/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Felix Brendel, Christoph Neuhauser
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Texture3d.hpp"

#ifdef USE_TBB
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#endif

#include <Utils/AppSettings.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Utils/Parallel/Reduction.hpp>
#include <Utils/Events/Stream/Stream.hpp>
#include <Utils/Defer.hpp>

inline float lerp(float a, float t, float b) {
    return a * (1.0f - t) + b * t;
}

Texture3D load_xyz_file(std::string file_name) {
    uint8_t* fileBuffer = nullptr;
    size_t bufferSize = 0;
    bool loaded = sgl::loadFileFromSource(file_name, fileBuffer, bufferSize, true);
    if (!loaded) {
        sgl::Logfile::get()->writeError(
            "Error in ScatteringLineTracingRequester::traceLines:"
            " Couldn't load data from grid data set file \""
            + file_name + "\".");
        return {};
    }

    Texture3D grid {};

    sgl::BinaryReadStream stream(fileBuffer, bufferSize);
    stream.read(grid.size_x);
    stream.read(grid.size_y);
    stream.read(grid.size_z);
    double voxel_size_x = 1.0, voxel_size_y = 1.0, voxel_size_z = 1.0;
    stream.read(voxel_size_x);
    stream.read(voxel_size_y);
    stream.read(voxel_size_z);
    grid.voxel_size_x = float(voxel_size_x);
    grid.voxel_size_y = float(voxel_size_y);
    grid.voxel_size_z = float(voxel_size_z);

    size_t num_floats =  grid.size_x * grid.size_y * grid.size_z;

    grid.data = new float[num_floats];
    float* transposed_data = new float[num_floats];
    defer {
        delete[] transposed_data;
    };

    stream.read(transposed_data, num_floats * sizeof(float));

    // NOTE(Felix): Since the .xyz file stores the densities in zyx order, and
    //   we want to have them in xyz order, we are transposing them here.
#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<uint32_t>(0, grid.size_z), [&](auto const& r) {
        for (auto z = r.begin(); z != r.end(); z++) {
#else
#if _OPENMP >= 201107
    #pragma omp parallel for shared(grid, transposed_data) default(none)
#endif
    for (uint32_t z = 0; z < grid.size_z; z++) {
#endif
        for (uint32_t y = 0; y < grid.size_y; y++) {
            for (uint32_t x = 0; x < grid.size_x; x++) {
                grid.data[x + (y + z * grid.size_y) * grid.size_x] =
                    transposed_data[z + (y + x * grid.size_y) * grid.size_z];
            }
        }
    }
#ifdef USE_TBB
    });
#endif

    // Normalization
    {
        // LLVM raises error "capturing a structured binding is not yet supported in OpenMP" when using bindings directly.
        auto [min_val_binding, max_val_binding] = sgl::reduceFloatArrayMinMax(
                grid.data, num_floats, std::make_pair(0.0f, std::numeric_limits<float>::lowest()));
        const float min_val = min_val_binding;
        const float max_val = max_val_binding;

#ifdef USE_TBB
        tbb::parallel_for(tbb::blocked_range<size_t>(0, num_floats), [&](auto const& r) {
            for (auto i = r.begin(); i != r.end(); i++) {
#else
#if _OPENMP >= 201107
        #pragma omp parallel for shared(num_floats, grid, min_val, max_val) default(none)
#endif
        for (size_t i = 0; i < num_floats; i++) {
#endif
            grid.data[i] = (grid.data[i] - min_val) / (max_val - min_val);
        }
#ifdef USE_TBB
        });
#endif
    }

    return grid;
}

float Texture3D::sample_at(glm::vec3 pos) {
    // cubic interpolation

# define IDX(x, y, z) ((x) + ((y) + (z) * size_y) * size_x)

    float fw =  pos.x * (this->size_x - 1); // we want indices between 0 and len -1
    float fh =  pos.y * (this->size_y - 1); // we want indices between 0 and len -1
    float fd =  pos.z * (this->size_z - 1); // we want indices between 0 and len -1

    float tw = fw - (int)fw;
    float th = fh - (int)fh;
    float td = fd - (int)fd;

    uint32_t idx_w_left = uint32_t(floor(fw));
    uint32_t idx_h_low  = uint32_t(floor(fh));
    uint32_t idx_d_near = uint32_t(floor(fd));

    uint32_t idx_w_right = uint32_t(ceil(fw));
    uint32_t idx_h_high  = uint32_t(ceil(fh));
    uint32_t idx_d_far   = uint32_t(ceil(fd));

    int idx_left_low_near   = IDX(idx_w_left,  idx_h_low,  idx_d_near);
    int idx_left_low_far    = IDX(idx_w_left,  idx_h_low,  idx_d_far);
    int idx_left_high_near  = IDX(idx_w_left,  idx_h_high, idx_d_near);
    int idx_left_high_far   = IDX(idx_w_left,  idx_h_high, idx_d_far);
    int idx_right_low_near  = IDX(idx_w_right, idx_h_low,  idx_d_near);
    int idx_right_low_far   = IDX(idx_w_right, idx_h_low,  idx_d_far);
    int idx_right_high_near = IDX(idx_w_right, idx_h_high, idx_d_near);
    int idx_right_high_far  = IDX(idx_w_right, idx_h_high, idx_d_far);

    float left_low_near   = this->data[idx_left_low_near];
    float left_low_far    = this->data[idx_left_low_far];
    float left_high_near  = this->data[idx_left_high_near];
    float left_high_far   = this->data[idx_left_high_far];
    float right_low_near  = this->data[idx_right_low_near];
    float right_low_far   = this->data[idx_right_low_far];
    float right_high_near = this->data[idx_right_high_near];
    float right_high_far  = this->data[idx_right_high_far];

    float low_near = lerp(left_low_near, tw, right_low_near);
    float low_far  = lerp(left_low_far,  tw, right_low_far);
    float low      = lerp(low_near,      td, low_far);

    float high_near = lerp(left_high_near, tw, right_high_near);
    float high_far  = lerp(left_high_far,  tw, right_high_far);
    float high      = lerp(high_near,      td, high_far);

    return lerp(low, th, high);

# undef IDX
}

void Texture3D::delete_maybe() {
    if (data) {
        delete[] data;
        data = nullptr;
    }
}
