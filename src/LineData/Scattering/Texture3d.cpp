#include "Texture3d.hpp"

#include <Utils/AppSettings.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Utils/Events/Stream/Stream.hpp>

inline float lerp(float a, float t, float b) {
    return a * (1.0 - t) + b * t;
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
    stream.read(grid.voxel_size_x);
    stream.read(grid.voxel_size_y);
    stream.read(grid.voxel_size_z);

    size_t num_floats =  grid.size_x * grid.size_y * grid.size_z;
    grid.data = new float[num_floats];
    stream.read(grid.data, num_floats * sizeof(float));

    return grid;
}

float Texture3D::sample_at(glm::vec3 pos) {
    // cubic interpolation

# define IDX(x, y, z) ((x * this->size_z * this->size_y) + (y * this->size_y) + z)

    float fw =  pos.x * (this->size_x - 1); // we want indices between 0 and len -1
    float fh =  pos.y * (this->size_y - 1); // we want indices between 0 and len -1
    float fd =  pos.z * (this->size_z - 1); // we want indices between 0 and len -1

    float tw = fw - (int)fw;
    float th = fh - (int)fh;
    float td = fd - (int)fd;

    uint32_t idx_w_left = floor(fw);
    uint32_t idx_h_low  = floor(fh);
    uint32_t idx_d_near = floor(fd);

    uint32_t idx_w_right = ceil(fw);
    uint32_t idx_h_high  = ceil(fh);
    uint32_t idx_d_far   = ceil(fd);

    float left_low_near   = this->data[IDX(idx_w_left,  idx_h_low,  idx_d_near)];
    float left_low_far    = this->data[IDX(idx_w_left,  idx_h_low,  idx_d_far)];
    float left_high_near  = this->data[IDX(idx_w_left,  idx_h_high, idx_d_near)];
    float left_high_far   = this->data[IDX(idx_w_left,  idx_h_high, idx_d_far)];
    float right_low_near  = this->data[IDX(idx_w_right, idx_h_low,  idx_d_near)];
    float right_low_far   = this->data[IDX(idx_w_right, idx_h_low,  idx_d_far)];
    float right_high_near = this->data[IDX(idx_w_right, idx_h_high, idx_d_near)];
    float right_high_far  = this->data[IDX(idx_w_right, idx_h_high, idx_d_far)];

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
