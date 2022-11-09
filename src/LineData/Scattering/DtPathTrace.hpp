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

#pragma once

#include <stdint.h>
#include <glm/glm.hpp>
#include "Texture3d.hpp"
#include "Loaders/DataSetList.hpp"
#include "LineDataScattering.hpp"
#include "Image.hpp"
#include "../LineData.hpp"


struct PathInfo {
    glm::vec3 camera_pos;
    glm::vec3 ray_direction;
    int32_t pass_number;
};

struct VolumeInfo {
    Texture3D grid;
    glm::vec3 extinction;
    glm::vec3 scattering_albedo;
    float g;
};


typedef std::vector<glm::vec3> Exit_Directions;

Image create_spherical_heatmap_image(KdTree<Empty>* kd_tree, uint32_t image_height);

void dt_path_trace(PathInfo path_info, VolumeInfo volume_info,
                   Trajectories* traj, Exit_Directions* exit_dirs);

namespace Random {
    void init(uint32_t seed);
}
