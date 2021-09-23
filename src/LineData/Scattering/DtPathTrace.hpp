#pragma once

#include <stdint.h>
#include <glm/glm.hpp>
#include "Texture3d.hpp"
#include "Loaders/DataSetList.hpp"
#include "LineDataScattering.hpp"


// pi.x = cam_pos
// pi.w = ray_dir
struct PathInfo {
    glm::vec3 camera_pos;
    glm::vec3 ray_direction;
    int32_t pass_number;
};

struct VolumeInfo {
    Texture3D grid;
    glm::vec3 extinction;
    glm::vec3 scattering_albedo;
    glm::vec3 g;
};


// void GetGridBox(texture3D grid, float3& minim, float3& maxim);

void dt_path_trace(PathInfo path_info, VolumeInfo volume_info, Trajectories* traj);

namespace Random {
    void init(uint32_t seed);
}
