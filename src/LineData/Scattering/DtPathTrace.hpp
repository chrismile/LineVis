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

void write_bmp_file(const char* file_name, Exit_Directions* exit_dirs);
void dt_path_trace(PathInfo path_info, VolumeInfo volume_info,
                   Trajectories* traj, Exit_Directions* exit_dirs);

namespace Random {
    void init(uint32_t seed);
}
