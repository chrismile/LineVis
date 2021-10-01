#pragma once
#include <string>
#include <stdint.h>
#include <glm/glm.hpp>

struct Texture3D {
    float* data;

    uint32_t size_x;
    uint32_t size_y;
    uint32_t size_z;

    float voxel_size_x;
    float voxel_size_y;
    float voxel_size_z;

    // ---------------------------- //

    float sample_at(glm::vec3 pos);
    void delete_maybe();
};

Texture3D load_xyz_file(std::string file_name);
