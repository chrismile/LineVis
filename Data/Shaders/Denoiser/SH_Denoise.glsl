-- header

// NOTE(Felix): These are set in cpp
// #define HASH_MAP_SIZE 10'000'000
#define LINEAR_SEARCH_LENGTH 10
#define SPACE_STRETCH 1

struct HM_Cell {
#if defined(SUPPORT_BUFFER_FLOAT_ATOMIC_ADD) || defined(FORCE_FLOAT)
    float ao_value;             // Accumulated AO value
#else
    uint ao_value;             // Accumulated AO value
#endif
    uint contribution_counter;  // Number of contributions to this cell
    uint checksum;              // Checksum for determining hash-collisions
    uint rc;                    // Replacement counter
    // float s_wd;
    // vec3 padding;
};

layout(scalar, binding = 0) uniform uniform_data_buffer {
    vec4  cam_pos;
    float f;       // camera aperture
    float s_nd;    // normal coarseness
    float s_p;     // user-defined level of coarseness in pixel
    float s_min;   // user-defined level of coarseness in pixel
    vec4 show_grid_cells;
};

uint wang_hash(uint seed) {
    seed = (seed ^ 61) ^ (seed >> 16);
    seed *= 9;
    seed = seed ^ (seed >> 4);
    seed *= 0x27d4eb2d;
    seed = seed ^ (seed >> 15);
    return seed;
}

uint murmur_hash(uint x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 13;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

void jenkins_one_at_a_time_hash_f(float data, out uint hash) {
    uint d = floatBitsToUint(data);
    for (int i = 0; i < 4; ++i) {
        uint b = d & 0xFF;
        d = d >> 8;

        hash += b;
        hash += hash << 10;
        hash ^= hash >> 6;
    }
}

uint jenkins_one_at_a_time_hash(vec3 position, vec3 normal, float swd) {
    uint hash = 0;

    jenkins_one_at_a_time_hash_f(swd, hash);
    jenkins_one_at_a_time_hash_f(position.x, hash);
    jenkins_one_at_a_time_hash_f(position.y, hash);
    jenkins_one_at_a_time_hash_f(position.z, hash);
    jenkins_one_at_a_time_hash_f(normal.x, hash);
    jenkins_one_at_a_time_hash_f(normal.y, hash);
    jenkins_one_at_a_time_hash_f(normal.z, hash);

    hash += hash << 3;
    hash ^= hash >> 11;
    hash += hash << 15;

    return hash;
}

// vec4 FAST_32_hash( float x )
// {
//     vec2 gridcell = vec2(x, 3.141592653);

//     //    gridcell is assumed to be an integer coordinate
//     const vec2 OFFSET = vec2( 26.0, 161.0 );
//     const float DOMAIN = 71.0;
//     const float SOMELARGEFLOAT = 951.135664;
//     vec4 P = vec4( gridcell.xy, gridcell.xy + 1.0.xx );
//     P = P - floor(P * ( 1.0 / DOMAIN )) * DOMAIN;    //    truncate the domain
//     P += OFFSET.xyxy;                                //    offset to interesting part of the noise
//     P *= P;                                          //    calculate and return the hash
//     return fract( P.xzxz * P.yyww * ( 1.0 / SOMELARGEFLOAT.x ).xxxx );
// }

// Hash position at cell size
uint H4D(vec3 position, float s_wd){
    // Clamp to smallest cell size
    s_wd = max(s_wd, s_min);

    return wang_hash(floatBitsToUint(s_wd)
         + wang_hash(floatBitsToUint(floor(position.z / s_wd))
         + wang_hash(floatBitsToUint(floor(position.y / s_wd))
         + wang_hash(floatBitsToUint(floor(position.x / s_wd))))));
}

// Actual all-inclusive hash function for normal + position
uint H7D(vec3 position, vec3 normal, float s_wd, float s_nd){
    normal = normalize(normal) * s_nd;
    s_nd = 0.0f; // HACK(Felix): remove this
    ivec3 normal_d = ivec3(normal);
    return
         + wang_hash(floatBitsToUint(s_nd + normal_d.z)
         + wang_hash(floatBitsToUint(s_nd + normal_d.y)
         + wang_hash(floatBitsToUint(s_nd + normal_d.x)
         + H4D(position, s_wd)
                    )))
                    ;
}

//
// Checksum functions
//

// // Actual all-inclusive checksum function for normal + position
// uint H7D_checksum(vec3 position, vec3 normal, float s_wd, float s_nd){
    // return jenkins_one_at_a_time_hash(position, ivec3(normal * s_nd), s_wd);
// }

// Hash position at cell size
uint H4D_checksum(vec3 position, float s_wd){
    // Clamp to smallest cell size
    s_wd = max(s_wd, s_min);

    return wang_hash(floatBitsToUint(floor(position.x / s_wd))
         + wang_hash(floatBitsToUint(floor(position.y / s_wd))
         + wang_hash(floatBitsToUint(floor(position.z / s_wd))
         + wang_hash(floatBitsToUint(s_wd)))));
}

// // Actual all-inclusive hash function for normal + position
uint H7D_checksum(vec3 position, vec3 normal, float s_wd, float s_nd){
    normal = normalize(normal) * s_nd;
    s_nd = 0.0f; // HACK(Felix): remove this
    ivec3 normal_d = ivec3(normal);
    return
        wang_hash(floatBitsToUint(s_nd + normal_d.x)
         + wang_hash(floatBitsToUint(s_nd + normal_d.y)
         + wang_hash(floatBitsToUint(s_nd + normal_d.z)
         + H4D_checksum(position, s_wd)
        )))
        ;
}

uint pow2[] = {1, 2, 4, 8, 16, 32, 64, 128};


vec3 _dismemberUint3(uint x){
    uint res[3];
    res[0] = (x & 0xFFC00000) >> 22;
    res[1] = (x & 0x003FF800) >> 11;
    res[2] = (x & 0x000007FF);
    return vec3(res[0], res[1], res[2]);
}


vec4 hash_to_color(uint hash){
    vec3 dis = _dismemberUint3(hash);

    return vec4(dis.x / 1023, dis.y / 2047, dis.z / 2047, 1);
}

//
// Calculate cell size in world-space
//

float s_wd_calc(vec3 position, vec2 res) {
    float dis = distance(position, cam_pos.xyz * SPACE_STRETCH);
    // float s_w = dis * tan(s_p * f * max(1/res.x, (res.x / (res.y*res.y))));
    float s_w = dis * tan(s_p * 1/res.y);
    float log_step = floor(log2(s_w / s_min));
    return pow(2, log_step) * s_min;
}


// --------------------------------------
//           Write Pass
// --------------------------------------
-- Compute-Write

#version 450
#extension GL_EXT_scalar_block_layout : enable
#extension GL_EXT_shader_atomic_float : require

#import ".header"

layout(local_size_x = BLOCK_SIZE, local_size_y = BLOCK_SIZE) in;


layout(scalar, binding = 1) buffer hash_map_buffer {
    HM_Cell hm_cells[/*HASH_MAP_SIZE*/];
};

layout(binding = 2) uniform sampler2D noisy_texture;
layout(binding = 3) uniform sampler2D normal_texture;
layout(binding = 4) uniform sampler2D position_texture;

layout(push_constant) uniform PushConstants {
    uint use_atomics;
};

void atomicAddAoValue(uint final_index, float value) {
#ifdef SUPPORT_BUFFER_FLOAT_ATOMIC_ADD
    atomicAdd(hm_cells[final_index].ao_value, value);
#else
    uint oldValue = hm_cells[final_index].ao_value;
    uint expectedValue, newValue;
    do {
        expectedValue = oldValue;
        newValue = floatBitsToUint(uintBitsToFloat(oldValue) + value);
        oldValue = atomicCompSwap(hm_cells[final_index].ao_value, expectedValue, newValue);
    } while (oldValue != expectedValue);
#endif
}

void main() {
    ivec2 i_pos = ivec2(gl_GlobalInvocationID.xy);
    ivec2 size = textureSize(noisy_texture, 0);
    if (i_pos.x >= size.x || i_pos.y >= size.y) {
        return;
    }

    vec3  position  = texelFetch(position_texture, i_pos, 0).xyz;
    vec3  normal    = texelFetch(normal_texture,   i_pos, 0).xyz;
    float occlusion = texelFetch(noisy_texture,    i_pos, 0).x;

    position *= SPACE_STRETCH;

    if (normal == vec3(0))
        return;

    float s_wd = s_wd_calc(position, size);

    for(int l = 0; l < 4; ++l) {
        float s_wd_it = s_wd * pow2[l];

        uint hash = H7D(position, normal, s_wd_it, s_nd);
        uint checksum = H7D_checksum(position, normal, s_wd_it, s_nd);

        bool slot_found = false;
        bool clear_slot_before_use = false;
        uint rc_collection[LINEAR_SEARCH_LENGTH];

        uint final_index;

         // Linear Search
        for(int i = 0; i < LINEAR_SEARCH_LENGTH; i++){
            uint idx_here = (hash + i) % HASH_MAP_SIZE;
            HM_Cell read_cell_i = hm_cells[idx_here];

            if(read_cell_i.contribution_counter == 0){
                // Found empty hashcell => create new entry
                final_index = idx_here;
                slot_found = true;
                clear_slot_before_use = true;
                break;
            } else if(read_cell_i.checksum == checksum){
                // Found fitting entry => add samples
                final_index = idx_here;
                slot_found = true;
                break;
            } else {
                // Found differing entry => continue search
                rc_collection[i] = read_cell_i.rc;
                hm_cells[idx_here].rc = read_cell_i.rc + 1;
            }
        }

        if (!slot_found) {
            // No space to insert value found => replace oldest entry
            uint final_offset = 0;
            uint oldest_entry = rc_collection[0];

            for(int i = 1; i < LINEAR_SEARCH_LENGTH; i++){
                if(rc_collection[i] > oldest_entry){
                    final_offset = i;
                    oldest_entry = rc_collection[i];
                }
            }

            // Found oldest entry to replace
            final_index = (hash + final_offset) % HASH_MAP_SIZE;
            clear_slot_before_use = true;
        }

        if (clear_slot_before_use) {
            hm_cells[final_index].ao_value             = 0;
            hm_cells[final_index].contribution_counter = 0;
            hm_cells[final_index].rc                   = 0;
            hm_cells[final_index].checksum             = checksum;
            // hm_cells[final_index].s_wd                 = s_wd_it;
        }

        if (use_atomics != 0) {
            atomicAddAoValue(final_index, occlusion);
            //atomicAdd(hm_cells[final_index].ao_value, occlusion);
            atomicAdd(hm_cells[final_index].contribution_counter, 1);
        } else {
#ifdef SUPPORT_BUFFER_FLOAT_ATOMIC_ADD
            hm_cells[final_index].ao_value += occlusion;
#else
            hm_cells[final_index].ao_value = floatBitsToUint(uintBitsToFloat(hm_cells[final_index].ao_value) + occlusion);
#endif
            hm_cells[final_index].contribution_counter += 1;
        }
    }
}

// --------------------------------------
//              Read Pass
// --------------------------------------
-- Compute-Read

#version 450
#extension GL_EXT_scalar_block_layout : enable

layout(local_size_x = BLOCK_SIZE, local_size_y = BLOCK_SIZE) in;

#define FORCE_FLOAT
#import ".header"

layout(scalar, binding = 1) readonly buffer hash_map_buffer {
    HM_Cell hm_cells[/*HASH_MAP_SIZE*/];
};

layout(binding = 2, rgba32f) uniform image2D temp_accum_texture;
layout(binding = 3) uniform sampler2D position_texture;
layout(binding = 4) uniform sampler2D normal_texture;

vec3 unlerp(vec3 a, vec3 x, vec3 b) {
    return (x - a) / (b - a);
}

vec3 remap(vec3 a, vec3 x, vec3 b, vec3 c, vec3 d) {
    return mix(c, d, unlerp(a, x, b));
}

void main() {
    ivec2 i_pos = ivec2(gl_GlobalInvocationID.xy);
    ivec2 size = imageSize(temp_accum_texture);
    if (i_pos.x >= size.x || i_pos.y >= size.y) {
        return;
    }

    vec3 position = texelFetch(position_texture, i_pos, 0).xyz;
    vec3 normal   = texelFetch(normal_texture, i_pos, 0).xyz;

    position *= SPACE_STRETCH;

    if (normal == vec3(0))
        return;


    const uint coarseness_level_increase = 0; // NOTE(Felix): same as Christiane's
    float s_wd = s_wd_calc(position, size) * pow2[coarseness_level_increase];

    // Sample coarser levels when not enough samples
    // const int min_nr_samples = config.min_nr_samples;
    const int min_nr_samples = 60; // NOTE(Felix): same as Christiane's
    uint current_samples = 0;
    float ao = 0;

    for(int l = 0; l < 4; ++l) {

        float s_wd_it = s_wd * pow2[l];

        uint hash     = H7D(position, normal, s_wd_it, s_nd);
        uint checksum = H7D_checksum(position, normal, s_wd_it, s_nd);

        for(int i = 0; i < LINEAR_SEARCH_LENGTH; ++i){
            uint index_here = (hash + i) % HASH_MAP_SIZE;
            HM_Cell read_cell_i = hm_cells[index_here];

            if(read_cell_i.checksum == checksum){
                // NOTE(Felix): make sure this cell is valid; Only ao >=
                //   read_cell_i.contribution_counter are equal since each
                //   sample can have a max ao of 1
                // if (read_cell_i.ao_value - 0.01 <= read_cell_i.contribution_counter) {
                    current_samples += read_cell_i.contribution_counter;
                    ao += read_cell_i.ao_value;
                // }
                break;
            }
        }

        if(current_samples >= min_nr_samples){
            // Enough samples => stop finding more values
            break;
        }
    }

    float final_ao = 0;
    if (current_samples > 0)
        final_ao = ao / current_samples;


    // imageStore(temp_accum_texture, i_pos, vec4(final_ao));

    // imageStore(temp_accum_texture, i_pos, mix(vec4(min(1, final_ao), ao, current_samples, s_wd),
    imageStore(temp_accum_texture, i_pos, mix(vec4(final_ao, ao, current_samples, s_wd),
                                              vec4(hash_to_color(H7D(position, normal, s_wd, s_nd)).xy, vec4(hash_to_color(H7D_checksum(position, normal, s_wd, s_nd))).xy),
                                              show_grid_cells.x));
        // imageStore(temp_accum_texture, i_pos, vec4(remap(vec3(-0.25), position.zzz, vec3(0.25), vec3(0) , vec3(1)), 0));
        // imageStore(temp_accum_texture, i_pos, vec4(remap(vec3(-1), normal.zzz, vec3(1), vec3(0) , vec3(1)), 0));
    // }
    // imageStore(temp_accum_texture, i_pos, vec4(min(1, ao)));
    // imageStore(temp_accum_texture, i_pos, vec4(ao));

}
