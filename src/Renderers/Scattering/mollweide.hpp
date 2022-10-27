/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Felix Brendel
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

#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <Math/Math.hpp>

typedef int8_t   s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

typedef uint8_t   u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef uint8_t byte;

typedef float       f32;
typedef double      f64;

template<class T>
T lerp(T point_a, float t, T point_b) {
    return point_a + t * (point_b - point_a);
}

template<class T>
T unlerp(T source_range_start, T source_point, T source_range_end) {
    return    (source_point - source_range_start) /
              /*--------------------------------------*/
              (source_range_end - source_range_start);
}

template<class T>
T remap(T source_range_start, T source_point, T source_range_end,
        T dest_range_start, T dest_range_end) {
    return lerp(dest_range_start,
                unlerp(source_range_start, source_point, source_range_end),
                dest_range_end);
}

template<class T>
T clamp01(T val) {
    if (val <= 0) {
        return 0;
    } else if (val >= 1.0f) {
        return 1;
    } else {
        return val;
    }
}

template <typename type>
struct Array_List {
    type* data;
    u32 length;
    u32 count;

    void init(u32 initial_capacity = 16) {
        data = (type*)malloc(initial_capacity * sizeof(type));
        count = 0;
        length = initial_capacity;
    }

    static Array_List<type> create_from(std::initializer_list<type> l) {
        Array_List<type> ret;
        ret.init_from(l);
        return ret;
    }

    void init_from(std::initializer_list<type> l) {
        length = l.size() > 1 ? l.size() : 1; // alloc at least one

        data = (type*)malloc(length * sizeof(type));
        count = 0;
        // TODO(Felix): Use memcpy here
        for (type t : l) {
            data[count++] = t;
        }
    }

    void extend(std::initializer_list<type> l) {
        // reserve((u32)l.size());
        // TODO(Felix): Use memcpy here
        for (type e : l) {
            append(e);
        }
    }

    void deinit() {
        free(data);
        data = nullptr;
    }

    void clear() {
        count = 0;
    }

    bool contains_linear_search(type elem) {
        for (u32 i = 0; i < count; ++i) {
            if (data[i] == elem)
                return true;
        }
        return false;
    }

    bool contains_binary_search(type elem) {
        return sorted_find(elem) != -1;
    }


    Array_List<type> clone() {
        Array_List<type> ret;
        ret.length = length;
        ret.count = count;

        ret.data = (type*)malloc(length * sizeof(type));
        // TODO(Felix): Maybe use memcpy here
        for (u32 i = 0; i < count; ++i) {
            ret.data[i] = data[i];
        }
        return ret;
    }

    void copy_values_from(Array_List<type> other) {
        // clear the array
        count = 0;
        // make sure we have allocated enough
        assure_allocated(other.count);
        // copy stuff
        count = other.count;
        memcpy(data, other.data, sizeof(type) * other.count);
    }

    type* begin() {
        return data;
    }

    type* end() {
        return data+(count);
    }

    void remove_index(u32 index) {
#ifdef FTB_INTERNAL_DEBUG
        if (index >= count) {
            fprintf(stderr, "ERROR: removing index that is not in use\n");
        }
#endif
        data[index] = data[--count];
    }

    void append(type element) {
        if (count == length) {
#ifdef FTB_INTERNAL_DEBUG
            if (length == 0) {
                fprintf(stderr, "ERROR: Array_List was not initialized.\n");
                length = 8;
            }
#endif
            length *= 2;

            data = (type*)realloc(data, length * sizeof(type));
        }
        data[count] = element;
        count++;
    }

    void assure_allocated(u32 allocated_elements) {
        if (length < allocated_elements) {
            if (allocated_elements > 1024) {
                length = allocated_elements;
            } else {
                // NOTE(Felix): find smallest power of two that is larger than
                // 'allocated_elements'
                for (u32 i = 9; i >= 1; --i) {
                    if  (allocated_elements > 1U << i) {
                        length = 1 << (i+1);
                        break;
                    }
                }
            }
            data = (type*)realloc(data, length * sizeof(type));
        }
    }


    void assure_available(u32 available_elements) {
        assure_allocated(available_elements+count);
    }

    operator bool() const {
        return count != 0;
    }

    type& operator[](u32 index) {
#ifdef FTB_DEBUG
        if (index >= length) {
            fprintf(stderr, "ERROR: Array_List access out of bounds (not even allocated).\n");
            debug_break();
        }
#endif
        return data[index];
    }


    type& last_element() {
#ifdef FTB_INTERNAL_DEBUG
        if (count == 0) {
            fprintf(stderr, "ERROR: Array_List was empty but last_element was called.\n");
        }
#endif
        return data[count-1];
    }

    typedef s32 (*compare_function_t)(const type*, const type*);
    typedef s32 (*void_compare_function_t)(const void*, const void*);
    typedef s32 (*pointer_compare_function_t)(const void**, const void**);
    void sort(compare_function_t comparer) {
        qsort(data, count, sizeof(type),
              (void_compare_function_t)comparer);
    }
    void sort(pointer_compare_function_t comparer) {
        qsort(data, count, sizeof(type),
              (void_compare_function_t)comparer);
    }

    s32 sorted_find(type elem,
                    compare_function_t compare_fun,
                    s32 left=-1, s32 right=-1)
    {
        if (left == -1) {
            return sorted_find(elem, compare_fun,
                               0, count - 1);
        } else if (left == right) {
            if (left < (s32)count) {
                if (compare_fun(&elem, &data[left]) == 0)
                    return left;
            }
            return -1;
        } else if (right < left)
            return -1;

        u32 middle = left + (right-left) / 2;

        s32 compare = compare_fun(&elem, &data[middle]);

        if (compare > 0)
            return sorted_find(elem, compare_fun, middle+1, right);
        if (compare < 0)
            return sorted_find(elem, compare_fun, left, middle-1);
        return middle;
    }

    s32 sorted_find(type elem,
                    pointer_compare_function_t compare_fun,
                    s32 left=-1, s32 right=-1)
    {
        return sorted_find(elem, (compare_function_t)compare_fun,
                           left, right);
    }

    bool is_sorted(compare_function_t compare_fun) {
        for (s32 i = 1; i < count; ++i) {
            if (compare_fun(&data[i-1], &data[i]) > 0)
                return false;
        }
        return true;
    }

    bool is_sorted(pointer_compare_function_t compare_fun) {
        return is_sorted((compare_function_t)compare_fun);
    }
};

template <typename type>
struct Bucket_Allocator {
    u32 next_index_in_latest_bucket;
    u32 next_bucket_index;
    u32 bucket_count;
    u32 bucket_size;

    Array_List<type*> free_list;
    type** buckets;

    void clear() {
        next_index_in_latest_bucket = 0;
        next_bucket_index = 0;
        free_list.clear();
    }

    void expand() {
        buckets = (type**)realloc(buckets, bucket_count * 2 * sizeof(type*));
        bucket_count *= 2;
    }

    void jump_to_next_bucket() {
        next_index_in_latest_bucket = 0;
        ++next_bucket_index;
        if (next_bucket_index >= bucket_count) {
            expand();
        }
        buckets[next_bucket_index] = (type*)malloc(bucket_size * sizeof(type));
    }

    void increment_pointers(s32 amount = 1) {
        next_index_in_latest_bucket += amount;
        if (next_index_in_latest_bucket >= bucket_size) {
            jump_to_next_bucket();
        }
    }

    void init(u32 bucket_size = 16, u32 initial_bucket_count = 8) {
        this->free_list.init();
        this->bucket_size = bucket_size;
        next_index_in_latest_bucket = 0;
        next_bucket_index = 0;
        bucket_count = initial_bucket_count;

        buckets = (type**)malloc(bucket_count * sizeof(type*));
        buckets[0] = (type*)malloc(bucket_size * sizeof(type));
    }

    void deinit() {
        for (u32 i = 0; i <= next_bucket_index; ++i) {
            free(buckets[i]);
        }
        this->free_list.deinit();
        free(buckets);
    }

    u32 count_elements() {
        // TODO(Felix): maybe we only need to take the last used element idx,
        //   with the next_bucket_index and next_index_in_latest_bucket nad
        //   subtract the length of the free list? So we dont have to sort it?
        auto voidp_cmp = [](const void** a, const void** b) -> s32 {
            return (s32)((byte*)*a - (byte*)*b);
        };

        free_list.sort(voidp_cmp);
        type* val;
        u32 count = 0;
        for (u32 i = 0; i < next_bucket_index; ++i) {
            for (u32 j = 0; j < bucket_size; ++j) {
                val = buckets[i]+j;
                if (free_list.sorted_find(val) == -1)
                    count++;
            }
        }
        for (u32 j = 0; j < next_index_in_latest_bucket; ++j) {
            val = buckets[next_bucket_index]+j;
            if (free_list.sorted_find(val) == -1)
                count++;
        }
        return count;
    }

    template <typename lambda>
    void for_each(lambda p) {
        auto voidp_cmp = [](const void** a, const void** b) -> s32 {
            return (s32)((byte*)*a - (byte*)*b);
        };

        free_list.sort(voidp_cmp);

        type* val;
        for (u32 i = 0; i < next_bucket_index; ++i) {
            for (u32 j = 0; j < bucket_size; ++j) {
                val = buckets[i]+j;
                if (free_list.sorted_find(val, voidp_cmp) == -1)
                    p(val);
            }
        }
        for (u32 j = 0; j < next_index_in_latest_bucket; ++j) {
            val = buckets[next_bucket_index]+j;
            if (free_list.sorted_find(val, voidp_cmp) == -1)
                p(val);
        }
    }

    type* allocate(u32 amount = 1) {
        type* ret;
        if (amount == 0) return nullptr;
        if (amount == 1) {
            if (free_list.count != 0) {
                return free_list.data[--free_list.count];
            }
            ret = buckets[next_bucket_index]+next_index_in_latest_bucket;
            increment_pointers(1);
            return ret;
        }
        if (amount > bucket_size)
            return nullptr;
        if ((bucket_size - next_index_in_latest_bucket) >= 4) {
            // if the current bucket is ahs enough free space
            ret = buckets[next_bucket_index]+(next_index_in_latest_bucket);
            increment_pointers(amount);
            return ret;
        } else {
            // the current bucket does not have enough free space
            // add all remainding slots to free list
            while (next_index_in_latest_bucket < bucket_size) {
                free_list.append(buckets[next_bucket_index]+next_index_in_latest_bucket);
                ++next_index_in_latest_bucket;
            }
            jump_to_next_bucket();
            return allocate(amount);
        }
    }

    void free_object(type* obj) {
        free_list.append(obj);
    }
};


template <typename type>
struct Bucket_List {
    Bucket_Allocator<type> allocator;

    void init(u32 bucket_size = 16, u32 initial_bucket_count = 8) {
        allocator.init(bucket_size, initial_bucket_count);
    }

    void deinit() {
        allocator.deinit();
    }

    void clear() {
        allocator.clear();
    }

    u32 count() {
        return
                allocator.next_bucket_index * allocator.bucket_size +
                allocator.next_index_in_latest_bucket;
    }

    void append(type elem) {
        type* mem = allocator.allocate();
        *mem = elem;
    }

    void extend(std::initializer_list<type> l) {
        for (type e : l) {
            append(e);
        }
    }

    void remove_index(u32 index) {
        u32 el_count = count();

#ifdef FTB_INTERNAL_DEBUG
        if (index >= el_count) {
            fprintf(stderr, "ERROR: removing index that is not in use\n");
        }
#endif

        type last = (*this)[el_count-1];

        (*this)[index] = last;

        if (allocator.next_index_in_latest_bucket == 0) {
            --allocator.next_bucket_index;
            allocator.next_index_in_latest_bucket = allocator.bucket_size-1;
        } else {
            --allocator.next_index_in_latest_bucket;
        }
    }

    type& operator[] (u32 index) {
#ifdef FTB_INTERNAL_DEBUG
        u32 el_count = count();
        panic_if(index >= el_count,
                 "ERROR: accessing index (%u) that is not in use (num elements: %u)\n",
                 index, el_count);
#endif

        u32 bucket_idx    = index / allocator.bucket_size;
        u32 idx_in_bucket = index % allocator.bucket_size;
        return allocator.buckets[bucket_idx][idx_in_bucket];
    }

    template <typename lambda>
    void for_each(lambda p) {
        allocator.for_each(p);
    }
};

template <typename type>
struct Bucket_Queue {
    Bucket_List<type> bucket_list;
    u32 start_idx;

    void init(u32 bucket_size = 16, u32 initial_bucket_count = 8) {
        bucket_list.init(bucket_size, initial_bucket_count);
        start_idx = 0;
    }

    void deinit() {
        bucket_list.deinit();
    }

    void push_back(type e) {
        bucket_list.append(e);
    }

    type get_next() {
#ifdef FTB_INTERNAL_DEBUG
        if (start_idx == 0 &&
            bucket_list.allocator.next_index_in_latest_bucket == 0 &&
            bucket_list.allocator.next_bucket_index == 0)
        {
            fprintf(stderr, "ERROR: queue has no next element\n");
        }
#endif

        type result = bucket_list[start_idx++];

        // maybe garbage collect left bucket
        if (start_idx >= bucket_list.allocator.bucket_size) {
            start_idx -= bucket_list.allocator.bucket_size;

            for (u32 i = 0; i < bucket_list.allocator.next_bucket_index; ++i) {
                bucket_list.allocator.buckets[i] = bucket_list.allocator.buckets[i+1];
            }
            --bucket_list.allocator.next_bucket_index;
        }


        // maybe reset if queue now empty
        if (start_idx == bucket_list.allocator.next_index_in_latest_bucket &&
            bucket_list.allocator.next_bucket_index == 0)
        {
            bucket_list.allocator.next_index_in_latest_bucket = 0;
            bucket_list.allocator.next_bucket_index = 0;
            start_idx = 0;
        }

        return result;


    }

    u32 count() {
        return bucket_list.count() - start_idx;
    }

    bool is_empty() {
        return count() == 0;
    }
};


void lat_lng_to_mollweide(f32 phi, f32 lambda, f32* out_x, f32* out_y);
void mollweide_to_lat_lng(f32 x, f32 y, f32* out_phi, f32* out_lambda);

template <typename Type>
struct Mollweide_Grid {
    struct Entry {
        float u; // [  -1;   1]
        float v; // [-0.5; 0.5]
        Type  payload;
    };

    u32 subdivision_level;
    Bucket_List<Entry>* buckets;

    void init(u32 subdivisions) {
        buckets = nullptr;
        subdivision_level = 0;
        change_subdivision_level(subdivisions);
    }

    void clear() {
        u32 cell_count = get_num_cells();
        for (u32 i = 0; i < cell_count; ++i) {
            if (buckets[i].allocator.buckets)
                buckets[i].clear();
        }
    }

    void deinit() {
        u32 cell_count = get_num_cells();
        for (u32 i = 0; i < cell_count; ++i) {
            if (buckets[i].allocator.buckets)
                buckets[i].deinit();
        }
        free(buckets);
    }

    void get_dimensions(u32* out_width, u32* out_height) {
        *out_width  = 1 << (subdivision_level + 1); // width  = 2 * 2 ** s_l
        *out_height = 1 << (subdivision_level);     // height = 2 ** s_l
    }

    u32 get_num_cells() {
        return 1 << (2 * subdivision_level + 1);
    }

    f32 find_theta(f32 lat) {
        f32 theta = lat;
        f32 old_t;
        do {
            old_t = theta;
            f32 denom = std::cos(theta);

            theta =  theta - (2*theta + std::sin(2*theta) - sgl::PI*std::sin(lat)) /
                             (4 * denom * denom);

        } while(std::abs(old_t - theta) > 0.001f);

        return theta;
    }

    // NOTE(Felix): point should be on the unit sphere (length == 1).
    void insert(glm::vec3 point, Type payload) {
        // calculate latitude and longiture from point
        f32 phi     = std::acos(clamp01(glm::dot(glm::normalize(glm::vec3{point.x, point.y, 0}), point)));      // [0; pi/2]
        f32 lambda  = std::acos(glm::dot(glm::normalize(glm::vec3{point.x, point.y, 0.0f}), {0.0f,1.0f,0.0f})); // [0; pi]

        if (point.z > 0)
            phi = -phi; // [-pi/2; pi/2]

        if (point.x < 0)
            lambda = -lambda; // [-pi; pi]

        f32 x, y;
        lat_lng_to_mollweide(phi, lambda, &x, &y);

        Entry new_entry {
                .u = x,
                .v = y,
                .payload = payload
        };

        insert(&new_entry);
    }

    void insert(Entry* e) {
        u32 grid_width, grid_height;
        get_dimensions(&grid_width, &grid_height);

        f32 f_x_idx = remap<float>(-1,   e->u,   1, 0, grid_width-0.001f);   // [0; grid_width-1]
        f32 f_y_idx = remap<float>(-0.5, e->v, 0.5, 0, grid_height-0.001f);  // [0; grid_height-1]

        u32 x_idx = (u32)f_x_idx;
        u32 y_idx = (u32)f_y_idx;

        u32 idx = grid_width * y_idx + x_idx;

        if (!buckets[idx].allocator.buckets) {
            buckets[idx].init(1024, 4);
        }

        buckets[idx].append(*e);
    }

    void change_subdivision_level(u32 new_subdivisions) {
        if (new_subdivisions == subdivision_level)
            return;

        auto old_buckets = buckets;
        u32 old_cell_count = get_num_cells();

        subdivision_level = new_subdivisions;
        u32 cell_count = get_num_cells();
        buckets = (Bucket_List<Entry>*)calloc(cell_count, sizeof(buckets[0]));

        for (u32 i = 0; i < cell_count; i++) {
            buckets[i].init();
        }

        if (old_buckets) {
            for (u32 i = 0; i < old_cell_count; ++i) {
                old_buckets[i].for_each([&] (Entry* e) {
                    insert(e);
                });

                old_buckets[i].deinit();
            }
            free(old_buckets);
        }
    }

    void range_query(glm::vec2 top_left, glm::vec2 bot_right, std::vector<Type>* list) {
        u32 grid_width, grid_height;
        get_dimensions(&grid_width, &grid_height);

        u32 box_left_index  = remap<float>(
                -1,  top_left.x, 1, 0, grid_width-0.001); // [0; grid_width-1]
        u32 box_right_index = remap<float>(
                -1, bot_right.x, 1, 0, grid_width-0.001); // [0; grid_width-1]

        u32 box_top_index = remap<float>(
                -0.5,  top_left.y, 0.5, 0, grid_height-0.001); // [0; grid_height-1]
        u32 box_bot_index = remap<float>(
                -0.5, bot_right.y, 0.5, 0, grid_height-0.001); // [0; grid_height-1]

        for (int y = box_top_index; y <= box_bot_index; ++y) {
            for (int x = box_left_index; x <= box_right_index; ++x) {
                u32 idx = grid_width * y + x;
                // continue if empty cell
                if (!buckets[idx].allocator.buckets)
                    continue;

                if (x == box_left_index || x == box_right_index ||
                    y == box_top_index  || y == box_bot_index)
                {
                    // have to check all
                    buckets[idx].for_each([=](Entry* elem) {
                        if (elem->u >= top_left.x && elem->u <= bot_right.x &&
                            elem->v >= top_left.y && elem->v <= bot_right.y)
                        {
                            list->append(elem->payload);
                        }
                    });
                } else {
                    // just include all
                    buckets[idx].for_each([=](Entry* elem) {
                        list->append(elem->payload);
                    });
                }
            }
        }
    }

    // NOTE(Felix): Retruned image uses 32 bit per pixel rgba. It is heap
    //   allocated and has to be freed by the user. The width will be 2*height.
    //
    //   Protip: The rendered image does not suffer from resampling issues when
    //   'height' is a multiple of the grid height (2 ** subdiv_level)
    auto render_heatmap(u32 height) -> u8* {
        struct Pixel {
            u8 r;
            u8 g;
            u8 b;
            u8 a;
        };

        glm::vec3 black = {0,0,0};
        glm::vec3 blue  = {0,0,1};
        glm::vec3 green = {0,1,0};
        glm::vec3 red   = {1,0,0};
        glm::vec3 white = {1,1,1};

        u32 width = 2 * height;

        u32 max_occupancy = 0;
        u32 cell_count = get_num_cells();
        for (u32 i = 0; i< cell_count; ++i) {
            if (buckets[i].allocator.buckets) {
                u32 count = buckets[i].count();
                if (count > max_occupancy)
                    max_occupancy = count;
            }
        }

        Pixel* pixels = (Pixel*)malloc(width * height * sizeof(pixels[0]));

        u32 grid_width, grid_height;
        get_dimensions(&grid_width, &grid_height);

        for (u32 y = 0; y < height; ++y) {
            for (u32 x = 0; x < width; ++x) {

                u32 b_x_idx = remap<float>(0, x, width,  0, grid_width);
                u32 b_y_idx = remap<float>(0, y, height, 0, grid_height);

                if (b_x_idx >= grid_width)
                    b_x_idx = grid_width-1;

                if (b_y_idx >= grid_height)
                    b_y_idx = grid_height-1;

                u32 b_idx = b_y_idx * grid_width + b_x_idx;

                u32 occu = 0;
                if (buckets[b_idx].allocator.buckets)
                    occu = buckets[b_idx].count();

                f32 norm_occ = (f32)occu / (f32)max_occupancy;

                glm::vec3 color;
                if (norm_occ < 0.25) {
                    color = glm::mix(black, blue, norm_occ*4);
                } else if (norm_occ < 0.5) {
                    color = glm::normalize(glm::mix(blue, green, (norm_occ-0.25)*4));
                } else if (norm_occ < 0.75) {
                    color = glm::normalize(glm::mix(green, red, (norm_occ-0.5)*4));
                } else {
                    color = glm::mix(red, white, (norm_occ-0.75)*4);
                }

                pixels[width * y + x] = {
                        .r = (u8)(color.r*255),
                        .g = (u8)(color.g*255),
                        .b = (u8)(color.b*255),
                        .a = 255,
                };
            }
        }

        return (u8*)pixels;
    }
};

#ifdef FTB_MOLLWEIDE_IMPL

void lat_lng_to_mollweide(f32 phi, f32 lambda, f32* out_x, f32* out_y) {
    f32 theta = phi;
    f32 old_t;
    do {
        old_t = theta;
        f32 denom = std::cos(theta);

        theta = theta - (2*theta + std::sin(2*theta) - sgl::PI*std::sin(phi)) /
                        (4 * denom * denom);

    } while(std::abs(old_t - theta) > 0.001f);

    *out_x = lambda * std::cos(theta) / sgl::PI; // [-1;     1]
    *out_y = std::sin(theta) / 2;           // [-0.5; 0.5]
}

void mollweide_to_lat_lng(f32 x, f32 y, f32* out_phi, f32* out_lambda) {
    f32 sqrt_2   = sqrtf(2.0f);
    f32 theta    = std::asin(y/sqrt_2);
    *out_phi     = std::asin((2.0f*theta + std::sin(2.0f*theta))/sgl::PI);
    *out_lambda  = (x*sgl::PI)/(2.0f*sqrt_2*std::cos(theta));
}

#endif