/**
 * MIT License
 *
 * Copyright (c) 2021-2022, Christoph Neuhauser, Ludwig Leonard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * For more details on decomposition tracking, please refer to:
 * P. Kutz, R. Habel, Y. K. Li, and J. Novák. Spectral and decomposition tracking for rendering heterogeneous volumes.
 * ACM Trans. Graph., 36(4), Jul. 2017.
 */

#ifdef USE_DECOMPOSITION_TRACKING

#if !defined(USE_NANOVDB) || defined(USE_SUPER_VOXEL_GRID)

vec3 analogDecompositionTracking(vec3 x, vec3 w, out ScatterEvent firstEvent) {
    firstEvent = ScatterEvent(false, x, 0.0, w, 0.0);

#ifdef USE_NANOVDB
    pnanovdb_readaccessor_t accessor = createAccessor();
#endif

    const vec3 EPSILON_VEC = vec3(1e-6);
    float tMinVal, tMaxVal;
    if (rayBoxIntersect(parameters.boxMin + EPSILON_VEC, parameters.boxMax - EPSILON_VEC, x, w, tMinVal, tMaxVal)) {
        float majorant = parameters.extinction.x;
        float absorptionAlbedo = 1.0 - parameters.scatteringAlbedo.x;

        ivec3 voxelGridSize = textureSize(gridImage, 0);
        vec3 boxDelta = parameters.boxMax - parameters.boxMin;
        vec3 superVoxelSize = parameters.superVoxelSize * boxDelta / voxelGridSize;

        x += w * tMinVal;
        vec3 startPoint = (x - parameters.boxMin) / boxDelta * voxelGridSize / parameters.superVoxelSize;
        ivec3 superVoxelIndex = ivec3(floor(startPoint));

        ivec3 cachedSuperVoxelIndex = ivec3(-1, -1, -1);
        vec2 superVoxelMinMaxDensity = vec2(0.0, 0.0);

        // Loop over all super voxels along the ray.
        while (all(greaterThanEqual(superVoxelIndex, ivec3(0))) && all(lessThan(superVoxelIndex, parameters.superVoxelGridSize))) {
            vec3 minSuperVoxelPos = parameters.boxMin + superVoxelIndex * superVoxelSize;
            vec3 maxSuperVoxelPos = minSuperVoxelPos + superVoxelSize;

            float tMinSuperVoxel = 0.0, tMaxSuperVoxel = 0.0;
            rayBoxIntersect(minSuperVoxelPos, maxSuperVoxelPos, x, w, tMinSuperVoxel, tMaxSuperVoxel);
            float d_max = tMaxSuperVoxel - tMinSuperVoxel; // + 1e-7
            x += w * tMinSuperVoxel;

            if (cachedSuperVoxelIndex != superVoxelIndex) {
                superVoxelMinMaxDensity = texelFetch(superVoxelGridImage, superVoxelIndex, 0).xy;
                cachedSuperVoxelIndex = superVoxelIndex;
            }
            if (superVoxelMinMaxDensity.y < 1e-5) {
                x += w * d_max;
            } else {
                float mu_c_t = max(0.0000000001, majorant * superVoxelMinMaxDensity.x);
                float majorant_r_local = max(0.0000000001, majorant * superVoxelMinMaxDensity.y - mu_c_t);
                //float mu_c_t = 0.0000000001;
                //float majorant_r_local = max(0.0000000001, majorant * 1 - mu_c_t);

                float t_c = -log(max(0.0000000001, 1 - random())) / mu_c_t;
                float t_r = 0.0;

                while (true) {
                    t_r -= log(max(0.0000000001, 1 - random())) / majorant_r_local;

                    if (t_c >= d_max && t_r >= d_max) {
                        x = x + d_max * w;
                        break; // null collision, proceed to next super voxel
                    }

                    vec3 xs = x + w * min(t_c, t_r);
                    bool isCollision = false;
                    if (t_c <= t_r) {
                        isCollision = true;
                    } else {
                        float density = sampleCloud(xs);
                        isCollision = random() * majorant_r_local < parameters.extinction.x * density - mu_c_t;
                    }

                    if (isCollision) {
                        x = xs;

                        if (random() < absorptionAlbedo) {
                            return vec3(0.0); // absorption event/emission
                        }

                        float pdf_w;
                        w = importanceSamplePhase(parameters.phaseG, w, pdf_w);
                        t_r = 0.0;
                        t_c = -log(max(0.0000000001, 1 - random())) / mu_c_t;
                        rayBoxIntersect(minSuperVoxelPos, maxSuperVoxelPos, x, w, tMinSuperVoxel, tMaxSuperVoxel);
                        d_max = tMaxSuperVoxel - tMinSuperVoxel; // + 1e-7
                    }
                }
            }

            vec3 cellCenter = (minSuperVoxelPos + maxSuperVoxelPos) * 0.5;
            vec3 mov = x + w * 0.00001 - cellCenter;
            vec3 smov = sign(mov);
            mov *= smov;

            ivec3 dims = ivec3(mov.x >= mov.y && mov.x >= mov.z, mov.y >= mov.x && mov.y >= mov.z, mov.z >= mov.x && mov.z >= mov.y);
            superVoxelIndex += dims * ivec3(smov);
        }
    }

    return sampleSkybox(w) + sampleLight(w);
}

#else

/**
 * Analog decomposition tracking implemented using NanoVDB's HDDA algorithm for traversing the sparse grid.
 */
vec3 analogDecompositionTracking(vec3 x, vec3 w, out ScatterEvent firstEvent) {
    firstEvent = ScatterEvent(false, x, 0.0, w, 0.0);

    pnanovdb_readaccessor_t accessor = createAccessor();
    pnanovdb_buf_t buf = pnanovdb_buf_t(0);
    pnanovdb_grid_handle_t gridHandle = pnanovdb_grid_handle_t(pnanovdb_address_null());

    // Convert world space to index space.
    vec3 origin = pnanovdb_grid_world_to_indexf(buf, gridHandle, x);
    vec3 direction = pnanovdb_grid_world_to_index_dirf(buf, gridHandle, w);
    float gridToWorldScale = 1.0 / length(direction);
    direction *= gridToWorldScale;
    float tMin = 0.0;
    float tMax = 1e10;

    // Check whether the ray hits the root node bounding box.
    ivec3 bbox_min = pnanovdb_root_get_bbox_min(buf, accessor.root);
    ivec3 bbox_max = pnanovdb_root_get_bbox_max(buf, accessor.root);
    vec3 bbox_minf = pnanovdb_coord_to_vec3(bbox_min);
    vec3 bbox_maxf = pnanovdb_coord_to_vec3(pnanovdb_coord_add(bbox_max, pnanovdb_coord_uniform(1)));
    bool hitsRootBoundingBox = pnanovdb_hdda_ray_clip(bbox_minf, bbox_maxf, origin, tMin, direction, tMax);

    if (hitsRootBoundingBox && tMax < 1e20) {
        float majorant = parameters.extinction.x;
        float absorptionAlbedo = 1.0 - parameters.scatteringAlbedo.x;

        /*
         * The lowest level is usually either 1 (i.e., iterating over the individual voxels) or 8 (the leaf nodes).
         * Level 1 is used if we have a really small grid, otherwise we will use the leaf nodes of size 8x8x8 voxels
         * as regions with homogeneous control component.
         */
        int lowestLevel = parameters.superVoxelSize.x;

        pnanovdb_vec3_t pos = pnanovdb_hdda_ray_start(origin, tMin - 1e-4, direction);
        pnanovdb_coord_t ijk = pnanovdb_hdda_pos_to_ijk(PNANOVDB_REF(pos));
        pnanovdb_int32_t dim = pnanovdb_uint32_as_int32(pnanovdb_readaccessor_get_dim(
                PNANOVDB_GRID_TYPE_FLOAT, buf, accessor, ijk));
        dim = lowestLevel;

        pnanovdb_hdda_t hdda;
        pnanovdb_hdda_init(hdda, origin, tMin - 1e-4, direction, tMax + 1e-4, dim);

        // Iterate over all leaves/voxels using the hierarchical DDA algorithm.
        int iteration = 0;
        while (true) {
            if (iteration > 0) {
                // Did we leave the domain?
                if (!pnanovdb_hdda_step(hdda)) {
                    break;
                }

                pnanovdb_int32_t dim = pnanovdb_uint32_as_int32(
                        pnanovdb_readaccessor_get_dim(PNANOVDB_GRID_TYPE_FLOAT, buf, accessor, hdda.voxel));
                dim = max(dim, lowestLevel);
                pnanovdb_hdda_update(hdda, origin, direction, dim);
            }
            iteration++;

            // Get the current visited node and whether it is active (i.e., contains a voxel with density larger than 0).
            bool isActive = false;
            pnanovdb_address_t nodeAddress;
            uint nodeAddressLevel;
            if (lowestLevel == 8) {
                pnanovdb_root_tile_handle_t tile = pnanovdb_root_find_tile(
                        PNANOVDB_GRID_TYPE_FLOAT, buf, accessor.root, hdda.voxel);
                if (!pnanovdb_int64_is_zero(pnanovdb_root_tile_get_child(buf, tile))) {
                    pnanovdb_upper_handle_t upperNode = pnanovdb_root_get_child(
                            PNANOVDB_GRID_TYPE_FLOAT, buf, accessor.root, tile);
                    pnanovdb_uint32_t upperNodeOffset = pnanovdb_upper_coord_to_offset(hdda.voxel);
                    if (pnanovdb_upper_get_child_mask(buf, upperNode, upperNodeOffset)) {
                        pnanovdb_lower_handle_t lowerNode = pnanovdb_upper_get_child(
                                PNANOVDB_GRID_TYPE_FLOAT, buf, upperNode, upperNodeOffset);
                        pnanovdb_uint32_t lowerNodeOffset = pnanovdb_lower_coord_to_offset(hdda.voxel);
                        if (pnanovdb_lower_get_child_mask(buf, lowerNode, lowerNodeOffset)) {
                            pnanovdb_leaf_handle_t leafNode = pnanovdb_lower_get_child(
                                    PNANOVDB_GRID_TYPE_FLOAT, buf, lowerNode, lowerNodeOffset);
                            nodeAddress = leafNode.address;
                            nodeAddressLevel = 0;
                            isActive = true;
                        } else {
                            nodeAddress = lowerNode.address;
                            nodeAddressLevel = 1;
                            isActive = pnanovdb_lower_get_value_mask(buf, lowerNode, lowerNodeOffset);
                        }
                    } else {
                        nodeAddress = upperNode.address;
                        nodeAddressLevel = 2;
                        isActive = pnanovdb_upper_get_value_mask(buf, upperNode, upperNodeOffset);
                    }
                } else {
                    nodeAddress = tile.address;
                    nodeAddressLevel = 3;
                    isActive = pnanovdb_root_tile_get_state(buf, tile) != 0u;
                }
            } else {
                isActive = pnanovdb_readaccessor_is_active(PNANOVDB_GRID_TYPE_FLOAT, buf, accessor, hdda.voxel);
            }

            // Skip nodes containing no voxel with density larger than 0.
            if (!isActive) {
                continue;
            }

            // Get the minimum, maximum and bounding box of the visited node.
            pnanovdb_address_t minAddress, maxAddress;
            vec3 leaf_bbox_minf;
            vec3 leaf_bbox_maxf;
            if (lowestLevel == 8) {
                if (nodeAddressLevel == 0) {
                    minAddress = pnanovdb_leaf_get_min_address(
                            PNANOVDB_GRID_TYPE_FLOAT, buf, pnanovdb_leaf_handle_t(nodeAddress));
                    maxAddress = pnanovdb_leaf_get_max_address(
                            PNANOVDB_GRID_TYPE_FLOAT, buf, pnanovdb_leaf_handle_t(nodeAddress));
                    ivec3 leaf_bbox_min = pnanovdb_leaf_get_bbox_min(buf, pnanovdb_leaf_handle_t(nodeAddress));
                    leaf_bbox_minf = pnanovdb_coord_to_vec3(leaf_bbox_min);
                    leaf_bbox_maxf = pnanovdb_coord_to_vec3(
                            pnanovdb_coord_add(leaf_bbox_min, pnanovdb_coord_uniform(8)));
                } else if (nodeAddressLevel == 1) {
                    minAddress = pnanovdb_lower_get_min_address(
                            PNANOVDB_GRID_TYPE_FLOAT, buf, pnanovdb_lower_handle_t(nodeAddress));
                    maxAddress = pnanovdb_lower_get_max_address(
                            PNANOVDB_GRID_TYPE_FLOAT, buf, pnanovdb_lower_handle_t(nodeAddress));
                    leaf_bbox_minf = pnanovdb_coord_to_vec3(
                            pnanovdb_lower_get_bbox_min(buf, pnanovdb_lower_handle_t(nodeAddress)));
                    leaf_bbox_maxf = pnanovdb_coord_to_vec3(
                            pnanovdb_lower_get_bbox_max(buf, pnanovdb_lower_handle_t(nodeAddress)));
                } else if (nodeAddressLevel == 2) {
                    minAddress = pnanovdb_upper_get_min_address(
                            PNANOVDB_GRID_TYPE_FLOAT, buf, pnanovdb_upper_handle_t(nodeAddress));
                    maxAddress = pnanovdb_upper_get_max_address(
                            PNANOVDB_GRID_TYPE_FLOAT, buf, pnanovdb_upper_handle_t(nodeAddress));
                    leaf_bbox_minf = pnanovdb_coord_to_vec3(
                            pnanovdb_upper_get_bbox_min(buf, pnanovdb_upper_handle_t(nodeAddress)));
                    leaf_bbox_maxf = pnanovdb_coord_to_vec3(
                            pnanovdb_upper_get_bbox_max(buf, pnanovdb_upper_handle_t(nodeAddress)));
                } else {
                    minAddress = pnanovdb_root_get_min_address(
                            PNANOVDB_GRID_TYPE_FLOAT, buf, pnanovdb_root_handle_t(nodeAddress));
                    maxAddress = pnanovdb_root_get_max_address(
                            PNANOVDB_GRID_TYPE_FLOAT, buf, pnanovdb_root_handle_t(nodeAddress));
                    leaf_bbox_minf = pnanovdb_coord_to_vec3(
                            pnanovdb_root_get_bbox_min(buf, pnanovdb_root_handle_t(nodeAddress)));
                    leaf_bbox_maxf = pnanovdb_coord_to_vec3(
                            pnanovdb_root_get_bbox_max(buf, pnanovdb_root_handle_t(nodeAddress)));
                }
            } else {
                // In case of a voxel, we just use the voxel value as the minimum and maximum.
                minAddress = pnanovdb_readaccessor_get_value_address(
                        PNANOVDB_GRID_TYPE_FLOAT, buf, accessor, hdda.voxel);
                maxAddress = pnanovdb_readaccessor_get_value_address(
                        PNANOVDB_GRID_TYPE_FLOAT, buf, accessor, hdda.voxel);
                leaf_bbox_minf = pnanovdb_coord_to_vec3(hdda.voxel);
                leaf_bbox_maxf = pnanovdb_coord_to_vec3(pnanovdb_coord_add(hdda.voxel, pnanovdb_coord_uniform(1)));
            }
            float minLeafValue = pnanovdb_read_float(buf, minAddress);
            float maxLeafValue = pnanovdb_read_float(buf, maxAddress);

            float tMinNode = 0.0;
            float tMaxNode = 1e10;
            bool rayHitsNode = pnanovdb_hdda_ray_clip(
                    leaf_bbox_minf, leaf_bbox_maxf, origin, tMinNode, direction, tMaxNode);
            x = pnanovdb_grid_index_to_worldf(buf, gridHandle, origin + tMinNode * direction);

            if (!rayHitsNode) {
                // TODO: Check cases where this happens.
                //return vec3(1.0, 1.0, 0.0);
                continue;
            }

            //float d_max = gridToWorldScale * (tMaxNode - tMinNode);
            float d_max = gridToWorldScale * (min(min(hdda.next.x, hdda.next.y), hdda.next.z) - hdda.tmin);

            // TODO: Boundary sampling
            //float mu_c_t = max(0.0000000001, majorant * minLeafValue);
            //float majorant_r_local = max(0.0000000001, majorant * maxLeafValue - mu_c_t);
            float mu_c_t = 0.0000000001;
            float majorant_r_local = max(0.0000000001, majorant * 1 - mu_c_t);
            bool directionChanged = false;

            float t_c = -log(max(0.0000000001, 1 - random())) / mu_c_t;
            float t_r = 0.0;

            while (true) {
                t_r -= log(max(0.0000000001, 1 - random())) / majorant_r_local;

                if (t_c >= d_max && t_r >= d_max) {
                    x = x + d_max * w;
                    break; // null collision, proceed to next super voxel
                }

                vec3 xs = x + w * min(t_c, t_r);
                bool isCollision = false;
                if (t_c <= t_r) {
                    isCollision = true;
                } else {
                    float density = sampleCloud(accessor, xs);
                    isCollision = random() * majorant_r_local < parameters.extinction.x * density - mu_c_t;
                }

                if (isCollision) {
                    x = xs;

                    if (random() < absorptionAlbedo) {
                        return vec3(0.0); // absorption event/emission
                    }

                    float pdf_w;
                    w = importanceSamplePhase(parameters.phaseG, w, pdf_w);
                    t_r = 0.0;
                    t_c = -log(max(0.0000000001, 1 - random())) / mu_c_t;

                    origin = pnanovdb_grid_world_to_indexf(buf, gridHandle, x);
                    direction = normalize(pnanovdb_grid_world_to_index_dirf(buf, gridHandle, w));

                    float tMinLeaf = 0.0, tMaxLeaf = 1e10;
                    rayBoxIntersect(leaf_bbox_minf, leaf_bbox_maxf, origin, direction, tMinLeaf, tMaxLeaf);
                    pnanovdb_hdda_ray_clip(leaf_bbox_minf, leaf_bbox_maxf, origin, tMinLeaf, direction, tMaxLeaf);
                    d_max = gridToWorldScale * (tMaxLeaf - tMinLeaf);
                    directionChanged = true;
                }
            }

            // Re-initialize the HDDA algorithm if the ray direction changed.
            if (directionChanged) {
                tMin = 0.0;
                tMax = 1e10;
                bool hit = pnanovdb_hdda_ray_clip(bbox_minf, bbox_maxf, origin, tMin, direction, tMax);
                dim = lowestLevel;
                pnanovdb_hdda_init(hdda, origin, tMin + d_max / gridToWorldScale + 1e-4, direction, tMax + 1e-4, dim);
            }
        }
    }
    return sampleSkybox(w) + sampleLight(w);
}

#endif

#endif
