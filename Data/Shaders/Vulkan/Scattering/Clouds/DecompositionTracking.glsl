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
vec3 analogDecompositionTracking(vec3 x, vec3 w, out ScatterEvent firstEvent) {
    firstEvent = ScatterEvent(false, x, 0.0, w, 0.0);

#ifdef USE_NANOVDB
    pnanovdb_readaccessor_t accessor = createAccessor();
#endif

    int it = 0;
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

                bool isNullCollision;
                float t_c = -log(max(0.0000000001, 1 - random())) / mu_c_t;
                float t_r = 0.0;
                isNullCollision = false;
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
#ifdef USE_NANOVDB
                        float density = sampleCloud(accessor, xs);
#else
                        float density = sampleCloud(xs);
#endif
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
#endif
