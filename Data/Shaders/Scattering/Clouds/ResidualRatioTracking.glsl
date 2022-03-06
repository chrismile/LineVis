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

// TODO: This code is still work in progress.

/**
 * For more details on residual ratio tracking, please refer to:
 * J. Novák, A. Selle, and W. Jarosz. Residual ratio tracking for estimating attenuation in participating media.
 * ACM Transactions on Graphics (Proceedings of SIGGRAPH Asia) , 33(6), Nov. 2014.
 */

#ifdef USE_RESIDUAL_RATIO_TRACKING
float residualRatioTrackingEstimator(
#ifdef USE_NANOVDB
        pnanovdb_readaccessor_t accessor,
#endif
        vec3 x, vec3 w, float dStart, float dEnd, float T,
        inout float reservoirWeightSum, out float reservoirT, out float reservoirDist,
        float absorptionAlbedo, float mu_c, float mu_r_bar) {
    float T_c = exp(-mu_c * (dEnd - dStart));
    float T_r = 1.0;
    float dTravelled = dStart;

    //if (mu_r_bar < 1e-5) {
    //    return T_c;
    //}

    do {
        float t = -log(max(0.0000000001, 1 - random())) / mu_r_bar;
        x += w * t;

        dTravelled += t;
        if (dTravelled >= dEnd) {
            break;
        }

#ifdef USE_NANOVDB
        float density = sampleCloud(accessor, x);
#else
        float density = sampleCloud(x);
#endif
        float mu = parameters.extinction.x * density;
        //T_r *= (1.0 - absorptionAlbedo * (mu - mu_c) / mu_r_bar);
        //T_r *= (1.0 - density) * (mu - mu_c) / mu_r_bar;
        T_r *= 1.0 - (mu - mu_c) / mu_r_bar;

        float Ps = parameters.scatteringAlbedo.x * density;
        //float T_c_local = exp(-absorptionAlbedo * parameters.scatteringAlbedo.x * mu_c * (dTravelled - dStart));
        float T_c_local = exp(-mu_c * (dTravelled - dStart));
        float T_local = T * T_r * T_c_local;
        // https://developer.download.nvidia.com//ray-tracing-gems/rtg2-chapter22-preprint.pdf
        float reservoirWeight = T_local * Ps;
        reservoirWeightSum += reservoirWeight;
        float xi = random();
        if (xi < reservoirWeight / reservoirWeightSum) {
            reservoirT = T_local;
            reservoirDist = dTravelled;
        }
    } while(true);

    return T_c * T_r;
}

vec3 residualRatioTracking(vec3 x, vec3 w, out ScatterEvent firstEvent) {
    firstEvent = ScatterEvent(false, x, 0.0, w, 0.0);

#ifdef USE_NANOVDB
    pnanovdb_readaccessor_t accessor = createAccessor();
#endif

    float absorptionAlbedo = 1.0 - parameters.scatteringAlbedo.x;
    float scatteringAlbedo = parameters.scatteringAlbedo.x;

    // Functional representation of the transmittance using reservoir sampling.
    // cf.: https://developer.nvidia.com/blog/learn-more-about-reservoir-sampling-in-free-ray-tracing-gems-ii-chapter/
    float reservoirWeightSum = 0.0;
    float reservoirT = 0.0;
    float reservoirDist = 0.0;

    float T = 1.0;

    vec3 accumulatedColor = vec3(0.0, 0.0, 0.0);

    const vec3 EPSILON_VEC = vec3(1e-6);
    float tMinVal, tMaxVal;
    vec3 oldX;

    ivec3 voxelGridSize = textureSize(gridImage, 0);
    vec3 boxDelta = parameters.boxMax - parameters.boxMin;

    float tMaxX, tMaxY, tMaxZ, tDeltaX, tDeltaY, tDeltaZ;
    ivec3 superVoxelIndex;

    // Loop over all in-scattering rays.
    int iteration = 0;
    while (true) {
        /// Does in-scattering ray intersect the box?
        if (rayBoxIntersect(parameters.boxMin + EPSILON_VEC, parameters.boxMax - EPSILON_VEC, x, w, tMinVal, tMaxVal)) {
            x += w * tMinVal;
            oldX = x;
            float dTotal = tMaxVal - tMinVal;

            vec3 startPoint = (x - parameters.boxMin) / boxDelta * voxelGridSize / parameters.superVoxelSize;
            vec3 endPoint = (x + w * dTotal - parameters.boxMin) / boxDelta * voxelGridSize / parameters.superVoxelSize;

            int stepX = int(sign(endPoint.x - startPoint.x));
            if (stepX != 0)
                tDeltaX = min(stepX / (endPoint.x - startPoint.x), 1e7);
            else
                tDeltaX = 1e7; // inf
            if (stepX > 0)
                tMaxX = tDeltaX * (1.0 - fract(startPoint.x));
            else
                tMaxX = tDeltaX * fract(startPoint.x);
            superVoxelIndex.x = int(floor(startPoint.x));

            int stepY = int(sign(endPoint.y - startPoint.y));
            if (stepY != 0)
                tDeltaY = min(stepY / (endPoint.y - startPoint.y), 1e7);
            else
                tDeltaY = 1e7; // inf
            if (stepY > 0)
                tMaxY = tDeltaY * (1.0 - fract(startPoint.y));
            else
                tMaxY = tDeltaY * fract(startPoint.y);
            superVoxelIndex.y = int(floor(startPoint.y));

            int stepZ = int(sign(endPoint.z - startPoint.z));
            if (stepZ != 0)
                tDeltaZ = min(stepZ / (endPoint.z - startPoint.z), 1e7);
            else
                tDeltaZ = 1e7; // inf
            if (stepZ > 0)
                tMaxZ = tDeltaZ * (1.0 - fract(startPoint.z));
            else
                tMaxZ = tDeltaZ * fract(startPoint.z);
            superVoxelIndex.z = int(floor(startPoint.z));

            if (stepX == 0 && stepY == 0 && stepZ == 0) {
                break;
            }
            ivec3 step = ivec3(stepX, stepY, stepZ);
            vec3 tMax = vec3(tMaxX, tMaxY, tMaxZ);
            vec3 tDelta = vec3(tDeltaX, tDeltaY, tDeltaZ);

            ivec3 startVoxelInt = clamp(ivec3(floor(startPoint)), ivec3(0), parameters.superVoxelGridSize - ivec3(1));
            ivec3 endVoxelInt = clamp(ivec3(ceil(endPoint)), ivec3(0), parameters.superVoxelGridSize - ivec3(1));

            // Loop over all super voxels along the ray.
            while (all(greaterThanEqual(superVoxelIndex, ivec3(0))) && all(lessThan(superVoxelIndex, parameters.superVoxelGridSize))) {
                vec2 superVoxel = texelFetch(superVoxelGridImage, superVoxelIndex, 0).rg;
                float mu_c = superVoxel.x;
                float mu_r_bar = superVoxel.y;
                //bool superVoxelEmpty = texelFetch(superVoxelGridOccupancyImage, ivec3(0), 0).r != 0;

                vec3 minVoxelPos = superVoxelIndex * parameters.superVoxelSize;
                vec3 maxVoxelPos = minVoxelPos + parameters.superVoxelSize;
                minVoxelPos = minVoxelPos / voxelGridSize * boxDelta + parameters.boxMin;
                maxVoxelPos = maxVoxelPos / voxelGridSize * boxDelta + parameters.boxMin;
                float tMinVoxel = 0.0, tMaxVoxel = 0.0;
                rayBoxIntersect(minVoxelPos, maxVoxelPos, oldX, w, tMinVoxel, tMaxVoxel);

                x = oldX + w * tMinVoxel;
                T *= residualRatioTrackingEstimator(
#ifdef USE_NANOVDB
                        accessor,
#endif
                        x, w, tMinVoxel, tMaxVoxel, T,
                        reservoirWeightSum, reservoirT, reservoirDist,
                        absorptionAlbedo, mu_c, mu_r_bar);

                if (tMaxX < tMaxY) {
                    if (tMaxX < tMaxZ) {
                        superVoxelIndex.x += stepX;
                        tMaxX += tDeltaX;
                    } else {
                        superVoxelIndex.z += stepZ;
                        tMaxZ += tDeltaZ;
                    }
                } else {
                    if (tMaxY < tMaxZ) {
                        superVoxelIndex.y += stepY;
                        tMaxY += tDeltaY;
                    } else {
                        superVoxelIndex.z += stepZ;
                        tMaxZ += tDeltaZ;
                    }
                }
            }
        } else {
            break;
        }

        float xi = random();
        if (xi > reservoirWeightSum || iteration >= 10) {
            break;
        }
        accumulatedColor += T * (sampleSkybox(w) + sampleLight(w));
        iteration++;

        //float pdf_w;
        //x = x + w * (tMinVal + xi * (tMaxVal - tMinVal)); // Uniform sampling.
        //w = importanceSamplePhase(parameters.phaseG, w, pdf_w);

        // https://developer.download.nvidia.com//ray-tracing-gems/rtg2-chapter22-preprint.pdf
        T = reservoirT;
        float pdf_w;
        x = oldX + w * reservoirDist;
        w = importanceSamplePhase(parameters.phaseG, w, pdf_w);

        reservoirWeightSum = 0.0;
        reservoirT = 0.0;
        reservoirDist = 0.0;
    }

    accumulatedColor += T * (sampleSkybox(w) + sampleLight(w));
    return accumulatedColor;
}
#endif
