/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser
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

#define _FILE_OFFSET_BITS 64

#include <iostream>
#include <cstdio>

#ifdef USE_TBB
#include <tbb/parallel_for.h>
#include <tbb/parallel_reduce.h>
#include <tbb/blocked_range.h>
#endif

#include <Utils/StringUtils.hpp>
#include <Utils/File/Logfile.hpp>
#include <Math/Geometry/AABB3.hpp>
#include <Utils/Events/Stream/Stream.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Utils/Parallel/Reduction.hpp>
#include <tracy/Tracy.hpp>

#include "ObjLoader.hpp"
#include "NetCdfLineLoader.hpp"
#include "BinLinesLoader.hpp"
#include "StressTrajectoriesDatLoader.hpp"
#include "TrajectoryFile.hpp"

sgl::AABB3 computeTrajectoriesAABB3(const Trajectories& trajectories) {
#ifdef USE_TBB

    return tbb::parallel_reduce(
            tbb::blocked_range<size_t>(0, trajectories.size()), sgl::AABB3(),
            [&trajectories](tbb::blocked_range<size_t> const& r, sgl::AABB3 init) {
                for (auto trajectoryIdx = r.begin(); trajectoryIdx != r.end(); trajectoryIdx++) {
                    const Trajectory& trajectory = trajectories.at(trajectoryIdx);
                    for (const glm::vec3& pt : trajectory.positions) {
                        init.min.x = std::min(init.min.x, pt.x);
                        init.min.y = std::min(init.min.y, pt.y);
                        init.min.z = std::min(init.min.z, pt.z);
                        init.max.x = std::max(init.max.x, pt.x);
                        init.max.y = std::max(init.max.y, pt.y);
                        init.max.z = std::max(init.max.z, pt.z);
                    }
                }
                return init;
            },
            [&](sgl::AABB3 lhs, sgl::AABB3 rhs) -> sgl::AABB3 {
                lhs.combine(rhs);
                return lhs;
            });

#else

    float minX, minY, minZ, maxX, maxY, maxZ;
    minX = minY = minZ = std::numeric_limits<float>::max();
    maxX = maxY = maxZ = std::numeric_limits<float>::lowest();
#if _OPENMP >= 201107
    #pragma omp parallel for shared(trajectories) default(none) reduction(min: minX) reduction(min: minY) \
    reduction(min: minZ) reduction(max: maxX) reduction(max: maxY) reduction(max: maxZ)
#endif
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
        const Trajectory& trajectory = trajectories.at(trajectoryIdx);
        for (const glm::vec3& pt : trajectory.positions) {
            minX = std::min(minX, pt.x);
            minY = std::min(minY, pt.y);
            minZ = std::min(minZ, pt.z);
            maxX = std::max(maxX, pt.x);
            maxY = std::max(maxY, pt.y);
            maxZ = std::max(maxZ, pt.z);
        }
    }
    sgl::AABB3 aabb;
    aabb.min = glm::vec3(minX, minY, minZ);
    aabb.max = glm::vec3(maxX, maxY, maxZ);
    return aabb;

#endif
}

void normalizeTrajectoriesVertexPositions(
        Trajectories& trajectories, const sgl::AABB3& aabb, const glm::mat4* vertexTransformationMatrixPtr) {
    glm::vec3 translation = -aabb.getCenter();
    glm::vec3 scale3D = 0.5f / aabb.getDimensions();
    float scale = std::min(scale3D.x, std::min(scale3D.y, scale3D.z));

#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<size_t>(0, trajectories.size()), [&](auto const& r) {
        for (auto trajectoryIdx = r.begin(); trajectoryIdx != r.end(); trajectoryIdx++) {
#else
#if _OPENMP >= 200805
    #pragma omp parallel for shared(trajectories, scale, translation) default(none)
#endif
    for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
#endif
        Trajectory& trajectory = trajectories.at(trajectoryIdx);
        for (glm::vec3& v : trajectory.positions) {
            v = (v + translation) * scale;
        }
    }
#ifdef USE_TBB
    });
#endif

    if (vertexTransformationMatrixPtr != nullptr) {
        glm::mat4 transformationMatrix = *vertexTransformationMatrixPtr;

#ifdef USE_TBB
        tbb::parallel_for(tbb::blocked_range<size_t>(0, trajectories.size()), [&](auto const& r) {
            for (auto trajectoryIdx = r.begin(); trajectoryIdx != r.end(); trajectoryIdx++) {
#else
#if _OPENMP >= 200805
        #pragma omp parallel for shared(trajectories, transformationMatrix) default(none)
#endif
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
#endif
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            for (glm::vec3& v : trajectory.positions) {
                glm::vec4 transformedVec = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
                v = glm::vec3(transformedVec.x, transformedVec.y, transformedVec.z);
            }
        }
#ifdef USE_TBB
        });
#endif
    }
}

void normalizeTrajectoriesVertexPositions(Trajectories& trajectories, const glm::mat4* vertexTransformationMatrixPtr) {
    sgl::AABB3 aabb = computeTrajectoriesAABB3(trajectories);
    normalizeTrajectoriesVertexPositions(trajectories, aabb, vertexTransformationMatrixPtr);
}

void normalizeVertexPositions(
        std::vector<glm::vec3>& vertexPositions, const sgl::AABB3& aabb,
        const glm::mat4* vertexTransformationMatrixPtr) {
    ZoneScoped;

    glm::vec3 translation = -aabb.getCenter();
    glm::vec3 scale3D = 0.5f / aabb.getDimensions();
    float scale = std::min(scale3D.x, std::min(scale3D.y, scale3D.z));

#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<size_t>(0, vertexPositions.size()), [&](auto const& r) {
        for (size_t vertexIdx = r.begin(); vertexIdx != r.end(); vertexIdx++) {
#else
#if _OPENMP >= 200805
    #pragma omp parallel for shared(vertexPositions, translation, scale) default(none)
#endif
    for (size_t vertexIdx = 0; vertexIdx < vertexPositions.size(); vertexIdx++) {
#endif
        glm::vec3& v = vertexPositions.at(vertexIdx);
        v = (v + translation) * scale;
    }
#ifdef USE_TBB
    });
#endif

    if (vertexTransformationMatrixPtr != nullptr) {
        glm::mat4 transformationMatrix = *vertexTransformationMatrixPtr;

#ifdef USE_TBB
        tbb::parallel_for(tbb::blocked_range<size_t>(0, vertexPositions.size()), [&](auto const& r) {
            for (auto vertexIdx = r.begin(); vertexIdx != r.end(); vertexIdx++) {
#else
#if _OPENMP >= 200805
        #pragma omp parallel for shared(vertexPositions, transformationMatrix) default(none)
#endif
        for (size_t vertexIdx = 0; vertexIdx < vertexPositions.size(); vertexIdx++) {
#endif
            glm::vec3& v = vertexPositions.at(vertexIdx);
            glm::vec4 transformedVec = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
            v = glm::vec3(transformedVec.x, transformedVec.y, transformedVec.z);
        }
#ifdef USE_TBB
        });
#endif
    }
}

void normalizeVertexPosition(
        glm::vec3& vertexPosition, const sgl::AABB3& aabb,
        const glm::mat4* vertexTransformationMatrixPtr) {
    glm::vec3 translation = -aabb.getCenter();
    glm::vec3 scale3D = 0.5f / aabb.getDimensions();
    float scale = std::min(scale3D.x, std::min(scale3D.y, scale3D.z));

    glm::vec3& v = vertexPosition;
    v = (v + translation) * scale;

    if (vertexTransformationMatrixPtr != nullptr) {
        glm::mat4 transformationMatrix = *vertexTransformationMatrixPtr;
        glm::vec4 transformedVec = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
        v = glm::vec3(transformedVec.x, transformedVec.y, transformedVec.z);
    }
}

void normalizeVertexNormals(
        std::vector<glm::vec3>& vertexNormals, const sgl::AABB3& aabb,
        const glm::mat4* vertexTransformationMatrixPtr) {
    ZoneScoped;

    if (vertexTransformationMatrixPtr != nullptr) {
        glm::mat4 transformationMatrix = *vertexTransformationMatrixPtr;
        transformationMatrix = glm::transpose(glm::inverse(transformationMatrix));

#ifdef USE_TBB
        tbb::parallel_for(tbb::blocked_range<size_t>(0, vertexNormals.size()), [&](auto const& r) {
            for (auto vertexIdx = r.begin(); vertexIdx != r.end(); vertexIdx++) {
#else
#if _OPENMP >= 200805
        #pragma omp parallel for shared(vertexNormals, transformationMatrix) default(none)
#endif
        for (size_t vertexIdx = 0; vertexIdx < vertexNormals.size(); vertexIdx++) {
#endif
            glm::vec3& v = vertexNormals.at(vertexIdx);
            glm::vec4 transformedVec = transformationMatrix * glm::vec4(v.x, v.y, v.z, 0.0f);
            v = glm::vec3(transformedVec.x, transformedVec.y, transformedVec.z);
        }
#ifdef USE_TBB
        });
#endif
    }
}

void normalizeVertexAttributes(std::vector<std::vector<float>>& vertexAttributesList) {
    const size_t numAttributes = vertexAttributesList.size();

    for (size_t attributeIdx = 0; attributeIdx < numAttributes; attributeIdx++) {
        std::vector<float>& vertexAttributes = vertexAttributesList.at(attributeIdx);
        auto [minVal, maxVal] = sgl::reduceFloatArrayMinMax(vertexAttributes);

#ifdef USE_TBB
        tbb::parallel_for(tbb::blocked_range<size_t>(0, vertexAttributes.size()), [&](auto const& r) {
            for (size_t i = r.begin(); i != r.end(); i++) {
#else
#if _OPENMP >= 200805
        #pragma omp parallel for shared(vertexAttributes, minVal, maxVal) default(none)
#endif
        for (size_t i = 0; i < vertexAttributes.size(); i++) {
#endif
            float& attrVal = vertexAttributes.at(i);
            attrVal = (attrVal - minVal) / (maxVal - minVal);
        }
#ifdef USE_TBB
        });
#endif
    }
}

void normalizeTrajectoriesVertexAttributes(Trajectories& trajectories) {
    const size_t numAttributes = trajectories.empty() ? 0 : trajectories.front().attributes.size();

    for (size_t attributeIdx = 0; attributeIdx < numAttributes; attributeIdx++) {
#ifdef USE_TBB
        auto [minVal, maxVal] = tbb::parallel_reduce(
                tbb::blocked_range<size_t>(0, trajectories.size()),
                std::make_pair(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()),
                [&trajectories, attributeIdx](tbb::blocked_range<size_t> const& r, std::pair<float, float> init) {
                    for (auto trajectoryIdx = r.begin(); trajectoryIdx != r.end(); trajectoryIdx++) {
                        const std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
                        for (const float& attrVal : attributes) {
                            init.first = std::min(init.first, attrVal);
                            init.second = std::max(init.second, attrVal);
                        }
                    }
                    return init;
                }, &sgl::reductionFunctionFloatMinMax);

#else
        float minVal = std::numeric_limits<float>::max();
        float maxVal = std::numeric_limits<float>::lowest();
#if _OPENMP >= 201107
        #pragma omp parallel for shared(trajectories, attributeIdx) default(none) \
        reduction(min: minVal) reduction(max: maxVal)
#endif
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
            const std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
            for (const float& attrVal : attributes) {
                minVal = std::min(minVal, attrVal);
                maxVal = std::max(maxVal, attrVal);
            }
        }
#endif

#ifdef USE_TBB
        tbb::parallel_for(tbb::blocked_range<size_t>(0, trajectories.size()), [&](auto const& r) {
            for (size_t trajectoryIdx = r.begin(); trajectoryIdx != r.end(); trajectoryIdx++) {
#else
#if _OPENMP >= 200805
        #pragma omp parallel for shared(trajectories, attributeIdx, minVal, maxVal) default(none)
#endif
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
#endif
            std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
            for (float& attrVal : attributes) {
                attrVal = (attrVal - minVal) / (maxVal - minVal);
            }
        }
#ifdef USE_TBB
        });
#endif
    }
}



sgl::AABB3 computeTrajectoriesPsAABB3(const std::vector<Trajectories>& trajectoriesPs) {
    sgl::AABB3 aabb;
    for (const Trajectories& trajectories : trajectoriesPs) {
        aabb.combine(computeTrajectoriesAABB3(trajectories));
    }
    return aabb;
}

void normalizeTrajectoriesPsVertexPositions(
        std::vector<Trajectories>& trajectoriesPs, const sgl::AABB3& aabb,
        const glm::mat4* vertexTransformationMatrixPtr) {
    glm::vec3 translation = -aabb.getCenter();
    glm::vec3 scale3D = 0.5f / aabb.getDimensions();
    float scale = std::min(scale3D.x, std::min(scale3D.y, scale3D.z));

    for (Trajectories& trajectories : trajectoriesPs) {
#ifdef USE_TBB
        tbb::parallel_for(tbb::blocked_range<size_t>(0, trajectories.size()), [&](auto const& r) {
            for (size_t trajectoryIdx = r.begin(); trajectoryIdx != r.end(); trajectoryIdx++) {
#else
#if _OPENMP >= 200805
        #pragma omp parallel for shared(trajectories, scale, translation) default(none)
#endif
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
#endif
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            for (glm::vec3& v : trajectory.positions) {
                v = (v + translation) * scale;
            }
        }
#ifdef USE_TBB
        });
#endif

        if (vertexTransformationMatrixPtr != nullptr) {
            glm::mat4 transformationMatrix = *vertexTransformationMatrixPtr;

#ifdef USE_TBB
            tbb::parallel_for(tbb::blocked_range<size_t>(0, trajectories.size()), [&](auto const& r) {
                for (size_t trajectoryIdx = r.begin(); trajectoryIdx != r.end(); trajectoryIdx++) {
#else
#if _OPENMP >= 200805
            #pragma omp parallel for shared(trajectories, transformationMatrix) default(none)
#endif
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
#endif
                Trajectory& trajectory = trajectories.at(trajectoryIdx);
                for (glm::vec3& v : trajectory.positions) {
                    glm::vec4 transformedVec = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
                    v = glm::vec3(transformedVec.x, transformedVec.y, transformedVec.z);
                }
            }
#ifdef USE_TBB
            });
#endif
        }
    }
}

void normalizeTrajectoriesPsVertexPositions(
        std::vector<Trajectories>& trajectoriesPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsUnsmoothedListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsUnsmoothedListRightPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsSmoothedListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsSmoothedListRightPs,
        const sgl::AABB3& aabb, const glm::mat4* vertexTransformationMatrixPtr) {
    glm::vec3 translation = -aabb.getCenter();
    glm::vec3 scale3D = 0.5f / aabb.getDimensions();
    float scale = std::min(scale3D.x, std::min(scale3D.y, scale3D.z));

    for (size_t psIdx = 0; psIdx < trajectoriesPs.size(); psIdx++) {
        Trajectories& trajectories = trajectoriesPs.at(psIdx);
        std::vector<std::vector<glm::vec3>>& bandPointsUnsmoothedListLeft = bandPointsUnsmoothedListLeftPs.at(psIdx);
        std::vector<std::vector<glm::vec3>>& bandPointsUnsmoothedListRight = bandPointsUnsmoothedListRightPs.at(psIdx);
        std::vector<std::vector<glm::vec3>>& bandPointsSmoothedListLeft = bandPointsSmoothedListLeftPs.at(psIdx);
        std::vector<std::vector<glm::vec3>>& bandPointsSmoothedListRight = bandPointsSmoothedListRightPs.at(psIdx);

#ifdef USE_TBB
        tbb::parallel_for(tbb::blocked_range<size_t>(0, trajectories.size()), [&](auto const& r) {
            for (size_t trajectoryIdx = r.begin(); trajectoryIdx != r.end(); trajectoryIdx++) {
#else
#if _OPENMP >= 200805
        #pragma omp parallel for shared(trajectories, bandPointsUnsmoothedListLeft, bandPointsUnsmoothedListRight) \
        shared(bandPointsSmoothedListLeft, bandPointsSmoothedListRight, scale, translation) default(none)
#endif
        for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
#endif
            Trajectory& trajectory = trajectories.at(trajectoryIdx);
            for (glm::vec3& v : trajectory.positions) {
                v = (v + translation) * scale;
            }

            std::vector<glm::vec3>& bandPointsUnsmoothedLeft = bandPointsUnsmoothedListLeft.at(trajectoryIdx);
            for (glm::vec3& v : bandPointsUnsmoothedLeft) {
                v = (v + translation) * scale;
            }

            std::vector<glm::vec3>& bandPointsUnsmoothedRight = bandPointsUnsmoothedListRight.at(trajectoryIdx);
            for (glm::vec3& v : bandPointsUnsmoothedRight) {
                v = (v + translation) * scale;
            }

            std::vector<glm::vec3>& bandPointsSmoothedLeft = bandPointsSmoothedListLeft.at(trajectoryIdx);
            for (glm::vec3& v : bandPointsSmoothedLeft) {
                v = (v + translation) * scale;
            }

            std::vector<glm::vec3>& bandPointsSmoothedRight = bandPointsSmoothedListRight.at(trajectoryIdx);
            for (glm::vec3& v : bandPointsSmoothedRight) {
                v = (v + translation) * scale;
            }
        }
#ifdef USE_TBB
        });
#endif

        if (vertexTransformationMatrixPtr != nullptr) {
            glm::mat4 transformationMatrix = *vertexTransformationMatrixPtr;

#ifdef USE_TBB
            tbb::parallel_for(tbb::blocked_range<size_t>(0, trajectories.size()), [&](auto const& r) {
                for (size_t trajectoryIdx = r.begin(); trajectoryIdx != r.end(); trajectoryIdx++) {
#else
#if _OPENMP >= 200805
            #pragma omp parallel for shared(trajectories, bandPointsUnsmoothedListLeft, bandPointsUnsmoothedListRight) \
            shared(bandPointsSmoothedListLeft, bandPointsSmoothedListRight, transformationMatrix) default(none)
#endif
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
#endif
                Trajectory& trajectory = trajectories.at(trajectoryIdx);
                for (glm::vec3& v : trajectory.positions) {
                    glm::vec4 transformedVec = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
                    v = glm::vec3(transformedVec.x, transformedVec.y, transformedVec.z);
                }

                std::vector<glm::vec3>& bandPointsUnsmoothedLeft = bandPointsUnsmoothedListLeft.at(trajectoryIdx);
                for (glm::vec3& v : bandPointsUnsmoothedLeft) {
                    glm::vec4 transformedVec = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
                    v = glm::vec3(transformedVec.x, transformedVec.y, transformedVec.z);
                }

                std::vector<glm::vec3>& bandPointsUnsmoothedRight = bandPointsUnsmoothedListRight.at(trajectoryIdx);
                for (glm::vec3& v : bandPointsUnsmoothedRight) {
                    glm::vec4 transformedVec = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
                    v = glm::vec3(transformedVec.x, transformedVec.y, transformedVec.z);
                }

                std::vector<glm::vec3>& bandPointsLeft = bandPointsSmoothedListLeft.at(trajectoryIdx);
                for (glm::vec3& v : bandPointsLeft) {
                    glm::vec4 transformedVec = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
                    v = glm::vec3(transformedVec.x, transformedVec.y, transformedVec.z);
                }

                std::vector<glm::vec3>& bandPointsRight = bandPointsSmoothedListRight.at(trajectoryIdx);
                for (glm::vec3& v : bandPointsRight) {
                    glm::vec4 transformedVec = transformationMatrix * glm::vec4(v.x, v.y, v.z, 1.0f);
                    v = glm::vec3(transformedVec.x, transformedVec.y, transformedVec.z);
                }
            }
#ifdef USE_TBB
            });
#endif
        }
    }
}


void normalizeTrajectoriesPsVertexPositions(
        std::vector<Trajectories>& trajectoriesPs, const glm::mat4* vertexTransformationMatrixPtr) {
    sgl::AABB3 aabb = computeTrajectoriesPsAABB3(trajectoriesPs);
    normalizeTrajectoriesPsVertexPositions(trajectoriesPs, aabb, vertexTransformationMatrixPtr);
}

void normalizeTrajectoriesPsVertexAttributes_Total(std::vector<Trajectories>& trajectoriesPs) {
    size_t numAttributes = 0;
    if (!trajectoriesPs.empty() && !trajectoriesPs.front().empty()) {
        numAttributes = trajectoriesPs.front().front().attributes.size();
    }

    for (size_t attributeIdx = 0; attributeIdx < numAttributes; attributeIdx++) {
        float minVal = std::numeric_limits<float>::max();
        float maxVal = std::numeric_limits<float>::lowest();
        for (Trajectories& trajectories : trajectoriesPs) {
#ifdef USE_TBB
            auto [minValNew, maxValNew] = tbb::parallel_reduce(
                    tbb::blocked_range<size_t>(0, trajectories.size()),
                    std::make_pair(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()),
                    [&trajectories, attributeIdx](tbb::blocked_range<size_t> const& r, std::pair<float, float> init) {
                        for (auto trajectoryIdx = r.begin(); trajectoryIdx != r.end(); trajectoryIdx++) {
                            const std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
                            for (const float& attrVal : attributes) {
                                init.first = std::min(init.first, attrVal);
                                init.second = std::max(init.second, attrVal);
                            }
                        }
                        return init;
                    }, &sgl::reductionFunctionFloatMinMax);
            minVal = std::min(minVal, minValNew);
            maxVal = std::min(maxVal, maxValNew);
#else
#if _OPENMP >= 201107
            #pragma omp parallel for shared(trajectories, attributeIdx) default(none) reduction(min: minVal) \
            reduction(max: maxVal)
#endif
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                const std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
                for (const float& attrVal : attributes) {
                    minVal = std::min(minVal, attrVal);
                    maxVal = std::max(maxVal, attrVal);
                }
            }
#endif
        }

        for (Trajectories& trajectories : trajectoriesPs) {
#ifdef USE_TBB
            tbb::parallel_for(tbb::blocked_range<size_t>(0, trajectories.size()), [&](auto const& r) {
                for (size_t trajectoryIdx = r.begin(); trajectoryIdx != r.end(); trajectoryIdx++) {
#else
#if _OPENMP >= 200805
            #pragma omp parallel for shared(trajectories, attributeIdx, minVal, maxVal) default(none)
#endif
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
#endif
                std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
                for (float& attrVal : attributes) {
                    attrVal = (attrVal - minVal) / (maxVal - minVal);
                }
            }
#ifdef USE_TBB
            });
#endif
        }
    }
}

void normalizeTrajectoriesPsVertexAttributes_PerPs(std::vector<Trajectories>& trajectoriesPs) {
    size_t numAttributes = 0;
    if (!trajectoriesPs.empty() && !trajectoriesPs.front().empty()) {
        numAttributes = trajectoriesPs.front().front().attributes.size();
    }

    for (size_t attributeIdx = 0; attributeIdx < numAttributes; attributeIdx++) {
        for (Trajectories& trajectories : trajectoriesPs) {
#ifdef USE_TBB
            auto [minVal, maxVal] = tbb::parallel_reduce(
                    tbb::blocked_range<size_t>(0, trajectories.size()),
                    std::make_pair(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()),
                    [&trajectories, attributeIdx](tbb::blocked_range<size_t> const& r, std::pair<float, float> init) {
                        for (auto trajectoryIdx = r.begin(); trajectoryIdx != r.end(); trajectoryIdx++) {
                            const std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
                            for (const float& attrVal : attributes) {
                                init.first = std::min(init.first, attrVal);
                                init.second = std::max(init.second, attrVal);
                            }
                        }
                        return init;
                    }, &sgl::reductionFunctionFloatMinMax);

#else
            float minVal = std::numeric_limits<float>::max();
            float maxVal = std::numeric_limits<float>::lowest();
#if _OPENMP >= 201107
            #pragma omp parallel for shared(trajectories, attributeIdx) default(none) reduction(min: minVal) \
            reduction(max: maxVal)
#endif
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                const std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
                for (const float& attrVal : attributes) {
                    minVal = std::min(minVal, attrVal);
                    maxVal = std::max(maxVal, attrVal);
                }
            }
#endif

#ifdef USE_TBB
            tbb::parallel_for(tbb::blocked_range<size_t>(0, trajectories.size()), [&](auto const& r) {
                for (size_t trajectoryIdx = r.begin(); trajectoryIdx != r.end(); trajectoryIdx++) {
#else
#if _OPENMP >= 200805
            #pragma omp parallel for shared(trajectories, attributeIdx, minVal, maxVal) default(none)
#endif
            for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
#endif
                std::vector<float>& attributes = trajectories.at(trajectoryIdx).attributes.at(attributeIdx);
                for (float& attrVal : attributes) {
                    attrVal = (attrVal - minVal) / (maxVal - minVal);
                }
            }
#ifdef USE_TBB
            });
#endif
        }
    }
}



BinLinesData loadFlowTrajectoriesFromFile(
        const std::string& filename, std::vector<std::string>& attributeNames,
        bool normalizeVertexPositions, bool normalizeAttributes, const glm::mat4* vertexTransformationMatrixPtr) {
    BinLinesData binLinesData;
    binLinesData.attributeNames = attributeNames;

    std::string lowerCaseFilename = sgl::toLowerCopy(filename);
    if (sgl::endsWith(lowerCaseFilename, ".obj")) {
        binLinesData.trajectories = loadTrajectoriesFromObj(filename, attributeNames);
    } else if (sgl::endsWith(lowerCaseFilename, ".nc")) {
        binLinesData = loadTrajectoriesFromNetCdf(filename);
    } else if (sgl::endsWith(lowerCaseFilename, ".binlines")) {
        binLinesData = loadTrajectoriesFromBinLines(filename);
    } else {
        sgl::Logfile::get()->writeError("ERROR in loadFlowTrajectoriesFromFile: Unknown file extension.");
    }

    Trajectories& trajectories = binLinesData.trajectories;
    if (normalizeVertexPositions && !binLinesData.verticesNormalized) {
        normalizeTrajectoriesVertexPositions(trajectories, vertexTransformationMatrixPtr);
    }
    if (normalizeAttributes) {
        normalizeTrajectoriesVertexAttributes(trajectories);
    }

    return binLinesData;
}

void loadStressTrajectoriesFromFile(
        const std::vector<std::string>& filenamesTrajectories, const std::vector<std::string>& filenamesHierarchy,
        int version, std::vector<int>& loadedPsIndices, MeshType& meshType,
        std::vector<Trajectories>& trajectoriesPs, std::vector<StressTrajectoriesData>& stressTrajectoriesDataPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsUnsmoothedListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsUnsmoothedListRightPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsSmoothedListLeftPs,
        std::vector<std::vector<std::vector<glm::vec3>>>& bandPointsSmoothedListRightPs,
        std::vector<uint32_t>& simulationMeshOutlineTriangleIndices,
        std::vector<glm::vec3>& simulationMeshOutlineVertexPositions,
        bool normalizeVertexPositions, bool normalizeAttributes,
        sgl::AABB3* oldAABB, const glm::mat4* vertexTransformationMatrixPtr) {
    std::string lowerCaseFilename = sgl::toLowerCopy(filenamesTrajectories.front());
    if (sgl::endsWith(lowerCaseFilename, ".dat")) {
        if (version == 1) {
            loadStressTrajectoriesFromDat_v1(
                    filenamesTrajectories, filenamesHierarchy, loadedPsIndices, trajectoriesPs,
                    stressTrajectoriesDataPs);
            meshType = MeshType::CARTESIAN;
        } else if (version == 2) {
            loadStressTrajectoriesFromDat_v2(
                    filenamesTrajectories, loadedPsIndices, trajectoriesPs, stressTrajectoriesDataPs,
                    bandPointsUnsmoothedListLeftPs, bandPointsUnsmoothedListRightPs);
            bandPointsSmoothedListLeftPs = bandPointsUnsmoothedListLeftPs;
            bandPointsSmoothedListRightPs = bandPointsUnsmoothedListRightPs;
            meshType = MeshType::CARTESIAN;
        } else if (version == 3) {
            loadStressTrajectoriesFromDat_v3(
                    filenamesTrajectories, loadedPsIndices, meshType, trajectoriesPs, stressTrajectoriesDataPs,
                    bandPointsUnsmoothedListLeftPs, bandPointsUnsmoothedListRightPs,
                    bandPointsSmoothedListLeftPs, bandPointsSmoothedListRightPs,
                    simulationMeshOutlineTriangleIndices, simulationMeshOutlineVertexPositions);
        } else {
            sgl::Logfile::get()->writeError("ERROR in loadStressTrajectoriesFromFile: Unknown version number.");
        }
    } else {
        sgl::Logfile::get()->writeError("ERROR in loadStressTrajectoriesFromFile: Unknown file extension.");
    }

    if (normalizeVertexPositions) {
        sgl::AABB3 aabb = computeTrajectoriesPsAABB3(trajectoriesPs);
        if (oldAABB) {
            *oldAABB = aabb;
        }
        if (version >= 2) {
            normalizeTrajectoriesPsVertexPositions(
                    trajectoriesPs, bandPointsUnsmoothedListLeftPs, bandPointsUnsmoothedListRightPs,
                    bandPointsSmoothedListLeftPs, bandPointsSmoothedListRightPs, aabb, vertexTransformationMatrixPtr);

            for (size_t psIdx = 0; psIdx < trajectoriesPs.size(); psIdx++) {
                Trajectories& trajectories = trajectoriesPs.at(psIdx);
                std::vector<std::vector<glm::vec3>>& bandPointsUnsmoothedListLeft = bandPointsUnsmoothedListLeftPs.at(psIdx);
                std::vector<std::vector<glm::vec3>>& bandPointsUnsmoothedListRight = bandPointsUnsmoothedListRightPs.at(psIdx);
                std::vector<std::vector<glm::vec3>>& bandPointsSmoothedListLeft = bandPointsSmoothedListLeftPs.at(psIdx);
                std::vector<std::vector<glm::vec3>>& bandPointsSmoothedListRight = bandPointsSmoothedListRightPs.at(psIdx);

                for (size_t trajectoryIdx = 0; trajectoryIdx < trajectories.size(); trajectoryIdx++) {
                    Trajectory& trajectory = trajectories.at(trajectoryIdx);
                    std::vector<glm::vec3>& bandPointsUnsmoothedLeft = bandPointsUnsmoothedListLeft.at(trajectoryIdx);
                    std::vector<glm::vec3>& bandPointsUnsmoothedRight = bandPointsUnsmoothedListRight.at(trajectoryIdx);
                    std::vector<glm::vec3>& bandPointsSmoothedLeft = bandPointsSmoothedListLeft.at(trajectoryIdx);
                    std::vector<glm::vec3>& bandPointsSmoothedRight = bandPointsSmoothedListRight.at(trajectoryIdx);
                    for (size_t linePos = 0; linePos < trajectory.positions.size(); linePos++) {
                        glm::vec3& trajectoryPoint = trajectory.positions.at(linePos);

                        glm::vec3& bandPointUnsmoothedLeft = bandPointsUnsmoothedLeft.at(linePos);
                        glm::vec3& bandPointUnsmoothedRight = bandPointsUnsmoothedRight.at(linePos);
                        bandPointUnsmoothedLeft = bandPointUnsmoothedLeft - trajectoryPoint;
                        bandPointUnsmoothedRight = bandPointUnsmoothedRight - trajectoryPoint;
                        //bandPointUnsmoothedLeft /= 0.005f;
                        //bandPointUnsmoothedRight /= 0.005f;
                        bandPointUnsmoothedLeft = glm::normalize(bandPointUnsmoothedLeft);
                        bandPointUnsmoothedRight = glm::normalize(bandPointUnsmoothedRight);

                        glm::vec3& bandPointSmoothedLeft = bandPointsSmoothedLeft.at(linePos);
                        glm::vec3& bandPointSmoothedRight = bandPointsSmoothedRight.at(linePos);
                        bandPointSmoothedLeft = bandPointSmoothedLeft - trajectoryPoint;
                        bandPointSmoothedRight = bandPointSmoothedRight - trajectoryPoint;
                        //bandPointSmoothedLeft /= 0.005f;
                        //bandPointSmoothedRight /= 0.005f;
                        bandPointSmoothedLeft = glm::normalize(bandPointSmoothedLeft);
                        bandPointSmoothedRight = glm::normalize(bandPointSmoothedRight);
                    }
                }
            }
        } else {
            normalizeTrajectoriesPsVertexPositions(trajectoriesPs, aabb, vertexTransformationMatrixPtr);
        }
    }
    if (normalizeAttributes) {
        normalizeTrajectoriesPsVertexAttributes_PerPs(trajectoriesPs);
    }
}

