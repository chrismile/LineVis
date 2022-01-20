/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser, Michael Kern
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

#ifndef STRESSLINEVIS_BEZIERTRAJECTORY_HPP
#define STRESSLINEVIS_BEZIERTRAJECTORY_HPP

#include "Loaders/TrajectoryFile.hpp"

// Describes the position of variables in the buffer and the total number of variable values for the entire line
struct LineDesc {
    float startIndex; // pointer to index in array
    float numValues; // number of variables along line after Bezier curve transformation
};

// Describes the range of values for each variable and the offset within each line
struct VarDesc {
    float startIndex;
    glm::vec2 minMax;
    float dummy;
};

class BezierTrajectory {
public:
    BezierTrajectory() = default;

    // Per Bezier point data.
    std::vector<glm::vec3> positions;
    std::vector<std::vector<float>> attributes;
    //std::vector<glm::vec3> tangents; // Not used for now - computed automatically by createLineTubesRenderDataCPU.
    //std::vector<uint32_t> segmentID;

    // Packed array of base trajectory attributes.
    std::vector<float> multiVarData;

    // Information about this line/trajectory.
    LineDesc lineDesc{};
    // Information about all variables.
    std::vector<VarDesc> multiVarDescs;
};
typedef std::vector<BezierTrajectory> BezierTrajectories;

BezierTrajectories convertTrajectoriesToBezierCurves(const Trajectories& inTrajectories, bool needsSubdiv);

#endif //STRESSLINEVIS_BEZIERTRAJECTORY_HPP
