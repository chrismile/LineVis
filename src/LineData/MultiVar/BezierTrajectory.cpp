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

#include <Utils/File/Logfile.hpp>

#include "BezierCurve.hpp"
#include "BezierTrajectory.hpp"

BezierTrajectories convertTrajectoriesToBezierCurves(const Trajectories &inTrajectories) {
    // 1) Determine Bezier segments
    std::vector<std::vector<BezierCurve>> curves(inTrajectories.size());
    // Store the arclength of all segments along a curve
    std::vector<float> curveArcLengths(inTrajectories.size(), 0.0f);

    // Average segment length;
    float avgSegLength = 0.0f;
    float minSegLength = std::numeric_limits<float>::max();
    float numSegments = 0;

    int32_t trajCounter = 0;
    for (const auto& trajectory : inTrajectories) {
        std::vector<BezierCurve> &curveSet = curves[trajCounter];

        const int maxVertices = int(trajectory.positions.size());

        float minT = 0.0f;
        float maxT = 1.0f;

        for (int v = 0; v < maxVertices - 1; ++v) {
            const glm::vec3& pos0 = trajectory.positions[std::max(0, v - 1)];
            const glm::vec3& pos1 = trajectory.positions[v];
            const glm::vec3& pos2 = trajectory.positions[v + 1];
            const glm::vec3& pos3 = trajectory.positions[std::min(v + 2, maxVertices - 1)];

            const glm::vec3 cotangent1 = glm::normalize(pos2 - pos0);
            const glm::vec3 cotangent2 = glm::normalize(pos3 - pos1);
            const glm::vec3 tangent = pos2 - pos1;
            const float lenTangent = glm::length(tangent);

            avgSegLength += lenTangent;
            minSegLength = std::min(minSegLength, lenTangent);
            numSegments++;

            glm::vec3 C0 = pos1;
            glm::vec3 C1 = pos1 + cotangent1 * lenTangent * 0.5f;
            glm::vec3 C2 = pos2 - cotangent2 * lenTangent * 0.5f;
            glm::vec3 C3 = pos2;

//            const std::vector<float>& attributes = trajectory.attributes[v];

//            curveSet.emplace_back({{ C0, C1, C2, C3 }}, minT, maxT);
            BezierCurve BCurve({{C0, C1, C2, C3}}, minT, maxT);

            curveSet.push_back(BCurve);
            curveArcLengths[trajCounter] += BCurve.totalArcLength;

            minT += 1.0f;
            maxT += 1.0f;
        }

        trajCounter++;
    }

    avgSegLength /= numSegments;

    // 2) Compute min max values of all attributes across all trajectories
    const uint32_t numVariables = uint32_t(inTrajectories[0].attributes.size());
    if (numVariables <= 0) {
        sgl::Logfile::get()->writeError("ERROR: No variable was found in trajectory file.");
        std::exit(-2);
    }

    std::vector<glm::vec2> attributesMinMax(
            numVariables, glm::vec2(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()));

    const uint32_t maxNumVariables = uint32_t(inTrajectories[0].attributes.size());
    const uint32_t numLines = uint32_t(inTrajectories.size());

    // 2.5) Create buffer array with all variables and statistics
    std::vector<std::vector<float>> multiVarData(numLines);
    std::vector<LineDesc> lineDescs(numLines);
    std::vector<std::vector<VarDesc>> lineMultiVarDescs(numLines);

    uint32_t lineOffset = 0;
    uint32_t lineID = 0;

    for (const auto& trajectory : inTrajectories) {
        uint32_t varOffsetPerLine = 0;

        for (uint32_t v = 0; v < maxNumVariables; ++v) {
            VarDesc varDescPerLine = {0};
            varDescPerLine.minMax = glm::vec2(
                    std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());
            varDescPerLine.startIndex = float(varOffsetPerLine);
            varDescPerLine.dummy = 0.0f;

            const std::vector<float>& variableArray = trajectory.attributes[v];

            for (const auto &variable : variableArray) {
                attributesMinMax[v].x = std::min(attributesMinMax[v].x, variable);
                attributesMinMax[v].y = std::max(attributesMinMax[v].y, variable);

                varDescPerLine.minMax.x = std::min(varDescPerLine.minMax.x, variable);
                varDescPerLine.minMax.y = std::max(varDescPerLine.minMax.y, variable);

                multiVarData[lineID].push_back(variable);
            }

            lineMultiVarDescs[lineID].push_back(varDescPerLine);
            varOffsetPerLine += uint32_t(variableArray.size());
        }
        lineDescs[lineID].startIndex = float(lineOffset);
        lineDescs[lineID].numValues = float(varOffsetPerLine);

        lineOffset += varOffsetPerLine;
        lineID++;
    }


    // 3) Compute several equally-distributed / equi-distant points along Bezier curves.
    // Store these points in a new trajectory
    float rollSegLength = avgSegLength / maxNumVariables;// avgSegLength * 0.2f;

    BezierTrajectories newTrajectories(inTrajectories.size());

    for (int32_t traj = 0; traj < int(inTrajectories.size()); ++traj) {
        float curArcLength = 0.0f;
        BezierTrajectory newTrajectory;

        // Obtain set of Bezier Curves
        std::vector<BezierCurve> &BCurves = curves[traj];
        // Obtain total arc length
        const float totalArcLength = curveArcLengths[traj];

        glm::vec3 pos;
        glm::vec3 tangent;
        uint32_t lineID = 0;
        uint32_t varIDPerLine = 0;
        // Start with first segment
        BCurves[0].evaluate(0, pos, tangent);

        newTrajectory.positions.push_back(pos);
        //newTrajectory.tangents.push_back(tangent);
        //newTrajectory.segmentID.push_back(lineID);

        // Now we store variable, min, and max, and var ID per vertex as new attributes
        newTrajectory.attributes.resize(8);
        float varValue = inTrajectories[traj].attributes[varIDPerLine][lineID];
        newTrajectory.attributes[0].push_back(varValue);
        newTrajectory.attributes[1].push_back(attributesMinMax[varIDPerLine].x);
        newTrajectory.attributes[2].push_back(attributesMinMax[varIDPerLine].y);
        // var ID
        newTrajectory.attributes[3].push_back(static_cast<float>(varIDPerLine));
        // var element index
        newTrajectory.attributes[4].push_back(static_cast<float>(lineID));
        // line ID
        newTrajectory.attributes[5].push_back(static_cast<float>(traj));
        // next line ID
        newTrajectory.attributes[6].push_back(static_cast<float>(std::min(lineID, uint32_t(BCurves.size() - 1))));
        // interpolant t
        newTrajectory.attributes[7].push_back(0.0f);


        curArcLength += rollSegLength;

        float sumArcLengths = 0.0f;
        float sumArcLengthsNext = BCurves[0].totalArcLength;

        varIDPerLine++;


        while (curArcLength <= totalArcLength) {
            // Obtain current Bezier segment index based on arc length
            while (sumArcLengthsNext <= curArcLength) {
                varIDPerLine = 0;
                lineID++;
                if (lineID >= BCurves.size()) {
                    break;
                }
                sumArcLengths = sumArcLengthsNext;
                sumArcLengthsNext += BCurves[lineID].totalArcLength;
            }
            if (lineID >= BCurves.size()) {
                break;
            }

            const auto &BCurve = BCurves[lineID];

            float t = BCurve.solveTForArcLength(curArcLength - sumArcLengths);

            BCurves[lineID].evaluate(t, pos, tangent);

            newTrajectory.positions.push_back(pos);
            //newTrajectory.tangents.push_back(tangent);
            //newTrajectory.segmentID.push_back(lineID);

            if (varIDPerLine < numVariables) {
                float varValue = inTrajectories[traj].attributes[varIDPerLine][lineID];
                newTrajectory.attributes[0].push_back(varValue);
                newTrajectory.attributes[1].push_back(attributesMinMax[varIDPerLine].x);
                newTrajectory.attributes[2].push_back(attributesMinMax[varIDPerLine].y);
                // var ID
                newTrajectory.attributes[3].push_back(static_cast<float>(varIDPerLine));

            } else {
                newTrajectory.attributes[0].push_back(0.0);
                newTrajectory.attributes[1].push_back(0.0);
                newTrajectory.attributes[2].push_back(0.0);
                newTrajectory.attributes[3].push_back(-1.0);
            }

            // var element index
            newTrajectory.attributes[4].push_back(static_cast<float>(lineID));
            // line ID
            newTrajectory.attributes[5].push_back(static_cast<float>(traj));
            // next line ID
            newTrajectory.attributes[6].push_back(
                    static_cast<float>(std::min(lineID + 1, uint32_t(BCurves.size() - 1))));
            // interpolant t
            float normalizedT = BCurve.normalizeT(t);
            newTrajectory.attributes[7].push_back(normalizedT);

            curArcLength += rollSegLength;
            varIDPerLine++;
        }

        newTrajectory.lineDesc = lineDescs[traj];
        newTrajectory.multiVarData = multiVarData[traj];
        newTrajectory.multiVarDescs = lineMultiVarDescs[traj];

        newTrajectories[traj] = newTrajectory;
    }

    return newTrajectories;
}
