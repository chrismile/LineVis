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

#ifndef STRESSLINEVIS_LINEDATA_HPP
#define STRESSLINEVIS_LINEDATA_HPP

#include <memory>

#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#include "Loaders/DataSetList.hpp"
#include "Loaders/TrajectoryFile.hpp"
#include "Graphics/Buffers/GeometryBuffer.hpp"

struct Trajectory;
typedef std::vector<Trajectory> Trajectories;

class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;

struct TubeRenderData {
    sgl::GeometryBufferPtr indexBuffer;
    sgl::GeometryBufferPtr vertexPositionBuffer;
    sgl::GeometryBufferPtr vertexAttributeBuffer;
    sgl::GeometryBufferPtr vertexNormalBuffer;
    sgl::GeometryBufferPtr vertexTangentBuffer;
    sgl::GeometryBufferPtr vertexPrincipalStressIndexBuffer; ///< Empty for flow lines.
};

/// For internal use of subclasses.
struct LinePointDataProgrammableFetch {
    glm::vec3 vertexPosition;
    float vertexAttribute;
    glm::vec3 vertexTangent;
    uint32_t principalStressIndex; ///< Padding in case of flow lines.
};

struct TubeRenderDataProgrammableFetch {
    sgl::GeometryBufferPtr indexBuffer;
    sgl::GeometryBufferPtr linePointsBuffer;
};

struct TubeRenderDataOpacityOptimization {
    sgl::GeometryBufferPtr indexBuffer;
    sgl::GeometryBufferPtr vertexPositionBuffer;
    sgl::GeometryBufferPtr vertexAttributeBuffer;
    sgl::GeometryBufferPtr vertexTangentBuffer;
    sgl::GeometryBufferPtr vertexPrincipalStressIndexBuffer; ///< Empty for flow lines.
};

struct PointRenderData {
    sgl::GeometryBufferPtr vertexPositionBuffer;
};

class LineData {
public:
    LineData(sgl::TransferFunctionWindow &transferFunctionWindow, DataSetType dataSetType);
    ~LineData();
    void setQualityMeasureIndex(int qualityMeasureIdx);
    void onTransferFunctionMapRebuilt();
    inline bool isDirty() { return dirty; }
    inline DataSetType getType() { return dataSetType; }

    // Statistics.
    virtual size_t getNumAttributes()=0;
    virtual size_t getNumLines()=0;
    virtual size_t getNumLinePoints()=0;
    virtual size_t getNumLineSegments()=0;

    // Get filtered line data (only containing points also shown when rendering).
    virtual Trajectories filterTrajectoryData()=0;
    virtual std::vector<std::vector<glm::vec3>> getFilteredLines()=0;


    // --- Retrieve data for rendering. ---
    virtual TubeRenderData getTubeRenderData()=0;
    virtual TubeRenderDataProgrammableFetch getTubeRenderDataProgrammableFetch()=0;
    virtual TubeRenderDataOpacityOptimization getTubeRenderDataOpacityOptimization()=0;

protected:
    void rebuildInternalRepresentationIfNecessary();
    virtual void recomputeHistogram()=0;

    DataSetType dataSetType;
    int qualityMeasureIdx = 0; ///< Selected attribute/importance criterion index.
    bool dirty = false;
    sgl::TransferFunctionWindow& transferFunctionWindow;
};

#endif //STRESSLINEVIS_LINEDATA_HPP
