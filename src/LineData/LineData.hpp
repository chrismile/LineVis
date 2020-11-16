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

#include <Graphics/Buffers/GeometryBuffer.hpp>
#include <Graphics/Shader/Shader.hpp>
#include <Graphics/Shader/ShaderAttributes.hpp>
#include <ImGui/Widgets/TransferFunctionWindow.hpp>
#include "Loaders/DataSetList.hpp"
#include "Loaders/TrajectoryFile.hpp"

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

enum LineRasterizationRenderingTechnique {
    // Render screen-oriented bands, based on programmable vertex fetching or a geometry shader.
    LINE_RASTERIZATION_BAND_PROGRAMMABLE_FETCH,
    LINE_RASTERIZATION_BAND_GEOMETRY_SHADER,
    // Render tubes.
    LINE_RASTERIZATION_TUBE_GEOMETRY_SHADER,
    LINE_RASTERIZATION_TUBE_TRIANGLE_MESH
};
const char *const LINE_RASTERIZATION_ALL_TECHNIQUE_NAMES[] = {
        "Screen-aligned band (Prog. Fetch)", "Screen aligned band (GS)", "Tubes (GS)", "Tubes (Triangle Mesh)"
};
const int NUM_LINE_RASTERIZATION_TECHNIQUES_TOTAL =
        ((int)(sizeof(LINE_RASTERIZATION_ALL_TECHNIQUE_NAMES) / sizeof(*LINE_RASTERIZATION_ALL_TECHNIQUE_NAMES)));

class LineData {
public:
    LineData(sgl::TransferFunctionWindow &transferFunctionWindow, DataSetType dataSetType);
    ~LineData();
    void setQualityMeasureIndex(int qualityMeasureIdx);
    void onTransferFunctionMapRebuilt();
    inline bool isDirty() { return dirty; }
    inline DataSetType getType() { return dataSetType; }

    /**
     * Load line data from the selected file(s).
     * @param fileNames The names of the files to load. More than one file makes primarily sense for, e.g., stress line
     * data with multiple principal stress directions.
     * @param dataSetInformation Metadata about the data set.
     * @param transformationMatrixPtr A transformation to apply to the loaded data (if any; can be nullptr).
     * @return Whether loading was successful.
     */
    virtual bool loadFromFile(
            const std::vector<std::string>& fileNames, DataSetInformation dataSetInformation,
            glm::mat4* transformationMatrixPtr)=0;

    // Statistics.
    inline const sgl::AABB3& getModelBoundingBox() { return modelBoundingBox; }
    virtual size_t getNumAttributes()=0;
    virtual size_t getNumLines()=0;
    virtual size_t getNumLinePoints()=0;
    virtual size_t getNumLineSegments()=0;

    // Get filtered line data (only containing points also shown when rendering).
    virtual Trajectories filterTrajectoryData()=0;
    virtual std::vector<std::vector<glm::vec3>> getFilteredLines()=0;

    // --- Retrieve data for rendering. Preferred way. ---
    sgl::ShaderProgramPtr reloadGatherShader();
    sgl::ShaderAttributesPtr getGatherShaderAttributes(sgl::ShaderProgramPtr& gatherShader);
    void setUniformGatherShaderData(sgl::ShaderProgramPtr& gatherShader);
    void setUniformGatherShaderData_AllPasses();
    void setUniformGatherShaderData_Pass(sgl::ShaderProgramPtr& gatherShader);

    // --- Retrieve data for rendering. Only for renderers needing direct access! ---
    virtual TubeRenderData getTubeRenderData()=0;
    virtual TubeRenderDataProgrammableFetch getTubeRenderDataProgrammableFetch()=0;
    virtual TubeRenderDataOpacityOptimization getTubeRenderDataOpacityOptimization()=0;

    /**
     * For selecting rendering technique (e.g., screen-oriented bands, tubes) and other line data settings.
     * @return true if the gather shader needs to be reloaded.
     */
    virtual bool renderGui(bool isRasterizer);

protected:
    void rebuildInternalRepresentationIfNecessary();
    virtual void recomputeHistogram()=0;

    DataSetType dataSetType;
    sgl::AABB3 modelBoundingBox;
    std::vector<std::string> attributeNames;
    int qualityMeasureIdx = 0; ///< Selected attribute/importance criterion index.
    bool dirty = false; ///< Should be set to true if the representation changed.
    sgl::TransferFunctionWindow& transferFunctionWindow;

    // Rendering settings.
    static bool useProgrammableFetch;
    std::vector<std::string> supportedRenderingModes;

    /// Stores line point data if useProgrammableFetch is true.
    sgl::GeometryBufferPtr linePointDataSSBO;
};

#endif //STRESSLINEVIS_LINEDATA_HPP
