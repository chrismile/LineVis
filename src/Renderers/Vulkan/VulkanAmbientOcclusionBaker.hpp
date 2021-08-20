/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Christoph Neuhauser
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

#ifndef LINEVIS_VULKANAMBIENTOCCLUSIONBAKER_HPP
#define LINEVIS_VULKANAMBIENTOCCLUSIONBAKER_HPP

#include <memory>
#include <thread>
#include <Graphics/Vulkan/Render/Passes/Pass.hpp>
#include "LineData/LineRenderData.hpp"
#include "../AmbientOcclusion/AmbientOcclusionBaker.hpp"

namespace sgl {

class TransferFunctionWindow;
class GeometryBuffer;
typedef std::shared_ptr<GeometryBuffer> GeometryBufferPtr;
class ShaderProgram;
typedef std::shared_ptr<ShaderProgram> ShaderProgramPtr;
class ShaderAttributes;
typedef std::shared_ptr<ShaderAttributes> ShaderAttributesPtr;
class SemaphoreVkGlInterop;
typedef std::shared_ptr<SemaphoreVkGlInterop> SemaphoreVkGlInteropPtr;

namespace vk {

class Renderer;
class Buffer;
typedef std::shared_ptr<Buffer> BufferPtr;
class Semaphore;
typedef std::shared_ptr<Semaphore> SemaphorePtr;

}

}

class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;

class AmbientOcclusionComputeRenderPass;

class VulkanAmbientOcclusionBaker : public AmbientOcclusionBaker {
public:
    VulkanAmbientOcclusionBaker(sgl::TransferFunctionWindow& transferFunctionWindow, sgl::vk::Renderer* rendererVk);
    void startAmbientOcclusionBaking(LineDataPtr& lineData) override;
    bool getHasComputationFinished() override;
    sgl::GeometryBufferPtr getAmbientOcclusionBuffer() override;
    sgl::GeometryBufferPtr getBlendingWeightsBuffer() override;
    sgl::vk::BufferPtr getAmbientOcclusionBufferVulkan() override;
    sgl::vk::BufferPtr getBlendingWeightsBufferVulkan() override;
    uint32_t getNumTubeSubdivisions() override;
    uint32_t getNumLineVertices() override;

    void renderGui() override;

private:
    // OpenGL-Vulkan interoperability data.
    sgl::vk::BufferPtr aoBufferVk;
    sgl::GeometryBufferPtr aoBufferGl;
    sgl::SemaphoreVkGlInteropPtr renderReadySemaphore, renderFinishedSemaphore;
    std::vector<sgl::vk::SemaphorePtr> waitSemaphores;
    std::vector<sgl::vk::SemaphorePtr> signalSemaphores;
    std::vector<VkCommandBuffer> commandBuffers;

    // Vulkan render data.
    std::shared_ptr<AmbientOcclusionComputeRenderPass> aoComputeRenderPass;

    // How many iterations do we want to use for the ray tracer until we assume the result has converged?
    int maxNumIterations = 1;
    int numIterations = 0;

    bool useMainThread = false;
    std::thread workerThread;
};

class AmbientOcclusionComputeRenderPass : public sgl::vk::ComputePass {
    friend class VulkanAmbientOcclusionBaker;
public:
    explicit AmbientOcclusionComputeRenderPass(
            sgl::vk::Renderer* renderer, sgl::vk::BufferPtr& aoBufferVk, sgl::GeometryBufferPtr& aoBufferGl);

    // Public interface.
    void setLineData(LineDataPtr& lineData);
    inline void setFrameNumber(uint32_t frame) { lineRenderSettings.frameNumber = frame; }
    inline sgl::GeometryBufferPtr getAmbientOcclusionBuffer() { return aoBufferGl; }
    inline sgl::GeometryBufferPtr getBlendingWeightsBuffer() { return blendingWeightParametrizationBufferGl; }
    inline sgl::vk::BufferPtr getAmbientOcclusionBufferVulkan() { return aoBufferVk; }
    inline sgl::vk::BufferPtr getBlendingWeightsBufferVulkan() { return blendingWeightParametrizationBuffer; }
    inline uint32_t getNumTubeSubdivisions() const { return numTubeSubdivisions; }
    inline uint32_t getNumLineVertices() const { return numLineVertices; }

private:
    void loadShader() override;
    void setComputePipelineInfo(sgl::vk::ComputePipelineInfo& pipelineInfo) override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    std::thread computeThread;

    // Line data.
    std::vector<std::vector<glm::vec3>> lines;

    // Blending weight parametrization for the line segments.
    void generateBlendingWeightParametrization();
    void recomputeStaticParametrization();
    float linesLengthSum = 0.0f; ///< Length sum of all polylineLengths.
    std::vector<float> polylineLengths; ///< Lengths of all polylines.

    // Resolution of the ambient occlusion data.
    float expectedParamSegmentLength = 0.001f;
    uint32_t numTubeSubdivisions = 8;
    uint32_t numAmbientOcclusionSamplesPerFrame = 64;
    float ambientOcclusionRadius = 0.05f;
    bool useDistance = true;

    // Information about geometry data.
    uint32_t numPolylineSegments = 0;
    uint32_t numLineSegments = 0;
    uint32_t numLineVertices = 0;

    sgl::vk::BufferPtr& aoBufferVk;
    sgl::GeometryBufferPtr& aoBufferGl;

    struct LinePoint {
        glm::vec4 position;
        glm::vec4 tangent;
        glm::vec4 normal;
    };
    sgl::vk::BufferPtr linePointsBuffer;

    sgl::vk::BufferPtr blendingWeightParametrizationBuffer;
    sgl::GeometryBufferPtr blendingWeightParametrizationBufferGl;
    sgl::vk::BufferPtr lineSegmentVertexConnectivityBuffer;
    sgl::vk::BufferPtr samplingLocationsBuffer;

    sgl::vk::TopLevelAccelerationStructurePtr topLevelAS;

    // Uniform buffer object storing the line rendering settings.
    struct LineRenderSettings {
        // The radius of the lines.
        float lineRadius;

        // How many line points exist in total (i.e., the number of entries in the buffer "LineGeometry").
        uint32_t numLinePoints;
        // How often should the tube be subdivided in the normal plane?
        uint32_t numTubeSubdivisions;
        // The number of this frame (used for accumulation of samples accross frames).
        uint32_t frameNumber;

        // How many rays should the shader shoot?
        uint32_t numAmbientOcclusionSamples;
        // What is the radius to take into account for ambient occlusion?
        float ambientOcclusionRadius;
        // Should the distance of the AO hits be used?
        int useDistance;

        int padding = 0;
    };
    LineRenderSettings lineRenderSettings{};
    sgl::vk::BufferPtr lineRenderSettingsBuffer;
};

#endif //LINEVIS_VULKANAMBIENTOCCLUSIONBAKER_HPP
