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
    sgl::vk::BufferPtr getAmbientOcclusionBufferVulkan() override;

    void renderGui() override;

private:
    // OpenGL-Vulkan interoperability data.
    sgl::vk::BufferPtr aoBufferVk;
    sgl::GeometryBufferPtr aoBufferGl;
    sgl::SemaphoreVkGlInteropPtr renderReadySemaphore, renderFinishedSemaphore;

    // Vulkan render data.
    std::shared_ptr<AmbientOcclusionComputeRenderPass> aoComputeRenderPass;

    // How many iterations do we want to use for the ray tracer until we assume the result has converged?
    int maxNumIterations = 1;
    int numIterations = 0;

    bool useMainThread = false;
    std::thread workerThread;
};

class AmbientOcclusionComputeRenderPass : public sgl::vk::ComputePass {
public:
    explicit AmbientOcclusionComputeRenderPass(
            sgl::vk::Renderer* renderer, sgl::vk::BufferPtr& aoBufferVk, sgl::GeometryBufferPtr& aoBufferGl);

    // Public interface.
    void setLineData(LineDataPtr& lineData);

private:
    void loadShader() override;
    void setComputePipelineInfo(sgl::vk::ComputePipelineInfo& pipelineInfo) override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    std::thread computeThread;

    sgl::vk::BufferPtr& aoBufferVk;
    sgl::GeometryBufferPtr& aoBufferGl;

    void setTubeTriangleRenderData(const VulkanTubeTriangleRenderData& triangleRenderData);
    VulkanTubeTriangleRenderData tubeTriangleRenderData;
    sgl::vk::TopLevelAccelerationStructurePtr topLevelAS;

    // Uniform buffer object storing the line rendering settings.
    struct LineRenderSettings {
        float dummyData = 0.0f;
    };
    LineRenderSettings lineRenderSettings{};
    sgl::vk::BufferPtr lineRenderSettingsBuffer;
};

#endif //LINEVIS_VULKANAMBIENTOCCLUSIONBAKER_HPP
