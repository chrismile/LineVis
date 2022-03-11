/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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

#ifndef LINEVIS_DEPTHCUES_HPP
#define LINEVIS_DEPTHCUES_HPP

#include <Graphics/Vulkan/Render/Passes/Pass.hpp>
#include "SceneData.hpp"

// For compute shaders.
constexpr size_t BLOCK_SIZE_DEPTH_CUES = 256;

class ComputeDepthValuesPass : public sgl::vk::ComputePass {
public:
    explicit ComputeDepthValuesPass(SceneData* sceneData);

    // Public interface.
    void setLineVerticesBuffer(const sgl::vk::BufferPtr& buffer);
    void setDepthMinMaxOutBuffer(const sgl::vk::BufferPtr& buffer);

private:
    void loadShader() override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    SceneData* sceneData;
    sgl::vk::BufferPtr lineVerticesBuffer;
    sgl::vk::BufferPtr depthMinMaxOutBuffer;

    // Uniform buffer object storing the camera settings.
    struct UniformData {
        float nearDist = 0.0f; ///< The distance of the near plane.
        float farDist = 0.0f; ///< The distance of the far plane.
        uint32_t numVertices = 0; ///< Number of entries in VertexPositionBuffer.
        uint32_t padding = 0;
        glm::mat4 cameraViewMatrix;
        glm::mat4 cameraProjectionMatrix;
    };
    UniformData uniformData{};
    sgl::vk::BufferPtr uniformDataBuffer;
};

class MinMaxDepthReductionPass : public sgl::vk::ComputePass {
public:
    explicit MinMaxDepthReductionPass(SceneData* sceneData);

    // Public interface.
    void setDepthMinMaxBuffers(const sgl::vk::BufferPtr& bufferIn, const sgl::vk::BufferPtr& bufferOut);
    inline void setInputSize(uint32_t _inputSize) { inputSize = _inputSize; }

private:
    void loadShader() override;
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    SceneData* sceneData;
    sgl::vk::BufferPtr depthMinMaxInBuffer;
    sgl::vk::BufferPtr depthMinMaxOutBuffer;
    uint32_t inputSize = 0;

    // Uniform buffer object storing the camera settings.
    struct UniformData {
        float nearDist = 0.0f; ///< The distance of the near plane.
        float farDist = 0.0f; ///< The distance of the far plane.
        uint numVertices = 0; ///< Number of entries in VertexPositionBuffer.
        uint padding = 0;
        glm::mat4 cameraViewMatrix;
        glm::mat4 cameraProjectionMatrix;
    };
    UniformData uniformData{};
    sgl::vk::BufferPtr uniformDataBuffer;
};

#endif //LINEVIS_DEPTHCUES_HPP
