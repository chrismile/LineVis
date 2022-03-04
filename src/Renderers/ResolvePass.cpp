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

#include <utility>

#include "LineRenderer.hpp"
#include "ResolvePass.hpp"

ResolvePass::ResolvePass(LineRenderer* lineRenderer, std::vector<std::string> customShaderIds)
        : sgl::vk::BlitRenderPass(
        *lineRenderer->getSceneData()->renderer, std::move(customShaderIds)),
          lineRenderer(lineRenderer) {
    this->setAttachmentLoadOp(VK_ATTACHMENT_LOAD_OP_CLEAR);
}

void ResolvePass::loadShader() {
    std::map<std::string, std::string> preprocessorDefines;
    lineRenderer->getVulkanShaderPreprocessorDefines(preprocessorDefines);

    // Resolve passes don't need fragment shader interlock.
    auto it = preprocessorDefines.find("__extensions");
    if (it != preprocessorDefines.end()) {
        if (it->second == "GL_ARB_fragment_shader_interlock") {
            preprocessorDefines.erase(it);
        }
    }
    preprocessorDefines.insert(std::make_pair("RESOLVE_PASS", ""));

    shaderStages = sgl::vk::ShaderManager->getShaderStages(shaderIds, preprocessorDefines);
}

void ResolvePass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    rasterData = std::make_shared<sgl::vk::RasterData>(renderer, graphicsPipeline);
    rasterData->setIndexBuffer(indexBuffer);
    rasterData->setVertexBuffer(vertexBuffer, 0);
    lineRenderer->setRenderDataBindings(rasterData);
}
