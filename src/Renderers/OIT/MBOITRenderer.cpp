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

#include <Math/Geometry/MatrixUtil.hpp>
#include <Graphics/Renderer.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Shader/ShaderManager.hpp>
#include <Graphics/Scene/Camera.hpp>
#include <Graphics/Texture/TextureManager.hpp>
#include <Graphics/Buffers/FBO.hpp>
#include <Graphics/OpenGL/GeometryBuffer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "Utils/AutomaticPerformanceMeasurer.hpp"
#include "MBOITRenderer.hpp"

// Internal mode
bool MBOITRenderer::useStencilBuffer = true; ///< Use stencil buffer to mask unused pixels
bool MBOITRenderer::usePowerMoments = true;
int MBOITRenderer::numMoments = 4;
MBOITPixelFormat MBOITRenderer::pixelFormat = MBOIT_PIXEL_FORMAT_FLOAT_32;
bool MBOITRenderer::USE_R_RG_RGBA_FOR_MBOIT6 = true;
float MBOITRenderer::overestimationBeta = 0.1;

MBOITRenderer::MBOITRenderer(
        SceneData* sceneData, sgl::TransferFunctionWindow& transferFunctionWindow)
        : LineRenderer("Moment-Based Order Independent Transparency",
                       sceneData, transferFunctionWindow) {
    syncMode = getSupportedSyncMode();

    // Create moment OIT uniform data buffer.
    momentUniformData.moment_bias = 5*1e-7;
    momentUniformData.overestimation = overestimationBeta;
    computeWrappingZoneParameters(momentUniformData.wrapping_zone_parameters);
    momentOITUniformBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(MomentOITUniformData), &momentUniformData, sgl::UNIFORM_BUFFER);

    updateMomentMode();

    // Create blitting data (fullscreen rectangle in normalized device coordinates).
    blitRenderData = sgl::ShaderManager->createShaderAttributes(blendShader);

    std::vector<glm::vec3> fullscreenQuad{
            glm::vec3(1,1,0), glm::vec3(-1,-1,0), glm::vec3(1,-1,0),
            glm::vec3(-1,-1,0), glm::vec3(1,1,0), glm::vec3(-1,1,0)};
    sgl::GeometryBufferPtr geomBuffer = sgl::Renderer->createGeometryBuffer(
            sizeof(glm::vec3)*fullscreenQuad.size(), fullscreenQuad.data());
    blitRenderData->addGeometryBuffer(
            geomBuffer, "vertexPosition", sgl::ATTRIB_FLOAT, 3);

    onResolutionChanged();
}

void MBOITRenderer::updateSyncMode() {
    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);

    spinlockViewportBuffer = sgl::GeometryBufferPtr();
    if (syncMode == SYNC_SPINLOCK) {
        spinlockViewportBuffer = sgl::Renderer->createGeometryBuffer(
                sizeof(uint32_t) * size_t(width) * size_t(height),
                nullptr, sgl::SHADER_STORAGE_BUFFER);

        // Set all values in the buffer to zero.
        GLuint bufferId = static_cast<sgl::GeometryBufferGL*>(spinlockViewportBuffer.get())->getBuffer();
        uint32_t val = 0;
        glClearNamedBufferData(bufferId, GL_R32UI, GL_RED_INTEGER, GL_UNSIGNED_INT, (const void*)&val);
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    }
}

void MBOITRenderer::reloadShaders() {
    if (lineData) {
        reloadGatherShader();
    }
    reloadResolveShader();
}

void MBOITRenderer::reloadGatherShader(bool canCopyShaderAttributes) {
    if (syncMode == SYNC_FRAGMENT_SHADER_INTERLOCK) {
        sgl::ShaderManager->addPreprocessorDefine("USE_SYNC_FRAGMENT_SHADER_INTERLOCK", "");
        if (!useOrderedFragmentShaderInterlock) {
            sgl::ShaderManager->addPreprocessorDefine("INTERLOCK_UNORDERED", "");
        }
    } else if (syncMode == SYNC_SPINLOCK) {
        sgl::ShaderManager->addPreprocessorDefine("USE_SYNC_SPINLOCK", "");
        // Do not discard while keeping the spinlock locked.
        sgl::ShaderManager->addPreprocessorDefine("GATHER_NO_DISCARD", "");
    }

    sgl::ShaderManager->addPreprocessorDefine("USE_SCREEN_SPACE_POSITION", "");

    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"MBOITPass1.glsl\"");
    mboitPass1Shader = lineData->reloadGatherShader();
    if (canCopyShaderAttributes && shaderAttributesPass1) {
        shaderAttributesPass1 = shaderAttributesPass1->copy(mboitPass1Shader);
    }

    if (syncMode == SYNC_FRAGMENT_SHADER_INTERLOCK) {
        sgl::ShaderManager->removePreprocessorDefine("USE_SYNC_FRAGMENT_SHADER_INTERLOCK");
        if (!useOrderedFragmentShaderInterlock) {
            sgl::ShaderManager->removePreprocessorDefine("INTERLOCK_UNORDERED");
        }
    } else if (syncMode == SYNC_SPINLOCK) {
        sgl::ShaderManager->removePreprocessorDefine("USE_SYNC_SPINLOCK");
        sgl::ShaderManager->removePreprocessorDefine("GATHER_NO_DISCARD");
    }

    sgl::ShaderManager->invalidateShaderCache();
    sgl::ShaderManager->addPreprocessorDefine("OIT_GATHER_HEADER", "\"MBOITPass2.glsl\"");
    mboitPass2Shader = lineData->reloadGatherShader();
    if (canCopyShaderAttributes && shaderAttributesPass2) {
        shaderAttributesPass2 = shaderAttributesPass2->copy(mboitPass2Shader);
    }

    sgl::ShaderManager->removePreprocessorDefine("USE_SCREEN_SPACE_POSITION");
}

void MBOITRenderer::reloadResolveShader() {
    blendShader = sgl::ShaderManager->getShaderProgram({"MBOITBlend.Vertex", "MBOITBlend.Fragment"});
    if (blitRenderData) {
        blitRenderData = blitRenderData->copy(blendShader);
    }
}

void MBOITRenderer::updateMomentMode() {
    // 1. Set shader state dependent on the selected mode
    sgl::ShaderManager->addPreprocessorDefine("ROV", "1"); // Always use fragment shader interlock.
    sgl::ShaderManager->addPreprocessorDefine("NUM_MOMENTS", sgl::toString(numMoments));
    sgl::ShaderManager->addPreprocessorDefine(
            "SINGLE_PRECISION", sgl::toString((int)(pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32)));
    sgl::ShaderManager->addPreprocessorDefine("TRIGONOMETRIC", sgl::toString((int)(!usePowerMoments)));
    sgl::ShaderManager->addPreprocessorDefine(
            "USE_R_RG_RGBA_FOR_MBOIT6", sgl::toString((int)USE_R_RG_RGBA_FOR_MBOIT6));

    // 2. Re-load the shaders
    reloadShaders();

    // 3. Load textures
    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);

    //const GLint internalFormat1 = pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32 ? GL_R32F : GL_R16;
    const GLint internalFormat1 = GL_R32F;
    const GLint internalFormat2 = pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32 ? GL_RG32F : GL_RG16;
    const GLint internalFormat4 = pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32 ? GL_RGBA32F : GL_RGBA16;
    const GLint pixelFormat1 = GL_RED;
    const GLint pixelFormat2 = GL_RG;
    const GLint pixelFormat4 = GL_RGBA;

    int depthB0 = 1;
    int depthB = 1;
    int depthBExtra = 0;
    GLint internalFormatB0 = internalFormat1;
    GLint internalFormatB = internalFormat4;
    GLint internalFormatBExtra = 0;
    GLint pixelFormatB0 = pixelFormat1;
    GLint pixelFormatB = pixelFormat4;
    GLint pixelFormatBExtra = 0;

    if (numMoments == 6) {
        if (USE_R_RG_RGBA_FOR_MBOIT6) {
            depthBExtra = 1;
            internalFormatB = internalFormat2;
            pixelFormatB = pixelFormat2;
            internalFormatBExtra = internalFormat4;
            pixelFormatBExtra = pixelFormat4;
        } else {
            depthB = 3;
            internalFormatB = internalFormat2;
            pixelFormatB = pixelFormat2;
        }
    } else if (numMoments == 8) {
        depthB = 2;
    }

    // Highest memory requirement: width * height * sizeof(DATATYPE) * #moments
    void *emptyData = calloc(width * height, sizeof(float) * 8);

    textureSettingsB0 = sgl::TextureSettings();
    textureSettingsB0.type = sgl::TEXTURE_2D_ARRAY;
    textureSettingsB0.internalFormat = internalFormatB0;
    b0 = sgl::TextureManager->createTexture(
            emptyData, width, height, depthB0, sgl::PixelFormat(pixelFormatB0, GL_FLOAT), textureSettingsB0);

    textureSettingsB = textureSettingsB0;
    textureSettingsB.internalFormat = internalFormatB;
    b = sgl::TextureManager->createTexture(
            emptyData, width, height, depthB, sgl::PixelFormat(pixelFormatB, GL_FLOAT), textureSettingsB);

    if (numMoments == 6 && USE_R_RG_RGBA_FOR_MBOIT6) {
        textureSettingsBExtra = textureSettingsB0;
        textureSettingsBExtra.internalFormat = internalFormatBExtra;
        bExtra = sgl::TextureManager->createTexture(
                emptyData, width, height, depthBExtra, sgl::PixelFormat(pixelFormatBExtra, GL_FLOAT),
                textureSettingsBExtra);
    }

    size_t baseSizeBytes = MBOIT_PIXEL_FORMAT_FLOAT_32 ? 4 : 2;
    if ((*sceneData->performanceMeasurer)) {
        (*sceneData->performanceMeasurer)->setCurrentAlgorithmBufferSizeBytes(
                baseSizeBytes * numMoments * width * height);
    }
    free(emptyData);


    // Set algorithm-dependent bias
    if (usePowerMoments) {
        if (numMoments == 4 && pixelFormat == MBOIT_PIXEL_FORMAT_UNORM_16) {
            momentUniformData.moment_bias = 6*1e-4; // 6*1e-5
        } else if (numMoments == 4 && pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32) {
            momentUniformData.moment_bias = 5*1e-7; // 5*1e-7
        } else if (numMoments == 6 && pixelFormat == MBOIT_PIXEL_FORMAT_UNORM_16) {
            momentUniformData.moment_bias = 6*1e-3; // 6*1e-4
        } else if (numMoments == 6 && pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32) {
            momentUniformData.moment_bias = 5*1e-6; // 5*1e-6
        } else if (numMoments == 8 && pixelFormat == MBOIT_PIXEL_FORMAT_UNORM_16) {
            momentUniformData.moment_bias = 2.5*1e-2; // 2.5*1e-3
        } else if (numMoments == 8 && pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32) {
            momentUniformData.moment_bias = 5*1e-5; // 5*1e-5
        }
    } else {
        if (numMoments == 4 && pixelFormat == MBOIT_PIXEL_FORMAT_UNORM_16) {
            momentUniformData.moment_bias = 4*1e-3; // 4*1e-4
        } else if (numMoments == 4 && pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32) {
            momentUniformData.moment_bias = 4*1e-7; // 4*1e-7
        } else if (numMoments == 6 && pixelFormat == MBOIT_PIXEL_FORMAT_UNORM_16) {
            momentUniformData.moment_bias = 6.5*1e-3; // 6.5*1e-4
        } else if (numMoments == 6 && pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32) {
            momentUniformData.moment_bias = 8*1e-6; // 8*1e-7
        } else if (numMoments == 8 && pixelFormat == MBOIT_PIXEL_FORMAT_UNORM_16) {
            momentUniformData.moment_bias = 8.5*1e-3; // 8.5*1e-4
        } else if (numMoments == 8 && pixelFormat == MBOIT_PIXEL_FORMAT_FLOAT_32) {
            momentUniformData.moment_bias = 1.5*1e-5; // 1.5*1e-6;
        }
    }

    momentOITUniformBuffer->subData(0, sizeof(MomentOITUniformData), &momentUniformData);
}

void MBOITRenderer::setNewState(const InternalState& newState) {
    numMoments = newState.rendererSettings.getIntValue("numMoments");
    pixelFormat =
            newState.rendererSettings.getValue("pixelFormat") == "Float"
            ? MBOIT_PIXEL_FORMAT_FLOAT_32 : MBOIT_PIXEL_FORMAT_UNORM_16;
    if (numMoments == 6) {
        USE_R_RG_RGBA_FOR_MBOIT6 = newState.rendererSettings.getBoolValue("USE_R_RG_RGBA_FOR_MBOIT6");
    }
    usePowerMoments = newState.rendererSettings.getBoolValue("usePowerMoments");

    if (newState.rendererSettings.getValueOpt("overestimationBeta", overestimationBeta)) {
        momentUniformData.overestimation = overestimationBeta;
        // subData already called in updateMomentMode
        //momentOITUniformBuffer->subData(0, sizeof(MomentOITUniformData), &momentUniformData);
    } else {
        overestimationBeta = 0.1f;
        momentUniformData.overestimation = overestimationBeta;
    }

    newState.rendererSettings.getValueOpt("useStencilBuffer", useStencilBuffer);

    bool recompileGatherShader = false;
    if (newState.rendererSettings.getValueOpt(
            "useOrderedFragmentShaderInterlock", useOrderedFragmentShaderInterlock)) {
        recompileGatherShader = true;
    }
    int syncModeInt = int(syncMode);
    if (newState.rendererSettings.getValueOpt("syncMode", syncModeInt)) {
        syncMode = SyncMode(syncModeInt);
        updateSyncMode();
        recompileGatherShader = true;
    }
    if (recompileGatherShader && lineData) {
        reloadGatherShader();
    }

    updateMomentMode();
}

void MBOITRenderer::setLineData(LineDataPtr& lineData, bool isNewData) {
    updateNewLineData(lineData, isNewData);

    // Unload old data.
    shaderAttributesPass1 = sgl::ShaderAttributesPtr();
    shaderAttributesPass2 = sgl::ShaderAttributesPtr();
    shaderAttributesPass1 = lineData->getGatherShaderAttributes(mboitPass1Shader);
    shaderAttributesPass2 = lineData->getGatherShaderAttributes(mboitPass2Shader);

    dirty = false;
    reRender = true;
}

void MBOITRenderer::onResolutionChanged() {
    int width = int(*sceneData->viewportWidth);
    int height = int(*sceneData->viewportHeight);

    // Create accumulator framebuffer object & texture
    blendFBO = sgl::Renderer->createFBO();
    sgl::TextureSettings textureSettings;
    textureSettings.internalFormat = GL_RGBA32F;
    blendRenderTexture = sgl::TextureManager->createEmptyTexture(width, height, textureSettings);
    blendFBO->bindTexture(blendRenderTexture);
    blendFBO->bindRenderbuffer(*sceneData->sceneDepthRBO, sgl::DEPTH_STENCIL_ATTACHMENT);

    updateMomentMode();
}

void MBOITRenderer::render() {
    LineRenderer::render();

    setUniformData();
    computeDepthRange();
    gather();
    resolve();
}

void MBOITRenderer::setUniformData() {
    //int width = int(*sceneData->viewportWidth);
    //int height = int(*sceneData->viewportHeight);

    if (syncMode == SYNC_SPINLOCK) {
        sgl::ShaderManager->bindShaderStorageBuffer(1, spinlockViewportBuffer);
    }

    lineData->setUniformGatherShaderData_AllPasses();

    mboitPass1Shader->setUniformImageTexture(
            0, b0, textureSettingsB0.internalFormat, GL_READ_WRITE,
            0, true, 0);
    mboitPass1Shader->setUniformImageTexture(
            1, b, textureSettingsB.internalFormat, GL_READ_WRITE,
            0, true, 0);
    //mboitPass1Shader->setUniform("viewportW", width);
    mboitPass1Shader->setUniform("cameraPosition", (*sceneData->camera)->getPosition());
    mboitPass1Shader->setUniform("lineWidth", lineWidth);
    if (mboitPass1Shader->hasUniform("backgroundColor")) {
        glm::vec3 backgroundColor = sceneData->clearColor->getFloatColorRGB();
        mboitPass1Shader->setUniform("backgroundColor", backgroundColor);
    }
    if (mboitPass1Shader->hasUniform("foregroundColor")) {
        glm::vec3 backgroundColor = sceneData->clearColor->getFloatColorRGB();
        glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;
        mboitPass1Shader->setUniform("foregroundColor", foregroundColor);
    }
    lineData->setUniformGatherShaderData_Pass(mboitPass1Shader);
    setUniformData_Pass(mboitPass1Shader);

    mboitPass2Shader->setUniformImageTexture(
            0, b0, textureSettingsB0.internalFormat, GL_READ_WRITE,
            0, true, 0); // GL_READ_ONLY? -> Shader
    mboitPass2Shader->setUniformImageTexture(
            1, b, textureSettingsB.internalFormat, GL_READ_WRITE,
            0, true, 0); // GL_READ_ONLY? -> Shader
    //mboitPass2Shader->setUniform("viewportW", width);
    mboitPass2Shader->setUniform("cameraPosition", (*sceneData->camera)->getPosition());
    mboitPass2Shader->setUniform("lineWidth", lineWidth);
    if (mboitPass2Shader->hasUniform("backgroundColor")) {
        glm::vec3 backgroundColor = sceneData->clearColor->getFloatColorRGB();
        mboitPass2Shader->setUniform("backgroundColor", backgroundColor);
    }
    if (mboitPass2Shader->hasUniform("foregroundColor")) {
        glm::vec3 backgroundColor = sceneData->clearColor->getFloatColorRGB();
        glm::vec3 foregroundColor = glm::vec3(1.0f) - backgroundColor;
        mboitPass2Shader->setUniform("foregroundColor", foregroundColor);
    }
    lineData->setUniformGatherShaderData(mboitPass2Shader);
    setUniformData_Pass(mboitPass2Shader);

    blendShader->setUniformImageTexture(
            0, b0, textureSettingsB0.internalFormat, GL_READ_WRITE,
            0, true, 0); // GL_READ_ONLY? -> Shader
    blendShader->setUniformImageTexture(
            1, b, textureSettingsB.internalFormat, GL_READ_WRITE,
            0, true, 0); // GL_READ_ONLY? -> Shader
    blendShader->setUniform("transparentSurfaceAccumulator", blendRenderTexture, 0);
    //blendShader->setUniform("viewportW", width);

    if (numMoments == 6 && USE_R_RG_RGBA_FOR_MBOIT6) {
        mboitPass1Shader->setUniformImageTexture(
                2, bExtra, textureSettingsBExtra.internalFormat, GL_READ_WRITE,
                0, true, 0);
        mboitPass2Shader->setUniformImageTexture(
                2, bExtra, textureSettingsBExtra.internalFormat, GL_READ_WRITE,
                0, true, 0); // GL_READ_ONLY? -> Shader
        blendShader->setUniformImageTexture(
                2, bExtra, textureSettingsBExtra.internalFormat, GL_READ_WRITE,
                0, true, 0); // GL_READ_ONLY? -> Shader
    }

    mboitPass1Shader->setUniformBuffer(1, "MomentOITUniformData", momentOITUniformBuffer);
    mboitPass2Shader->setUniformBuffer(1, "MomentOITUniformData", momentOITUniformBuffer);
}

void MBOITRenderer::computeDepthRange() {
    const sgl::AABB3& boundingBox = lineData->getModelBoundingBox();
    sgl::AABB3 screenSpaceBoundingBox = boundingBox.transformed((*sceneData->camera)->getViewMatrix());

    // Add offset of 0.1 for e.g. point data sets where additonal vertices may be added in the shader for quads.
    float minViewZ = screenSpaceBoundingBox.getMaximum().z + 0.1f;
    float maxViewZ = screenSpaceBoundingBox.getMinimum().z - 0.1f;
    minViewZ = std::max(-minViewZ, (*sceneData->camera)->getNearClipDistance());
    maxViewZ = std::min(-maxViewZ, (*sceneData->camera)->getFarClipDistance());
    minViewZ = std::min(minViewZ, (*sceneData->camera)->getFarClipDistance());
    maxViewZ = std::max(maxViewZ, (*sceneData->camera)->getNearClipDistance());
    float logmin = log(minViewZ);
    float logmax = log(maxViewZ);
    mboitPass1Shader->setUniform("logDepthMin", logmin);
    mboitPass1Shader->setUniform("logDepthMax", logmax);
    mboitPass2Shader->setUniform("logDepthMin", logmin);
    mboitPass2Shader->setUniform("logDepthMax", logmax);
}

void MBOITRenderer::gather() {
    setUniformData();

    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);

    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());
    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    glEnable(GL_DEPTH_TEST);

    if (useStencilBuffer) {
        glEnable(GL_STENCIL_TEST);
        glStencilFunc(GL_ALWAYS, 1, 0xFF);
        glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
        glStencilMask(0xFF);
        glClear(GL_STENCIL_BUFFER_BIT);
    }

    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glDisable(GL_CULL_FACE);
    }

    sgl::Renderer->setProjectionMatrix((*sceneData->camera)->getProjectionMatrix());
    sgl::Renderer->setViewMatrix((*sceneData->camera)->getViewMatrix());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());

    sgl::Renderer->render(shaderAttributesPass1);

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

    sgl::Renderer->bindFBO(blendFBO);
    sgl::Renderer->clearFramebuffer(GL_COLOR_BUFFER_BIT, sgl::Color(0, 0, 0, 0));

    glBlendFuncSeparate(GL_ONE, GL_ONE, GL_ONE, GL_ONE);

    sgl::Renderer->render(shaderAttributesPass2);

    if (lineData->getLinePrimitiveMode() == LineData::LINE_PRIMITIVES_BAND) {
        glEnable(GL_CULL_FACE);
    }

    glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
}

void MBOITRenderer::resolve() {
    glDisable(GL_DEPTH_TEST);

    sgl::Renderer->setProjectionMatrix(sgl::matrixIdentity());
    sgl::Renderer->setViewMatrix(sgl::matrixIdentity());
    sgl::Renderer->setModelMatrix(sgl::matrixIdentity());


    if (useStencilBuffer) {
        glStencilFunc(GL_EQUAL, 1, 0xFF);
        glStencilMask(0x00);
    }

    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE);

    sgl::Renderer->bindFBO(*sceneData->framebuffer);
    sgl::Renderer->render(blitRenderData);

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_STENCIL_TEST);

    glDepthMask(GL_TRUE);
}

void MBOITRenderer::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    LineRenderer::renderGuiPropertyEditorNodes(propertyEditor);

    // USE_R_RG_RGBA_FOR_MBOIT6
    const char *const momentModes[] = {
            "Power Moments: 4", "Power Moments: 6 (Layered)", "Power Moments: 6 (R_RG_RGBA)", "Power Moments: 8",
            "Trigonometric Moments: 2", "Trigonometric Moments: 3 (Layered)", "Trigonometric Moments: 3 (R_RG_RGBA)",
            "Trigonometric Moments: 4"};
    const int momentModesNumMoments[] = {4, 6, 6, 8, 4, 6, 6, 8};
    static int momentModeIndex = -1;
    if (true) { // momentModeIndex == -1
        // Initialize
        momentModeIndex = usePowerMoments ? 0 : 4;
        momentModeIndex += numMoments/2 - 2;
        momentModeIndex += (USE_R_RG_RGBA_FOR_MBOIT6 && numMoments == 6) ? 1 : 0;
        momentModeIndex += (numMoments == 8) ? 1 : 0;
    }

    if (propertyEditor.addCombo("Moment Mode", &momentModeIndex, momentModes, IM_ARRAYSIZE(momentModes))) {
        usePowerMoments = (momentModeIndex / 4) == 0;
        numMoments = momentModesNumMoments[momentModeIndex]; // Count complex moments * 2
        USE_R_RG_RGBA_FOR_MBOIT6 = (momentModeIndex == 2) || (momentModeIndex == 6);
        updateMomentMode();
        reRender = true;
    }

    const char *const pixelFormatModes[] = {"Float 32-bit", "UNORM Integer 16-bit"};
    if (propertyEditor.addCombo(
            "Pixel Format", (int*)&pixelFormat, pixelFormatModes, IM_ARRAYSIZE(pixelFormatModes))) {
        updateMomentMode();
        reRender = true;
    }

    if (propertyEditor.addSliderFloat("Overestimation", &overestimationBeta, 0.0f, 1.0f, "%.2f")) {
        momentUniformData.overestimation = overestimationBeta;
        momentOITUniformBuffer->subData(0, sizeof(MomentOITUniformData), &momentUniformData);
        reRender = true;
    }

    const char *syncModeNames[] = { "No Sync (Unsafe)", "Fragment Shader Interlock", "Spinlock" };
    if (propertyEditor.addCombo(
            "Sync Mode", (int*)&syncMode, syncModeNames, IM_ARRAYSIZE(syncModeNames))) {
        updateSyncMode();
        reloadGatherShader();
        reRender = true;
    }
    if (syncMode == SYNC_FRAGMENT_SHADER_INTERLOCK && propertyEditor.addCheckbox(
            "Ordered Sync", &useOrderedFragmentShaderInterlock)) {
        reloadGatherShader();
        reRender = true;
    }
}
