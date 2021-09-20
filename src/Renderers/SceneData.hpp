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

#ifndef LINEVIS_SCENEDATA_HPP
#define LINEVIS_SCENEDATA_HPP

#include <Graphics/Scene/Camera.hpp>
#include <Graphics/Buffers/FBO.hpp>
#include <Graphics/Color.hpp>

class AutomaticPerformanceMeasurer;

struct SceneData {
    SceneData(
            sgl::FramebufferObjectPtr& framebuffer, sgl::TexturePtr& sceneTexture,
            sgl::RenderbufferObjectPtr& sceneDepthRBO, sgl::CameraPtr& camera,
            sgl::Color& clearColor, bool& screenshotTransparentBackground,
            AutomaticPerformanceMeasurer*& performanceMeasurer, bool& recordingMode, bool& useCameraFlight)
            : framebuffer(framebuffer), sceneTexture(sceneTexture), sceneDepthRBO(sceneDepthRBO), camera(camera),
              clearColor(clearColor), screenshotTransparentBackground(screenshotTransparentBackground),
              performanceMeasurer(performanceMeasurer), recordingMode(recordingMode), useCameraFlight(useCameraFlight)
              {}
    sgl::FramebufferObjectPtr& framebuffer;
    sgl::TexturePtr& sceneTexture;
    sgl::RenderbufferObjectPtr& sceneDepthRBO;

    sgl::CameraPtr& camera;
    sgl::Color& clearColor;
    bool& screenshotTransparentBackground;
    AutomaticPerformanceMeasurer*& performanceMeasurer;
    bool& recordingMode;
    bool& useCameraFlight;
};

#endif //LINEVIS_SCENEDATA_HPP
