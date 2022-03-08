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
#include <Graphics/Color.hpp>
#include <utility>

namespace sgl { namespace dialog {
class MsgBoxHandle;
typedef std::shared_ptr<MsgBoxHandle> MsgBoxHandlePtr;
}}

namespace sgl { namespace vk {
class Renderer;
class Texture;
typedef std::shared_ptr<Texture> TexturePtr;
}}

class AutomaticPerformanceMeasurer;

struct SceneData {
    SceneData(
            sgl::vk::Renderer** renderer, sgl::vk::TexturePtr* sceneTexture, sgl::vk::TexturePtr* sceneDepthTexture,
            uint32_t* viewportWidth, uint32_t* viewportHeight,
            sgl::CameraPtr camera, sgl::Color* clearColor, bool* screenshotTransparentBackground,
            AutomaticPerformanceMeasurer** performanceMeasurer,
            bool* continuousRendering, bool* recordingMode, bool* useCameraFlight,
            float* MOVE_SPEED, float* MOUSE_ROT_SPEED,
            std::vector<sgl::dialog::MsgBoxHandlePtr>* nonBlockingMsgBoxHandles)
            : renderer(renderer), sceneTexture(sceneTexture), sceneDepthTexture(sceneDepthTexture),
            viewportWidth(viewportWidth), viewportHeight(viewportHeight),
              camera(std::move(camera)), clearColor(clearColor),
              screenshotTransparentBackground(screenshotTransparentBackground),
              performanceMeasurer(performanceMeasurer),
              continuousRendering(continuousRendering), recordingMode(recordingMode), useCameraFlight(useCameraFlight),
              MOVE_SPEED(MOVE_SPEED), MOUSE_ROT_SPEED(MOUSE_ROT_SPEED),
              nonBlockingMsgBoxHandles(nonBlockingMsgBoxHandles)
            {}
    sgl::vk::Renderer** renderer;
    sgl::vk::TexturePtr* sceneTexture;
    sgl::vk::TexturePtr* sceneDepthTexture;
    uint32_t* viewportWidth;
    uint32_t* viewportHeight;

    sgl::CameraPtr camera;
    sgl::Color* clearColor;
    bool* screenshotTransparentBackground;
    AutomaticPerformanceMeasurer** performanceMeasurer;
    bool* continuousRendering;
    bool* recordingMode;
    bool* useCameraFlight;

    float* MOVE_SPEED;
    float* MOUSE_ROT_SPEED;

    std::vector<sgl::dialog::MsgBoxHandlePtr>* nonBlockingMsgBoxHandles;
};

#endif //LINEVIS_SCENEDATA_HPP
