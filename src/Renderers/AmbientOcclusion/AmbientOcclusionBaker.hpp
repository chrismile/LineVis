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

#ifndef LINEVIS_AMBIENTOCCLUSIONBAKER_HPP
#define LINEVIS_AMBIENTOCCLUSIONBAKER_HPP

#include <memory>

namespace sgl {

class TransferFunctionWindow;
class GeometryBuffer;
typedef std::shared_ptr<GeometryBuffer> GeometryBufferPtr;

namespace vk {

class Renderer;
class Buffer;
typedef std::shared_ptr<Buffer> BufferPtr;

}

}

class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;

class AmbientOcclusionBaker {
public:
    AmbientOcclusionBaker(sgl::TransferFunctionWindow& transferFunctionWindow, sgl::vk::Renderer* rendererVk)
            : transferFunctionWindow(transferFunctionWindow), rendererVk(rendererVk) {}

    virtual void startAmbientOcclusionBaking(LineDataPtr& lineData)=0;
    virtual bool getHasComputationFinished()=0;
    virtual sgl::GeometryBufferPtr& getAmbientOcclusionBuffer()=0;
    virtual sgl::GeometryBufferPtr& getBlendingWeightsBuffer()=0;
#ifdef USE_VULKAN_INTEROP
    virtual sgl::vk::BufferPtr& getAmbientOcclusionBufferVulkan()=0;
    virtual sgl::vk::BufferPtr& getBlendingWeightsBufferVulkan()=0;
#endif
    virtual uint32_t getNumTubeSubdivisions()=0;
    virtual uint32_t getNumLineVertices()=0;
    virtual uint32_t getNumParametrizationVertices()=0;

    /// Returns whether the baking process was re-run.
    virtual bool renderGui() { return false; }

protected:
    sgl::TransferFunctionWindow& transferFunctionWindow;
    sgl::vk::Renderer* rendererVk;
    bool showWindow = true;
};

typedef std::shared_ptr<AmbientOcclusionBaker> AmbientOcclusionBakerPtr;

#endif //LINEVIS_AMBIENTOCCLUSIONBAKER_HPP
