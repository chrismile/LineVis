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
#include <Graphics/Vulkan/Buffers/Buffer.hpp>

namespace IGFD {
class FileDialog;
}
typedef IGFD::FileDialog ImGuiFileDialog;

namespace sgl {

class TransferFunctionWindow;
class PropertyEditor;
class GeometryBuffer;
typedef std::shared_ptr<GeometryBuffer> GeometryBufferPtr;
class Texture;
typedef std::shared_ptr<Texture> TexturePtr;

namespace vk {

class Renderer;
class Buffer;
typedef std::shared_ptr<Buffer> BufferPtr;
class Texture;
typedef std::shared_ptr<Texture> TexturePtr;

}

}

class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;
class SettingsMap;

/**
 * Ambient occlusion baking modes:
 * - Immediate: Halt the program until the baking process has finished.
 * - Iterative update: Update the AO texture once every frame until enough samples have been collected.
 * - Multi-threaded: Let the baking process run in a separate thread. Display the AO texture once baking has finished.
 */
enum class BakingMode {
    IMMEDIATE, ITERATIVE_UPDATE, MULTI_THREADED
};
const char* const BAKING_MODE_NAMES[] = {
        "Immediate", "Iterative", "Multi-Threaded"
};

#define AMBIENT_OCCLUSION_TYPES                         \
    AO_TYPE("RTAO (Prebaker)",     RTAO_PREBAKER, 0)    \
    AO_TYPE("RTAO (Screen Space)", RTAO,          1)    \
    AO_TYPE("SSAO",                SSAO,          2)    \
    AO_TYPE("GTAO (reference)",    GTAO,          3)

enum class AmbientOcclusionBakerType {
    NONE = -1,

  #define AO_TYPE(_, name, value) name = value,
    AMBIENT_OCCLUSION_TYPES
  #undef AO_TYPE
};
const char* const AMBIENT_OCCLUSION_BAKER_TYPE_NAMES[] = {
  #define AO_TYPE(str, _1, _2) str,
    AMBIENT_OCCLUSION_TYPES
  #undef AO_TYPE
};

class AmbientOcclusionBaker {
public:
    explicit AmbientOcclusionBaker(sgl::vk::Renderer* renderer) : rendererMain(renderer) {}
    virtual ~AmbientOcclusionBaker();

    [[nodiscard]] inline BakingMode getBakingMode() const { return bakingMode; }

    virtual AmbientOcclusionBakerType getType()=0;
    virtual bool getIsStaticPrebaker()=0;
    virtual void startAmbientOcclusionBaking(LineDataPtr& lineData, bool isNewData)=0;
    virtual void updateIterative(VkPipelineStageFlags pipelineStageFlags)=0;
    virtual void updateMultiThreaded(VkPipelineStageFlags pipelineStageFlags)=0;
    virtual bool getIsDataReady()=0;
    virtual bool getIsComputationRunning()=0;
    virtual bool getHasComputationFinished()=0;
    virtual bool getHasThreadUpdate()=0;

    // Ambient occlusion baker type 1: Preprocessing of static ambient occlusion.
    virtual sgl::vk::BufferPtr getAmbientOcclusionBuffer()=0;
    virtual sgl::vk::BufferPtr getBlendingWeightsBuffer()=0;
    virtual uint32_t getNumTubeSubdivisions()=0;
    virtual uint32_t getNumLineVertices()=0;
    virtual uint32_t getNumParametrizationVertices()=0;

    // Ambient occlusion baker type 2: Compute ambient occlusion per frame.
    virtual sgl::vk::TexturePtr getAmbientOcclusionFrameTexture()=0;
    virtual bool getHasTextureResolutionChanged() { return false; }

    /// Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    virtual bool needsReRender() { return false; }
    /// Called when the camera has moved.
    virtual void onHasMoved() {}
    /// Called when the resolution of the application window has changed.
    virtual void onResolutionChanged() {}

    virtual bool setNewSettings(const SettingsMap& settings) { return false; }

    /// Returns whether the baking process was re-run.
    virtual bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) { return false; }
    virtual void setFileDialogInstance(ImGuiFileDialog* _fileDialogInstance) {}

protected:
    sgl::vk::Renderer* rendererMain;
    BakingMode bakingMode = BakingMode::ITERATIVE_UPDATE;
};

typedef std::shared_ptr<AmbientOcclusionBaker> AmbientOcclusionBakerPtr;

#endif //LINEVIS_AMBIENTOCCLUSIONBAKER_HPP
