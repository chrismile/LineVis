/**
 * MIT License
 *
 * Copyright (c) 2021, Christoph Neuhauser, Ludwig Leonard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef LINEVIS_VOLUMETRICPATHTRACINGPASS_HPP
#define LINEVIS_VOLUMETRICPATHTRACINGPASS_HPP

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <Graphics/Scene/Camera.hpp>
#include <Graphics/Vulkan/Render/Passes/Pass.hpp>
#include <Graphics/Vulkan/Render/Passes/BlitRenderPass.hpp>
#include <Graphics/Vulkan/Utils/Timer.hpp>
#include "../Denoiser/Denoiser.hpp"

namespace sgl {
class PropertyEditor;
}

class LineDataScattering;
typedef std::shared_ptr<LineDataScattering> LineDataScatteringPtr;

class BlitMomentTexturePass;
class SuperVoxelGridResidualRatioTracking;
class SuperVoxelGridDecompositionTracking;

namespace IGFD {
class FileDialog;
}
typedef IGFD::FileDialog ImGuiFileDialog;

enum class FeatureMapType {
    RESULT, FIRST_X, FIRST_W, PRIMARY_RAY_ABSORPTION_MOMENTS, SCATTER_RAY_ABSORPTION_MOMENTS
};
const char* const FEATURE_MAP_NAMES[] = {
        "Result", "First X", "First W", "Primary Ray Absorption Moments", "Scatter Ray Absorption Moments"
};

class VolumetricPathTracingPass : public sgl::vk::ComputePass {
public:
    explicit VolumetricPathTracingPass(sgl::vk::Renderer* renderer, sgl::CameraPtr* camera);
    ~VolumetricPathTracingPass() override;

    // Public interface.
    void setOutputImage(sgl::vk::ImageViewPtr& colorImage);
    void recreateSwapchain(uint32_t width, uint32_t height) override;
    void setLineData(LineDataScatteringPtr& data, bool isNewData);
    void setUseLinearRGB(bool useLinearRGB);
    void setFileDialogInstance(ImGuiFileDialog* fileDialogInstance);

    // Called when the camera has moved.
    void onHasMoved();
    /// Returns if the data needs to be re-rendered, but the visualization mapping is valid.
    virtual bool needsReRender() { bool tmp = reRender; reRender = false; return tmp; }
    /// Renders the GUI. The "reRender" flag might be set depending on the user's actions.
    virtual bool renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor);

private:
    void loadShader() override;
    void setComputePipelineInfo(sgl::vk::ComputePipelineInfo& pipelineInfo) override {}
    void createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) override;
    void _render() override;

    sgl::CameraPtr* camera;

    bool reRender = true;

    const glm::ivec2 blockSize2D = glm::ivec2(16, 16);
    sgl::vk::ImageViewPtr sceneImageView;
    LineDataScatteringPtr lineData;
    FeatureMapType featureMapType = FeatureMapType::RESULT;

    void updateVptMode();
    enum class VptMode {
        DELTA_TRACKING, SPECTRAL_DELTA_TRACKING, RATIO_TRACKING, RESIDUAL_RATIO_TRACKING, DECOMPOSITION_TRACKING
    };
    const char* const VPT_MODE_NAMES[5] = {
            "Delta Tracking", "Delta Tracking (Spectral)", "Ratio Tracking", "Residual Ratio Tracking",
            "Decomposition Tracking"
    };
    VptMode vptMode = VptMode::DECOMPOSITION_TRACKING;
    std::shared_ptr<SuperVoxelGridResidualRatioTracking> superVoxelGridResidualRatioTracking;
    std::shared_ptr<SuperVoxelGridDecompositionTracking> superVoxelGridDecompositionTracking;
    int superVoxelSize = 8;
    const bool clampToZeroBorder = true; ///< Whether to use a zero valued border for densityFieldTexture.

    uint32_t lastViewportWidth = 0, lastViewportHeight = 0;

    sgl::vk::ImageViewPtr resultImageView;
    sgl::vk::TexturePtr resultImageTexture;
    sgl::vk::TexturePtr resultTexture;
    sgl::vk::ImageViewPtr denoisedImageView;
    sgl::vk::TexturePtr densityFieldTexture;
    sgl::vk::TexturePtr accImageTexture;
    sgl::vk::TexturePtr firstXTexture;
    sgl::vk::TexturePtr firstWTexture;

    std::string getCurrentEventName();
    int targetNumSamples = 1024;
    bool reachedTarget = true;
    bool changedDenoiserSettings = false;
    bool timerStopped = false;
    bool createNewAccumulationTimer = false;
    sgl::vk::TimerPtr accumulationTimer;

    glm::vec3 sunlightColor = glm::vec3(1.0f, 0.961538462f, 0.884615385f);
    float sunlightIntensity = 2.6f;
    glm::vec3 sunlightDirection = glm::normalize(glm::vec3(0.5826f, 0.7660f, 0.2717f));
    float cloudExtinctionScale = 1024.0f;
    glm::vec3 cloudExtinctionBase = glm::vec3(1.0, 1.0, 1.0);
    glm::vec3 cloudScatteringAlbedo = glm::vec3(1.0, 1.0, 1.0);

    // Environment map data.
    void loadEnvironmentMapImage();
    bool isEnvironmentMapLoaded = false;
    bool useEnvironmentMapImage = false;
    std::string environmentMapFilenameGui;
    std::string loadedEnvironmentMapFilename;
    sgl::vk::TexturePtr environmentMapTexture;
    float environmentMapIntensityFactor = 1.5f;
    ImGuiFileDialog* fileDialogInstance = nullptr;

    sgl::vk::BlitRenderPassPtr blitResultRenderPass;
    std::shared_ptr<BlitMomentTexturePass> blitPrimaryRayMomentTexturePass;
    std::shared_ptr<BlitMomentTexturePass> blitScatterRayMomentTexturePass;

    void createDenoiser();
    void setDenoiserFeatureMaps();
    DenoiserType denoiserType = DenoiserType::EAW;
    bool useDenoiser = true;
    bool denoiserChanged = false;
    std::shared_ptr<Denoiser> denoiser;

    // Uniform buffer object storing the camera settings.
    struct UniformData {
        glm::mat4 inverseViewProjMatrix;

        // Cloud properties
        glm::vec3 boxMin; float pad0;
        glm::vec3 boxMax; float pad1;
        glm::vec3 extinction; float pad2;
        glm::vec3 scatteringAlbedo;
        float G = 0.875f;
        glm::vec3 sunDirection; float pad3;
        glm::vec3 sunIntensity;
        float environmentMapIntensityFactor;

        // For decomposition and residual ratio tracking.
        glm::ivec3 superVoxelSize; int pad5;
        glm::ivec3 superVoxelGridSize;

        // Whether to use linear RGB or sRGB.
        int useLinearRGB;
    };
    UniformData uniformData{};
    sgl::vk::BufferPtr uniformBuffer;

    struct FrameInfo {
        uint32_t frameCount;
        glm::uvec3 padding;
    };
    FrameInfo frameInfo{};
    sgl::vk::BufferPtr frameInfoBuffer;

    struct MomentUniformData {
        glm::vec4 wrapping_zone_parameters;
    };
    MomentUniformData momentUniformData{};
    sgl::vk::BufferPtr momentUniformDataBuffer;
};

class BlitMomentTexturePass : public sgl::vk::BlitRenderPass {
public:
    explicit BlitMomentTexturePass(sgl::vk::Renderer* renderer, std::string prefix);

    enum class MomentType {
        POWER, TRIGONOMETRIC
    };

    // Public interface.
    void setOutputImage(sgl::vk::ImageViewPtr& colorImage) override;
    void setVisualizeMomentTexture(bool visualizeMomentTexture);
    [[nodiscard]] inline MomentType getMomentType() const { return momentType; }
    [[nodiscard]] inline int getNumMoments() const { return numMoments; }
    inline sgl::vk::TexturePtr getMomentTexture() { return momentTexture; }

    /// Renders the GUI. Returns whether re-rendering has become necessary due to the user's actions.
    virtual bool renderGuiPropertyEditorNodes(
            sgl::PropertyEditor& propertyEditor, bool& shallRecreateMomentTexture, bool& momentTypeChanged);

private:
    void createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) override;
    void _render() override;
    void recreateMomentTexture();

    const char* const MOMENT_TYPE_NAMES[2] = {
            "Power", "Trigonometric"
    };
    const int NUM_MOMENTS_SUPPORTED[3] = {
            4, 6, 8
    };
    const char* const NUM_MOMENTS_NAMES[3] = {
            "4", "6", "8"
    };

    std::string prefix; ///< What moments - e.g., "primary", "scatter" for primary and scatter ray moments.
    bool visualizeMomentTexture = false;
    MomentType momentType = MomentType::POWER;
    int numMomentsIdx = 2;
    int numMoments = 8;
    int selectedMomentBlitIdx = 0;
    sgl::vk::TexturePtr momentTexture;
};

#endif //LINEVIS_VOLUMETRICPATHTRACINGPASS_HPP
