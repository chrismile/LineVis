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

#include <memory>
#include <utility>
#include <glm/vec3.hpp>

#include <Math/Math.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Graphics/Texture/Bitmap.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/RayTracingPipeline.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/Passes/BlitRenderPass.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>
#include <ImGui/ImGuiFileDialog/ImGuiFileDialog.h>
#include <ImGui/imgui_stdlib.h>

#include "../Denoiser/EAWDenoiser.hpp"
#ifdef SUPPORT_OPTIX
#include "../Denoiser/OptixVptDenoiser.hpp"
#endif

#ifdef SUPPORT_OPENEXR
#include "OpenExrLoader.hpp"
#endif

#include "Renderers/OIT/MBOITUtils.hpp"
#include "LineData/Scattering/CloudData.hpp"
#include "SuperVoxelGrid.hpp"
#include "VolumetricPathTracingPass.hpp"

VolumetricPathTracingPass::VolumetricPathTracingPass(sgl::vk::Renderer* renderer, sgl::CameraPtr* camera)
        : ComputePass(renderer), camera(camera) {
    uniformBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(UniformData),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    frameInfoBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(FrameInfo),
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);
    computeWrappingZoneParameters(momentUniformData.wrapping_zone_parameters);
    momentUniformDataBuffer = std::make_shared<sgl::vk::Buffer>(
            device, sizeof(MomentUniformData), &momentUniformData,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
            VMA_MEMORY_USAGE_GPU_ONLY);

    if (sgl::AppSettings::get()->getSettings().getValueOpt(
            "vptEnvironmentMapImage", environmentMapFilenameGui)) {
        useEnvironmentMapImage = true;
        sgl::AppSettings::get()->getSettings().getValueOpt("vptUseEnvironmentMap", useEnvironmentMapImage);
        loadEnvironmentMapImage();
    }

    blitResultRenderPass = std::make_shared<sgl::vk::BlitRenderPass>(renderer);
    blitPrimaryRayMomentTexturePass = std::make_shared<BlitMomentTexturePass>(renderer, "Primary");
    blitScatterRayMomentTexturePass = std::make_shared<BlitMomentTexturePass>(renderer, "Scatter");

    createDenoiser();
    updateVptMode();
}

VolumetricPathTracingPass::~VolumetricPathTracingPass() {
    if (isEnvironmentMapLoaded) {
        sgl::AppSettings::get()->getSettings().addKeyValue(
                "vptEnvironmentMapImage", loadedEnvironmentMapFilename);
        sgl::AppSettings::get()->getSettings().addKeyValue(
                "vptUseEnvironmentMap", useEnvironmentMapImage);
    }
}

void VolumetricPathTracingPass::createDenoiser() {
    denoiser = createDenoiserObject(denoiserType, renderer, camera, DenoisingMode::VOLUMETRIC_PATH_TRACING);
    if (denoiser) {
        denoiser->setFileDialogInstance(fileDialogInstance);
    }

    if (resultImageTexture) {
        setDenoiserFeatureMaps();
        if (denoiser) {
            denoiser->recreateSwapchain(lastViewportWidth, lastViewportHeight);
        }
    }
}

void VolumetricPathTracingPass::setOutputImage(sgl::vk::ImageViewPtr& imageView) {
    sceneImageView = imageView;

    sgl::vk::ImageSamplerSettings samplerSettings;
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

    sgl::vk::ImageSettings imageSettings = imageView->getImage()->getImageSettings();
    imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
    imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    resultImageView = std::make_shared<sgl::vk::ImageView>(
            std::make_shared<sgl::vk::Image>(device, imageSettings));
    resultTexture = std::make_shared<sgl::vk::Texture>(
            resultImageView, sgl::vk::ImageSamplerSettings());
    imageSettings.usage =
            VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_STORAGE_BIT
            | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    denoisedImageView = std::make_shared<sgl::vk::ImageView>(
            std::make_shared<sgl::vk::Image>(device, imageSettings));

    resultImageTexture = std::make_shared<sgl::vk::Texture>(resultImageView, samplerSettings);

    imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    accImageTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    firstXTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    firstWTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);

    blitResultRenderPass->setInputTexture(resultTexture);
    blitResultRenderPass->setOutputImage(imageView);
    blitPrimaryRayMomentTexturePass->setOutputImage(imageView);
    blitScatterRayMomentTexturePass->setOutputImage(imageView);

    setDenoiserFeatureMaps();

    frameInfo.frameCount = 0;
    setDataDirty();
}

void VolumetricPathTracingPass::setDenoiserFeatureMaps() {
    if (denoiser) {
        if (denoiser->getUseFeatureMap(FeatureMapType::COLOR)) {
            denoiser->setFeatureMap(FeatureMapType::COLOR, resultImageTexture);
        }
        if (denoiser->getUseFeatureMap(FeatureMapType::POSITION)) {
            denoiser->setFeatureMap(FeatureMapType::POSITION, firstXTexture);
        }
        if (denoiser->getUseFeatureMap(FeatureMapType::NORMAL)) {
            denoiser->setFeatureMap(FeatureMapType::NORMAL, firstWTexture);
        }
        denoiser->setOutputImage(denoisedImageView);

        featureMapUsedArray.resize(IM_ARRAYSIZE(FEATURE_MAP_NAMES));
        for (int i = 0; i < IM_ARRAYSIZE(FEATURE_MAP_NAMES); i++) {
            featureMapUsedArray.at(i) = denoiser->getUseFeatureMap(FeatureMapType(i));
        }
    }
}

void VolumetricPathTracingPass::checkResetDenoiserFeatureMaps() {
    bool shallResetFeatureMaps = false;
    if (denoiser) {
        for (int i = 0; i < IM_ARRAYSIZE(FEATURE_MAP_NAMES); i++) {
            if (denoiser->getUseFeatureMap(FeatureMapType(i)) != featureMapUsedArray.at(i)) {
                shallResetFeatureMaps = true;
            }
        }
    }

    if (shallResetFeatureMaps) {
        setDenoiserFeatureMaps();
        //changedDenoiserSettings = false;
    }
}

void VolumetricPathTracingPass::recreateSwapchain(uint32_t width, uint32_t height) {
    lastViewportWidth = width;
    lastViewportHeight = height;

    blitResultRenderPass->recreateSwapchain(width, height);
    blitScatterRayMomentTexturePass->recreateSwapchain(width, height);
    blitPrimaryRayMomentTexturePass->recreateSwapchain(width, height);

    if (useDenoiser && denoiser) {
        denoiser->recreateSwapchain(width, height);
    }
}

void VolumetricPathTracingPass::setGridData() {
    nanoVdbBuffer = {};
    densityFieldTexture = {};

    if (!cloudData) {
        return;
    }

    if (useSparseGrid) {
        uint8_t* sparseDensityField;
        uint64_t sparseDensityFieldSize;
        cloudData->getSparseDensityField(sparseDensityField, sparseDensityFieldSize);

        uint64_t bufferSize = sizeof(uint32_t) * sgl::iceil(int(sparseDensityFieldSize), sizeof(uint32_t));
        auto* sparseDensityFieldCopy = new uint8_t[bufferSize];
        memset(sparseDensityFieldCopy, 0, bufferSize);
        memcpy(sparseDensityFieldCopy, sparseDensityField, sparseDensityFieldSize);

        nanoVdbBuffer = std::make_shared<sgl::vk::Buffer>(
                device, bufferSize, sparseDensityFieldCopy,
                VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                VMA_MEMORY_USAGE_GPU_ONLY);
        delete[] sparseDensityFieldCopy;
    } else {
        sgl::vk::ImageSettings imageSettings;
        imageSettings.width = cloudData->getGridSizeX();
        imageSettings.height = cloudData->getGridSizeY();
        imageSettings.depth = cloudData->getGridSizeZ();
        imageSettings.imageType = VK_IMAGE_TYPE_3D;
        imageSettings.format = VK_FORMAT_R32_SFLOAT;
        imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;

        sgl::vk::ImageSamplerSettings samplerSettings;
        if (clampToZeroBorder) {
            samplerSettings.addressModeU = samplerSettings.addressModeV = samplerSettings.addressModeW =
                    VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
        } else {
            samplerSettings.addressModeU = samplerSettings.addressModeV = samplerSettings.addressModeW =
                    VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        }
        samplerSettings.borderColor = VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK;
        if (gridInterpolationType == GridInterpolationType::TRILINEAR) {
            samplerSettings.minFilter = VK_FILTER_LINEAR;
            samplerSettings.magFilter = VK_FILTER_LINEAR;
        } else {
            samplerSettings.minFilter = VK_FILTER_NEAREST;
            samplerSettings.magFilter = VK_FILTER_NEAREST;
        }
        sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

        densityFieldTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
        densityFieldTexture->getImage()->uploadData(
                cloudData->getGridSizeX() * cloudData->getGridSizeY() * cloudData->getGridSizeZ() * sizeof(float),
                cloudData->getDenseDensityField());
    }
}

void VolumetricPathTracingPass::updateGridSampler() {
    if (!densityFieldTexture) {
        return;
    }

    sgl::vk::ImageSamplerSettings samplerSettings = densityFieldTexture->getImageSampler()->getImageSamplerSettings();
    if (clampToZeroBorder) {
        samplerSettings.addressModeU = samplerSettings.addressModeV = samplerSettings.addressModeW =
                VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
    } else {
        samplerSettings.addressModeU = samplerSettings.addressModeV = samplerSettings.addressModeW =
                VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    }
    if (gridInterpolationType == GridInterpolationType::TRILINEAR) {
        samplerSettings.minFilter = VK_FILTER_LINEAR;
        samplerSettings.magFilter = VK_FILTER_LINEAR;
    } else {
        samplerSettings.minFilter = VK_FILTER_NEAREST;
        samplerSettings.magFilter = VK_FILTER_NEAREST;
    }
    densityFieldTexture = std::make_shared<sgl::vk::Texture>(
            densityFieldTexture->getImageView(), samplerSettings);
}

void VolumetricPathTracingPass::setCloudData(const CloudDataPtr& data, bool isNewData) {
    cloudData = data;
    frameInfo.frameCount = 0;

    setGridData();
    setDataDirty();
    updateVptMode();
}

void VolumetricPathTracingPass::setVptMode(VptMode vptMode) {
    this->vptMode = vptMode;
    updateVptMode();
    setShaderDirty();
    setDataDirty();
}

void VolumetricPathTracingPass::setUseSparseGrid(bool useSparse) {
    this->useSparseGrid = useSparse;
    setGridData();
    updateVptMode();
    setShaderDirty();
    setDataDirty();
}

void VolumetricPathTracingPass::setSparseGridInterpolationType(GridInterpolationType type) {
    this->gridInterpolationType = type;
    updateGridSampler();
    setShaderDirty();
}

void VolumetricPathTracingPass::setCustomSeedOffset(uint32_t offset) {
    customSeedOffset = offset;
    setShaderDirty();
}

void VolumetricPathTracingPass::setUseLinearRGB(bool useLinearRGB) {
    uniformData.useLinearRGB = useLinearRGB;
    frameInfo.frameCount = 0;
    setShaderDirty();
}

void VolumetricPathTracingPass::setFileDialogInstance(ImGuiFileDialog* _fileDialogInstance) {
    this->fileDialogInstance = _fileDialogInstance;
}

void VolumetricPathTracingPass::onHasMoved() {
    frameInfo.frameCount = 0;
}

void VolumetricPathTracingPass::updateVptMode() {
    if (accumulationTimer && !reachedTarget) {
        createNewAccumulationTimer = true;
    }
    if (vptMode == VptMode::RESIDUAL_RATIO_TRACKING && cloudData && !useSparseGrid) {
        superVoxelGridDecompositionTracking = {};
        superVoxelGridResidualRatioTracking = std::make_shared<SuperVoxelGridResidualRatioTracking>(
                device, cloudData->getGridSizeX(), cloudData->getGridSizeY(),
                cloudData->getGridSizeZ(), cloudData->getDenseDensityField(),
                superVoxelSize, clampToZeroBorder, gridInterpolationType);
        superVoxelGridResidualRatioTracking->setExtinction((cloudExtinctionBase * cloudExtinctionScale).x);
    } else if (vptMode == VptMode::DECOMPOSITION_TRACKING && cloudData && !useSparseGrid) {
        superVoxelGridResidualRatioTracking = {};
        superVoxelGridDecompositionTracking = std::make_shared<SuperVoxelGridDecompositionTracking>(
                device, cloudData->getGridSizeX(), cloudData->getGridSizeY(),
                cloudData->getGridSizeZ(), cloudData->getDenseDensityField(),
                superVoxelSize, clampToZeroBorder, gridInterpolationType);
    } else {
        superVoxelGridResidualRatioTracking = {};
        superVoxelGridDecompositionTracking = {};
    }
}

void VolumetricPathTracingPass::loadEnvironmentMapImage() {
    if (!sgl::FileUtils::get()->exists(environmentMapFilenameGui)) {
        sgl::Logfile::get()->writeError(
                "Error in VolumetricPathTracingPass::loadEnvironmentMapImage: The file \""
                + environmentMapFilenameGui + "\" does not exist.");
        return;
    }

    bool newEnvMapImageUsesLinearRgb = true;
    sgl::BitmapPtr bitmap;
#ifdef SUPPORT_OPENEXR
    OpenExrImageInfo imageInfo;
#endif
    if (sgl::FileUtils::get()->hasExtension(environmentMapFilenameGui.c_str(), ".png")) {
        bitmap = std::make_shared<sgl::Bitmap>();
        bitmap->fromFile(environmentMapFilenameGui.c_str());
        newEnvMapImageUsesLinearRgb = false; // Assume by default that .png images store sRGB.
    }
#ifdef SUPPORT_OPENEXR
    else if (sgl::FileUtils::get()->hasExtension(environmentMapFilenameGui.c_str(), ".exr")) {
        bool isLoaded = loadOpenExrImageFile(environmentMapFilenameGui, imageInfo);
        if (!isLoaded) {
            sgl::Logfile::get()->writeError(
                    "Error in VolumetricPathTracingPass::loadEnvironmentMapImage: The file \""
                    + environmentMapFilenameGui + "\" couldn't be opened using OpenEXR.");
            return;
        }
    }
#endif
    else {
        sgl::Logfile::get()->writeError(
                "Error in VolumetricPathTracingPass::loadEnvironmentMapImage: The file \""
                + environmentMapFilenameGui + "\" has an unknown file extension.");
        return;
    }

    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

    sgl::vk::ImageSettings imageSettings;
    imageSettings.imageType = VK_IMAGE_TYPE_2D;
    imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;

    sgl::vk::ImageSamplerSettings samplerSettings;
    samplerSettings.addressModeU = samplerSettings.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;

    void* pixelData;
    uint32_t bytesPerPixel;
    uint32_t width;
    uint32_t height;
    if (bitmap) {
        pixelData = bitmap->getPixels();
        bytesPerPixel = bitmap->getBPP() / 8;
        width = uint32_t(bitmap->getWidth());
        height = uint32_t(bitmap->getHeight());
        imageSettings.format = VK_FORMAT_R8G8B8A8_UNORM;
    }
#ifdef SUPPORT_OPENEXR
    else {
        pixelData = imageInfo.pixelData;
        bytesPerPixel = 8; // 4 * half
        width = imageInfo.width;
        height = imageInfo.height;
        imageSettings.format = VK_FORMAT_R16G16B16A16_SFLOAT;
    }
#endif
    imageSettings.width = width;
    imageSettings.height = height;

    environmentMapTexture = std::make_shared<sgl::vk::Texture>(device, imageSettings, samplerSettings);
    environmentMapTexture->getImage()->uploadData(width * height * bytesPerPixel, pixelData);
    loadedEnvironmentMapFilename = environmentMapFilenameGui;
    isEnvironmentMapLoaded = true;
    frameInfo.frameCount = 0;

    if (envMapImageUsesLinearRgb != newEnvMapImageUsesLinearRgb) {
        envMapImageUsesLinearRgb = newEnvMapImageUsesLinearRgb;
        setShaderDirty();
    }

#ifdef SUPPORT_OPENEXR
    if (!bitmap) {
        delete[] imageInfo.pixelData;
    }
#endif
}

void VolumetricPathTracingPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> customPreprocessorDefines;
    if (customSeedOffset != 0) {
        customPreprocessorDefines.insert({ "CUSTOM_SEED_OFFSET", std::to_string(customSeedOffset) });
    }
    if (vptMode == VptMode::DELTA_TRACKING) {
        customPreprocessorDefines.insert({ "USE_DELTA_TRACKING", "" });
    } else if (vptMode == VptMode::SPECTRAL_DELTA_TRACKING) {
        customPreprocessorDefines.insert({ "USE_SPECTRAL_DELTA_TRACKING", "" });
        if (sdtCollisionProbability == SpectralDeltaTrackingCollisionProbability::MAX_BASED) {
            customPreprocessorDefines.insert({ "MAX_BASED_PROBABILITY", "" });
        } else if (sdtCollisionProbability == SpectralDeltaTrackingCollisionProbability::AVG_BASED) {
            customPreprocessorDefines.insert({ "AVG_BASED_PROBABILITY", "" });
        } else { // SpectralDeltaTrackingCollisionProbability::PATH_HISTORY_AVG_BASED
            customPreprocessorDefines.insert({ "PATH_HISTORY_AVG_BASED_PROBABILITY", "" });
        }
    } else if (vptMode == VptMode::RATIO_TRACKING) {
        customPreprocessorDefines.insert({ "USE_RATIO_TRACKING", "" });
    } else if (vptMode == VptMode::RESIDUAL_RATIO_TRACKING) {
        customPreprocessorDefines.insert({ "USE_RESIDUAL_RATIO_TRACKING", "" });
    } else if (vptMode == VptMode::DECOMPOSITION_TRACKING) {
        customPreprocessorDefines.insert({ "USE_DECOMPOSITION_TRACKING", "" });
    }
    if (gridInterpolationType == GridInterpolationType::NEAREST) {
        customPreprocessorDefines.insert({ "GRID_INTERPOLATION_NEAREST", "" });
    } else if (gridInterpolationType == GridInterpolationType::STOCHASTIC) {
        customPreprocessorDefines.insert({ "GRID_INTERPOLATION_STOCHASTIC", "" });
    } else if (gridInterpolationType == GridInterpolationType::TRILINEAR) {
        customPreprocessorDefines.insert({ "GRID_INTERPOLATION_TRILINEAR", "" });
    }
    if (useSparseGrid) {
        customPreprocessorDefines.insert({ "USE_NANOVDB", "" });
    }
    if (blitPrimaryRayMomentTexturePass->getMomentType() != BlitMomentTexturePass::MomentType::NONE) {
        customPreprocessorDefines.insert({ "COMPUTE_PRIMARY_RAY_ABSORPTION_MOMENTS", "" });
        customPreprocessorDefines.insert(
                { "NUM_PRIMARY_RAY_ABSORPTION_MOMENTS",
                  std::to_string(blitPrimaryRayMomentTexturePass->getNumMoments()) });
        if (blitPrimaryRayMomentTexturePass->getMomentType() == BlitMomentTexturePass::MomentType::POWER) {
            customPreprocessorDefines.insert({ "USE_POWER_MOMENTS_PRIMARY_RAY", "" });
        }
    }
    if (blitScatterRayMomentTexturePass->getMomentType() != BlitMomentTexturePass::MomentType::NONE) {
        customPreprocessorDefines.insert({ "COMPUTE_SCATTER_RAY_ABSORPTION_MOMENTS", "" });
        customPreprocessorDefines.insert(
                { "NUM_SCATTER_RAY_ABSORPTION_MOMENTS",
                  std::to_string(blitScatterRayMomentTexturePass->getNumMoments()) });
        if (blitScatterRayMomentTexturePass->getMomentType() == BlitMomentTexturePass::MomentType::POWER) {
            customPreprocessorDefines.insert({ "USE_POWER_MOMENTS_SCATTER_RAY", "" });
        }
    }
    if (useEnvironmentMapImage) {
        customPreprocessorDefines.insert({ "USE_ENVIRONMENT_MAP_IMAGE", "" });
    }
    if (uniformData.useLinearRGB) {
        customPreprocessorDefines.insert({ "USE_LINEAR_RGB", "" });
    }
    if (envMapImageUsesLinearRgb) {
        customPreprocessorDefines.insert({ "ENV_MAP_IMAGE_USES_LINEAR_RGB", "" });
    }

    if (device->getPhysicalDeviceProperties().limits.maxComputeWorkGroupInvocations >= 1024) {
        customPreprocessorDefines.insert({ "LOCAL_SIZE", "32" });
    } else {
        customPreprocessorDefines.insert({ "LOCAL_SIZE", "16" });
    }

    shaderStages = sgl::vk::ShaderManager->getShaderStages({"Clouds.Compute"}, customPreprocessorDefines);
}

void VolumetricPathTracingPass::createComputeData(
        sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& computePipeline) {
    computeData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
    computeData->setStaticImageView(resultImageView, "resultImage");
    if (useSparseGrid) {
        computeData->setStaticBuffer(nanoVdbBuffer, "NanoVdbBuffer");
    } else {
        computeData->setStaticTexture(densityFieldTexture, "gridImage");
        if (vptMode == VptMode::RESIDUAL_RATIO_TRACKING) {
            computeData->setStaticTexture(
                    superVoxelGridResidualRatioTracking->getSuperVoxelGridTexture(),
                    "superVoxelGridImage");
            computeData->setStaticTexture(
                    superVoxelGridResidualRatioTracking->getSuperVoxelGridOccupancyTexture(),
                    "superVoxelGridOccupancyImage");
        } else if (vptMode == VptMode::DECOMPOSITION_TRACKING) {
            computeData->setStaticTexture(
                    superVoxelGridDecompositionTracking->getSuperVoxelGridTexture(),
                    "superVoxelGridImage");
            computeData->setStaticTexture(
                    superVoxelGridDecompositionTracking->getSuperVoxelGridOccupancyTexture(),
                    "superVoxelGridOccupancyImage");
        }
    }
    computeData->setStaticBuffer(uniformBuffer, "Parameters");
    computeData->setStaticBuffer(frameInfoBuffer, "FrameInfo");
    computeData->setStaticImageView(accImageTexture->getImageView(), "accImage");
    computeData->setStaticImageView(firstXTexture->getImageView(), "firstX");
    computeData->setStaticImageView(firstWTexture->getImageView(), "firstW");
    if (useEnvironmentMapImage) {
        computeData->setStaticTexture(environmentMapTexture, "environmentMapTexture");
    }
    if (blitPrimaryRayMomentTexturePass->getMomentType() != BlitMomentTexturePass::MomentType::NONE) {
        computeData->setStaticImageView(
                blitPrimaryRayMomentTexturePass->getMomentTexture()->getImageView(),
                "primaryRayAbsorptionMomentsImage");
    }
    if (blitScatterRayMomentTexturePass->getMomentType() != BlitMomentTexturePass::MomentType::NONE) {
        computeData->setStaticImageView(
                blitScatterRayMomentTexturePass->getMomentTexture()->getImageView(),
                "scatterRayAbsorptionMomentsImage");
    }
    computeData->setStaticBuffer(momentUniformDataBuffer, "MomentUniformData");
}

std::string VolumetricPathTracingPass::getCurrentEventName() {
    return std::string() + VPT_MODE_NAMES[int(vptMode)] + " " + std::to_string(targetNumSamples) + "spp";
}

void VolumetricPathTracingPass::_render() {
    if (denoiserChanged) {
        createDenoiser();
        denoiserChanged = false;
    }

    std::string eventName = getCurrentEventName();
    if (createNewAccumulationTimer) {
        accumulationTimer = {};
        accumulationTimer = std::make_shared<sgl::vk::Timer>(renderer);
        createNewAccumulationTimer = false;
    }

    if (!reachedTarget) {
        if (int(frameInfo.frameCount) > targetNumSamples) {
            frameInfo.frameCount = 0;
        }
        if (int(frameInfo.frameCount) < targetNumSamples) {
            reRender = true;
        }
        if (int(frameInfo.frameCount) == targetNumSamples) {
            reachedTarget = true;
            accumulationTimer->finishGPU();
            accumulationTimer->printTimeMS(eventName);
            timerStopped = true;
        }
    }

    if (!reachedTarget) {
        accumulationTimer->startGPU(eventName);
    }

    if (!changedDenoiserSettings && !timerStopped) {
        uniformData.inverseViewProjMatrix = glm::inverse(
                (*camera)->getProjectionMatrix() * (*camera)->getViewMatrix());
        uniformData.boxMin = cloudData->getWorldSpaceBoxMin();
        uniformData.boxMax = cloudData->getWorldSpaceBoxMax();
        uniformData.extinction = cloudExtinctionBase * cloudExtinctionScale;
        uniformData.scatteringAlbedo = cloudScatteringAlbedo;
        uniformData.sunDirection = sunlightDirection;
        uniformData.sunIntensity = sunlightIntensity * sunlightColor;
        uniformData.environmentMapIntensityFactor = environmentMapIntensityFactor;
        if (useSparseGrid) {
            if (cloudData->getGridSizeX() >= 8 && cloudData->getGridSizeY() >= 8 && cloudData->getGridSizeZ() >= 8) {
                uniformData.superVoxelSize = glm::ivec3(8);
            } else {
                uniformData.superVoxelSize = glm::ivec3(1);
            }
        } else if (superVoxelGridResidualRatioTracking) {
            uniformData.superVoxelSize = superVoxelGridResidualRatioTracking->getSuperVoxelSize();
            uniformData.superVoxelGridSize = superVoxelGridResidualRatioTracking->getSuperVoxelGridSize();
        } else if (superVoxelGridDecompositionTracking) {
            uniformData.superVoxelSize = superVoxelGridDecompositionTracking->getSuperVoxelSize();
            uniformData.superVoxelGridSize = superVoxelGridDecompositionTracking->getSuperVoxelGridSize();
        }
        uniformBuffer->updateData(
                sizeof(UniformData), &uniformData, renderer->getVkCommandBuffer());

        frameInfoBuffer->updateData(
                sizeof(FrameInfo), &frameInfo, renderer->getVkCommandBuffer());
        frameInfo.frameCount++;

        renderer->insertMemoryBarrier(
                VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
                VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT);

        renderer->transitionImageLayout(resultImageView->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        if (!useSparseGrid) {
            renderer->transitionImageLayout(
                    densityFieldTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
        }
        renderer->transitionImageLayout(accImageTexture->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        renderer->transitionImageLayout(firstXTexture->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        renderer->transitionImageLayout(firstWTexture->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        renderer->transitionImageLayout(
                blitPrimaryRayMomentTexturePass->getMomentTexture()->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        renderer->transitionImageLayout(
                blitScatterRayMomentTexturePass->getMomentTexture()->getImage(), VK_IMAGE_LAYOUT_GENERAL);
        auto& imageSettings = resultImageView->getImage()->getImageSettings();
        renderer->dispatch(
                computeData,
                sgl::iceil(int(imageSettings.width), blockSize2D.x),
                sgl::iceil(int(imageSettings.height), blockSize2D.y),
                1);
    }
    changedDenoiserSettings = false;
    timerStopped = false;

    if (featureMapType == FeatureMapTypeVpt::RESULT) {
        if (useDenoiser && denoiser && denoiser->getIsEnabled()) {
            denoiser->denoise();
            renderer->transitionImageLayout(
                    denoisedImageView->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            renderer->transitionImageLayout(
                    sceneImageView->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
            denoisedImageView->getImage()->blit(
                    sceneImageView->getImage(), renderer->getVkCommandBuffer());
        } else {
            /*renderer->transitionImageLayout(
                    resultImageView->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            blitResultRenderPass->render();*/
            renderer->transitionImageLayout(
                    resultImageView->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
            renderer->transitionImageLayout(
                    sceneImageView->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
            resultImageView->getImage()->blit(
                    sceneImageView->getImage(), renderer->getVkCommandBuffer());
        }
    } else if (featureMapType == FeatureMapTypeVpt::FIRST_X) {
        renderer->transitionImageLayout(firstXTexture->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        renderer->transitionImageLayout(sceneImageView->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        firstXTexture->getImage()->blit(sceneImageView->getImage(), renderer->getVkCommandBuffer());
    } else if (featureMapType == FeatureMapTypeVpt::FIRST_W) {
        renderer->transitionImageLayout(firstWTexture->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        renderer->transitionImageLayout(sceneImageView->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        firstWTexture->getImage()->blit(sceneImageView->getImage(), renderer->getVkCommandBuffer());
    } else if (featureMapType == FeatureMapTypeVpt::PRIMARY_RAY_ABSORPTION_MOMENTS) {
        blitPrimaryRayMomentTexturePass->render();
    } else if (featureMapType == FeatureMapTypeVpt::SCATTER_RAY_ABSORPTION_MOMENTS) {
        blitScatterRayMomentTexturePass->render();
    }

    if (!reachedTarget) {
        accumulationTimer->endGPU(eventName);
    }
}

bool VolumetricPathTracingPass::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool optionChanged = false;

    if (IGFD_DisplayDialog(
            fileDialogInstance,
            "ChooseEnvironmentMapImage", ImGuiWindowFlags_NoCollapse,
            sgl::ImGuiWrapper::get()->getScaleDependentSize(1000, 580),
            ImVec2(FLT_MAX, FLT_MAX))) {
        if (IGFD_IsOk(fileDialogInstance)) {
            std::string filePathName = IGFD_GetFilePathNameString(fileDialogInstance);
            std::string filePath = IGFD_GetCurrentPathString(fileDialogInstance);
            std::string filter = IGFD_GetCurrentFilterString(fileDialogInstance);
            std::string userDatas;
            if (IGFD_GetUserDatas(fileDialogInstance)) {
                userDatas = std::string((const char*)IGFD_GetUserDatas(fileDialogInstance));
            }
            auto selection = IGFD_GetSelection(fileDialogInstance);

            // Is this line data set or a volume data file for the scattering line tracer?
            std::string currentPath = IGFD_GetCurrentPathString(fileDialogInstance);
            std::string filename = currentPath;
            if (!filename.empty() && filename.back() != '/' && filename.back() != '\\') {
                filename += "/";
            }
            filename += selection.table[0].fileName;
            IGFD_Selection_DestroyContent(&selection);

            environmentMapFilenameGui = filename;
            loadEnvironmentMapImage();
            setShaderDirty();
            reRender = true;
        }
        IGFD_CloseDialog(fileDialogInstance);
    }

    if (propertyEditor.beginNode("VPT Renderer")) {
        std::string numSamplesText = "#Samples: " + std::to_string(frameInfo.frameCount) + "###numSamplesText";
        propertyEditor.addCustomWidgets(numSamplesText);
        if (ImGui::Button(" = ")) {
            createNewAccumulationTimer = true;
            reachedTarget = false;
            reRender = true;

            if (int(frameInfo.frameCount) >= targetNumSamples) {
                frameInfo.frameCount = 0;
            }
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(sgl::ImGuiWrapper::get()->getScaleDependentSize(220.0f));
        ImGui::InputInt("##targetNumSamples", &targetNumSamples);
        if (propertyEditor.addColorEdit3("Sunlight Color", &sunlightColor.x)) {
            optionChanged = true;
        }
        if (propertyEditor.addSliderFloat("Sunlight Intensity", &sunlightIntensity, 0.0f, 10.0f)) {
            optionChanged = true;
        }
        if (propertyEditor.addSliderFloat3("Sunlight Direction", &sunlightDirection.x, 0.0f, 1.0f)) {
            optionChanged = true;
        }
        if (propertyEditor.addSliderFloat("Extinction Scale", &cloudExtinctionScale, 1.0f, 2048.0f)) {
            optionChanged = true;
        }
        if (propertyEditor.addSliderFloat3("Extinction Base", &cloudExtinctionBase.x, 0.01f, 1.0f)) {
            optionChanged = true;
        }
        if (propertyEditor.addColorEdit3("Scattering Albedo", &cloudScatteringAlbedo.x)) {
            optionChanged = true;
        }
        if (propertyEditor.addSliderFloat("G", &uniformData.G, 0.0f, 1.0f)) {
            optionChanged = true;
        }
        if (propertyEditor.addCombo(
                "Feature Map", (int*)&featureMapType, VPT_FEATURE_MAP_NAMES,
                IM_ARRAYSIZE(VPT_FEATURE_MAP_NAMES))) {
            optionChanged = true;
            blitPrimaryRayMomentTexturePass->setVisualizeMomentTexture(
                    featureMapType == FeatureMapTypeVpt::PRIMARY_RAY_ABSORPTION_MOMENTS);
            blitScatterRayMomentTexturePass->setVisualizeMomentTexture(
                    featureMapType == FeatureMapTypeVpt::SCATTER_RAY_ABSORPTION_MOMENTS);
        }
        if (propertyEditor.addCombo(
                "VPT Mode", (int*)&vptMode, VPT_MODE_NAMES,
                IM_ARRAYSIZE(VPT_MODE_NAMES) - 1)) {
            optionChanged = true;
            updateVptMode();
            setShaderDirty();
            setDataDirty();
        }

        if (vptMode == VptMode::SPECTRAL_DELTA_TRACKING) {
            if (propertyEditor.addCombo(
                    "Collision Probability", (int*)&sdtCollisionProbability,
                    SPECTRAL_DELTA_TRACKING_COLLISION_PROBABILITY_NAMES,
                    IM_ARRAYSIZE(SPECTRAL_DELTA_TRACKING_COLLISION_PROBABILITY_NAMES))) {
                optionChanged = true;
                setShaderDirty();
            }
        }

        if (vptMode == VptMode::RESIDUAL_RATIO_TRACKING || vptMode == VptMode::DECOMPOSITION_TRACKING) {
            if (propertyEditor.addSliderInt("Super Voxel Size", &superVoxelSize, 1, 64)) {
                optionChanged = true;
                updateVptMode();
                setShaderDirty();
                setDataDirty();
            }
        }

        if (propertyEditor.addCheckbox("Use Sparse Grid", &useSparseGrid)) {
            optionChanged = true;
            setGridData();
            updateVptMode();
            setShaderDirty();
            setDataDirty();
        }

        if (propertyEditor.addCombo(
                "Grid Interpolation", (int*)&gridInterpolationType,
                GRID_INTERPOLATION_TYPE_NAMES, IM_ARRAYSIZE(GRID_INTERPOLATION_TYPE_NAMES))) {
            optionChanged = true;
            if (vptMode == VptMode::RESIDUAL_RATIO_TRACKING || vptMode == VptMode::DECOMPOSITION_TRACKING) {
                updateVptMode();
            }
            updateGridSampler();
            setShaderDirty();
        }

        propertyEditor.addInputAction("Environment Map", &environmentMapFilenameGui);
        if (propertyEditor.addButton("", "Load")) {
            loadEnvironmentMapImage();
            setShaderDirty();
            reRender = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Open from Disk...")) {
            IGFD_OpenModal(
                    fileDialogInstance,
                    "ChooseEnvironmentMapImage", "Choose an Environment Map Image",
                    ".*,.png,.exr",
                    sgl::AppSettings::get()->getDataDirectory().c_str(),
                    "", 1, nullptr,
                    ImGuiFileDialogFlags_None);
        }

        if (isEnvironmentMapLoaded && propertyEditor.addCheckbox(
                "Use Env. Map Image", &useEnvironmentMapImage)) {
            setShaderDirty();
            reRender = true;
            frameInfo.frameCount = 0;
        }

        if (useEnvironmentMapImage && propertyEditor.addSliderFloat(
                "Env. Map Intensity", &environmentMapIntensityFactor, 0.0f, 5.0f)) {
            reRender = true;
            frameInfo.frameCount = 0;
        }

        if (useEnvironmentMapImage && propertyEditor.addCheckbox(
                "Env. Map Linear RGB", &envMapImageUsesLinearRgb)) {
            reRender = true;
            frameInfo.frameCount = 0;
            setShaderDirty();
        }

        bool shallRecreateMomentTextureA = false;
        bool momentTypeChangedA = false;
        bool shallRecreateMomentTextureB = false;
        bool momentTypeChangedB = false;
        optionChanged =
                blitPrimaryRayMomentTexturePass->renderGuiPropertyEditorNodes(
                        propertyEditor, shallRecreateMomentTextureA, momentTypeChangedA) || optionChanged;
        optionChanged =
                blitScatterRayMomentTexturePass->renderGuiPropertyEditorNodes(
                        propertyEditor, shallRecreateMomentTextureB, momentTypeChangedB) || optionChanged;
        if (shallRecreateMomentTextureA || shallRecreateMomentTextureB) {
            setShaderDirty();
            setDataDirty();
        }
        if (momentTypeChangedA || momentTypeChangedB) {
            setShaderDirty();
        }

        propertyEditor.endNode();
    }

    int numDenoisersSupported = IM_ARRAYSIZE(DENOISER_NAMES);
#ifdef SUPPORT_OPTIX
    if (!OptixVptDenoiser::isOptixEnabled()) {
        numDenoisersSupported--;
    }
#endif
    if (propertyEditor.addCombo(
            "Denoiser", (int*)&denoiserType, DENOISER_NAMES, numDenoisersSupported)) {
        denoiserChanged = true;
        reRender = true;
        changedDenoiserSettings = true;
    }

    if (useDenoiser && denoiser) {
        if (propertyEditor.beginNode(denoiser->getDenoiserName())) {
            bool denoiserReRender = denoiser->renderGuiPropertyEditorNodes(propertyEditor);
            reRender = denoiserReRender || reRender;
            changedDenoiserSettings = denoiserReRender || changedDenoiserSettings;
            if (denoiserReRender) {
                checkResetDenoiserFeatureMaps();
            }
            propertyEditor.endNode();
        }
    }

    if (optionChanged) {
        frameInfo.frameCount = 0;
        reRender = true;
    }

    return optionChanged;
}



BlitMomentTexturePass::BlitMomentTexturePass(sgl::vk::Renderer* renderer, std::string prefix)
        : BlitRenderPass(renderer, {"BlitMomentTexture.Vertex", "BlitMomentTexture.Fragment"}),
          prefix(std::move(prefix)) {
}

// Public interface.
void BlitMomentTexturePass::setOutputImage(sgl::vk::ImageViewPtr& colorImage) {
    BlitRenderPass::setOutputImage(colorImage);
    recreateMomentTexture();
}

void BlitMomentTexturePass::setVisualizeMomentTexture(bool visualizeMomentTexture) {
    this->visualizeMomentTexture = visualizeMomentTexture;
}

void BlitMomentTexturePass::recreateMomentTexture() {
    sgl::vk::ImageSamplerSettings samplerSettings;
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

    sgl::vk::ImageSettings imageSettings = outputImageViews.front()->getImage()->getImageSettings();
    imageSettings.format = VK_FORMAT_R32_SFLOAT;
    imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    imageSettings.arrayLayers = numMoments + 1;
    momentTexture = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, VK_IMAGE_VIEW_TYPE_2D_ARRAY, samplerSettings);
    BlitRenderPass::setInputTexture(momentTexture);
}

bool BlitMomentTexturePass::renderGuiPropertyEditorNodes(
        sgl::PropertyEditor& propertyEditor, bool& shallRecreateMomentTexture, bool& momentTypeChanged) {
    bool reRender = false;
    shallRecreateMomentTexture = false;
    momentTypeChanged = false;

    std::string headerName = prefix;
    if (propertyEditor.beginNode(headerName)) {
        std::string momentTypeName = "Moment Type## (" + prefix + ")";
        if (propertyEditor.addCombo(
                momentTypeName, (int*)&momentType, MOMENT_TYPE_NAMES,
                IM_ARRAYSIZE(MOMENT_TYPE_NAMES))) {
            reRender = true;
            momentTypeChanged = true;
        }
        std::string numMomentName = "#Moments## (" + prefix + ")";
        if (propertyEditor.addCombo(
                numMomentName, &numMomentsIdx, NUM_MOMENTS_NAMES,
                IM_ARRAYSIZE(NUM_MOMENTS_NAMES))) {
            numMoments = NUM_MOMENTS_SUPPORTED[numMomentsIdx];
            reRender = true;
            shallRecreateMomentTexture = true;
        }

        if (visualizeMomentTexture) {
            if (propertyEditor.addSliderInt("Visualized Moment", &selectedMomentBlitIdx, 0, numMoments)) {
                reRender = true;
            }
        }

        propertyEditor.endNode();
    }

    if (shallRecreateMomentTexture) {
        selectedMomentBlitIdx = std::min(selectedMomentBlitIdx, numMoments);
        recreateMomentTexture();
    }

    return reRender;
}

void BlitMomentTexturePass::createRasterData(sgl::vk::Renderer* renderer, sgl::vk::GraphicsPipelinePtr& graphicsPipeline) {
    BlitRenderPass::createRasterData(renderer, graphicsPipeline);
}

void BlitMomentTexturePass::_render() {
    renderer->transitionImageLayout(momentTexture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    renderer->pushConstants(
            rasterData->getGraphicsPipeline(), VK_SHADER_STAGE_FRAGMENT_BIT,
            0, selectedMomentBlitIdx);
    BlitRenderPass::_render();
    renderer->transitionImageLayout(momentTexture->getImage(), VK_IMAGE_LAYOUT_GENERAL);
}
