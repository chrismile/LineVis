/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022-2023, Felix Brendel, Christoph Neuhauser
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

#include <Math/Math.hpp>

#include <Utils/Defer.hpp>
#include <Graphics/Scene/Camera.hpp>
//#include <corecrt_math.h>
#include <stdio.h>
#include "Graphics/Vulkan/Utils/Device.hpp"
#include "Graphics/Vulkan/Render/Renderer.hpp"
#include <Graphics/Vulkan/Render/Data.hpp>
#include <Graphics/Vulkan/Render/ComputePipeline.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>
#include "SpatialHashingDenoiser.hpp"

const size_t hm_cells = 10'000'000u;

Spatial_Hashing_Denoiser::Spatial_Hashing_Denoiser(sgl::vk::Renderer* renderer, sgl::CameraPtr* camera) {
    this->renderer = renderer;
    this->camera   = camera;

    auto device = renderer->getDevice();
    textures.vk_uniform_buffer = std::make_shared<sgl::vk::Buffer>(
        device, sizeof(Spatial_Hashing_Uniform_Buffer),
        VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VMA_MEMORY_USAGE_GPU_ONLY);

    struct HM_Cell {
        float ao_value;
        int contribution_counter;
        int checksum;
        int rc;
        // float s_wd;
        // glm::vec3 padding;
    };
    const size_t hm_buffer_size = sizeof(HM_Cell) * hm_cells;

    printf("Spatial Hashmap Buffer size: %f MB\n\n", hm_buffer_size/1024.0f/1024.0f);
    fflush(stdout);
    
    textures.vk_hash_map_buffer = std::make_shared<sgl::vk::Buffer>(
        device, hm_buffer_size,
        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VMA_MEMORY_USAGE_GPU_ONLY);

    {
        auto cmd_buff = device->beginSingleTimeCommands();
        defer { device->endSingleTimeCommands(cmd_buff); };

        textures.vk_hash_map_buffer->fill(0, cmd_buff);
    }


    write_pass = std::make_shared<Spatial_Hashing_Write_Pass>(renderer, &textures);
    read_pass  = std::make_shared<Spatial_Hashing_Read_Pass>(renderer, &textures);
    eaw_pass   = std::make_shared<EAWBlitPass>(renderer, 3);

    auto eaw_settings = eaw_pass->get_settings();
    eaw_settings.useColorWeights = false;
    eaw_settings.phiPositionScale = 0.000001f;
    eaw_pass->set_settings(eaw_settings);

    textures.uniform_buffer.s_nd    = 3.0f;      // NOTE(Felix): same as Christiane's
    textures.uniform_buffer.s_p     = 8;
    textures.uniform_buffer.show_grid = glm::vec4(0);
    textures.uniform_buffer.s_min = powf(10.0f, (float)s_min_exp);

    if (renderer->getDevice()->getPhysicalDeviceShaderAtomicFloatFeatures().shaderBufferFloat32AtomicAdd) {
        spatialHashingAtomicsMode = SpatialHashingAtomicsMode::FLOAT_ATOMIC_ADD;
    } else {
        spatialHashingAtomicsMode = SpatialHashingAtomicsMode::UINT_QUANTIZED_ATOMIC_ADD;
    }
    read_pass->setAtomicsMode(spatialHashingAtomicsMode);
    write_pass->setAtomicsMode(spatialHashingAtomicsMode);
}


void Spatial_Hashing_Denoiser::denoise() {
    textures.uniform_buffer.cam_pos = glm::vec4((*camera)->getPosition(), 0);
    textures.uniform_buffer.f       = (*camera)->getFOVy();

    textures.vk_uniform_buffer->updateData(
                sizeof(Spatial_Hashing_Uniform_Buffer),
                &textures.uniform_buffer, renderer->getVkCommandBuffer());

    renderer->insertBufferMemoryBarrier(
        VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_UNIFORM_READ_BIT,
        VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        textures.vk_uniform_buffer);

    write_pass->render();

    renderer->insertBufferMemoryBarrier(
        VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_SHADER_READ_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        textures.vk_hash_map_buffer);

    read_pass->render();

    auto dest_stage = eaw_pass->getUseComputeShader() ? VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT : VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    renderer->insertImageMemoryBarrier(textures.temp_accum_texture->getImage(),
                                       VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                       VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, dest_stage,
                                       VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT);

    eaw_pass->render();
}

bool Spatial_Hashing_Denoiser::getIsEnabled() const { return true; };

void Spatial_Hashing_Denoiser::setOutputImage(sgl::vk::ImageViewPtr& outputImage) {
    eaw_pass->setOutputImage(outputImage);
}

bool Spatial_Hashing_Denoiser::getUseFeatureMap(FeatureMapType featureMapType) const {
    return
        featureMapType == FeatureMapType::COLOR          ||
        featureMapType == FeatureMapType::NORMAL_WORLD   ||
        featureMapType == FeatureMapType::POSITION_WORLD;
}

void Spatial_Hashing_Denoiser::setFeatureMap(FeatureMapType featureMapType, const sgl::vk::TexturePtr& featureTexture) {
    switch (featureMapType) {
        case FeatureMapType::COLOR:          {textures.noisy_texture    = featureTexture; } return;
        case FeatureMapType::NORMAL_WORLD:   {textures.normal_texture   = featureTexture; eaw_pass->setNormalTexture(featureTexture);   } return;
        case FeatureMapType::POSITION_WORLD: {textures.position_texture = featureTexture; eaw_pass->setPositionTexture(featureTexture); } return;
        default: return;
    }
}

void Spatial_Hashing_Denoiser::recreateSwapchain(uint32_t width, uint32_t height) {
    auto device = renderer->getDevice();
    resolution = glm::ivec2((int)width, (int)height);

    sgl::vk::ImageSettings imageSettings;
    imageSettings.width  = width;
    imageSettings.height = height;
    imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
    imageSettings.usage =
        VK_IMAGE_USAGE_SAMPLED_BIT |
        VK_IMAGE_USAGE_STORAGE_BIT |
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
        VK_IMAGE_USAGE_TRANSFER_DST_BIT |
        VK_IMAGE_USAGE_TRANSFER_SRC_BIT;


    textures.temp_accum_texture = {};
    textures.temp_accum_texture =
        std::make_shared<sgl::vk::Texture>(device, imageSettings);

    eaw_pass->setColorTexture(textures.temp_accum_texture);
    eaw_pass->recreateSwapchain(width, height);
}

bool Spatial_Hashing_Denoiser::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool redraw = false;
    redraw |= eaw_pass->renderGuiPropertyEditorNodes(propertyEditor);
    redraw |= propertyEditor.addSliderFloat("textures.uniform_buffer.s_p", &textures.uniform_buffer.s_p, 0.1f, 20.0f, "%.10f");
    redraw |= propertyEditor.addSliderInt("s_min_exp", &s_min_exp, -20, -1);
    redraw |=propertyEditor.addSliderFloat("s_nd", &textures.uniform_buffer.s_nd, 1, 10);
    redraw |= propertyEditor.addSliderFloat("Show Grid", &textures.uniform_buffer.show_grid.x, 0, 1);

    int offset =
            renderer->getDevice()->getPhysicalDeviceShaderAtomicFloatFeatures().shaderBufferFloat32AtomicAdd ? 0 : 1;
    if (propertyEditor.addCombo(
            "Atomics Mode", (int*)&spatialHashingAtomicsMode,
            SPATIAL_HASHING_ATOMICS_MODE_NAMES + offset, IM_ARRAYSIZE(SPATIAL_HASHING_ATOMICS_MODE_NAMES) - offset)) {
        redraw = true;
        read_pass->setAtomicsMode(spatialHashingAtomicsMode);
        write_pass->setAtomicsMode(spatialHashingAtomicsMode);
    }

    if (redraw) {
        // auto device = renderer->getDevice();
        // auto cmd_buff = device->beginSingleTimeCommands();
        // defer { device->endSingleTimeCommands(cmd_buff); };
        read_pass->setDataDirty();
        write_pass->setDataDirty();
        eaw_pass->setDataDirty();
        resetFrameNumber();

        textures.uniform_buffer.s_min = powf(10.0f, (float)s_min_exp);
        textures.vk_hash_map_buffer->fill(0, renderer->getVkCommandBuffer());

        setWantsFrameNumberReset();
    }

    return redraw;
}


// -------------------------------
//        SH WRITE PASS
// -------------------------------
Spatial_Hashing_Write_Pass::Spatial_Hashing_Write_Pass(sgl::vk::Renderer* renderer, Spatial_Hashing_Texture_Pack *textures)
    : sgl::vk::ComputePass(renderer), textures(textures) {}

void Spatial_Hashing_Write_Pass::createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& compute_pipeline) {
    compute_data = std::make_shared<sgl::vk::ComputeData>(renderer, compute_pipeline);

    compute_data->setStaticBuffer(textures->vk_uniform_buffer,  "uniform_data_buffer");
    compute_data->setStaticBuffer(textures->vk_hash_map_buffer, "hash_map_buffer");

    compute_data->setStaticTexture(textures->noisy_texture,    "noisy_texture");
    compute_data->setStaticTexture(textures->normal_texture,   "normal_texture");
    compute_data->setStaticTexture(textures->position_texture, "position_texture");
}

void Spatial_Hashing_Write_Pass::_render() {
    sgl::vk::TexturePtr to_read[] {
        textures->noisy_texture,
        textures->normal_texture,
        textures->position_texture,
    };
    for (auto& tex : to_read) {
        renderer->transitionImageLayout(tex->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }

    auto width = int(textures->noisy_texture->getImage()->getImageSettings().width);
    auto height = int(textures->noisy_texture->getImage()->getImageSettings().height);
    int numWorkgroupsX = sgl::iceil(width, computeBlockSize);
    int numWorkgroupsY = sgl::iceil(height, computeBlockSize);
    renderer->dispatch(compute_data, numWorkgroupsX, numWorkgroupsY, 1);
}

void Spatial_Hashing_Write_Pass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;

    preprocessorDefines.insert(std::make_pair("BLOCK_SIZE", std::to_string(computeBlockSize)));
    preprocessorDefines.insert(std::make_pair("HASH_MAP_SIZE", std::to_string(hm_cells)));
    if (spatialHashingAtomicsMode == SpatialHashingAtomicsMode::FLOAT_ATOMIC_ADD) {
        preprocessorDefines.insert(std::make_pair("__extensions", "GL_EXT_shader_atomic_float"));
        preprocessorDefines.insert(std::make_pair("SUPPORT_BUFFER_FLOAT_ATOMIC_ADD", ""));
    } else if (spatialHashingAtomicsMode == SpatialHashingAtomicsMode::UINT_QUANTIZED_ATOMIC_ADD) {
        preprocessorDefines.insert(std::make_pair("USE_QUANTIZED_AO_VALUES", ""));
    } else if (spatialHashingAtomicsMode == SpatialHashingAtomicsMode::NO_ATOMICS) {
        preprocessorDefines.insert(std::make_pair("NO_ATOMICS", ""));
    }

    shaderStages = sgl::vk::ShaderManager->getShaderStages({ "SH_Denoise.Compute-Write" }, preprocessorDefines);
}

// -------------------------------
//        SH READ PASS
// -------------------------------
Spatial_Hashing_Read_Pass::Spatial_Hashing_Read_Pass(sgl::vk::Renderer* renderer, Spatial_Hashing_Texture_Pack *textures)
    : sgl::vk::ComputePass(renderer), textures(textures) {}

void Spatial_Hashing_Read_Pass::createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& compute_pipeline) {
    compute_data = std::make_shared<sgl::vk::ComputeData>(renderer, compute_pipeline);

    compute_data->setStaticBuffer(textures->vk_uniform_buffer,  "uniform_data_buffer");
    compute_data->setStaticBuffer(textures->vk_hash_map_buffer, "hash_map_buffer");

    compute_data->setStaticTexture(textures->position_texture, "position_texture");
    compute_data->setStaticTexture(textures->normal_texture,   "normal_texture");
    compute_data->setStaticImageView(textures->temp_accum_texture->getImageView(), "temp_accum_texture");
}

void Spatial_Hashing_Read_Pass::_render() {
    sgl::vk::TexturePtr to_read[] {
        textures->position_texture,
        textures->normal_texture,
    };
    for (auto& tex : to_read) {
        renderer->transitionImageLayout(tex->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }


    renderer->insertImageMemoryBarrier(textures->temp_accum_texture->getImage(),
                                       VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                                       VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_NONE_KHR, VK_ACCESS_SHADER_WRITE_BIT);

    auto width = int(textures->noisy_texture->getImage()->getImageSettings().width);
    auto height = int(textures->noisy_texture->getImage()->getImageSettings().height);
    int numWorkgroupsX = sgl::iceil(width, computeBlockSize);
    int numWorkgroupsY = sgl::iceil(height, computeBlockSize);
    renderer->dispatch(compute_data, numWorkgroupsX, numWorkgroupsY, 1);
}

void Spatial_Hashing_Read_Pass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();
    std::map<std::string, std::string> preprocessorDefines;

    preprocessorDefines.insert(
        std::make_pair("BLOCK_SIZE", std::to_string(computeBlockSize)));
    preprocessorDefines.insert(
        std::make_pair("HASH_MAP_SIZE", std::to_string(hm_cells)));
    if (spatialHashingAtomicsMode == SpatialHashingAtomicsMode::FLOAT_ATOMIC_ADD) {
        preprocessorDefines.insert(std::make_pair("SUPPORT_BUFFER_FLOAT_ATOMIC_ADD", ""));
    } else if (spatialHashingAtomicsMode == SpatialHashingAtomicsMode::UINT_QUANTIZED_ATOMIC_ADD) {
        preprocessorDefines.insert(std::make_pair("USE_QUANTIZED_AO_VALUES", ""));
    } else if (spatialHashingAtomicsMode == SpatialHashingAtomicsMode::NO_ATOMICS) {
        preprocessorDefines.insert(std::make_pair("NO_ATOMICS", ""));
    }

    shaderStages = sgl::vk::ShaderManager->getShaderStages(
        { "SH_Denoise.Compute-Read" }, preprocessorDefines);
}
