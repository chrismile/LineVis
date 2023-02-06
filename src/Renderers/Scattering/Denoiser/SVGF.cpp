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

#include <Math/Math.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Buffers/Framebuffer.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Render/Data.hpp>
#include <Graphics/Vulkan/Render/ComputePipeline.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/Widgets/PropertyEditor.hpp>
#include <Graphics/Vulkan/Utils/Device.hpp>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <initializer_list>

/*
  SVGF NEEDS
  ----------
  Current Frame:
    - AO (Color)
    - Motion
    - Normal
    - Depth
    - Mesh ID

  Last Frame:
    - Moment History (intgrated variance)
    - Color History (result of first iteration of last frame)
    - Normal
    - Depth
    - Mesh ID

 */

#include "SVGF.hpp"


SVGFDenoiser::SVGFDenoiser(sgl::vk::Renderer* renderer) {
    this->renderer = renderer;

    svgf_atrous_pass = std::make_shared<SVGF_ATrous_Pass>(renderer, &textures);
    svgf_reproj_pass = std::make_shared<SVGF_Reproj_Pass>(renderer, &textures);
}

bool SVGFDenoiser::getIsEnabled() const {
    return svgf_atrous_pass->getMaxNumIterations() > 0;
}

void SVGFDenoiser::setOutputImage(sgl::vk::ImageViewPtr& outputImage) {
    textures.denoised_image = outputImage;
}

void SVGFDenoiser::setFeatureMap(FeatureMapType featureMapType, const sgl::vk::TexturePtr& feature_map) {
    switch (featureMapType) {
        case FeatureMapType::COLOR:  textures.current_frame.noisy_texture  = feature_map; return;
        case FeatureMapType::NORMAL: textures.current_frame.normal_texture = feature_map; return;
        case FeatureMapType::DEPTH:  textures.current_frame.depth_texture  = feature_map; return;
        case FeatureMapType::FLOW:   textures.current_frame.motion_texture = feature_map; return;
        default: return;
    }
}

bool SVGFDenoiser::getUseFeatureMap(FeatureMapType featureMapType) const {
    return
        featureMapType == FeatureMapType::COLOR  ||
        featureMapType == FeatureMapType::NORMAL ||
        featureMapType == FeatureMapType::FLOW   ||
        featureMapType == FeatureMapType::DEPTH;
}

void SVGFDenoiser::setUseFeatureMap(FeatureMapType featureMapType, bool useFeature) {
    // We use the maps from the paper
}

void SVGFDenoiser::denoise() {
    svgf_reproj_pass->render();
    svgf_atrous_pass->render();

    // update previous frame images
    {
        // normal texture
        renderer->insertImageMemoryBarrier(
            textures.previous_frame.normal_texture->getImage(),
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_WRITE_BIT);

        renderer->insertImageMemoryBarrier(
            textures.current_frame.normal_texture->getImage(),
            textures.current_frame.normal_texture->getImage()->getVkImageLayout(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_READ_BIT);


        textures.current_frame.normal_texture->getImage()->copyToImage(textures.previous_frame.normal_texture->getImage(),
                                                                       VK_IMAGE_ASPECT_COLOR_BIT,
                                                                       renderer->getVkCommandBuffer());

        // renderer->syncWithCpu();

        // depth texture
        renderer->insertImageMemoryBarrier(
            textures.previous_frame.depth_texture->getImage(),
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_WRITE_BIT);

        renderer->insertImageMemoryBarrier(
            textures.current_frame.depth_texture->getImage(),
            textures.current_frame.depth_texture->getImage()->getVkImageLayout(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_READ_BIT);


        textures.current_frame.depth_texture->getImage()->copyToImage(textures.previous_frame.depth_texture->getImage(),
                                                                      VK_IMAGE_ASPECT_COLOR_BIT,
                                                                      renderer->getVkCommandBuffer());

        // renderer->syncWithCpu();
    }

}


void SVGFDenoiser::recreateSwapchain(uint32_t width, uint32_t height) {
    auto device = renderer->getDevice();

    sgl::vk::ImageSettings imageSettings;
    imageSettings.width  = width;
    imageSettings.height = height;
    imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
    imageSettings.usage =
        VK_IMAGE_USAGE_SAMPLED_BIT |
        VK_IMAGE_USAGE_STORAGE_BIT |
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    // temp_accum_image

    {
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


        textures.current_frame.temp_accum_texture = {};
        textures.current_frame.temp_accum_texture =
            std::make_shared<sgl::vk::Texture>(device, imageSettings);

    }

    for (int i = 0; i < 2; i++) {
        textures.pingPongRenderTextures[i] = std::make_shared<sgl::vk::Texture>(device, imageSettings);
    }

    // recreate previous frame images
    {
        // color_history
        textures.previous_frame.color_history_texture = {};
        textures.previous_frame.color_history_texture =
            std::make_shared<sgl::vk::Texture>(
                textures.current_frame.noisy_texture->getImageView()->copy(true, false),
                textures.current_frame.noisy_texture->getImageSampler());

        // moments_histories
        {
            sgl::vk::ImageSettings imageSettings;
            imageSettings.width  = width;
            imageSettings.height = height;
            imageSettings.format = VK_FORMAT_R32G32_SFLOAT;
            imageSettings.usage =
                VK_IMAGE_USAGE_SAMPLED_BIT |
                VK_IMAGE_USAGE_STORAGE_BIT;

            for (int i = 0; i < 2; ++i) {
                textures.previous_frame.moments_history_texture[i] = {};
                textures.previous_frame.moments_history_texture[i] =
                    std::make_shared<sgl::vk::Texture>(device, imageSettings);
            }
        }

        // normal
        sgl::vk::ImageSettings im_settings = textures.current_frame.normal_texture->getImage()->getImageSettings();
        im_settings.usage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;

        textures.previous_frame.normal_texture = {};
        textures.previous_frame.normal_texture = std::make_shared<sgl::vk::Texture>(device, im_settings);

        // depth
        im_settings = textures.current_frame.depth_texture->getImage()->getImageSettings();
        im_settings.usage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;

        textures.previous_frame.depth_texture = {};
        textures.previous_frame.depth_texture = std::make_shared<sgl::vk::Texture>(device, im_settings);
    }

    svgf_atrous_pass->setDataDirty();
    svgf_reproj_pass->setDataDirty();

}

bool SVGFDenoiser::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    return
        svgf_reproj_pass->renderGuiPropertyEditorNodes(propertyEditor) ||
        svgf_atrous_pass->renderGuiPropertyEditorNodes(propertyEditor);
}

void SVGFDenoiser::resetFrameNumber() {}

// ------------------------------
//       À-Trous-Pass
// ------------------------------

SVGF_ATrous_Pass::SVGF_ATrous_Pass(sgl::vk::Renderer* renderer, Texture_Pack* textures)
    : sgl::vk::ComputePass(renderer), textures(textures)
{}


void SVGF_ATrous_Pass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();

    std::map<std::string, std::string> preprocessorDefines;

    preprocessorDefines.insert(std::make_pair("BLOCK_SIZE", std::to_string(computeBlockSize)));
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
        { "SVGF.Compute-ATrous" }, preprocessorDefines);
}

void SVGF_ATrous_Pass::createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& compute_pipeline) {
    for (int i = 0; i < 3; i++) {
        computeDataPingPong[i] = std::make_shared<sgl::vk::ComputeData>(renderer, compute_pipeline);
        computeDataPingPongFinal[i] = std::make_shared<sgl::vk::ComputeData>(renderer, compute_pipeline);

        if (i == 0) {
            computeDataPingPong[i]->setStaticTexture(textures->current_frame.temp_accum_texture, "color_texture");
            computeDataPingPongFinal[i]->setStaticTexture(textures->current_frame.temp_accum_texture, "color_texture");
            computeDataPingPong[i]->setStaticImageView(textures->pingPongRenderTextures[(i + 1) % 2]->getImageView(), "outputImage");
            computeDataPingPongFinal[i]->setStaticImageView(textures->denoised_image, "outputImage");
        } else {
            computeDataPingPong[i]->setStaticTexture(textures->pingPongRenderTextures[i % 2], "color_texture");
            computeDataPingPongFinal[i]->setStaticTexture(textures->pingPongRenderTextures[i % 2], "color_texture");
            computeDataPingPong[i]->setStaticImageView(textures->pingPongRenderTextures[(i + 1) % 2]->getImageView(), "outputImage");
            computeDataPingPongFinal[i]->setStaticImageView(textures->denoised_image, "outputImage");
        }

        computeDataPingPong[i]->setStaticTexture(textures->current_frame.depth_texture, "depth_texture");
        computeDataPingPongFinal[i]->setStaticTexture(textures->current_frame.depth_texture, "depth_texture");

        computeDataPingPong[i]->setStaticTexture(textures->current_frame.normal_texture, "normal_texture");
        computeDataPingPongFinal[i]->setStaticTexture(textures->current_frame.normal_texture, "normal_texture");

        computeDataPingPong[i]->setStaticTexture(textures->current_frame.motion_texture, "motion_texture");
        computeDataPingPongFinal[i]->setStaticTexture(textures->current_frame.motion_texture, "motion_texture");

        // prev frame
        computeDataPingPong[i]->setStaticTexture(textures->previous_frame.color_history_texture, "color_history_texture");
        computeDataPingPongFinal[i]->setStaticTexture(textures->previous_frame.color_history_texture, "color_history_texture");

        // TODO(Felix): warum war moments_history_texture hier pingpong??
        computeDataPingPong[i]->setStaticTexture(textures->previous_frame.moments_history_texture[0], "variance_history_texture");
        computeDataPingPongFinal[i]->setStaticTexture(textures->previous_frame.moments_history_texture[0], "variance_history_texture");

        computeDataPingPong[i]->setStaticTexture(textures->previous_frame.depth_texture, "depth_history_texture");
        computeDataPingPongFinal[i]->setStaticTexture(textures->previous_frame.depth_texture, "depth_history_texture");

        computeDataPingPong[i]->setStaticTexture(textures->previous_frame.normal_texture, "normals_history_texture");
        computeDataPingPongFinal[i]->setStaticTexture(textures->previous_frame.normal_texture, "normals_history_texture");

    }
}

void SVGF_ATrous_Pass::_render() {
    if (maxNumIterations < 1) {
        renderer->transitionImageLayout(textures->current_frame.temp_accum_texture->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        renderer->transitionImageLayout(textures->denoised_image->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        textures->current_frame.temp_accum_texture->getImage()->blit(textures->denoised_image->getImage(), renderer->getVkCommandBuffer());
        // renderer->syncWithCpu();
        return;
    }


    renderer->transitionImageLayout(textures->current_frame.temp_accum_texture->getImage(),  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    renderer->transitionImageLayout(textures->current_frame.depth_texture->getImage(),  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    renderer->transitionImageLayout(textures->current_frame.normal_texture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    renderer->transitionImageLayout(textures->current_frame.motion_texture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    // render_compute
    {
        auto width = int(textures->current_frame.temp_accum_texture->getImage()->getImageSettings().width);
        auto height = int(textures->current_frame.temp_accum_texture->getImage()->getImageSettings().height);

        for (int i = 0; i < maxNumIterations; i++) {
            int computeDataIdx;
            sgl::vk::ComputeDataPtr computeData;
            if (i == 0) {
                computeDataIdx = 0;
            } else {
                computeDataIdx = 2 - (i % 2);
            }
            if (i == maxNumIterations - 1) {
                computeData = computeDataPingPongFinal[computeDataIdx];
            } else {
                computeData = computeDataPingPong[computeDataIdx];
            }

            pc.i = i;
            renderer->pushConstants(
                std::static_pointer_cast<sgl::vk::Pipeline>(computeData->getComputePipeline()),
                VK_SHADER_STAGE_COMPUTE_BIT, 0, pc);

            renderer->transitionImageLayout(
                computeData->getImageView("color_texture")->getImage(),
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            renderer->transitionImageLayout(
                computeData->getImageView("depth_texture")->getImage(),
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            renderer->transitionImageLayout(
                computeData->getImageView("normal_texture")->getImage(),
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            renderer->transitionImageLayout(
                computeData->getImageView("motion_texture")->getImage(),
                VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);


            renderer->insertImageMemoryBarrier(
                computeData->getImageView("outputImage")->getImage(),
                VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_ACCESS_NONE_KHR, VK_ACCESS_SHADER_WRITE_BIT);

            renderer->insertImageMemoryBarrier(
                computeData->getImageView("color_history_texture")->getImage(),
                VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_ACCESS_NONE_KHR, VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT);

            renderer->insertImageMemoryBarrier(
                computeData->getImageView("variance_history_texture")->getImage(),
                VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_ACCESS_NONE_KHR, VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_SHADER_READ_BIT);

            renderer->insertImageMemoryBarrier(
                computeData->getImageView("depth_history_texture")->getImage(),
                VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_ACCESS_NONE_KHR, VK_ACCESS_SHADER_READ_BIT);

            renderer->insertImageMemoryBarrier(
                computeData->getImageView("normals_history_texture")->getImage(),
                VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_ACCESS_NONE_KHR, VK_ACCESS_SHADER_READ_BIT);

            int numWorkgroupsX = sgl::iceil(width, computeBlockSize);
            int numWorkgroupsY = sgl::iceil(height, computeBlockSize);
            renderer->dispatch(computeData, numWorkgroupsX, numWorkgroupsY, 1);

            // renderer->syncWithCpu();
        }
    }
}

bool SVGF_ATrous_Pass::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    if (propertyEditor.addSliderInt("#Iterations", &maxNumIterations, 0, 5)      |
        propertyEditor.addSliderFloat("z multiplier", &pc.z_multiplier, 1, 1000) |
        propertyEditor.addSliderFloat("fwidth_h", &pc.fwidth_h, 0, 1, "%.5f")    |
        propertyEditor.addSliderFloat("nabla_max", &pc.nabla_max, 0, 1, "%.5f"))
    {
        setDataDirty();
        device->waitIdle();
        return true;
    }

    return false;
}


// ------------------------------
//       Reproj-Pass
// ------------------------------

SVGF_Reproj_Pass::SVGF_Reproj_Pass(sgl::vk::Renderer* renderer, Texture_Pack* textures)
    : sgl::vk::ComputePass(renderer), textures(textures)
{}


void SVGF_Reproj_Pass::createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& compute_pipeline) {
    compute_data = std::make_shared<sgl::vk::ComputeData>(renderer, compute_pipeline);

    compute_data->setStaticTexture(textures->current_frame.noisy_texture, "noisy_texture");
    compute_data->setStaticTexture(textures->previous_frame.color_history_texture, "color_history");
    compute_data->setStaticTexture(textures->current_frame.motion_texture, "motion_texture");
    compute_data->setStaticTexture(textures->current_frame.normal_texture, "normal_texture");
    compute_data->setStaticTexture(textures->current_frame.depth_texture, "depth_texture");
    compute_data->setStaticTexture(textures->previous_frame.depth_texture, "depth_history_texture");
    compute_data->setStaticTexture(textures->previous_frame.normal_texture, "normal_history_texture");

    compute_data->setStaticTexture(textures->current_frame.temp_accum_texture, "temp_accum_texture");
}

void SVGF_Reproj_Pass::_render() {
    sgl::vk::TexturePtr to_read[] {
        textures->current_frame.noisy_texture,
        textures->previous_frame.color_history_texture,
        textures->current_frame.motion_texture,
        textures->current_frame.normal_texture,
        textures->current_frame.depth_texture,
        textures->previous_frame.depth_texture,
        textures->previous_frame.normal_texture,
    };
    for (auto& tex : to_read) {
            renderer->transitionImageLayout(tex->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }

    renderer->pushConstants(
        std::static_pointer_cast<sgl::vk::Pipeline>(compute_data->getComputePipeline()),
        VK_SHADER_STAGE_COMPUTE_BIT, 0, pc);

    renderer->insertImageMemoryBarrier(
                compute_data->getImageView("temp_accum_texture")->getImage(),
                VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_ACCESS_NONE_KHR, VK_ACCESS_SHADER_WRITE_BIT);


    auto width = int(textures->current_frame.temp_accum_texture->getImage()->getImageSettings().width);
    auto height = int(textures->current_frame.temp_accum_texture->getImage()->getImageSettings().height);
    int numWorkgroupsX = sgl::iceil(width, computeBlockSize);
    int numWorkgroupsY = sgl::iceil(height, computeBlockSize);
    renderer->dispatch(compute_data, numWorkgroupsX, numWorkgroupsY, 1);
}

void SVGF_Reproj_Pass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();

    std::map<std::string, std::string> preprocessorDefines;

    preprocessorDefines.insert(std::make_pair("BLOCK_SIZE", std::to_string(computeBlockSize)));
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
        { "SVGF.Compute-Reproject" }, preprocessorDefines);
}


bool SVGF_Reproj_Pass::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    if (propertyEditor.addSliderFloat("allowed_normal_dist", &pc.allowed_normal_dist, 0, 2) ||
        propertyEditor.addSliderFloat("allowed_z_dist", &pc.allowed_z_dist, 0, 5))
    {
        setDataDirty();
        device->waitIdle();
        return true;
    }

    return false;
}
