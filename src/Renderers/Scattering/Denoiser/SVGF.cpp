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
    svgfBlitPass = std::make_shared<SVGFBlitPass>(renderer);
}

bool SVGFDenoiser::getIsEnabled() const {
    return svgfBlitPass->getMaxNumIterations() > 0;
}

void SVGFDenoiser::setOutputImage(sgl::vk::ImageViewPtr& outputImage) {
    svgfBlitPass->setOutputImage(outputImage);
}

void SVGFDenoiser::setFeatureMap(FeatureMapType featureMapType, const sgl::vk::TexturePtr& featureTexture) {
    switch (featureMapType) {
        case FeatureMapType::COLOR:  svgfBlitPass->set_color_texture(featureTexture);  return;
        case FeatureMapType::NORMAL: svgfBlitPass->set_normal_texture(featureTexture); return;
        case FeatureMapType::DEPTH:  svgfBlitPass->set_depth_texture(featureTexture);  return;
        case FeatureMapType::FLOW:   svgfBlitPass->set_motion_texture(featureTexture); return;
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
    svgfBlitPass->render();
    // svgfBlitPass->set_previous_frame_data();
}


void SVGFDenoiser::recreateSwapchain(uint32_t width, uint32_t height) {
    svgfBlitPass->recreateSwapchain(width, height);
}

bool SVGFDenoiser::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    return svgfBlitPass->renderGuiPropertyEditorNodes(propertyEditor);
}

void SVGFDenoiser::resetFrameNumber() {

}

SVGFBlitPass::SVGFBlitPass(sgl::vk::Renderer* renderer)
    : sgl::vk::ComputePass(renderer)
{}

// Public interface.
void SVGFBlitPass::setOutputImage(sgl::vk::ImageViewPtr& colorImage) {
    output_image = colorImage;
}

void SVGFBlitPass::recreateSwapchain(uint32_t width, uint32_t height) {
    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = width;
    imageSettings.height = height;
    imageSettings.format = VK_FORMAT_R32G32B32A32_SFLOAT;
    imageSettings.usage =
        VK_IMAGE_USAGE_SAMPLED_BIT |
        VK_IMAGE_USAGE_STORAGE_BIT |
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    for (int i = 0; i < 2; i++) {
        pingPongRenderTextures[i] = std::make_shared<sgl::vk::Texture>(device, imageSettings);
    }

    // recreate previous frame images
    {
        // color_history
        previous_frame.color_history_texture = {};
        previous_frame.color_history_texture =
            std::make_shared<sgl::vk::Texture>(
                current_frame.color_texture->getImageView()->copy(true, false),
                current_frame.color_texture->getImageSampler());

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
                previous_frame.moments_history_texture[i] = {};
                previous_frame.moments_history_texture[i] =
                    std::make_shared<sgl::vk::Texture>(device, imageSettings);
            }
        }

        // normal
        sgl::vk::ImageSettings im_settings = current_frame.normal_texture->getImage()->getImageSettings();
        im_settings.usage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;

        previous_frame.normal_texture = {};
        previous_frame.normal_texture = std::make_shared<sgl::vk::Texture>(device, im_settings);

        // depth
        im_settings = current_frame.depth_texture->getImage()->getImageSettings();
        im_settings.usage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;

        previous_frame.depth_texture = {};
        previous_frame.depth_texture = std::make_shared<sgl::vk::Texture>(device, im_settings);
    }


    dataDirty = true;
}

void SVGFBlitPass::loadShader() {
    sgl::vk::ShaderManager->invalidateShaderCache();

    std::map<std::string, std::string> preprocessorDefines;

    preprocessorDefines.insert(std::make_pair("BLOCK_SIZE", std::to_string(computeBlockSize)));
    shaderStages = sgl::vk::ShaderManager->getShaderStages(
        { "SVGF.Compute" }, preprocessorDefines);
}

void SVGFBlitPass::createComputeData(sgl::vk::Renderer* renderer, sgl::vk::ComputePipelinePtr& compute_pipeline) {

    for (int i = 0; i < 3; i++) {
        computeDataPingPong[i] = std::make_shared<sgl::vk::ComputeData>(renderer, compute_pipeline);
        computeDataPingPongFinal[i] = std::make_shared<sgl::vk::ComputeData>(renderer, compute_pipeline);

        if (i == 0) {
            computeDataPingPong[i]->setStaticTexture(current_frame.color_texture, "color_texture");
            computeDataPingPongFinal[i]->setStaticTexture(current_frame.color_texture, "color_texture");
            computeDataPingPong[i]->setStaticImageView(pingPongRenderTextures[(i + 1) % 2]->getImageView(), "outputImage");
            computeDataPingPongFinal[i]->setStaticImageView(output_image, "outputImage");
        } else {
            computeDataPingPong[i]->setStaticTexture(pingPongRenderTextures[i % 2], "color_texture");
            computeDataPingPongFinal[i]->setStaticTexture(pingPongRenderTextures[i % 2], "color_texture");
            computeDataPingPong[i]->setStaticImageView(pingPongRenderTextures[(i + 1) % 2]->getImageView(), "outputImage");
            computeDataPingPongFinal[i]->setStaticImageView(output_image, "outputImage");
        }

        computeDataPingPong[i]->setStaticTexture(current_frame.depth_texture, "depth_texture");
        computeDataPingPongFinal[i]->setStaticTexture(current_frame.depth_texture, "depth_texture");

        computeDataPingPong[i]->setStaticTexture(current_frame.normal_texture, "normal_texture");
        computeDataPingPongFinal[i]->setStaticTexture(current_frame.normal_texture, "normal_texture");

        computeDataPingPong[i]->setStaticTexture(current_frame.motion_texture, "motion_texture");
        computeDataPingPongFinal[i]->setStaticTexture(current_frame.motion_texture, "motion_texture");

        // prev frame
        computeDataPingPong[i]->setStaticTexture(previous_frame.color_history_texture, "color_history_texture");
        computeDataPingPongFinal[i]->setStaticTexture(previous_frame.color_history_texture, "color_history_texture");

        // TODO(Felix): warum war moments_history_texture hier pingpong??
        computeDataPingPong[i]->setStaticTexture(previous_frame.moments_history_texture[0], "variance_history_texture");
        computeDataPingPongFinal[i]->setStaticTexture(previous_frame.moments_history_texture[0], "variance_history_texture");

        computeDataPingPong[i]->setStaticTexture(previous_frame.depth_texture, "depth_history_texture");
        computeDataPingPongFinal[i]->setStaticTexture(previous_frame.depth_texture, "depth_history_texture");

        // computeDataPingPong[i]->setStaticTexture(previous_frame.normal_texture, "normals_history_texture");
        // computeDataPingPongFinal[i]->setStaticTexture(previous_frame.normal_texture, "normals_history_texture");

    }
}

void SVGFBlitPass::_render() {
    if (maxNumIterations < 1) {
        renderer->transitionImageLayout(current_frame.color_texture->getImage(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
        renderer->transitionImageLayout(output_image->getImage(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
        current_frame.color_texture->getImage()->blit(output_image->getImage(), renderer->getVkCommandBuffer());
        renderer->syncWithCpu();
        return;
    }


    renderer->transitionImageLayout(current_frame.color_texture->getImage(),  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    renderer->transitionImageLayout(current_frame.depth_texture->getImage(),  VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    renderer->transitionImageLayout(current_frame.normal_texture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    renderer->transitionImageLayout(current_frame.motion_texture->getImage(), VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

    // render_compute
    {
        auto width = int(current_frame.color_texture->getImage()->getImageSettings().width);
        auto height = int(current_frame.color_texture->getImage()->getImageSettings().height);

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

            struct {
                int i;
                float z_multiplier;
            } pc;

            pc.i = i;
            pc.z_multiplier = z_multiplier;

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

            int numWorkgroupsX = sgl::iceil(width, computeBlockSize);
            int numWorkgroupsY = sgl::iceil(height, computeBlockSize);
            renderer->dispatch(computeData, numWorkgroupsX, numWorkgroupsY, 1);

            renderer->syncWithCpu();
        }

        // update previous frame images
        {
            // normal texture
            renderer->insertImageMemoryBarrier(
                previous_frame.normal_texture->getImage(),
                VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_WRITE_BIT);

            renderer->insertImageMemoryBarrier(
                current_frame.normal_texture->getImage(),
                current_frame.normal_texture->getImage()->getVkImageLayout(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_READ_BIT);


            current_frame.normal_texture->getImage()->copyToImage(previous_frame.normal_texture->getImage(),
                                                                 VK_IMAGE_ASPECT_COLOR_BIT,
                                                                 renderer->getVkCommandBuffer());

            renderer->syncWithCpu();

            // depth texture
            renderer->insertImageMemoryBarrier(
                previous_frame.depth_texture->getImage(),
                VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_WRITE_BIT);

            renderer->insertImageMemoryBarrier(
                current_frame.depth_texture->getImage(),
                current_frame.depth_texture->getImage()->getVkImageLayout(), VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
                VK_ACCESS_SHADER_READ_BIT, VK_ACCESS_TRANSFER_READ_BIT);


            current_frame.depth_texture->getImage()->copyToImage(previous_frame.depth_texture->getImage(),
                                                                 VK_IMAGE_ASPECT_COLOR_BIT,
                                                                 renderer->getVkCommandBuffer());

            renderer->syncWithCpu();
        }
    }
}

bool SVGFBlitPass::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    bool reRender = false;

    if (propertyEditor.addSliderInt("#Iterations", &maxNumIterations, 0, 5)) {
        reRender = true;
        setDataDirty();
        device->waitIdle();
    }


    if (propertyEditor.addSliderFloat("z multiplier", &z_multiplier, 1, 1000)) {
        reRender = true;
        setDataDirty();
        device->waitIdle();
    }


    return reRender;
}
