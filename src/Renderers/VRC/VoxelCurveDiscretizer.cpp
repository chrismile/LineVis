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

#include <chrono>

#include <Math/Math.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Render/Data.hpp>
#include <Graphics/Vulkan/Render/ComputePipeline.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <Graphics/Vulkan/Shader/ShaderManager.hpp>
#include <memory>

#include "VoxelCurveDiscretizer.hpp"

VoxelCurveDiscretizer::VoxelCurveDiscretizer(sgl::vk::Device* device) {
    renderer = new sgl::vk::Renderer(device, 100);

    sgl::vk::ComputePipelineInfo computePipelineInfo(sgl::vk::ShaderManager->getShaderStages(
            { "PrefixSumBlockIncrement.Compute" }));
    prefixSumBlockIncrementPipeline = std::make_shared<sgl::vk::ComputePipeline>(device, computePipelineInfo);

    computePipelineInfo = sgl::vk::ComputePipelineInfo(sgl::vk::ShaderManager->getShaderStages(
            { "PrefixSumScan.Compute" }));
    prefixSumScanPipeline = std::make_shared<sgl::vk::ComputePipeline>(device, computePipelineInfo);

    computePipelineInfo = sgl::vk::ComputePipelineInfo(sgl::vk::ShaderManager->getShaderStages(
            { "PrefixSumWriteFinalElement.Compute" }));
    sgl::vk::ComputePipelinePtr computePipeline = std::make_shared<sgl::vk::ComputePipeline>(
            device, computePipelineInfo);
    prefixSumWriteFinalElementData = std::make_shared<sgl::vk::ComputeData>(renderer, computePipeline);
}

VoxelCurveDiscretizer::~VoxelCurveDiscretizer() {
    prefixSumBlockIncrementData = {};
    prefixSumScanData = {};
    prefixSumWriteFinalElementData = {};

    renderer->getDevice()->waitIdle();
    delete renderer;
    renderer = nullptr;
}

void VoxelCurveDiscretizer::loadLineData(
        LineDataPtr& lineData, uint32_t gridResolution1D, uint32_t quantizationResolution1D) {
    linesBoundingBox = lineData->getFocusBoundingBox();
    gridResolution = glm::uvec3(gridResolution1D);
    quantizationResolution = glm::uvec3(quantizationResolution1D);

    glm::vec3 gridDimensions = linesBoundingBox.getDimensions();
    float maxDimensionLength = 0.0f;
    for (int i = 0; i < 3; i++) {
        maxDimensionLength = std::max(maxDimensionLength, gridDimensions[i]);
    }

    // Make sure that each dimension has a size of at least one voxel.
    for (int i = 0; i < 3; i++) {
        if (gridDimensions[i] < maxDimensionLength / float(gridResolution1D)) {
            float diff = maxDimensionLength / float(gridResolution1D) - gridDimensions[i];
            linesBoundingBox.min[i] -= diff / 2.0f;
            linesBoundingBox.max[i] += diff / 2.0f;
        }
    }
    gridDimensions = linesBoundingBox.getDimensions();

    for (int i = 0; i < 3; i++) {
        float sideLengthFactor = gridDimensions[i] / maxDimensionLength;
        gridResolution[i] = std::max(int(std::ceil(float(gridResolution[i]) * sideLengthFactor)), 1);
    }

    linesToVoxel =
            sgl::matrixScaling(1.0f / linesBoundingBox.getDimensions() * glm::vec3(gridResolution))
            * sgl::matrixTranslation(-linesBoundingBox.getMinimum());
    voxelToLines = glm::inverse(linesToVoxel);

    curves.clear();
    int selectedAttributeIndex = lineData->getSelectedAttributeIndex();
    lineData->rebuildInternalRepresentationIfNecessary();
    lineData->iterateOverTrajectories([this, selectedAttributeIndex](const Trajectory& trajectory) {
        Curve curve;
        for (size_t pointIdx = 0; pointIdx < trajectory.positions.size(); pointIdx++) {
            curve.points.push_back(sgl::transformPoint(linesToVoxel, trajectory.positions.at(pointIdx)));
            curve.attributes.push_back(trajectory.attributes.at(selectedAttributeIndex).at(pointIdx));
        }
        curve.lineID = uint32_t(curves.size());
        curves.push_back(curve);
    });
}
