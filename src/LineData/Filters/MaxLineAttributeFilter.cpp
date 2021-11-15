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

#include <limits>

#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "LineData/LineData.hpp"
#include "MaxLineAttributeFilter.hpp"

void MaxLineAttributeFilter::onDataLoaded(LineDataPtr lineDataIn) {
    trajectoryFilteringThresholdMin = 0.0f;
    minGlobalAttribute = std::numeric_limits<float>::max();
    maxGlobalAttribute = std::numeric_limits<float>::lowest();
    minTrajectoryAttributes.clear();
    maxTrajectoryAttributes.clear();

    lineDataIn->iterateOverTrajectories([this, lineDataIn](const Trajectory& trajectory) {
        int n = int(trajectory.positions.size());
        int attributeIdx = lineDataIn->getSelectedAttributeIndex();

        float minTrajectoryAttribute = std::numeric_limits<float>::max();
        float maxTrajectoryAttribute = std::numeric_limits<float>::lowest();
        for (int i = 0; i < n - 1; i++) {
            float attr = trajectory.attributes.at(attributeIdx).at(i);
            minTrajectoryAttribute = std::min(minTrajectoryAttribute, attr);
            maxTrajectoryAttribute = std::max(maxTrajectoryAttribute, attr);
        }

        minTrajectoryAttributes.push_back(minTrajectoryAttribute);
        maxTrajectoryAttributes.push_back(maxTrajectoryAttribute);
        minGlobalAttribute = std::min(minGlobalAttribute, minTrajectoryAttribute);
        maxGlobalAttribute = std::max(maxGlobalAttribute, maxTrajectoryAttribute);
    });
    trajectoryFilteringThresholdMin = minGlobalAttribute;
    trajectoryFilteringThresholdMax = maxGlobalAttribute;

    canUseLiveUpdate = lineDataIn->getCanUseLiveUpdate(LineDataAccessType::FILTERED_LINES);
}

void MaxLineAttributeFilter::filterData(LineDataPtr lineDataIn) {
    if (maxTrajectoryAttributes.empty()) {
        onDataLoaded(lineDataIn);
    }

    size_t trajectoryIdx = 0;
    lineDataIn->filterTrajectories([this, &trajectoryIdx, lineDataIn](const Trajectory& trajectory) -> bool {
        float attributeValue = maxTrajectoryAttributes.at(trajectoryIdx++);
        return attributeValue < trajectoryFilteringThresholdMin || attributeValue > trajectoryFilteringThresholdMax;
    });
    dirty = false;
}

void MaxLineAttributeFilter::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    glm::vec2 newRange(trajectoryFilteringThresholdMin, trajectoryFilteringThresholdMax);
    ImGui::EditMode editMode = propertyEditor.addSliderFloat2Edit(
            "Attribute Range", &newRange.x, minGlobalAttribute, maxGlobalAttribute);
    if (editMode != ImGui::EditMode::NO_CHANGE && newRange.x > newRange.y) {
        if (newRange.x != trajectoryFilteringThresholdMin) {
            newRange.x = newRange.y;
        } else {
            newRange.y = newRange.x;
        }
    }
    trajectoryFilteringThresholdMin = newRange.x;
    trajectoryFilteringThresholdMax = newRange.y;
    if ((canUseLiveUpdate && editMode != ImGui::EditMode::NO_CHANGE)
        || (!canUseLiveUpdate && editMode == ImGui::EditMode::INPUT_FINISHED)) {
        dirty = true;
    }
}
