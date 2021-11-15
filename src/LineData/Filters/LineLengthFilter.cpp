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

#include <ImGui/imgui_custom.h>
#include <ImGui/Widgets/PropertyEditor.hpp>

#include "LineData/LineData.hpp"
#include "LineLengthFilter.hpp"

void LineLengthFilter::onDataLoaded(LineDataPtr lineDataIn) {
    trajectoryFilteringThresholdMin = 0.0f;
    maxTrajectoryLength = 0.0f;
    trajectoryLengths.clear();

    lineDataIn->iterateOverTrajectories([this](const Trajectory& trajectory) {
        int n = int(trajectory.positions.size());

        float trajectoryLength = 0.0f;
        for (int i = 0; i < n - 1; i++) {
            trajectoryLength += glm::length(trajectory.positions.at(i) - trajectory.positions.at(i + 1));
        }

        trajectoryLengths.push_back(trajectoryLength);
        maxTrajectoryLength = std::max(maxTrajectoryLength, trajectoryLength);
    });

    trajectoryFilteringThresholdMax = maxTrajectoryLength;
    canUseLiveUpdate = lineDataIn->getCanUseLiveUpdate(LineDataAccessType::FILTERED_LINES);
}

void LineLengthFilter::filterData(LineDataPtr lineDataIn) {
    if (trajectoryLengths.empty()) {
        onDataLoaded(lineDataIn);
    }

    size_t trajectoryIdx = 0;
    lineDataIn->filterTrajectories([&trajectoryIdx, this](const Trajectory& trajectory) -> bool {
        float trajectoryLength = trajectoryLengths.at(trajectoryIdx++);
        return trajectoryLength < trajectoryFilteringThresholdMin || trajectoryLength > trajectoryFilteringThresholdMax;
    });
    dirty = false;
}

void LineLengthFilter::renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor) {
    glm::vec2 newRange(trajectoryFilteringThresholdMin, trajectoryFilteringThresholdMax);
    ImGui::EditMode editMode = propertyEditor.addSliderFloat2Edit(
            "Length Range", &newRange.x, 0.0f, maxTrajectoryLength);
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
