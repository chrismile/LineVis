/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser
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

#ifndef LINEDENSITYCONTROL_DATASETLIST_HPP
#define LINEDENSITYCONTROL_DATASETLIST_HPP

#include <Math/Geometry/MatrixUtil.hpp>
#include <vector>

const std::string lineDataSetsDirectory = "Data/LineDataSets/";

enum DataSetType {
    DATA_SET_TYPE_NONE, DATA_SET_TYPE_FLOW_LINES, DATA_SET_TYPE_STRESS_LINES, DATA_SET_TYPE_FLOW_LINES_MULTIVAR
};

struct DataSetInformation {
    DataSetType type = DATA_SET_TYPE_FLOW_LINES;
    std::string name;
    std::vector<std::string> filenames;

    // Optional attributes.
    bool hasCustomLineWidth = false;
    float lineWidth = 0.002f;
    bool hasCustomTransform = false;
    glm::mat4 transformMatrix = sgl::matrixIdentity();
    std::vector<std::string> attributeNames; ///< Names of the associated importance criteria.
    int version = 1;

    // Stress lines: Additional information (optional).
    std::string meshFilename;
    std::string degeneratePointsFilename;
    std::vector<std::string> filenamesStressLineHierarchy;
};

std::vector<DataSetInformation> loadDataSetList(const std::string& filename);

#endif //LINEDENSITYCONTROL_DATASETLIST_HPP
