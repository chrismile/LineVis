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

#include <vector>
#include <memory>

#include <Math/Geometry/MatrixUtil.hpp>

enum DataSetType {
    DATA_SET_TYPE_NONE,
    DATA_SET_TYPE_NODE, //< Hierarchical container.
    DATA_SET_TYPE_FLOW_LINES, //< Streamlines or streamribbons.
    DATA_SET_TYPE_STRESS_LINES, //< Principal stress lines (PSLs).
    DATA_SET_TYPE_SCATTERING_LINES, //< Lines created through path scattering in participating media.
    DATA_SET_TYPE_TRIANGLE_MESH //< Pre-generated triangle mesh.
};

const float STANDARD_LINE_WIDTH = 0.002f;
const float STANDARD_BAND_WIDTH = 0.005f;

struct DataSetInformation;
typedef std::shared_ptr<DataSetInformation> DataSetInformationPtr;

struct DataSetInformation {
    DataSetType type = DATA_SET_TYPE_NODE;
    std::string name;
    std::vector<std::string> filenames;

    // For type DATA_SET_TYPE_NODE.
    std::vector<DataSetInformationPtr> children;
    int sequentialIndex = 0;

    // Optional attributes.
    bool hasCustomLineWidth = false;
    float lineWidth = STANDARD_LINE_WIDTH;
    bool hasCustomTransform = false;
    glm::mat4 transformMatrix = sgl::matrixIdentity();
    std::vector<std::string> attributeNames; ///< Names of the associated importance criteria.
    int version = 3;
    float heightScale = 1.0f;

    // Stress lines: Additional information (optional).
    std::string meshFilename;
    std::string degeneratePointsFilename;
    std::vector<std::string> filenamesStressLineHierarchy;

    // For type DATA_SET_TYPE_TRIANGLE_MESH.
    bool shallComputeSharedVertexRepresentation = true;
    bool useVertexCacheOptimization = true;
};

DataSetInformationPtr loadDataSetList(const std::string& filename);

#endif //LINEDENSITYCONTROL_DATASETLIST_HPP
