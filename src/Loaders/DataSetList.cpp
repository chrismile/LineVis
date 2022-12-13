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

#include <iostream>

#include <boost/algorithm/string/predicate.hpp>
#include <json/json.h>

#include <Utils/File/Logfile.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/Regex/TransformString.hpp>

#include "DataSetList.hpp"

bool jsonValueToBool(const Json::Value& value) {
    if (value.isString()) {
        std::string valueString = value.asString();
        if (valueString == "true") {
            return true;
        } else if (valueString == "false") {
            return false;
        } else {
            sgl::Logfile::get()->throwError("Error in jsonValueToBool: Invalid value \"" + valueString + "\".");
            return false;
        }
    } else {
        return value.asBool();
    }
}

void processDataSetNodeChildren(Json::Value& childList, DataSetInformation* dataSetInformationParent) {
    for (Json::Value& source : childList) {
        auto* dataSetInformation = new DataSetInformation;

        // Get the type information.
        Json::Value type = source["type"];
        std::string typeName = type.asString();
        if (typeName == "node") {
            dataSetInformation->type = DATA_SET_TYPE_NODE;
        } else if (typeName == "flow") {
            dataSetInformation->type = DATA_SET_TYPE_FLOW_LINES;
        } else if (typeName == "stress") {
            dataSetInformation->type = DATA_SET_TYPE_STRESS_LINES;
        } else if (typeName == "trimesh") {
            dataSetInformation->type = DATA_SET_TYPE_TRIANGLE_MESH;
        } else {
            sgl::Logfile::get()->writeError(
                    "Error in processDataSetNodeChildren: Invalid type name \"" + typeName + "\".");
            return;
        }

        dataSetInformation->name = source["name"].asString();

        if (dataSetInformation->type == DATA_SET_TYPE_NODE) {
            dataSetInformationParent->children.emplace_back(dataSetInformation);
            processDataSetNodeChildren(source["children"], dataSetInformation);
            continue;
        }

        Json::Value filenames = source["filenames"];
        const std::string lineDataSetsDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";
        if (filenames.isArray()) {
            for (const auto& filename : filenames) {
                std::string pathString = filename.asString();
#ifdef _WIN32
                bool isAbsolutePath =
                    (pathString.size() > 1 && pathString.at(1) == ':')
                    || boost::starts_with(pathString, "/") || boost::starts_with(pathString, "\\");
#else
                bool isAbsolutePath = boost::starts_with(pathString, "/");
#endif
                if (isAbsolutePath) {
                    dataSetInformation->filenames.push_back(pathString);
                } else {
                    dataSetInformation->filenames.push_back(lineDataSetsDirectory + pathString);
                }
            }
        } else {
            std::string pathString = filenames.asString();
#ifdef _WIN32
            bool isAbsolutePath =
                    (pathString.size() > 1 && pathString.at(1) == ':')
                    || boost::starts_with(pathString, "/") || boost::starts_with(pathString, "\\");
#else
            bool isAbsolutePath = boost::starts_with(pathString, "/");
#endif
            if (isAbsolutePath) {
                dataSetInformation->filenames.push_back(pathString);
            } else {
                dataSetInformation->filenames.push_back(lineDataSetsDirectory + pathString);
            }
        }

        // Optional data: Line width.
        dataSetInformation->hasCustomLineWidth = source.isMember("linewidth");
        if (dataSetInformation->hasCustomLineWidth) {
            dataSetInformation->lineWidth = source["linewidth"].asFloat();
        }

        // Optional data: Transform.
        dataSetInformation->hasCustomTransform = source.isMember("transform");
        if (dataSetInformation->hasCustomTransform) {
            glm::mat4 transformMatrix = parseTransformString(source["transform"].asString());
            dataSetInformation->transformMatrix = transformMatrix;
        } else if (dataSetInformation->type == DATA_SET_TYPE_STRESS_LINES) {
            dataSetInformation->hasCustomTransform = true;
            dataSetInformation->transformMatrix = parseTransformString("rotate(270°, 1, 0, 0)");
        }

        // Optional data: The version of the file format.
        if (source.isMember("version")) {
            dataSetInformation->version = source["version"].asInt();
        }

        // Optional data: Attribute (importance criteria) display names.
        if (source.isMember("attributes")) {
            Json::Value attributes = source["attributes"];
            if (attributes.isArray()) {
                for (Json::Value::const_iterator attributesIt = attributes.begin();
                     attributesIt != attributes.end(); ++attributesIt) {
                    dataSetInformation->attributeNames.push_back(attributesIt->asString());
                }
            } else {
                dataSetInformation->attributeNames.push_back(attributes.asString());
            }
        } else if (dataSetInformation->type == DATA_SET_TYPE_STRESS_LINES && dataSetInformation->version >= 3) {
            dataSetInformation->attributeNames.emplace_back("Principal Stress");
            dataSetInformation->attributeNames.emplace_back("Principal Stress Magnitude");
            dataSetInformation->attributeNames.emplace_back("von Mises Stress");
            dataSetInformation->attributeNames.emplace_back("Normal Stress (xx)");
            dataSetInformation->attributeNames.emplace_back("Normal Stress (yy)");
            dataSetInformation->attributeNames.emplace_back("Normal Stress (zz)");
            dataSetInformation->attributeNames.emplace_back("Shear Stress (yz)");
            dataSetInformation->attributeNames.emplace_back("Shear Stress (zx)");
            dataSetInformation->attributeNames.emplace_back("Shear Stress (xy)");
        }

        // Optional data: The scaling in y direction.
        if (source.isMember("heightscale")) {
            dataSetInformation->heightScale = source["heightscale"].asFloat();
        }

        // Optional stress line data: Mesh file.
        if (source.isMember("mesh")) {
            dataSetInformation->meshFilename = lineDataSetsDirectory + source["mesh"].asString();
        }

        // Optional stress line data: Degenerate points file.
        if (source.isMember("degenerate_points")) {
            dataSetInformation->degeneratePointsFilename =
                    lineDataSetsDirectory + source["degenerate_points"].asString();
        }

        // Optional stress line data: Line hierarchy.
        if (source.isMember("line_hierarchy")) {
            Json::Value lineHierarchyFilenames = source["line_hierarchy"];
            if (lineHierarchyFilenames.isArray()) {
                for (Json::Value::const_iterator filenameIt = lineHierarchyFilenames.begin();
                     filenameIt != lineHierarchyFilenames.end(); ++filenameIt) {
                    dataSetInformation->filenamesStressLineHierarchy.push_back(
                            lineDataSetsDirectory + filenameIt->asString());
                }
            } else {
                dataSetInformation->filenamesStressLineHierarchy.push_back(
                        lineDataSetsDirectory + lineHierarchyFilenames.asString());
            }
        }

        // Optional triangle mesh information: Convert disconnected triangles with a shared vertex representation?
        if (dataSetInformation->type == DATA_SET_TYPE_TRIANGLE_MESH && source.isMember("shared_vertex_representation")) {
            Json::Value sharedVertexRepresentation = source["shared_vertex_representation"];
            dataSetInformation->shallComputeSharedVertexRepresentation = jsonValueToBool(sharedVertexRepresentation);
        }

        // Optional triangle mesh information: Use vertex cache optimization after loading the data.
        if (dataSetInformation->type == DATA_SET_TYPE_TRIANGLE_MESH && source.isMember("vertex_cache_optimization")) {
            Json::Value vertexCacheOptimization = source["vertex_cache_optimization"];
            dataSetInformation->useVertexCacheOptimization = jsonValueToBool(vertexCacheOptimization);
        }

        dataSetInformationParent->children.emplace_back(dataSetInformation);
    }
}

DataSetInformationPtr loadDataSetList(const std::string& filename) {
    // Parse the passed JSON file.
    std::ifstream jsonFileStream(filename.c_str());
    Json::CharReaderBuilder builder;
    JSONCPP_STRING errorString;
    Json::Value root;
    if (!parseFromStream(builder, jsonFileStream, &root, &errorString)) {
        sgl::Logfile::get()->writeError(errorString);
        return {};
    }
    jsonFileStream.close();

    DataSetInformationPtr dataSetInformationRoot(new DataSetInformation);
    Json::Value& dataSetNode = root["datasets"];
    processDataSetNodeChildren(dataSetNode, dataSetInformationRoot.get());
    return dataSetInformationRoot;
}
