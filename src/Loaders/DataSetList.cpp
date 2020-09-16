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

#include <jsoncpp/json/json.h>

#include <Utils/File/Logfile.hpp>

#include "TransformString.hpp"
#include "DataSetList.hpp"

std::vector<DataSetInformation> loadDataSetList(const std::string& filename) {
    std::vector<DataSetInformation> dataSetList;

    // Parse the passed JSON file.
    std::ifstream jsonFileStream(filename.c_str());
    Json::CharReaderBuilder builder;
    JSONCPP_STRING errorString;
    Json::Value root;
    if (!parseFromStream(builder, jsonFileStream, &root, &errorString)) {
        sgl::Logfile::get()->writeError(errorString);
        return dataSetList;
    }
    jsonFileStream.close();

    Json::Value sources = root["datasets"];
    for (Json::Value::const_iterator sourceIt = sources.begin(); sourceIt != sources.end(); ++sourceIt) {
        DataSetInformation dataSetInformation;
        Json::Value source = *sourceIt;

        // Get the type information.
        Json::Value type = source["type"];
        std::string typeName = type.asString();
        if (typeName == "flow") {
            dataSetInformation.type = DATA_SET_TYPE_FLOW_LINES;
        } else if (typeName == "stress") {
            dataSetInformation.type = DATA_SET_TYPE_STRESS_LINES;
        } else {
            sgl::Logfile::get()->writeError("ERROR in loadDataSetList: Invalid type name \"" + typeName + "\".");
            return std::vector<DataSetInformation>();
        }

        // Get the display name and the associated filenames.
        dataSetInformation.name = source["name"].asString();
        Json::Value filenames = source["filenames"];
        if (filenames.isArray()) {
            for (Json::Value::const_iterator filenameIt = filenames.begin();
                 filenameIt != filenames.end(); ++filenameIt) {
                dataSetInformation.filenames.push_back(filenameIt->asString());
            }
        } else {
            dataSetInformation.filenames.push_back(filenames.asString());
        }

        // Optional data: Line width.
        dataSetInformation.hasCustomLineWidth = source.isMember("linewidth");
        if (dataSetInformation.hasCustomLineWidth) {
            dataSetInformation.lineWidth = source["linewidth"].asFloat();
        }

        // Optional data: Transform.
        dataSetInformation.hasCustomTransform = source.isMember("transform");
        if (dataSetInformation.hasCustomTransform) {
            glm::mat4 transformMatrix = parseTransformString(source["transform"].asString());
            dataSetInformation.transformMatrix = transformMatrix;
        }

        // Optional data: Attribute (importance criteria) display names.
        if (source.isMember("attributes")) {
            Json::Value attributes = source["attributes"];
            if (attributes.isArray()) {
                for (Json::Value::const_iterator attributesIt = attributes.begin();
                     attributesIt != attributes.end(); ++attributesIt) {
                    dataSetInformation.attributeNames.push_back(attributesIt->asString());
                }
            } else {
                dataSetInformation.attributeNames.push_back(attributes.asString());
            }
        }

        dataSetList.push_back(dataSetInformation);
    }
    return dataSetList;
}
