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

#include <boost/algorithm/string/predicate.hpp>
#include <regex>

#include <Utils/Convert.hpp>
#include <Utils/File/Logfile.hpp>
#include <Math/Math.hpp>
#include <Math/Geometry/MatrixUtil.hpp>

#include "Tokens.hpp"
#include "TransformString.hpp"

glm::mat4 parseTransformString(std::string transformString) {
    glm::mat4 matrix = sgl::matrixIdentity();

    // Iterate over all of the concatenated transformations (pattern: "type(content)").
    // First match: Transform type. Second match: Transform information in the brackets.
    std::regex transformRegex("\\s*(\\S+)\\s*\\(\\s*(.+)\\s*\\)(.*)");
    std::smatch what;
    while (std::regex_search(transformString, what, transformRegex)) {
        std::string transformType = what[1];
        std::string transformContent = what[2];
        transformString = what[3]; // Continue with next part of string

        // Now split the transform content in the brackets (multiple float values separated by space or commas).
        std::vector<std::string> transformData = getTokenList(transformContent, NUMBER_AND_DEGREES_REGEX_STRING);

        if (transformType == "translate") {
            float x = sgl::fromString<float>(transformData.at(0));
            float y = sgl::fromString<float>(transformData.at(1));
            float z = sgl::fromString<float>(transformData.at(1));
            matrix = matrix * sgl::matrixTranslation(glm::vec3(x, y, z));
        } else if (transformType == "scale") {
            float scaleX = sgl::fromString<float>(transformData.at(0));
            if (transformData.size() == 1) {
                matrix = matrix * sgl::matrixScaling(glm::vec3(scaleX, scaleX, scaleX));
            } else {
                float scaleY = sgl::fromString<float>(transformData.at(1));
                float scaleZ = sgl::fromString<float>(transformData.at(2));
                matrix = matrix * sgl::matrixScaling(glm::vec3(scaleX, scaleY, scaleZ));
            }
        } else if (transformType == "rotate") {
            float rotationAngleRadians = 0.0f;
            // Degree or radians?
            if (boost::ends_with(transformData.at(0), "°")) {
                std::string numberString =
                        transformData.at(0).substr(0, transformData.at(0).find_last_of("°") - 1);
                rotationAngleRadians = sgl::fromString<float>(numberString) / 180.0f * sgl::PI;
            } else {
                rotationAngleRadians = sgl::fromString<float>(transformData.at(0));
            }
            float axisX = sgl::fromString<float>(transformData.at(1));
            float axisY = sgl::fromString<float>(transformData.at(2));
            float axisZ = sgl::fromString<float>(transformData.at(3));
            matrix = matrix * glm::rotate(rotationAngleRadians, glm::vec3(axisX, axisY, axisZ));
        } else if (transformType == "matrix") {
            float m[16];
            for (size_t i = 0; i < transformData.size(); i++) {
                m[i] = sgl::fromString<float>(transformData.at(i));
            }

            if (transformData.size() == 9) {
                matrix = matrix * glm::mat4(
                        m[0], m[3], m[6], 0.0f,
                        m[1], m[4], m[7], 0.0f,
                        m[2], m[5], m[8], 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f
                );
            } else if (transformData.size() == 12) {
                matrix = matrix * glm::mat4(
                        m[0], m[4], m[8], 0.0f,
                        m[1], m[5], m[9], 0.0f,
                        m[2], m[6], m[10], 0.0f,
                        m[3], m[7], m[11], 1.0f
                );
            } else if (transformData.size() == 16) {
                matrix = matrix * glm::mat4(
                        m[0], m[4], m[8], m[12],
                        m[1], m[5], m[9], m[13],
                        m[2], m[6], m[10], m[14],
                        m[3], m[7], m[11], m[15]
                );
            } else {
                sgl::Logfile::get()->writeError(
                        "ERROR in parseTransformString: Invalid number of elements in matrix.");
            }
        } else if (transformType == "matrixc") {
            float m[16];
            for (size_t i = 0; i < transformData.size(); i++) {
                m[i] = sgl::fromString<float>(transformData.at(i));
            }

            if (transformData.size() == 9) {
                matrix = matrix * glm::mat4(
                        m[0], m[1], m[2], 0.0f,
                        m[3], m[4], m[5], 0.0f,
                        m[6], m[7], m[8], 0.0f,
                        0.0f, 0.0f, 0.0f, 1.0f
                );
            } else if (transformData.size() == 12) {
                matrix = matrix * glm::mat4(
                        m[0], m[1], m[2], 0.0f,
                        m[3], m[4], m[5], 0.0f,
                        m[6], m[7], m[8], 0.0f,
                        m[9], m[10], m[11], 1.0f
                );
            } else if (transformData.size() == 16) {
                matrix = matrix * glm::mat4(
                        m[0], m[1], m[2], m[3],
                        m[4], m[5], m[6], m[7],
                        m[8], m[9], m[10], m[11],
                        m[12], m[13], m[14], m[15]
                );
            } else {
                sgl::Logfile::get()->writeError(
                        "ERROR in parseTransformString: Invalid number of elements in matrix.");
            }
        }
    }

    return matrix;
}
