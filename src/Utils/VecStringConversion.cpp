/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020-2021, Christoph Neuhauser
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

#include <glm/gtx/quaternion.hpp>
#include <Utils/Convert.hpp>
#include "VecStringConversion.hpp"

std::string vec2ToString(const glm::vec2& v) {
    return std::string() + std::to_string(v.x) + "|" + std::to_string(v.y);
}
std::string vec3ToString(const glm::vec3& v) {
    return std::string() + std::to_string(v.x) + "|" + std::to_string(v.y) + "|" + std::to_string(v.z);
}
std::string vec4ToString(const glm::vec4& v) {
    return std::string() + std::to_string(v.x) + "|" + std::to_string(v.y) + "|" + std::to_string(v.z) + "|" + std::to_string(v.w);
}
std::string quatToString(const glm::quat& q) {
    return std::string() + std::to_string(q.x) + "|" + std::to_string(q.y) + "|" + std::to_string(q.z) + "|" + std::to_string(q.w);
}

glm::vec2 stringToVec2(const std::string& s) {
    std::vector<float> components;
    std::string buffer;
    for (char c : s) {
        if (c == '|') {
            components.push_back(sgl::fromString<float>(buffer));
            buffer.clear();
        } else {
            buffer += c;
        }
    }
    if (!buffer.empty()) {
        components.push_back(sgl::fromString<float>(buffer));
    }
    assert(components.size() == 2);
    return glm::vec2(components.at(0), components.at(1));
}
glm::vec3 stringToVec3(const std::string& s) {
    std::vector<float> components;
    std::string buffer;
    for (char c : s) {
        if (c == '|') {
            components.push_back(sgl::fromString<float>(buffer));
            buffer.clear();
        } else {
            buffer += c;
        }
    }
    if (!buffer.empty()) {
        components.push_back(sgl::fromString<float>(buffer));
    }
    assert(components.size() == 3);
    return glm::vec3(components.at(0), components.at(1), components.at(2));
}
glm::vec4 stringToVec4(const std::string& s) {
    std::vector<float> components;
    std::string buffer;
    for (char c : s) {
        if (c == '|') {
            components.push_back(sgl::fromString<float>(buffer));
            buffer.clear();
        } else {
            buffer += c;
        }
    }
    if (!buffer.empty()) {
        components.push_back(sgl::fromString<float>(buffer));
    }
    assert(components.size() == 3 || components.size() == 4);
    if (components.size() == 3) {
        return glm::vec4(components.at(0), components.at(1), components.at(2), 1.0f);
    } else {
        return glm::vec4(components.at(0), components.at(1), components.at(2), components.at(3));
    }
}
glm::quat stringToQuat(const std::string& s) {
    std::vector<float> components;
    std::string buffer;
    for (char c : s) {
        if (c == '|') {
            components.push_back(sgl::fromString<float>(buffer));
            buffer.clear();
        } else {
            buffer += c;
        }
    }
    if (!buffer.empty()) {
        components.push_back(sgl::fromString<float>(buffer));
    }
    assert(components.size() == 4);
    return glm::quat(components.at(0), components.at(1), components.at(2), components.at(3));
}
