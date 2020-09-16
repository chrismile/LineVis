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
#include <fstream>
#include <boost/algorithm/string/predicate.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/norm.hpp>

#include <Utils/Events/Stream/Stream.hpp>
#include <Utils/File/Logfile.hpp>
#include <Math/Math.hpp>

#include "CameraPath.hpp"

//#define ROTATION_AND_ZOOMING_MODE
//#define ROTATION_MODE
//#define ZOOMING_MODE
#define VIDEO_MODE

ControlPoint::ControlPoint(float time, float tx, float ty, float tz, float yaw, float pitch) {
    this->time = time;
    this->position = glm::vec3(tx, ty, tz);
    this->orientation = glm::angleAxis(-pitch, glm::vec3(1, 0, 0))
            * glm::angleAxis(yaw + sgl::PI / 2.0f, glm::vec3(0, 1, 0));
}

void CameraPath::fromCirclePath(
        sgl::AABB3& sceneBoundingBox, const std::string& modelFilename, float totalTime,
        bool performanceMeasurementMode) {
    const size_t NUM_CIRCLE_POINTS = 64;
    controlPoints.clear();

    glm::vec3 centerOffset(0.0f, 0.0f, 0.0f);
    float startAngle = 0.0f;
    float pulseFactor = 2.0f;
    float standardZoom = performanceMeasurementMode ? 1.4f : 1.6f;
    if (boost::starts_with(
            modelFilename,
            "Data/Meshes/2011 - All-Hex Mesh Generation via Volumetric PolyCube Deformation/anc101_a1.mesh")) {
        centerOffset = glm::vec3(0.0f, -0.02f, 0.0f);
        pulseFactor = 3.0f;
        standardZoom = 1.6f;
    }
    if (boost::starts_with(
            modelFilename,
            "Data/Meshes/2014 - l1-Based Construction of Polycube Maps from Complex Shapes/cognit/hex.vtk")) {
        centerOffset = glm::vec3(0.0f, -0.02f, 0.0f);
        pulseFactor = 3.0f;
        standardZoom = 1.6f;
    }
    if (performanceMeasurementMode && boost::ends_with(modelFilename, "cubic128.vtk")) {
        standardZoom = 1.3f;
    }

    for (size_t i = 0; i <= NUM_CIRCLE_POINTS; i++) {
        float time = float(i) / NUM_CIRCLE_POINTS * totalTime;
        float angle = float(i) / NUM_CIRCLE_POINTS * sgl::TWO_PI + startAngle;
        float pulseRadius = (cos(2.0f * angle) - 1.0f) / (8.0f / pulseFactor) + standardZoom;
        glm::vec3 cameraPos =
                glm::vec3(cos(angle)*pulseRadius, 0.0f, sin(angle)*pulseRadius)
                * glm::length(sceneBoundingBox.getExtent()) + sceneBoundingBox.getCenter() + centerOffset;
        controlPoints.push_back(
                ControlPoint(time, cameraPos.x, cameraPos.y, cameraPos.z, sgl::PI + angle, 0.0f));
    }
    update(0.0f);
}

void CameraPath::fromControlPoints(const std::vector<ControlPoint>& controlPoints) {
    this->controlPoints = controlPoints;
    update(0.0f);
}

bool CameraPath::fromBinaryFile(const std::string& filename) {
    std::ifstream file(filename.c_str(), std::ifstream::binary);
    if (!file.is_open()) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in CameraPath::fromBinaryFile: File \"" + filename + "\" not found.");
        return false;
    }

    file.seekg(0, file.end);
    size_t size = file.tellg();
    file.seekg(0);
    char *buffer = new char[size];
    file.read(buffer, size);
    file.close();

    sgl::BinaryReadStream stream(buffer, size);
    uint32_t version;
    stream.read(version);
    if (version != CAMERA_PATH_FORMAT_VERSION) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in CameraPath::fromBinaryFile: "
                + "Invalid version in file \"" + filename + "\".");
        return false;
    }

    controlPoints.clear();
    stream.readArray(controlPoints);
    update(0.0f);

    return true;
}

bool CameraPath::saveToBinaryFile(const std::string& filename) {
    std::ofstream file(filename.c_str(), std::ofstream::binary);
    if (!file.is_open()) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in CameraPath::saveToBinaryFile: File \"" + filename + "\" not found.");
        return false;
    }

    sgl::BinaryWriteStream stream;
    stream.write((uint32_t)CAMERA_PATH_FORMAT_VERSION);
    stream.writeArray(controlPoints);
    file.write((const char*)stream.getBuffer(), stream.getSize());
    file.close();

    return true;
}


void CameraPath::normalizeToTotalTime(float totalTime) {
    float factor = totalTime / getEndTime();
    int n = (int)controlPoints.size();
    for (int i = 0; i < n; i++) {
        controlPoints.at(i).time *= factor;
    }
}

void CameraPath::update(float currentTime) {
    time = fmod(currentTime, controlPoints.back().time);

    // Find either exact control point or two to interpolate between
    int n = (int)controlPoints.size();
    for (int i = 0; i < n; i++) {
        // Perfect match?
        ControlPoint& currentCtrlPoint = controlPoints.at(i);
        if (i == n-1 || glm::abs(time - currentCtrlPoint.time) < 0.0001f) {
            currentTransform = toTransform(currentCtrlPoint.position, currentCtrlPoint.orientation);
            break;
        }
        ControlPoint& nextCtrlPoint = controlPoints.at(i+1);
        if (time > currentCtrlPoint.time && time < nextCtrlPoint.time) {
            float percentage = (time - currentCtrlPoint.time) / (nextCtrlPoint.time - currentCtrlPoint.time);
            glm::vec3 interpPos = glm::mix(currentCtrlPoint.position, nextCtrlPoint.position, percentage);
            glm::quat interpQuat = glm::slerp(currentCtrlPoint.orientation, nextCtrlPoint.orientation, percentage);
            currentTransform = toTransform(interpPos, interpQuat);
            break;
        }
    }
}

void CameraPath::resetTime() {
    update(0.0f);
}

glm::mat4x4 CameraPath::toTransform(const glm::vec3& position, const glm::quat& orientation) {
    return glm::toMat4(orientation) * sgl::matrixTranslation(-position);
}
