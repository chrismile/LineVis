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

#ifndef HEXVOLUMERENDERER_CAMERAPATH_HPP
#define HEXVOLUMERENDERER_CAMERAPATH_HPP

#include <vector>
#include <glm/gtc/quaternion.hpp>

#include <Math/Geometry/MatrixUtil.hpp>
#include <Math/Geometry/AABB3.hpp>

class ControlPoint
{
public:
    ControlPoint() {}
    ControlPoint(float time, float tx, float ty, float tz, float yaw, float pitch);

    float time = 0.0f;
    glm::vec3 position;
    glm::quat orientation;
};

class CameraPath
{
public:
    CameraPath() {}
    void fromCirclePath(
            sgl::AABB3& sceneBoundingBox, const std::string& modelFilename, float totalTime,
            bool performanceMeasurementMode);
    void fromControlPoints(const std::vector<ControlPoint>& controlPoints);
    bool fromBinaryFile(const std::string& filename);
    bool saveToBinaryFile(const std::string& filename);
    void normalizeToTotalTime(float totalTime);
    void update(float currentTime);
    void resetTime();
    inline const glm::mat4x4& getViewMatrix() const { return currentTransform; }
    inline float getEndTime() { return controlPoints.back().time; }

private:
    glm::mat4x4 toTransform(const glm::vec3 &position, const glm::quat &orientation);

    const uint32_t CAMERA_PATH_FORMAT_VERSION = 1u;
    glm::mat4x4 currentTransform;
    std::vector<ControlPoint> controlPoints;
    float time = 0.0f;
};

#endif //HEXVOLUMERENDERER_CAMERAPATH_HPP
