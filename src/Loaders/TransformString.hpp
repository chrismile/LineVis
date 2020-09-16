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

#ifndef STRESSLINEVIS_TRANSFORMSTRING_HPP
#define STRESSLINEVIS_TRANSFORMSTRING_HPP

#include <string>
#include <glm/glm.hpp>

/**
 * Parses SVG/CSS-like transform strings. The strings are read from left to right.
 * Example: "translation(-1, 2, 1.7) rotate(90° 0 -1 0) scale(1e-3)".
 * Supported commands:
 * - translate(tx, ty, tz): Translation by (tx, ty, tz).
 * - scale(s): Uniform scaling in x, y, z by s.
 * - scale(sx, sy, sz): Non-uniform scaling.
 * - rotate(angle-rad, ax, ay, az): Rotation around axis (ax, ay, az) by passed radians (0 to 2*PI).
 * - rotate(angle-deg°, ax, ay, az): Rotation around axis (ax, ay, az) by passed degrees (0° to 360°).
 * - matrix(a11, a12, a13, a21, a22, a23, a31, a32, a33): A 3x3 row-major matrix.
 * - matrix(a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34): A 3x4 row-major matrix.
 * - matrix(a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34, a41, a42, a43, a44): A 4x4 row-major matrix.
 * - matrixc(a11, a21, a31, a12, a22, a32, a13, a23, a33): A 3x3 column-major matrix.
 * - matrixc(a11, a21, a31, a12, a22, a32, a13, a23, a33, a14, a24, a34): A 3x4 column-major matrix.
 * - matrixc(a11, a21, a31, a41, a12, a22, a32, a42, a13, a23, a33, a43, a14, a24, a34, a44): A 4x4 column-major matrix.
 * @param transformString Transform string.
 * @return Parsed 4x4 transform matrix.
 */
glm::mat4 parseTransformString(std::string transformString);

#endif //STRESSLINEVIS_TRANSFORMSTRING_HPP
