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

#include <cmath>
#include <algorithm>
#include <glm/glm.hpp>
#include "HsvColor.hpp"

glm::vec3 rgbToHsv(const glm::vec3& color) {
    float minValue = glm::min(color.r, glm::min(color.g, color.b));
    float maxValue = glm::max(color.r, glm::max(color.g, color.b));

    float C = maxValue - minValue;

    // 1) Compute hue H
    float H = 0;
    if (maxValue == color.r) {
        H = glm::mod((color.g - color.b) / C, 6.0f);
    } else if (maxValue == color.g) {
        H = (color.b - color.r) / C + 2;
    } else if (maxValue == color.b) {
        H = (color.r - color.g) / C + 4;
    } else {
        H = 0;
    }

    H *= 60; // hue is in degree

    // 2) Compute the value V
    float V = maxValue;

    // 3) Compute saturation S
    float S = 0;
    if (V == 0) {
        S = 0;
    } else {
        S = C / V;
        //        S = C / (1 - abs(maxValue + minValue - 1));
    }

    return glm::vec3(H, S, V);
}

// https://en.wikipedia.org/wiki/HSL_and_HSV
// https://de.wikipedia.org/wiki/HSV-Farbraum

glm::vec3 hsvToRgb(const glm::vec3& color) {
    const float H = color.r;
    const float S = color.g;
    const float V = color.b;

    float h = H / 60.0;

    int hi = int(glm::floor(h));
    float f = (h - float(hi));

    float p = V * (1.0 - S);
    float q = V * (1.0 - S * f);
    float t = V * (1.0 - S * (1.0 - f));

    if (hi == 1) {
        return glm::vec3(q, V, p);
    } else if (hi == 2) {
        return glm::vec3(p, V, t);
    }
    else if (hi == 3) {
        return glm::vec3(p, q, V);
    } else if (hi == 4) {
        return glm::vec3(t, p, V);
    } else if (hi == 5) {
        return glm::vec3(V, p, q);
    } else {
        return glm::vec3(V, t, p);
    }
}
