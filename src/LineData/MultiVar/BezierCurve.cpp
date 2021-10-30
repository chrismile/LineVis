/*
 * This file contains adapted code from the Embree library.
 * The original code is covered by the Apache License, Version 2.0, and the changes
 * are covered by the BSD 2-Clause License.
 *
 * -------------------------------------------------------------------------------
 * Copyright 2009-2020 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * -------------------------------------------------------------------------------
 *
 * -------------------------------------------------------------------------------
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser, Michael Kern
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
 * -------------------------------------------------------------------------------
 */

#include "BezierCurve.hpp"

// Bezier Curve code --> quick evaluation
// https://github.com/embree/embree/blob/master/kernels/subdiv/bezier_curve.h

BezierCurve::BezierCurve(const std::array<glm::vec3, 4> &points, const float _minT, const float _maxT)
        : controlPoints(points), minT(_minT), maxT(_maxT) {
    totalArcLength = evalArcLength(minT, maxT, 20);
}

void BezierCurve::evaluate(const float t, glm::vec3 &P, glm::vec3 &dt) const {
    assert(minT <= t && t <= maxT);

    float tN = normalizeT(t);
    //float negT = 1.0f - tN;
    // Check old algorithm to verify the Bezier Curve
    // Original algebraic solution
//    glm::vec3 pos = float(std::pow(negT, 3)) * controlPoints[0]
//            + 3 * float(std::pow(negT, 2)) * tN * controlPoints[1]
//            + 3 * negT * float(std::pow(tN, 2)) * controlPoints[2]
//            + float(std::pow(tN, 3)) * controlPoints[3];

    // De-Casteljau algorithm
    auto P01 = glm::mix(controlPoints[0], controlPoints[1], tN);
    auto P02 = glm::mix(controlPoints[1], controlPoints[2], tN);
    auto P03 = glm::mix(controlPoints[2], controlPoints[3], tN);
    auto P11 = glm::mix(P01, P02, tN);
    auto P12 = glm::mix(P02, P03, tN);
    P = glm::mix(P11, P12, tN);

//    auto testDT = derivative(t);

    dt = 3.0f * (P12 - P11);
}

glm::vec3 BezierCurve::derivative(const float t) const {
    assert(minT <= t && t <= maxT);

    float tN = normalizeT(t);
    //float negT = 1.0f - tN;

    // Original algebraic solution
//    auto DP01 = controlPoints[1] - controlPoints[0];
//    auto DP12 = controlPoints[2] - controlPoints[1];
//    auto DP23 = controlPoints[3] - controlPoints[2];
//
//    glm::vec3 tangent = 3.0f * negT * negT * DP01 + 6.0f * negT * tN * DP12 + 3 * tN * tN * DP23;

    // De-Casteljau algorithm
    auto P01 = glm::mix(controlPoints[0], controlPoints[1], tN);
    auto P02 = glm::mix(controlPoints[1], controlPoints[2], tN);
    auto P03 = glm::mix(controlPoints[2], controlPoints[3], tN);
    auto P11 = glm::mix(P01, P02, tN);
    auto P12 = glm::mix(P02, P03, tN);

    auto dt = 3.0f * (P12 - P11);

    return dt;
}

glm::vec3 BezierCurve::curvature(const float t) const {
    assert(minT <= t && t <= maxT);

    //    glm::vec3 tangent = 6.0f * negT * (controlPoints[2] - 2.0f * controlPoints[1] + controlPoints[0])
    //         + 6.0f * tN * (controlPoints[3] - 2.0f * controlPoints[2] + controlPoints[1]);

    return glm::vec3(0);
}

float BezierCurve::evalArcLength(const float _minT, const float _maxT, const uint16_t numSteps) const {
    assert(minT <= _minT && _minT <= maxT);
    assert(minT <= _maxT && _maxT <= maxT);
    assert(_minT <= _maxT);

    // Normalize min and max borders
//    float minTN = normalizeT(_minT);
//    float maxTN = normalizeT(_maxT);
    float minTN = _minT;
    float maxTN = _maxT;

    // Trapezoidal rule for integration
    float h = (maxTN - minTN) / float(numSteps - 1);

    float L = 0.0f;

    for (auto i = 0; i < numSteps; ++i) {
        // Avoid overflow due to numerical error
        float curT = std::min(minTN + i * h, maxTN);
        float segmentL = glm::length(derivative(curT));

        if (i > 0 || i < numSteps - 1) {
            segmentL *= 2.0f;
        }

        L += segmentL;
    }

    L *= (h / 2.0f);
    return L;
}


// See Eberly Paper: Moving along curve with constant speed
float BezierCurve::solveTForArcLength(const float _arcLength) const {
    assert(_arcLength <= totalArcLength && _arcLength >= 0.0);

    // Initial guess
    float t = minT + _arcLength / totalArcLength * (maxT - minT);

    float delta = 1E-5;
    // We use bisection to get closer to our solution
    float lower = minT;
    float upper = maxT;

    const uint32_t numIterations = 20;
    // Use Newton-Raphson algorithm to find t at arc length along each curve
    for (uint32_t i = 0; i < numIterations; ++i) {
        float C = evalArcLength(minT, t, 20) - _arcLength;

        // Early termination if t was found
        if (std::abs(C) <= delta) {
            return t;
        }

        // Derivative should be >= 0
        float dCdt = glm::length(derivative(t));

        // Newton-Raphson step
        float tCandidate = t - C / dCdt;

        // Bisection
        if (C > 0) {
            upper = t;
            if (tCandidate <= lower) { t = (upper + lower) / 2.0f; }
            else { t = tCandidate; }
        } else {
            lower = t;
            if (tCandidate >= upper) { t = (upper + lower) / 2.0f; }
            else { t = tCandidate; }
        }
    }

    return t;
}
