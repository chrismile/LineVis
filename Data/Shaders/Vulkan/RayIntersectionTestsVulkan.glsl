/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Christoph Neuhauser
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

#define SQR(x) ((x)*(x))

float squareVec(vec3 v) {
    return SQR(v.x) + SQR(v.y) + SQR(v.z);
}

/**
 * Implementation of ray-sphere intersection (idea from A. Glassner et al., "An Introduction to Ray Tracing").
 * For more details see: https://www.siggraph.org//education/materials/HyperGraph/raytrace/rtinter1.htm
 */
bool raySphereIntersection(vec3 rayOrigin, vec3 rayDirection, vec3 sphereCenter, float sphereRadius, out float hitT) {
    float A = SQR(rayDirection.x) + SQR(rayDirection.y) + SQR(rayDirection.z);
    float B = 2.0 * (
            rayDirection.x * (rayOrigin.x - sphereCenter.x)
            + rayDirection.y * (rayOrigin.y - sphereCenter.y)
            + rayDirection.z * (rayOrigin.z - sphereCenter.z)
    );
    float C =
            SQR(rayOrigin.x - sphereCenter.x)
            + SQR(rayOrigin.y - sphereCenter.y)
            + SQR(rayOrigin.z - sphereCenter.z)
            - SQR(sphereRadius);

    float discriminant = SQR(B) - 4.0*A*C;
    if (discriminant < 0.0) {
        return false; // No intersection
    }

    float discriminantSqrt = sqrt(discriminant);
    float t0 = (-B - discriminantSqrt) / (2.0 * A);
    float t1 = (-B + discriminantSqrt) / (2.0 * A);

    // Intersection(s) behind the ray origin?
    hitT = t0;
    if (t0 >= 0.0) {
        hitT = t0;
    } else if (t1 >= 0.0) {
        hitT = t1;
    } else {
        return false;
    }

    return true;
}

/**
 * Implementation of ray-tube intersection (idea from "Advanced Rendering" lecture by Denis Zorin,
 * NYU Media Research Lab). For more details see: https://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf
 */
bool rayTubeIntersection(
        vec3 rayOrigin, vec3 rayDirection, vec3 tubeStart, vec3 tubeEnd, float tubeRadius, out float hitT) {
    vec3 tubeDirection = normalize(tubeEnd - tubeStart);

    vec3 deltaP = rayOrigin - tubeStart;
    float A = squareVec(rayDirection - dot(rayDirection, tubeDirection) * tubeDirection);
    float B = 2.0 * dot(rayDirection - dot(rayDirection, tubeDirection) * tubeDirection,
            deltaP - dot(deltaP, tubeDirection) * tubeDirection);
    float C = squareVec(deltaP - dot(deltaP, tubeDirection) * tubeDirection) - SQR(tubeRadius);

    float discriminant = SQR(B) - 4.0*A*C;
    if (discriminant < 0.0) {
        return false; // No intersection.
    }

    float discriminantSqrt = sqrt(discriminant);
    float t0 = (-B - discriminantSqrt) / (2.0 * A);

    // Intersection(s) behind the ray origin?
    if (t0 >= 0.0) {
        vec3 intersectionPosition = rayOrigin + t0 * rayDirection;
        if (dot(tubeDirection, intersectionPosition - tubeStart) > 0
                && dot(tubeDirection, intersectionPosition - tubeEnd) < 0) {
            // Inside of finite cylinder.
            hitT = t0;
            return true;
        }
    }

    float t1 = (-B + discriminantSqrt) / (2.0 * A);
    if (t1 >= 0.0) {
        vec3 intersectionPosition = rayOrigin + t1 * rayDirection;
        if (dot(tubeDirection, intersectionPosition - tubeStart) > 0
                && dot(tubeDirection, intersectionPosition - tubeEnd) < 0) {
            // Inside of finite cylinder.
            hitT = t1;
            return true;
        }
    }

    return false;
}
