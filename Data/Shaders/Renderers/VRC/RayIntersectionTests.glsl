#ifndef RAY_INTERSECTION_TESTS_GLSL
#define RAY_INTERSECTION_TESTS_GLSL

#define SQR(x) ((x)*(x))
#define BIAS 1e-3

float squareVec(vec3 v) {
    return SQR(v.x) + SQR(v.y) + SQR(v.z);
}

/**
 * Helper function for rayBoxIntersection (see below).
 */
bool rayBoxPlaneIntersection(
        float rayOriginX, float rayDirectionX, float lowerX, float upperX, inout float tNear, inout float tFar) {
    if (abs(rayDirectionX) < BIAS) {
        // Ray is parallel to the x planes
        if (rayOriginX < lowerX || rayOriginX > upperX) {
            return false;
        }
    } else {
        // Not parallel to the x planes. Compute the intersection distance to the planes.
        float t0 = (lowerX - rayOriginX) / rayDirectionX;
        float t1 = (upperX - rayOriginX) / rayDirectionX;
        if (t0 > t1) {
            // Since t0 intersection with near plane
            float tmp = t0;
            t0 = t1;
            t1 = tmp;
        }

        if (t0 > tNear) {
            // We want the largest tNear
            tNear = t0;
        }
        if (t1 < tFar) {
            // We want the smallest tFar
            tFar = t1;
        }
        if (tNear > tFar) {
            // Box is missed
            return false;
        }
        if (tFar < 0) {
            // Box is behind ray
            return false;
        }
    }
    return true;
}

/**
 * Implementation of ray-box intersection (idea from A. Glassner et al., "An Introduction to Ray Tracing").
 * For more details see: https://www.siggraph.org//education/materials/HyperGraph/raytrace/rtinter3.htm
 */
bool rayBoxIntersection(
        vec3 rayOrigin, vec3 rayDirection, vec3 lower, vec3 upper, out vec3 entrancePoint, out vec3 exitPoint) {
    float tNear = -1e7, tFar = 1e7; // Infinite values
    for (int i = 0; i < 3; i++) {
        if (!rayBoxPlaneIntersection(rayOrigin[i], rayDirection[i], lower[i], upper[i], tNear, tFar)) {
            return false;
        }
    }

    entrancePoint = rayOrigin + tNear * rayDirection;
    exitPoint = rayOrigin + tFar * rayDirection;
    return true;
}


/**
 * Implementation of ray-box intersection (idea from A. Glassner et al., "An Introduction to Ray Tracing").
 * For more details see: https://www.siggraph.org//education/materials/HyperGraph/raytrace/rtinter3.htm
 */
bool rayBoxIntersectionRayCoords(
        vec3 rayOrigin, vec3 rayDirection, vec3 lower, vec3 upper, out float tNearOut, out float tFarOut) {
    float tNear = -1e7, tFar = 1e7; // Infinite values
    for (int i = 0; i < 3; i++) {
        if (!rayBoxPlaneIntersection(rayOrigin[i], rayDirection[i], lower[i], upper[i], tNear, tFar)) {
            return false;
        }
    }

    tNearOut = tNear;
    tFarOut = tFar;
    return true;
}


/**
 * Returns whether the bounding box with bounds lower, upper contains the passed point.
 */
bool boxContainsPoint(vec3 point, vec3 lower, vec3 upper) {
    return (point.x >= lower.x) && (point.y >= lower.y) && (point.z >= lower.z)
    && (point.x <= upper.x) && (point.y <= upper.y) && (point.z <= upper.z);
}


/**
 * Implementation of ray-sphere intersection (idea from A. Glassner et al., "An Introduction to Ray Tracing").
 * For more details see: https://www.siggraph.org//education/materials/HyperGraph/raytrace/rtinter1.htm
 */
bool raySphereIntersection(
        vec3 rayOrigin, vec3 rayDirection, vec3 sphereCenter, float sphereRadius,
        out vec3 intersectionPosition, in vec3 centerVoxelPosMin, in vec3 centerVoxelPosMax) {
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
        return false; // No intersection.
    }

    float discriminantSqrt = sqrt(discriminant);
    float t0 = (-B - discriminantSqrt) / (2.0 * A);
    // Intersection(s) behind the ray origin?
    intersectionPosition = rayOrigin + t0 * rayDirection;
    if (t0 >= 0.0
            && all(greaterThanEqual(intersectionPosition, centerVoxelPosMin))
            && all(lessThanEqual(intersectionPosition, centerVoxelPosMax))) {
        return true;
    }/* else {
        float t1 = (-B + discriminantSqrt) / (2.0 * A);
        intersectionPosition = rayOrigin + t1 * rayDirection;
        if (t1 >= 0.0
                && all(greaterThanEqual(intersectionPosition, centerVoxelPosMin))
                && all(lessThanEqual(intersectionPosition, centerVoxelPosMax))) {
            return true;
        }
    }*/

    return false;
}


/**
 * Implementation of ray-tube intersection (idea from "Advanced Rendering" lecture by Denis Zorin,
 * NYU Media Research Lab). For more details see: https://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf
 */
bool rayTubeIntersection(
        vec3 rayOrigin, vec3 rayDirection, vec3 tubeStart, vec3 tubeEnd, float tubeRadius,
        out vec3 intersectionPosition, in vec3 centerVoxelPosMin, in vec3 centerVoxelPosMax) {
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
        intersectionPosition = rayOrigin + t0 * rayDirection;
        if (dot(tubeDirection, intersectionPosition - tubeStart) > 0
                && dot(tubeDirection, intersectionPosition - tubeEnd) < 0) {
            // Inside of finite cylinder
            if (all(greaterThanEqual(intersectionPosition, centerVoxelPosMin))
                    && all(lessThanEqual(intersectionPosition, centerVoxelPosMax))) {
                return true;
            }
        }
    }

    /*float t1 = (-B + discriminantSqrt) / (2.0 * A);
    if (t1 >= 0.0) {
        intersectionPosition = rayOrigin + t1 * rayDirection;
        if (dot(tubeDirection, intersectionPosition - tubeStart) > 0
                && dot(tubeDirection, intersectionPosition - tubeEnd) < 0
        #if defined(FAST_NEIGHBOR_SEARCH) || !defined(VOXEL_RAY_CASTING_FAST)
                && all(greaterThanEqual(intersectionPosition, centerVoxelPosMin))
                && all(lessThanEqual(intersectionPosition, centerVoxelPosMax))
        #endif
        ) {
            // Inside of finite cylinder
            return true; // TODO
        }
    }*/

    return false;
}

/**
 * Implementation of ray-tube intersection (idea from "Advanced Rendering" lecture by Denis Zorin,
 * NYU Media Research Lab). For more details see: https://mrl.nyu.edu/~dzorin/rend05/lecture2.pdf
 */
bool rayTubeInfIntersection(
        vec3 rayOrigin, vec3 rayDirection, vec3 tubeStart, vec3 tubeEnd, float tubeRadius,
        out vec3 intersectionPosition) {
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
        intersectionPosition = rayOrigin + t0 * rayDirection;
        return true;
    }

    float t1 = (-B + discriminantSqrt) / (2.0 * A);
    if (t1 >= 0.0) {
        intersectionPosition = rayOrigin + t1 * rayDirection;
        //return true;
    }

    return false;
}

#endif
