hitAttributeEXT vec2 attribs;

#define M_PI 3.14159265358979323846

float interpolateFloat(float v0, float v1, float v2, vec3 barycentricCoordinates) {
    return v0 * barycentricCoordinates.x + v1 * barycentricCoordinates.y + v2 * barycentricCoordinates.z;
}

vec3 interpolateVec3(vec3 v0, vec3 v1, vec3 v2, vec3 barycentricCoordinates) {
    return v0 * barycentricCoordinates.x + v1 * barycentricCoordinates.y + v2 * barycentricCoordinates.z;
}

/// Assumes all angles are 0 <= phi <= 2pi
float interpolateAngle(float v0, float v1, float v2, vec3 barycentricCoordinates) {
    if (v1 - v0 > M_PI || v2 - v0 > M_PI) {
        v0 += 2.0 * M_PI;
    }
    if (v0 - v1 > M_PI || v2 - v1 > M_PI) {
        v1 += 2.0 * M_PI;
    }
    if (v0 - v2 > M_PI || v1 - v2 > M_PI) {
        v2 += 2.0 * M_PI;
    }
    return v0 * barycentricCoordinates.x + v1 * barycentricCoordinates.y + v2 * barycentricCoordinates.z;
}
