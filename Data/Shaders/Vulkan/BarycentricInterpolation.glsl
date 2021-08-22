hitAttributeEXT vec2 attribs;

float interpolateFloat(float v0, float v1, float v2, vec3 barycentricCoordinates) {
    return v0 * barycentricCoordinates.x + v1 * barycentricCoordinates.y + v2 * barycentricCoordinates.z;
}
vec3 interpolateVec3(vec3 v0, vec3 v1, vec3 v2, vec3 barycentricCoordinates) {
    return v0 * barycentricCoordinates.x + v1 * barycentricCoordinates.y + v2 * barycentricCoordinates.z;
}
