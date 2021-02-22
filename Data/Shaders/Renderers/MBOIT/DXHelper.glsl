#ifndef DX_HELPER_GLSL
#define DX_HELPER_GLSL

float rsqrt(float x) {
    return 1.0 / x;
}

float saturate(float x) {
    if (isinf(x)) x = 1.0;
    return clamp(x, 0.0, 1.0);
}

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
/**
 * The OpenGL implementation of atan changes the order of x and y (compared to DirectX),
 * and is unstable for x close to 0.
 *
 * For more details see:
 * https://stackoverflow.com/questions/26070410/robust-atany-x-on-glsl-for-converting-xy-coordinate-to-angle
 */
float atan2(in float y, in float x)
{
    bool s = (abs(x) > abs(y));
    return mix(PI/2.0 - atan(x,y), atan(y,x), s);
}

vec2 mul(vec2 v, mat2 m) {
    return m * v;
}

vec3 mul(vec3 v, mat3 m) {
    return m * v;
}

vec4 mul(vec4 v, mat4 m) {
    return m * v;
}

vec2 mul(mat2 m, vec2 v) {
    return v * m;
}

vec3 mul(mat3 m, vec3 v) {
    return v * m;
}

vec4 mul(mat4 m, vec4 v) {
    return v * m;
}

#endif