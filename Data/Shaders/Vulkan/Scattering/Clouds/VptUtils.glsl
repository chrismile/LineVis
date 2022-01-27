/**
 * MIT License
 *
 * Copyright (c) 2021-2022, Christoph Neuhauser, Ludwig Leonard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

//--- Constants

const float PI = 3.14159265359;
const float TWO_PI = 2*3.14159265359;


//--- Structs

struct ScatterEvent {
    bool hasValue;
    vec3 x; float pdf_x;
    vec3 w; float pdf_w;
};


//--- Random Number Generator (Hybrid Taus)

uvec4 rngState = uvec4(0);
uint tausStep(uint z, int S1, int S2, int S3, uint M) { uint b = (((z << S1) ^ z) >> S2); return ((z & M) << S3) ^ b; }
uint lcgStep(uint z, uint A, uint C) { return A * z + C; }

float random() {
    rngState.x = tausStep(rngState.x, 13, 19, 12, 4294967294);
    rngState.y = tausStep(rngState.y, 2, 25, 4, 4294967288);
    rngState.z = tausStep(rngState.z, 3, 11, 17, 4294967280);
    rngState.w = lcgStep(rngState.w, 1664525, 1013904223);
    return 2.3283064365387e-10 * (rngState.x ^ rngState.y ^ rngState.z ^ rngState.w);
}

void initializeRandom(uint seed) {
    rngState = uvec4(seed);
    for (int i = 0; i < seed % 7 + 2; i++) {
        random();
    }
}

void createOrthonormalBasis(vec3 D, out vec3 B, out vec3 T) {
    vec3 other = abs(D.z) >= 0.9999 ? vec3(1, 0, 0) : vec3(0, 0, 1);
    B = normalize(cross(other, D));
    T = normalize(cross(D, B));
}

vec3 randomDirection(vec3 D) {
    float r1 = random();
    float r2 = random() * 2 - 1;
    float sqrR2 = r2 * r2;
    float two_pi_by_r1 = TWO_PI * r1;
    float sqrt_of_one_minus_sqrR2 = sqrt(1.0 - sqrR2);
    float x = cos(two_pi_by_r1) * sqrt_of_one_minus_sqrR2;
    float y = sin(two_pi_by_r1) * sqrt_of_one_minus_sqrR2;
    float z = r2;

    vec3 t0, t1;
    createOrthonormalBasis(D, t0, t1);

    return t0 * x + t1 * y + D * z;
}


//--- Scattering functions

#define oneMinusG2 (1.0 - (GFactor) * (GFactor))
#define onePlusG2 (1.0 + (GFactor) * (GFactor))
#define oneOver2G (0.5 / (GFactor))

float invertcdf(float GFactor, float xi) {
    float t = (oneMinusG2) / (1.0f - GFactor + 2.0f * GFactor * xi);
    return oneOver2G * (onePlusG2 - t * t);
}

vec3 importanceSamplePhase(float GFactor, vec3 D, out float pdf) {
    if (abs(GFactor) < 0.001) {
        pdf = 1.0 / (4 * PI);
        return randomDirection(-D);
    }

    float phi = random() * 2 * PI;
    float cosTheta = invertcdf(GFactor, random());
    float sinTheta = sqrt(max(0, 1.0f - cosTheta * cosTheta));

    vec3 t0, t1;
    createOrthonormalBasis(D, t0, t1);

    pdf = 0.25 / PI * (oneMinusG2) / pow(onePlusG2 - 2 * GFactor * cosTheta, 1.5);

    return sinTheta * sin(phi) * t0 + sinTheta * cos(phi) * t1 + cosTheta * D;
}


//--- Tools

/**
 * Converts linear RGB to sRGB.
 * For more details see: https://en.wikipedia.org/wiki/SRGB
 */
vec3 linearRGBTosRGB(in vec3 sRGB) {
    return mix(1.055 * pow(sRGB, vec3(1.0 / 2.4)) - 0.055, sRGB * 12.92, lessThanEqual(sRGB, vec3(0.0031308)));
}

/**
 * Converts sRGB to linear RGB.
 * For more details see: https://en.wikipedia.org/wiki/SRGB
 */
vec3 sRGBToLinearRGB(in vec3 linearRGB) {
    return mix(pow((linearRGB + 0.055) / 1.055, vec3(2.4)), linearRGB / 12.92, lessThanEqual(linearRGB, vec3(0.04045)));
}


#ifdef USE_ENVIRONMENT_MAP_IMAGE
vec3 sampleSkybox(in vec3 dir) {
    // Sample from equirectangular projection.
    vec2 texcoord = vec2(atan(dir.z, dir.x) / TWO_PI + 0.5, -asin(dir.y) / PI + 0.5);
    vec3 textureColor = texture(environmentMapTexture, texcoord).rgb;
#ifdef USE_LINEAR_RGB
    return parameters.environmentMapIntensityFactor * sRGBToLinearRGB(textureColor);
#else
    return parameters.environmentMapIntensityFactor * textureColor;
#endif
}
vec3 sampleLight(in vec3 dir) {
    return vec3(0.0);
}
#else
vec3 sampleSkybox(in vec3 dir) {
    vec3 L = dir;

    vec3 BG_COLORS[5] = {
            vec3(0.1, 0.05, 0.01), // GROUND DARKER BLUE
            vec3(0.01, 0.05, 0.2), // HORIZON GROUND DARK BLUE
            vec3(0.8, 0.9, 1.0), // HORIZON SKY WHITE
            vec3(0.1, 0.3, 1.0),  // SKY LIGHT BLUE
            vec3(0.01, 0.1, 0.7)  // SKY BLUE
    };

    float BG_DISTS[5] = {
            -1.0,
            -0.1,
            0.0,
            0.4,
            1.0
    };

    vec3 col = BG_COLORS[0];
    col = mix(col, BG_COLORS[1], vec3(smoothstep(BG_DISTS[0], BG_DISTS[1], L.y)));
    col = mix(col, BG_COLORS[2], vec3(smoothstep(BG_DISTS[1], BG_DISTS[2], L.y)));
    col = mix(col, BG_COLORS[3], vec3(smoothstep(BG_DISTS[2], BG_DISTS[3], L.y)));
    col = mix(col, BG_COLORS[4], vec3(smoothstep(BG_DISTS[3], BG_DISTS[4], L.y)));

#ifdef USE_LINEAR_RGB
    return sRGBToLinearRGB(col);
#else
    return col;
#endif
}
vec3 sampleLight(in vec3 dir) {
    int N = 10;
    float phongNorm = (N + 2) / (2 * 3.14159);
    return parameters.sunIntensity * pow(max(0, dot(dir, parameters.sunDirection)), N) * phongNorm;
}
#endif

float sampleCloud(in vec3 pos) {
    ivec3 dim = textureSize(gridImage, 0);
    vec3 coord = (pos - parameters.boxMin) / (parameters.boxMax - parameters.boxMin);
    coord += vec3(random() - 0.5, random() - 0.5, random() - 0.5) / dim;
    return texture(gridImage, coord).x;
}

void createCameraRay(in vec2 coord, out vec3 x, out vec3 w) {
    vec4 ndcP = vec4(coord, 0, 1);
    vec4 ndcT = ndcP + vec4(0, 0, 1, 0);

    vec4 viewP = parameters.inverseViewProjMatrix * ndcP;
    viewP.xyz /= viewP.w;
    vec4 viewT = parameters.inverseViewProjMatrix * ndcT;
    viewT.xyz /= viewT.w;

    x = viewP.xyz;
    w = normalize(viewT.xyz - viewP.xyz);
}

bool rayBoxIntersect(vec3 bMin, vec3 bMax, vec3 P, vec3 D, out float tMin, out float tMax) {
    // Un-parallelize D.
    D.x = abs(D).x <= 0.000001 ? 0.000001 : D.x;
    D.y = abs(D).y <= 0.000001 ? 0.000001 : D.y;
    D.z = abs(D).z <= 0.000001 ? 0.000001 : D.z;
    vec3 C_Min = (bMin - P)/D;
    vec3 C_Max = (bMax - P)/D;
    tMin = max(max(min(C_Min[0], C_Max[0]), min(C_Min[1], C_Max[1])), min(C_Min[2], C_Max[2]));
    tMin = max(0.0, tMin);
    tMax = min(min(max(C_Min[0], C_Max[0]), max(C_Min[1], C_Max[1])), max(C_Min[2], C_Max[2]));
    if (tMax <= tMin || tMax <= 0) {
        return false;
    }
    return true;
}

float maxComponent(vec3 v) {
    return max(v.x, max(v.y, v.z));
}
