// Binding 0-7 is reserved for the individual rendering modes.
#define LINE_UNIFORM_DATA_BUFFER_BINDING 8
#define SPECIALIZATION_LINE_UNIFORM_DATA_BUFFER_BINDING 9
#define LINE_POINTS_BUFFER_BINDING 10 // for GeometryPassNormal.glsl
#define LINE_HIERARCHY_LEVELS_BUFFER_BINDING 11 // for GeometryPassNormal.glsl
#define DEPTH_MIN_MAX_BUFFER_BINDING 12
#define AMBIENT_OCCLUSION_TEXTURE_BINDING 13 // for AmbientOcclusion.glsl
#define AMBIENT_OCCLUSION_FACTORS_BUFFER_BINDING 13 // for AmbientOcclusion.glsl
#define AMBIENT_OCCLUSION_BLENDING_WEIGHTS_BUFFER_BINDING 14 // for AmbientOcclusion.glsl
#define MIN_MAX_BUFFER_BINDING 15 // for TransferFunction.glsl
#define TRANSFER_FUNCTION_TEXTURE_BINDING 16 // for TransferFunction.glsl
#define LINE_HIERARCHY_IMPORTANCE_MAP_BINDING 17

layout(binding = LINE_UNIFORM_DATA_BUFFER_BINDING) uniform LineUniformDataBuffer {
    // Camera data.
    vec3 cameraPosition;
    float fieldOfViewY;
    mat4 viewMatrix;
    mat4 projectionMatrix;
    mat4 inverseViewMatrix;
    mat4 inverseProjectionMatrix;
    vec4 backgroundColor;
    vec4 foregroundColor;

    // Line & band render settings.
    float lineWidth;
    float bandWidth;
    float minBandThickness;
    float depthCueStrength;
    float ambientOcclusionStrength;
    float ambientOcclusionGamma;
    float lineUniformDataPadding0, lineUniformDataPadding1;

    // Pre-baked ambient occlusion settings (STATIC_AMBIENT_OCCLUSION_PREBAKING).
    uint numAoTubeSubdivisions;
    uint numLineVertices;
    uint numParametrizationVertices;
    uint lineUniformDataPadding2;

    // Hull render settings.
    vec4 hullColor;
    uint hasHullMesh;
    uint hullUseShading;

    // Antialiasing.glsl needs a viewport size. Change this value if downsampling/upscaling is used!
    uvec2 viewportSize;
};

#ifdef STRESS_LINE_DATA
layout(binding = SPECIALIZATION_LINE_UNIFORM_DATA_BUFFER_BINDING) uniform StressLineUniformDataBuffer {
    vec3 lineHierarchySlider; // USE_LINE_HIERARCHY_LEVEL && !USE_TRANSPARENCY
    float paddingStressLineSettings;
    ivec3 psUseBands;
    int currentSeedIdx; // VISUALIZE_SEEDING_PROCESS
};
#endif

