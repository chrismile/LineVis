struct LinePointData {
    vec3 vertexPosition;
    float vertexAttribute;
    vec3 vertexTangent;
    float vertexRotation;
    vec3 vertexNormal;
    uint lineStartIndex;
};
layout(std430, binding = LINE_POINTS_BUFFER_BINDING) readonly buffer LinePointDataBuffer {
    LinePointData linePoints[];
};

#if defined(USE_PRINCIPAL_STRESS_DIRECTION_INDEX) || defined(USE_LINE_HIERARCHY_LEVEL) || defined(VISUALIZE_SEEDING_PROCESS)
struct StressLinePointData {
    uint vertexPrincipalStressIndex;
    uint vertexLineAppearanceOrder;
    float vertexLineHierarchyLevel;
    float stressLinePointPadding;
};
layout(std430, binding = STRESS_LINE_POINTS_BUFFER_BINDING) readonly buffer StressLinePointDataBuffer {
    StressLinePointData stressLinePoints[];
};
#endif

#ifdef USE_PRINCIPAL_STRESSES
struct StressLinePointPrincipalStressData {
    float vertexMajorStress;
    float vertexMediumStress;
    float vertexMinorStress;
    float principalStressPadding;
};
layout(std430, binding = STRESS_LINE_POINTS_PRINCIPAL_STRESS_BUFFER_BINDING) readonly buffer StressLinePointPrincipalStressDataBuffer {
    StressLinePointPrincipalStressData principalStressLinePoints[];
};
#endif
