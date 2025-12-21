#ifdef VISUALIZE_BVH_HIERARCHY
struct NodeAabb {
    vec3 worldSpaceAabbMin;
    float normalizedHierarchyLevel;
    vec3 worldSpaceAabbMax;
    uint passIdx;
};
layout(std430, binding = 15) coherent buffer NodeAabbCountBuffer {
    uint numNodeAabbs;
};
layout(std430, binding = 16) writeonly buffer NodeAabbBuffer {
    NodeAabb nodeAabbs[];
};
#endif
