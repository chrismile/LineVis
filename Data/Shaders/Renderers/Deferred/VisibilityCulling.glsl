
layout(binding = 6) uniform sampler2D depthBuffer;

layout(binding = 7) uniform VisibilityCullingUniformBuffer {
    mat4 modelViewProjectionMatrix;
    ivec2 viewportSize;
    uint numMeshlets; // only for linear meshlet list.
    uint rootNodeIdx; // only for meshlet node tree.
};

/*
 * For more details see: https://dev.theomader.com/transform-bounding-boxes/
 */
/*void aabbWorldSpaceToNdc(in mat4 T, in vec4 worldMin, in vec4 worldMax, out vec4 ndcMin, out vec4 ndcMax) {
    vec4 xa = T[0] * worldMin.x;
    vec4 xb = T[0] * worldMax.x;
    vec4 ya = T[1] * worldMin.y;
    vec4 yb = T[1] * worldMax.y;
    vec4 za = T[2] * worldMin.z;
    vec4 zb = T[2] * worldMax.z;
    ndcMin = min(xa, xb) + min(ya, yb) + min(za, zb) + T[3];
    ndcMax = max(xa, xb) + max(ya, yb) + max(za, zb) + T[3];
}*/

/*
 * Use hierarchical Z (Hi-Z) occlusion culling using an hierarchical Z buffer (HZB). For more details see:
 * https://www.rastergrid.com/blog/2010/10/hierarchical-z-map-based-occlusion-culling/
 * https://frostbite-wp-prd.s3.amazonaws.com/wp-content/uploads/2016/03/29204330/GDC_2016_Compute.pdf
 * Returns true if the object is visible and false otherwise.
 */
bool occlusionCulling(vec2 bbNdcMin, vec2 bbNdcMax, float bbNdcMinDepth) {
    const float EPSILON = 1e-5;
    float viewSizeX = (bbNdcMax.x - bbNdcMin.x) * float(viewportSize.x);
    float viewSizeY = (bbNdcMax.y - bbNdcMin.y) * float(viewportSize.y);
    float longestEdge = max(viewSizeX, viewSizeY);
#define USE_2X2_DEPTH_CULLING
#ifdef USE_2X2_DEPTH_CULLING
    float lodLevel = ceil(log2(longestEdge / 2.0));
    float depth0 = textureLod(depthBuffer, vec2(bbNdcMin.x, bbNdcMin.y), lodLevel).x;
    float depth1 = textureLod(depthBuffer, vec2(bbNdcMax.x, bbNdcMin.y), lodLevel).x;
    float depth2 = textureLod(depthBuffer, vec2(bbNdcMin.x, bbNdcMax.y), lodLevel).x;
    float depth3 = textureLod(depthBuffer, vec2(bbNdcMax.x, bbNdcMax.y), lodLevel).x;
    float maxDepth = max(max(depth0, depth1), max(depth2, depth3));
    return bbNdcMinDepth - EPSILON <= maxDepth;
#else
    float lodLevel = ceil(log2(max(longestEdge, 1.0))).x;
    float maxOcclusionDepth = textureLod(depthBuffer, (vec2(bbNdcMin.x, bbNdcMin.y) + vec2(bbNdcMax.x, bbNdcMax.y)) * 0.5, lodLevel).x;
    return bbNdcMinDepth - EPSILON <= maxOcclusionDepth;
#endif
}

// Checks whether the clip space point lies outside of the 6 view frustum planes.
void incrementFrustumCounters(
        vec4 bbClipSpacePoint, inout int ctr0, inout int ctr1, inout int ctr2, inout int ctr3, inout int ctr4,
        inout int ctr5) {
    if (bbClipSpacePoint.x >  bbClipSpacePoint.w) ctr0++;
    if (bbClipSpacePoint.x < -bbClipSpacePoint.w) ctr1++;
    if (bbClipSpacePoint.y >  bbClipSpacePoint.w) ctr2++;
    if (bbClipSpacePoint.y < -bbClipSpacePoint.w) ctr3++;
    if (bbClipSpacePoint.z >  bbClipSpacePoint.w) ctr4++;
    if (bbClipSpacePoint.z < -bbClipSpacePoint.w) ctr5++;
}

/*
 * Returns true if the node is visible and false otherwise.
 */
bool visibilityCulling(vec3 worldSpaceAabbMin, vec3 worldSpaceAabbMax) {
    // Unroll transformed bounding box points to avoid array stored in local memory.
    // Compute the bounding box points in clip space.
    vec4 bb0 = modelViewProjectionMatrix * vec4(worldSpaceAabbMin, 1.0);
    vec4 bb1 = modelViewProjectionMatrix * vec4(vec3(worldSpaceAabbMax.x, worldSpaceAabbMin.y, worldSpaceAabbMin.z), 1.0);
    vec4 bb2 = modelViewProjectionMatrix * vec4(vec3(worldSpaceAabbMin.x, worldSpaceAabbMax.y, worldSpaceAabbMin.z), 1.0);
    vec4 bb3 = modelViewProjectionMatrix * vec4(vec3(worldSpaceAabbMax.x, worldSpaceAabbMax.y, worldSpaceAabbMin.z), 1.0);
    vec4 bb4 = modelViewProjectionMatrix * vec4(vec3(worldSpaceAabbMin.x, worldSpaceAabbMin.y, worldSpaceAabbMax.z), 1.0);
    vec4 bb5 = modelViewProjectionMatrix * vec4(vec3(worldSpaceAabbMax.x, worldSpaceAabbMin.y, worldSpaceAabbMax.z), 1.0);
    vec4 bb6 = modelViewProjectionMatrix * vec4(vec3(worldSpaceAabbMin.x, worldSpaceAabbMax.y, worldSpaceAabbMax.z), 1.0);
    vec4 bb7 = modelViewProjectionMatrix * vec4(worldSpaceAabbMax, 1.0);

    // Check whether each clip space point lies beyond view frustum plane 0 to 5.
    int ctr0 = 0;
    int ctr1 = 0;
    int ctr2 = 0;
    int ctr3 = 0;
    int ctr4 = 0;
    int ctr5 = 0;
    incrementFrustumCounters(bb0, ctr0, ctr1, ctr2, ctr3, ctr4, ctr5);
    incrementFrustumCounters(bb1, ctr0, ctr1, ctr2, ctr3, ctr4, ctr5);
    incrementFrustumCounters(bb2, ctr0, ctr1, ctr2, ctr3, ctr4, ctr5);
    incrementFrustumCounters(bb3, ctr0, ctr1, ctr2, ctr3, ctr4, ctr5);
    incrementFrustumCounters(bb4, ctr0, ctr1, ctr2, ctr3, ctr4, ctr5);
    incrementFrustumCounters(bb5, ctr0, ctr1, ctr2, ctr3, ctr4, ctr5);
    incrementFrustumCounters(bb6, ctr0, ctr1, ctr2, ctr3, ctr4, ctr5);
    incrementFrustumCounters(bb7, ctr0, ctr1, ctr2, ctr3, ctr4, ctr5);

    // Do view frustum culling check - do all points lie past one view frustum plane?
    if (ctr0 == 8 || ctr1 == 8 || ctr2 == 8 || ctr3 == 8 || ctr4 == 8 || ctr5 == 8) {
        return false;
    }

    // Convert from clip space to NDC space.
    bb0.xyz /= bb0.w;
    bb1.xyz /= bb1.w;
    bb2.xyz /= bb2.w;
    bb3.xyz /= bb3.w;
    bb4.xyz /= bb4.w;
    bb5.xyz /= bb5.w;
    bb6.xyz /= bb6.w;
    bb7.xyz /= bb7.w;

    // Convert the clip space bounding box points to a NDC bounding box.
    vec2 bbNdcMin, bbNdcMax;
    bbNdcMin.x = min(min(min(bb0.x, bb1.x), min(bb2.x, bb3.x)), min(min(bb4.x, bb5.x), min(bb6.x, bb7.x))) / 2.0 + 0.5;
    bbNdcMin.y = min(min(min(bb0.y, bb1.y), min(bb2.y, bb3.y)), min(min(bb4.y, bb5.y), min(bb6.y, bb7.y))) / 2.0 + 0.5;
    bbNdcMax.x = max(max(max(bb0.x, bb1.x), max(bb2.x, bb3.x)), max(max(bb4.x, bb5.x), max(bb6.x, bb7.x))) / 2.0 + 0.5;
    bbNdcMax.y = max(max(max(bb0.y, bb1.y), max(bb2.y, bb3.y)), max(max(bb4.y, bb5.y), max(bb6.y, bb7.y))) / 2.0 + 0.5;
    float bbNdcMinDepth = min(min(min(bb0.z, bb1.z), min(bb2.z, bb3.z)), min(min(bb4.z, bb5.z), min(bb6.z, bb7.z)));

    // Do occlusion culling check using Hi-Z buffer (HZB).
    return occlusionCulling(bbNdcMin, bbNdcMax, bbNdcMinDepth);
}
