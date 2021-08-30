#ifndef VOXEL_DATA_GLSL
#define VOXEL_DATA_GLSL

//#define QUANTIZATION_RESOLUTION 8
//#define QUANTIZATION_RESOLUTION_LOG2 3
//#define gridResolution ivec3(256, 256, 256)
//#define TUBE_RADIUS 0.2

#define PACK_LINES

uniform float lineRadius = 0.2;

struct LineSegment {
    vec3 v1; // Vertex position
    float a1; // Vertex attribute
    vec3 v2; // Vertex position
    float a2; // Vertex attribute
    uint lineID;
};

// Works until quantization resolution of 64^2 (6 + 2 * 2log2(64) = 30)
struct LineSegmentCompressed {
    // Bit 0-2, 3-5: Face ID of start/end point.
    // For c = log2(QUANTIZATION_RESOLUTION^2) = 2*log2(QUANTIZATION_RESOLUTION):
    // Bit 6-(5+c), (6+c)-(5+2c): Quantized face position of start/end point.
    uint linePosition;
    // Bit 11-15: Line ID (5 bits for bitmask of 2^5 bits = 32 bits).
    // Bit 16-23, 24-31: Attribute of start/end point (normalized to [0,1]).
    uint attributes;
};

// Offset in the line segments buffer for each voxel.
layout (std430, binding = 0) readonly buffer VoxelLineListOffsetBuffer {
    uint voxelLineListOffsets[];
};

// Number of line segments in each voxel.
layout (std430, binding = 1) readonly buffer NumLinesBuffer {
    uint numLinesInVoxel[];
};

// Buffer containing all line segments
layout (std430, binding = 2) readonly buffer LineSegmentBuffer {
#ifdef PACK_LINES
    LineSegmentCompressed lineSegments[];
#else
    LineSegment lineSegments[];
#endif
};


// Density of voxels (with LODs)
uniform sampler3D densityTexture;

// Density of voxels (with LODs)
uniform usampler3D octreeTexture;


// --- Functions ---

vec3 getQuantizedPositionOffset(uint faceIndex, uint quantizedPos1D) {
    vec2 quantizedFacePosition = vec2(
    float(quantizedPos1D % QUANTIZATION_RESOLUTION),
    float(quantizedPos1D / QUANTIZATION_RESOLUTION))
    / float(QUANTIZATION_RESOLUTION);

    // Whether the face is the face in x/y/z direction with greater dimensions (offset factor)
    float face0or1 = float(faceIndex % 2);

    vec3 offset;
    if (faceIndex <= 1) {
        offset = vec3(face0or1, quantizedFacePosition.x, quantizedFacePosition.y);
    } else if (faceIndex <= 3) {
        offset = vec3(quantizedFacePosition.x, face0or1, quantizedFacePosition.y);
    } else if (faceIndex <= 5) {
        offset = vec3(quantizedFacePosition.x, quantizedFacePosition.y, face0or1);
    }
    return offset;
}


#ifdef PACK_LINES
void decompressLine(in vec3 voxelPosition, in LineSegmentCompressed compressedLine, out LineSegment decompressedLine) {
    const uint c = 2*QUANTIZATION_RESOLUTION_LOG2;
    const uint bitmaskQuantizedPos = QUANTIZATION_RESOLUTION*QUANTIZATION_RESOLUTION-1;
    uint faceStartIndex = compressedLine.linePosition & 0x7u;
    uint faceEndIndex = (compressedLine.linePosition >> 3) & 0x7u;
    uint quantizedStartPos1D = (compressedLine.linePosition >> 6) & bitmaskQuantizedPos;
    uint quantizedEndPos1D = (compressedLine.linePosition >> (6+c)) & bitmaskQuantizedPos;
    if (c > 12) {
        quantizedEndPos1D |= (compressedLine.attributes << (c - (6 + 2*c - 32))) & bitmaskQuantizedPos;
    }
    uint lineID = (compressedLine.attributes >> 11) & 31u;
    uint attr1 = (compressedLine.attributes >> 16) & 0xFFu;
    uint attr2 = (compressedLine.attributes >> 24) & 0xFFu;

    decompressedLine.v1 = voxelPosition + getQuantizedPositionOffset(faceStartIndex, quantizedStartPos1D);
    decompressedLine.v2 = voxelPosition + getQuantizedPositionOffset(faceEndIndex, quantizedEndPos1D);
    decompressedLine.a1 = float(attr1) / 255.0;
    decompressedLine.a2 = float(attr2) / 255.0;
    decompressedLine.lineID = lineID;
}
#endif

uint getVoxelIndex1D(ivec3 voxelIndex) {
    return uint(voxelIndex.x + voxelIndex.y * gridResolution.x + voxelIndex.z *gridResolution.x * gridResolution.y);
}

uint getNumLinesInVoxel(uint voxelIndex1D) {
    return min(numLinesInVoxel[voxelIndex1D], MAX_NUM_LINES_PER_VOXEL);
}

uint getLineListOffset(uint voxelIndex1D) {
    return voxelLineListOffsets[voxelIndex1D];
}

void loadLineInVoxel(vec3 voxelPosition, uint voxelLineListOffset, uint lineIndex, out LineSegment currVoxelLine) {
#ifdef PACK_LINES
    decompressLine(voxelPosition, lineSegments[voxelLineListOffset + lineIndex], currVoxelLine);
#else
    currVoxelLine = lineSegments[voxelLineListOffset + lineIndex];
#endif
}

/// Stores lines of voxel in global variables "currVoxelLines", "currVoxelNumLines"
/*void loadLinesInVoxel(ivec3 voxelIndex, out uint currVoxelNumLines,
        out LineSegment currVoxelLines[MAX_NUM_LINES_PER_VOXEL]) {
    int voxelIndex1D = voxelIndex.x + voxelIndex.y*gridResolution.x + voxelIndex.z*gridResolution.x*gridResolution.y;
    currVoxelNumLines = min(numLinesInVoxel[voxelIndex1D], MAX_NUM_LINES_PER_VOXEL);
    if (currVoxelNumLines <= 0) {
        return;
    }

    vec3 voxelPosition = vec3(voxelIndex);
    uint lineListOffset = voxelLineListOffsets[voxelIndex1D];
    for (uint i = 0; i < currVoxelNumLines && i < MAX_NUM_LINES_PER_VOXEL; i++) {
#ifdef PACK_LINES
        decompressLine(voxelPosition, lineSegments[lineListOffset+i], currVoxelLines[i]);
#else
        currVoxelLines[i] = lineSegments[lineListOffset+i];
#endif
    }
}*/

// Get density at specified lod index
float getVoxelDensity(vec3 coords, float lod) {
    return textureLod(densityTexture, coords / vec3(gridResolution), lod).r;
}

#endif
