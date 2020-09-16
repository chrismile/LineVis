/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UTILS_MESHSERIALIZER_HPP_
#define UTILS_MESHSERIALIZER_HPP_

#include <glm/glm.hpp>
#include <vector>
#include <set>

#include <Math/Geometry/AABB3.hpp>
#include <Math/Geometry/Sphere.hpp>
#include <Graphics/Shader/ShaderAttributes.hpp>

/**
 * Parsing text-based mesh files, like .obj files, is really slow compared to binary formats.
 * The utility functions below serialize 3D mesh data to a file/read the data back from such a file.
 */

 /**
  * A simple material definition. For now, no textures are supported.
  */
struct ObjMaterial
{
    ObjMaterial() : ambientColor(0.75f, 0.75f, 0.75f), diffuseColor(0.75f, 0.75f, 0.75f),
                    specularColor(0.0f, 0.0f, 0.0f), specularExponent(1.0f), opacity(1.0f) {}

    glm::vec3 ambientColor;
    glm::vec3 diffuseColor;
    glm::vec3 specularColor;
    float specularExponent;
    float opacity;
};

struct ObjSubmesh
{
    ObjMaterial material;
    std::vector<uint32_t> indices;
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec2> texcoords;
    std::vector<glm::vec3> normals;
};

struct ObjMesh
{
    std::vector<ObjSubmesh> submeshes;
};

/**
 * Binmesh Format: A mesh consists of one or many submeshes.
 * A submesh consists of:
 *  - A material encoding ambient color, specular exponent etc. for Phong Shading.
 *    This is ignored for e.g. the aneurism dataset, as color and opacity are computed using a transfer function.
 *  - A vertex mode (points, lines, triangles, ...).
 *  - A list of indices (optional, can be empty). 32-bit indices were chosen, as all scientific datasets have more than 2^16 vertices anyways.
 *  - A list of vertex attributes.
 *  - A list of uniform attributes.
 *
 * A vertex attribute could be e.g. the vertex position, normal, color, vorticity, ... and consists of:
 *  - A name (string), which is used as the binding point for the vertex shader.
 *  - The attribute format (e.g. byte, unsigned int, float, ...).
 *  - Die number of components, e.g. 3 for a vector containing three elements or 1 for a scalar value.
 *  - The actual attribute data as an array of bytes. It is expected that the number of vertices is the same for each attribute.
 *    The number of vertices can be explicitly computed by "data.size() / numComponents / dataFormatNumBytes".
 *
 * A uniform attribute is an attribute constant over all vertices.
 */

struct BinaryMeshAttribute
{
    std::string name; // e.g. "vertexPosition"
    sgl::VertexAttributeFormat attributeFormat;
    uint32_t numComponents;
    std::vector<uint8_t> data;
};

struct BinaryMeshUniform
{
    std::string name; // e.g. "maxVorticity"
    sgl::VertexAttributeFormat attributeFormat;
    uint32_t numComponents;
    std::vector<uint8_t> data;
};

struct BinarySubMesh
{
    ObjMaterial material;
    sgl::VertexMode vertexMode;
    std::vector<uint32_t> indices;
    std::vector<BinaryMeshAttribute> attributes;
    std::vector<BinaryMeshUniform> uniforms;
};

struct BinaryMesh
{
    std::vector<BinarySubMesh> submeshes;
};

/**
 * Writes a mesh to a binary file. The mesh data vectors may also be empty (i.e. size 0).
 * @param indices, vertices, texcoords, normals: The mesh data.
 */
void writeMesh3D(const std::string &filename, const BinaryMesh &mesh);

/**
 * Reads a mesh from a binary file. The mesh data vectors may also be empty (i.e. size 0).
 * @param indices, vertices, texcoords, normals: The mesh data.
 */
void readMesh3D(const std::string &filename, BinaryMesh &mesh);

struct ImportanceCriterionAttribute {
    std::string name;
    std::vector<float> attributes;
    float minAttribute;
    float maxAttribute;
};

// For programmable vertex fetching/pulling
struct SSBOEntry {
    SSBOEntry(int bindingPoint, const std::string &attributeName, sgl::GeometryBufferPtr &attributeBuffer)
        : bindingPoint(bindingPoint), attributeName(attributeName), attributeBuffer(attributeBuffer) {}
    int bindingPoint;
    std::string attributeName;
    sgl::GeometryBufferPtr attributeBuffer;
};

class MeshRenderer
{
public:
    MeshRenderer() : useProgrammableFetch(false) {}
    MeshRenderer(bool useProgrammableFetch) : useProgrammableFetch(useProgrammableFetch) {}

    // attributeIndex: For programmable vertex fetching/pulling. We need to bind the correct line attribute SSBO!
    void render(sgl::ShaderProgramPtr passShader, bool isGBufferPass, int attributeIndex);
    void setNewShader(sgl::ShaderProgramPtr newShader);
    bool isLoaded() { return shaderAttributes.size() > 0; }
    bool hasAttributeWithName(const std::string &name) {
        return shaderAttributeNames.find(name) != shaderAttributeNames.end();
    }

    bool useProgrammableFetch;
    std::vector<sgl::ShaderAttributesPtr> shaderAttributes;
    std::vector<SSBOEntry> ssboEntries; // For programmable vertex fetching/pulling
    std::set<std::string> shaderAttributeNames;
    std::vector<ObjMaterial> materials;
    sgl::AABB3 boundingBox;
    sgl::Sphere boundingSphere;
    std::vector<ImportanceCriterionAttribute> importanceCriterionAttributes;
};


/**
 * Uses parseMesh3d to read the mesh data from a file and assigns the data to a ShaderAttributesPtr object.
 * @param shader: The shader to use for the mesh.
 * @return: The loaded mesh stored in a ShaderAttributes object.
 */
MeshRenderer parseMesh3d(const std::string &filename, sgl::ShaderProgramPtr shader, bool shuffleData = false,
        bool useProgrammableFetch = false, bool programmableFetchUseAoS = true, float lineRadius = 0.001f);

#endif /* UTILS_MESHSERIALIZER_HPP_ */
