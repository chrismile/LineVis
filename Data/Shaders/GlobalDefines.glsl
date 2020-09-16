// Visible to all shaders
layout (std140, binding = 0) uniform MatrixBlock
{
    mat4 mMatrix; // Model matrix
    mat4 vMatrix; // View matrix
    mat4 pMatrix; // Projection matrix
    mat4 mvpMatrix; // Model-view-projection matrix
};
