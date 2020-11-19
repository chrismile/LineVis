// Global inputs for line vertices
// Position in 3D world space
layout(location = 0) in vec3 vertexPosition;
// Normal of current line segment
layout(location = 1) in vec3 vertexLineNormal;
// Tangent of current line segment
layout(location = 2) in vec3 vertexLineTangent;
// Description of current variable at line vertex
layout(location = 3) in vec4 multiVariable;
// Description of line segment such as curve interpolant t, neighboring element:
// (x: elementID, y: lineID, z: nextElementID, w: elementInterpolant)
layout(location = 4) in vec4 variableDesc;