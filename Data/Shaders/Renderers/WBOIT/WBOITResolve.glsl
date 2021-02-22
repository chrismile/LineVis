-- Vertex

#version 430 core

in vec3 vertexPosition;
in vec2 vertexTexCoord;
out vec2 fragTexCoord;

void main()
{
    fragTexCoord = vertexTexCoord;
    gl_Position = mvpMatrix * vec4(vertexPosition, 1.0);
}

-- Fragment

#version 430 core

/**
 * Based on: Morgan McGuire and Louis Bavoil, Weighted Blended Order-Independent Transparency, Journal of Computer
 * Graphics Techniques (JCGT), vol. 2, no. 2, 122-141, 2013.
 *
 * For more details regarding the implementation see:
 * http://casual-effects.blogspot.com/2015/03/implemented-weighted-blended-order.html
 */

uniform sampler2D accumulationTexture;
uniform sampler2D revealageTexture;
in vec2 fragTexCoord;
out vec4 fragmentColor;

const float EPSILON = 1e-5;

float maxComponent(vec3 v) {
    return max(max(v.x, v.y), v.z);
}

void main()
{
    float revealage = texture2D(revealageTexture, fragTexCoord).r;
    if (revealage > 0.9999) {
        discard;
    }
    vec4 accumulatedColor = texture2D(accumulationTexture, fragTexCoord);
    if (isinf(maxComponent(abs(accumulatedColor.rgb)))) {
        accumulatedColor.rgb = vec3(accumulatedColor.a);
    }
    fragmentColor = vec4(accumulatedColor.rgb / max(accumulatedColor.a, EPSILON), 1.0 - revealage);
}
