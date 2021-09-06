#ifndef VULKAN
uniform float fieldOfViewY;
uniform ivec2 viewportSize;
#endif

float getAntialiasingFactor(float distance) {
    return distance / float(viewportSize.y) * fieldOfViewY;
}
