float getAntialiasingFactor(float distance) {
    return distance / float(viewportSize.y) * fieldOfViewY;
}
