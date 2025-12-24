#ifdef USE_COVERAGE_MASK
#ifdef VULKAN
// Use same binding as in OpacityOptimizationUniformData.glsl.
layout(binding = 6) uniform SampleInfoUniformBuffer {
    uint numSamples;
};
#else
in int gl_SampleMaskIn[];
#endif
#endif
