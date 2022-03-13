// Parameters for opacity optimization.
layout(binding = 6) uniform OpacityOptimizationUniformDataBuffer {
    float q; // = 2000.0f; ///< Overall opacity, q >= 0.
    float r; // = 20.0f; ///< Clearing of background, r >= 0.
    int s; // = 15; ///< Iterations for smoothing.
    float lambda; // = 2.0f; ///< Emphasis of important structures, lambda > 0.
    float relaxationConstant; // = 0.1f; ///< Relaxation constant for spatial smoothing, relaxationConstant > 0.
    /// Temporal smoothing constant, 0 <= temporalSmoothingFactor <= 1. 1 means immediate update, 0 no update at all.
    float temporalSmoothingFactor; // = 0.15f;
    float opacityOptimizationUniformDataPadding0;
    float opacityOptimizationUniformDataPadding1;
};
