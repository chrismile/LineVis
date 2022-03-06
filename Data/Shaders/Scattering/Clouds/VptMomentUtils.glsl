//---------------------------------------------------------
// Absorption moment computation
//---------------------------------------------------------

#ifdef COMPUTE_PRIMARY_RAY_ABSORPTION_MOMENTS
void computePrimaryRayAbsorptionMoments(
        vec3 x, vec3 w, out float primaryRayAbsorptionMoments[NUM_PRIMARY_RAY_ABSORPTION_MOMENTS + 1]) {
    for (int i = 0; i <= NUM_PRIMARY_RAY_ABSORPTION_MOMENTS; i++) {
        primaryRayAbsorptionMoments[i] = 0.0;
    }
    float depth = 0.0;

    float majorant = parameters.extinction.x;
    float absorptionAlbedo = 1.0 - parameters.scatteringAlbedo.x;
    float scatteringAlbedo = parameters.scatteringAlbedo.x;
    float PA = absorptionAlbedo * parameters.extinction.x;
    float PS = scatteringAlbedo * parameters.extinction.x;

    float tMin, tMax;
    if (rayBoxIntersect(parameters.boxMin, parameters.boxMax, x, w, tMin, tMax)) {
        x += w * tMin;
        //depth += tMin;
        float d = tMax - tMin;

        float pdf_x = 1.0;
        float transmittance = 1.0;

        while (true) {
            float absorbance = -log(transmittance);
            if (absorbance > ABSORBANCE_MAX_VALUE) {
                absorbance = ABSORBANCE_MAX_VALUE;
            }
#ifdef USE_POWER_MOMENTS_PRIMARY_RAY
            for (int i = 0; i <= NUM_PRIMARY_RAY_ABSORPTION_MOMENTS; i++) {
                primaryRayAbsorptionMoments[i] += absorbance * pow(depth, i);
            }
#else
            float phase = fma(depth, wrapping_zone_parameters.y, wrapping_zone_parameters.y);
            vec2 circlePoint = vec2(cos(phase), sin(phase));
            primaryRayAbsorptionMoments[0] = absorbance;
            primaryRayAbsorptionMoments[1] = absorbance * circlePoint.x;
            primaryRayAbsorptionMoments[2] = absorbance * circlePoint.y;
            vec2 circlePointNext = circlePoint;
            for (int i = 2; i <= NUM_PRIMARY_RAY_ABSORPTION_MOMENTS / 2; i++) {
                circlePointNext = Multiply(circlePointNext, circlePoint);
                primaryRayAbsorptionMoments[i * 2] = absorbance * circlePointNext.x;
                primaryRayAbsorptionMoments[i * 2 + 1] = absorbance * circlePointNext.y;
            }
#endif
            transmittance = 1.0;

            float t = -log(max(0.0000000001, 1 - random())) / majorant;

            if (t > d)
            break;

            x += w * t;
            depth += t;

            float density = sampleCloud(x);
            transmittance *= 1.0 - density;

            float sigma_a = PA * density;
            float sigma_s = PS * density;
            float sigma_n = majorant - parameters.extinction.x * density;

            float Pa = sigma_a / majorant;
            float Ps = sigma_s / majorant;
            float Pn = sigma_n / majorant;

            float xi = random();

            if (xi < Pa)
            return; // weights * sigma_a / (majorant * Pa) * L_e; // 0 - No emission

            if (xi < 1 - Pn) { // scattering event
                if (rayBoxIntersect(parameters.boxMin, parameters.boxMax, x, w, tMin, tMax)) {
                    x += w*tMin;
                    depth += tMin;
                    d = tMax - tMin;
                }
            } else {
                pdf_x *= exp(-parameters.extinction.x * density);
                d -= t;
            }
        }
    }
}
#endif
