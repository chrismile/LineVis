#ifdef USE_CAPPED_TUBES
    uint vertexLinePointIndex0 = vertexData0.vertexLinePointIndex & 0x7FFFFFFFu;
    uint vertexLinePointIndex1 = vertexData1.vertexLinePointIndex & 0x7FFFFFFFu;
    uint vertexLinePointIndex2 = vertexData2.vertexLinePointIndex & 0x7FFFFFFFu;
#ifndef USE_PRELOADED_LINE_DATA
    LinePointData linePointData0 = linePoints[vertexLinePointIndex0];
    LinePointData linePointData1 = linePoints[vertexLinePointIndex1];
    LinePointData linePointData2 = linePoints[vertexLinePointIndex2];
#endif
    bool isCap =
            bitfieldExtract(vertexData0.vertexLinePointIndex, 31, 1) > 0u
            || bitfieldExtract(vertexData1.vertexLinePointIndex, 31, 1) > 0u
            || bitfieldExtract(vertexData2.vertexLinePointIndex, 31, 1) > 0u;
#else
    uint vertexLinePointIndex0 = vertexData0.vertexLinePointIndex;
    uint vertexLinePointIndex1 = vertexData1.vertexLinePointIndex;
    uint vertexLinePointIndex2 = vertexData2.vertexLinePointIndex;
#ifndef USE_PRELOADED_LINE_DATA
    LinePointData linePointData0 = linePoints[vertexLinePointIndex0];
    LinePointData linePointData1 = linePoints[vertexLinePointIndex1];
    LinePointData linePointData2 = linePoints[vertexLinePointIndex2];
#endif
#endif

    vec3 fragmentNormal = interpolateVec3(
            vertexData0.vertexNormal, vertexData1.vertexNormal, vertexData2.vertexNormal, barycentricCoordinates);
    fragmentNormal = normalize(fragmentNormal);
    vec3 fragmentTangent = interpolateVec3(
            linePointData0.lineTangent, linePointData1.lineTangent, linePointData2.lineTangent, barycentricCoordinates);
    fragmentTangent = normalize(fragmentTangent);
    float fragmentAttribute = interpolateFloat(
            linePointData0.lineAttribute, linePointData1.lineAttribute, linePointData2.lineAttribute,
            barycentricCoordinates);

#if defined (USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
    float phi = interpolateAngle(
            vertexData0.phi, vertexData1.phi, vertexData2.phi, barycentricCoordinates);
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING)
    float fragmentVertexId = interpolateFloat(
            float(vertexLinePointIndex0), float(vertexLinePointIndex1), float(vertexLinePointIndex2),
            barycentricCoordinates);
#endif

#ifdef USE_BANDS
    vec3 linePosition = interpolateVec3(
            linePointData0.linePosition, linePointData1.linePosition, linePointData2.linePosition, barycentricCoordinates);
    vec3 lineNormal = interpolateVec3(
            linePointData0.lineNormal, linePointData1.lineNormal, linePointData2.lineNormal, barycentricCoordinates);
#endif

#ifdef USE_ROTATING_HELICITY_BANDS
    float fragmentRotation = interpolateFloat(
            linePointData0.lineRotation * helicityRotationFactor,
            linePointData1.lineRotation * helicityRotationFactor,
            linePointData2.lineRotation * helicityRotationFactor,
            barycentricCoordinates);
#endif

#if defined(USE_ROTATING_HELICITY_BANDS) && defined(USE_CAPPED_TUBES)
    if (isCap) {
        float fragmentRotationDelta = 0.0;
        float segmentLength = 1.0;
        vec3 planeNormal = vec3(0.0);
        LinePointData linePointDataOther;
        bool found = false;
        if (vertexLinePointIndex0 != 0) {
            linePointDataOther = linePoints[vertexLinePointIndex0 - 1];
            found = linePointDataOther.lineStartIndex == linePointData0.lineStartIndex;
        }
        if (!found) {
            linePointDataOther = linePoints[vertexLinePointIndex0 + 1];
        }
        fragmentRotationDelta =
                (linePointData0.lineRotation - linePointDataOther.lineRotation) * helicityRotationFactor;
        planeNormal = linePointData0.linePosition - linePointDataOther.linePosition;
        segmentLength = length(planeNormal);
        planeNormal /= segmentLength;
        //planeNormal = linePointData0.lineTangent;
        float planeDist = -dot(planeNormal, linePointData0.linePosition);
        float distToPlane = dot(planeNormal, fragmentPositionWorld) + planeDist;
        fragmentRotation += fragmentRotationDelta * distToPlane / segmentLength;
    }
#endif

#if defined(USE_ROTATING_HELICITY_BANDS) && defined(UNIFORM_HELICITY_BAND_WIDTH)
    float rotDx;
    float rotDy;
    bool found = false;
    LinePointData linePointDataOther;
    if (vertexLinePointIndex0 != 0) {
        linePointDataOther = linePoints[vertexLinePointIndex0 - 1];
        found = linePointDataOther.lineStartIndex == linePointData0.lineStartIndex;
        rotDy = (linePointData0.lineRotation - linePointDataOther.lineRotation) * helicityRotationFactor;
    }
    if (!found) {
        linePointDataOther = linePoints[vertexLinePointIndex0 + 1];
        rotDy = (linePointDataOther.lineRotation - linePointData0.lineRotation) * helicityRotationFactor;
    }
    rotDx = length(linePointData0.linePosition - linePointDataOther.linePosition);
    // Space conversion world <-> surface: circumference / arc length == M_PI * lineWidth / (2.0 * M_PI) == 0.5 * lineWidth
    float rotationSeparatorScale = cos(atan(rotDy * 0.5 * lineWidth, rotDx));
#endif

#ifdef STRESS_LINE_DATA
#ifndef USE_PRELOADED_LINE_DATA
    StressLinePointData stressLinePointData0 = stressLinePoints[vertexLinePointIndex0];
#endif
    uint principalStressIndex = stressLinePointData0.linePrincipalStressIndex;
    float lineAppearanceOrder = stressLinePointData0.lineLineAppearanceOrder;
#ifdef USE_PRINCIPAL_STRESSES
#ifndef USE_PRELOADED_LINE_DATA
    StressLinePointPrincipalStressData stressLinePointPrincipalStressData0 = principalStressLinePoints[vertexLinePointIndex0];
    StressLinePointPrincipalStressData stressLinePointPrincipalStressData1 = principalStressLinePoints[vertexLinePointIndex1];
    StressLinePointPrincipalStressData stressLinePointPrincipalStressData2 = principalStressLinePoints[vertexLinePointIndex2];
#endif
    float fragmentMajorStress = interpolateFloat(
            stressLinePointPrincipalStressData0.lineMajorStress,
            stressLinePointPrincipalStressData1.lineMajorStress,
            stressLinePointPrincipalStressData2.lineMajorStress,
            barycentricCoordinates);
    float fragmentMediumStress = interpolateFloat(
            stressLinePointPrincipalStressData0.lineMediumStress,
            stressLinePointPrincipalStressData1.lineMediumStress,
            stressLinePointPrincipalStressData2.lineMediumStress,
            barycentricCoordinates);
    float fragmentMinorStress = interpolateFloat(
            stressLinePointPrincipalStressData0.lineMinorStress,
            stressLinePointPrincipalStressData1.lineMinorStress,
            stressLinePointPrincipalStressData2.lineMinorStress,
            barycentricCoordinates);
#endif
#endif

    computeFragmentColor(
            fragmentPositionWorld, fragmentNormal, fragmentTangent,
#ifdef USE_CAPPED_TUBES
            isCap,
#endif
#if defined (USE_BANDS) || defined(USE_AMBIENT_OCCLUSION) || defined(USE_ROTATING_HELICITY_BANDS)
            phi,
#endif
#if defined(USE_AMBIENT_OCCLUSION) || defined(USE_MULTI_VAR_RENDERING)
            fragmentVertexId,
#endif
#ifdef USE_BANDS
            linePosition, lineNormal,
#endif
#ifdef USE_ROTATING_HELICITY_BANDS
            fragmentRotation,
#endif
#if defined(USE_ROTATING_HELICITY_BANDS) && defined(UNIFORM_HELICITY_BAND_WIDTH)
            rotationSeparatorScale,
#endif
#ifdef STRESS_LINE_DATA
            principalStressIndex, lineAppearanceOrder,
#ifdef USE_PRINCIPAL_STRESSES
            fragmentMajorStress, fragmentMediumStress, fragmentMinorStress,
#endif
#endif
            fragmentAttribute
    );
