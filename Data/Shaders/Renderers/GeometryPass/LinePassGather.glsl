#if defined(DIRECT_BLIT_GATHER)
    // To counteract depth fighting with overlay wireframe.
    float depthOffset = -0.00001;
    if (absCoords >= WHITE_THRESHOLD - EPSILON_WHITE) {
        depthOffset = 0.002;
    }
    //gl_FragDepth = clamp(gl_FragCoord.z + depthOffset, 0.0, 0.999);
    gl_FragDepth = convertLinearDepthToDepthBufferValue(
            convertDepthBufferValueToLinearDepth(gl_FragCoord.z) + fragmentDepth
            - length(fragmentPositionWorld - cameraPosition) - 0.0001);
#ifdef LOW_OPACITY_DISCARD
    if (colorOut.a < 0.01) {
        discard;
    }
#endif
    colorOut.a = 1.0;
    fragColor = colorOut;
#elif defined(USE_SYNC_FRAGMENT_SHADER_INTERLOCK)
    // Area of mutual exclusion for fragments mapping to the same pixel
    beginInvocationInterlockARB();
    gatherFragment(colorOut);
    endInvocationInterlockARB();
#elif defined(USE_SYNC_SPINLOCK)
    uint x = uint(gl_FragCoord.x);
    uint y = uint(gl_FragCoord.y);
    uint pixelIndex = addrGen(uvec2(x,y));
    /**
     * Spinlock code below based on code in:
     * Brüll, Felix. (2018). Order-Independent Transparency Acceleration. 10.13140/RG.2.2.17568.84485.
     */
    if (!gl_HelperInvocation) {
        bool keepWaiting = true;
        while (keepWaiting) {
            if (atomicCompSwap(spinlockViewportBuffer[pixelIndex], 0, 1) == 0) {
                gatherFragment(colorOut);
                memoryBarrier();
                atomicExchange(spinlockViewportBuffer[pixelIndex], 0);
                keepWaiting = false;
            }
        }
    }
#else
    gatherFragment(colorOut);
#endif
