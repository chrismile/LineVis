/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "LineData/LineData.hpp"
#include "Renderers/OIT/SyncMode.hpp"
#include "Renderers/Deferred/DeferredModes.hpp"
#include "Renderers/Deferred/Tree/PersistentThreadHelper.hpp"
#include "InternalState.hpp"

void getTestModesOpaque(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_OPAQUE;
    state.name = "Opaque";
    states.push_back(state);
}

void getTestModesDepthComplexity(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_DEPTH_COMPLEXITY;
    state.name = "Depth Complexity";
    states.push_back(state);
}

void getTestModesPerPixelLinkedLists(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_PER_PIXEL_LINKED_LIST;
    state.name = "PPLL";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{})};
    states.push_back(state);
}

void getTestModesOpacityOptimization(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_PER_PIXEL_LINKED_LIST;
    state.name = "Opacity Optimization";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{})};
    states.push_back(state);
}

void getTestModesMlab(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_MLAB;

    state.name = "MLAB (No Sync)";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{
            { "syncMode", std::to_string((int)NO_SYNC) }
    })};
    states.push_back(state);

    state.name = "MLAB (Spinlock)";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{
            { "syncMode", std::to_string((int)SYNC_SPINLOCK) }
    })};
    states.push_back(state);

    state.name = "MLAB (Unordered Interlock)";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{
            { "syncMode", std::to_string((int)SYNC_FRAGMENT_SHADER_INTERLOCK) },
            { "useOrderedFragmentShaderInterlock", "false" }
    })};
    states.push_back(state);

    state.name = "MLAB (Ordered Interlock)";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{
            { "syncMode", std::to_string((int)SYNC_FRAGMENT_SHADER_INTERLOCK) },
            { "useOrderedFragmentShaderInterlock", "true" }
    })};
    states.push_back(state);
}

void getTestModesOITForDataSet(std::vector<InternalState>& states, InternalState state) {
    //getTestModesDepthComplexity(states, state);
    //getTestModesPerPixelLinkedLists(states, state);
    //getTestModesOpacityOptimization(states, state);
    //getTestModesOpacityOptimization(states, state);
    getTestModesOpaque(states, state);
    getTestModesDepthComplexity(states, state);
    getTestModesPerPixelLinkedLists(states, state);
    getTestModesMlab(states, state);
}

std::vector<InternalState> getTestModesOIT() {
    std::vector<InternalState> states;
    //std::vector<glm::ivec2> windowResolutions = {
    //        glm::ivec2(1280, 720), glm::ivec2(1920, 1080), glm::ivec2(2560, 1440) };
    std::vector<glm::ivec2> windowResolutions = { glm::ivec2(1920, 1080) };
    //std::vector<glm::ivec2> windowResolutions = { glm::ivec2(2560, 1440) };
    //std::vector<glm::ivec2> windowResolutions = { glm::ivec2(2186, 1358) };
    std::vector<DataSetDescriptor> dataSetDescriptors = {
            //DataSetDescriptor("Aneurysm"),
            DataSetDescriptor("Femur (Vis2021)"),
    };
    std::vector<std::string> transferFunctionNames = {
            //"Transparent_Aneurysm.xml",
            "Standard.xml",
    };
    InternalState state;

    /*for (size_t i = 0; i < windowResolutions.size(); i++) {
        state.windowResolution = windowResolutions.at(i);
        for (size_t j = 0; j < dataSetDescriptors.size(); j++) {
            state.dataSetDescriptor = dataSetDescriptors.at(j);
            if (transferFunctionNames.size() > 0) {
                state.transferFunctionName = transferFunctionNames.at(j);
            }
            getTestModesOITForDataSet(states, state);
        }
    }*/
    for (size_t i = 0; i < dataSetDescriptors.size(); i++) {
        state.dataSetDescriptor = dataSetDescriptors.at(i);
        for (size_t j = 0; j < windowResolutions.size(); j++) {
            state.windowResolution = windowResolutions.at(j);
            if (!transferFunctionNames.empty()) {
                state.transferFunctionName = transferFunctionNames.at(i);
            }
            getTestModesOITForDataSet(states, state);
        }
    }

    // Append model name to state name if more than one model is loaded
    if (!dataSetDescriptors.empty() || windowResolutions.size() > 1) {
        for (InternalState& state : states) {
            state.name =
                    sgl::toString(state.windowResolution.x) + "x" + sgl::toString(state.windowResolution.y)
                    + " " + state.dataSetDescriptor.name + " " + state.name;
        }
    }

    return states;
}


void setLinePrimitiveMode(InternalState& state, LineData::LinePrimitiveMode linePrimitiveMode) {
    state.dataSetSettings = { SettingsMap(std::map<std::string, std::string>{
            { "line_primitive_mode_index", std::to_string(int(linePrimitiveMode)) }
    })};
}
void getTestModesRasterization(std::vector<InternalState>& states, InternalState state) {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

    state.renderingMode = RENDERING_MODE_OPAQUE;
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{
            { "numSamples", "1" }
    })};

    if (device->getPhysicalDeviceFeatures().geometryShader) {
        state.name = "Warm-up Run";
        setLinePrimitiveMode(state, LineData::LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER);
        states.push_back(state);

        state.name = "Quads Geometry Shader";
        setLinePrimitiveMode(state, LineData::LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER);
        states.push_back(state);

        state.name = "Geometry Shader";
        setLinePrimitiveMode(state, LineData::LINE_PRIMITIVES_TUBE_RIBBONS_GEOMETRY_SHADER);
        states.push_back(state);
    }

    state.name = "Programmable Pull";
    setLinePrimitiveMode(state, LineData::LINE_PRIMITIVES_TUBE_RIBBONS_PROGRAMMABLE_PULL);
    states.push_back(state);

    if (device->getPhysicalDeviceMeshShaderFeaturesNV().meshShader) {
        state.name = "Mesh Shader (NV)";
        setLinePrimitiveMode(state, LineData::LINE_PRIMITIVES_TUBE_RIBBONS_MESH_SHADER_NV);
        states.push_back(state);
    }

#ifdef VK_EXT_mesh_shader
    if (device->getPhysicalDeviceMeshShaderFeaturesEXT().meshShader) {
        state.name = "Mesh Shader (EXT)";
        setLinePrimitiveMode(state, LineData::LINE_PRIMITIVES_TUBE_RIBBONS_MESH_SHADER);
        states.push_back(state);
    }
#endif

    // An out of memory error is triggered on Intel iGPUs for large data sets when trying to store the triangle mesh.
    bool isIntelIntegratedGpu =
            (device->getDeviceDriverId() == VK_DRIVER_ID_INTEL_PROPRIETARY_WINDOWS
             || device->getDeviceDriverId() == VK_DRIVER_ID_INTEL_OPEN_SOURCE_MESA)
            && device->getDeviceType() == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU;
    bool isLargeDataSet =
            state.dataSetDescriptor.name == "Convection Rolls" || state.dataSetDescriptor.name == "Turbulence";
    if (!isIntelIntegratedGpu || !isLargeDataSet) {
        state.name = "Triangle Mesh";
        setLinePrimitiveMode(state, LineData::LINE_PRIMITIVES_TUBE_RIBBONS_TRIANGLE_MESH);
        states.push_back(state);
    }
}

void getTestModesVulkanRayTracing(std::vector<InternalState>& states, InternalState state) {
    if (sgl::AppSettings::get()->getPrimaryDevice()->getRayTracingPipelineSupported()) {
        state.renderingMode = RENDERING_MODE_VULKAN_RAY_TRACER;

        //state.name = "VRT Analytic";
        //state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{
        //        { "useAnalyticIntersections", "true" },
        //        { "numSamplesPerFrame", "1" }
        //})};
        //states.push_back(state);

        /*if (state.dataSetDescriptor.name != "Convection Rolls"
                || sgl::AppSettings::get()->getPrimaryDevice()->getDeviceDriverId() != VK_DRIVER_ID_AMD_PROPRIETARY) {*/
        state.name = "VRT Triangle Mesh";
        state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{
                { "useAnalyticIntersections", "false" },
                { "numSamplesPerFrame", "1" }
        })};
        states.push_back(state);
        //}
    }
}

void getTestModesVoxelRayCasting(std::vector<InternalState>& states, InternalState state) {
    //sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    // Intel integrated GPUs are too slow at the time of writing this code and TDR might be triggered for this mode.
    // I increased the timeout using the following method to allow it to run anyways:
    // https://substance3d.adobe.com/documentation/spdoc/gpu-drivers-crash-with-long-computations-tdr-crash-128745489.html
    //if ((device->getDeviceDriverId() != VK_DRIVER_ID_INTEL_PROPRIETARY_WINDOWS
    //         && device->getDeviceDriverId() != VK_DRIVER_ID_INTEL_OPEN_SOURCE_MESA)
    //        || device->getDeviceType() != VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) {
    state.renderingMode = RENDERING_MODE_VOXEL_RAY_CASTING;
    state.name = "Voxel Ray Casting";
    if (state.dataSetDescriptor.name == "Convection Rolls") {
        state.rendererSettings.addKeyValue("computeNearestFurthestHitsUsingHull", false);
    }
    states.push_back(state);
    //}
}

#ifdef USE_OSPRAY
void getTestModesOspray(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_OSPRAY_RAY_TRACER;

    state.name = "OSPRay Linear Curves";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{
            { "geometryMode", "curves" }
    })};
    states.push_back(state);

    state.name = "OSPRay Triangle Mesh";
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{
            { "geometryMode", "triangle_mesh" }
    })};
    states.push_back(state);
}
#endif

void getTestModesPaperVMV(std::vector<InternalState>& states, const InternalState& state) {
    getTestModesRasterization(states, state);
    getTestModesVulkanRayTracing(states, state);
    //getTestModesVoxelRayCasting(states, state);
#ifdef USE_OSPRAY
    //getTestModesOspray(states, state);
#endif
}

std::vector<InternalState> getTestModesOpaqueRendering() {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    std::vector<InternalState> states;
    std::vector<glm::ivec2> windowResolutions = {
            //glm::ivec2(1920, 1080),
            glm::ivec2(3840, 2160)
    };
    bool isIntegratedGpu =
            device->getDeviceDriverId() == VK_DRIVER_ID_MOLTENVK
            || ((device->getDeviceDriverId() == VK_DRIVER_ID_INTEL_PROPRIETARY_WINDOWS
                 || device->getDeviceDriverId() == VK_DRIVER_ID_INTEL_OPEN_SOURCE_MESA)
                && device->getDeviceType() == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU);
    if (isIntegratedGpu) {
        windowResolutions = { glm::ivec2(1920, 1080) };
    } else {
        windowResolutions = { glm::ivec2(3840, 2160) };
    }
    std::vector<DataSetDescriptor> dataSetDescriptors = {
            //DataSetDescriptor("Rings"),
            //DataSetDescriptor("Aneurysm"),
            //DataSetDescriptor("Convection Rolls"),
            //DataSetDescriptor("Femur (Vis2021)"),
            //DataSetDescriptor("Bearing"),
            //DataSetDescriptor("Convection Rolls"),
            //DataSetDescriptor("Tangaroa (t=200)"),
            DataSetDescriptor("Centrifugal Pump (DES.res_t2564)"),
    };
    std::vector<std::string> transferFunctionNames = {
            //"Standard.xml"
    };
    InternalState state;

    for (size_t i = 0; i < dataSetDescriptors.size(); i++) {
        state.dataSetDescriptor = dataSetDescriptors.at(i);
        for (size_t j = 0; j < windowResolutions.size(); j++) {
            state.windowResolution = windowResolutions.at(j);
            if (!transferFunctionNames.empty()) {
                state.transferFunctionName = transferFunctionNames.at(i);
            }
            getTestModesPaperVMV(states, state);
        }
    }

    // Testing performance without the contributions introduced by the paper.
    bool useContributions = true;

    bool testRotatingHelicityBands = true;
    if (testRotatingHelicityBands) {
        std::vector<InternalState> oldStates = states;
        states.clear();
        for (size_t i = 0; i < oldStates.size(); i++) {
            InternalState state = oldStates.at(i);
            std::string nameBase = state.name;

            state.name = nameBase + "(Ribbon)";
            if (!useContributions) {
                state.dataSetSettings.addKeyValue("use_halos", "false");
            }
            state.dataSetSettings.addKeyValue("use_ribbons", "true");
            state.dataSetSettings.addKeyValue("rotating_helicity_bands", "false");
            state.rendererSettings.addKeyValue("line_width", "0.006");
            state.rendererSettings.addKeyValue("band_width", "0.007");
            states.push_back(state);

            state.name = nameBase + "(Twist)";
            state.dataSetSettings.addKeyValue("use_ribbons", "false");
            if (useContributions) {
                int linePrimitiveModeIndex = 0;
                state.dataSetSettings.getValueOpt("line_primitive_mode_index", linePrimitiveModeIndex);
                if (LineData::LinePrimitiveMode(linePrimitiveModeIndex) != LineData::LINE_PRIMITIVES_RIBBON_QUADS_GEOMETRY_SHADER) {
                    state.dataSetSettings.addKeyValue("rotating_helicity_bands", "true");
                }
            } else {
                state.dataSetSettings.addKeyValue(
                        "twist_line_texture",
                        sgl::AppSettings::get()->getDataDirectory() + "Texture/Stripes.png");
                state.dataSetSettings.addKeyValue(
                        "use_twist_line_texture", "false");
                state.dataSetSettings.addKeyValue(
                        "twist_line_texture_filtering_mode", "Linear Mipmap Linear");
                state.dataSetSettings.addKeyValue(
                        "twist_line_texture_max_anisotropy", "16");
            }
            states.push_back(state);
        }
    }

    bool runStatesTwoTimesForErrorMeasure = true;
    if (runStatesTwoTimesForErrorMeasure) {
        std::vector<InternalState> oldStates = states;
        states.clear();
        for (size_t i = 0; i < oldStates.size(); i++) {
            InternalState state = oldStates.at(i);
            states.push_back(state);
            state.name += "(2)";
            states.push_back(state);
        }
    }

    for (InternalState& state : states) {
        state.nameRaw = state.name;
    }

    // Append model name to state name if more than one model is loaded
    if (dataSetDescriptors.size() > 1 || windowResolutions.size() > 1) {
        for (InternalState& state : states) {
            state.name =
                    sgl::toString(state.windowResolution.x)
                    + "x" + sgl::toString(state.windowResolution.y)
                    + " " + state.dataSetDescriptor.name + " " + state.name;
        }
    }

    return states;
}

void getTestModesRasterizationTriMesh(std::vector<InternalState>& states, InternalState state) {
    state.renderingMode = RENDERING_MODE_OPAQUE;
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{
            { "numSamples", "1" }
    })};

    setLinePrimitiveMode(state, LineData::LINE_PRIMITIVES_TUBE_RIBBONS_TRIANGLE_MESH);
    state.name = "Warm Up";
    states.push_back(state);
    state.name = "Triangle Mesh";
    states.push_back(state);
}

void getTestModesDeferred(std::vector<InternalState>& states, InternalState state) {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();

    state.renderingMode = RENDERING_MODE_DEFERRED_SHADING;
    state.rendererSettings = { SettingsMap(std::map<std::string, std::string>{
            { "numSamples", "1" }
    })};

    state.name = "Deferred Draw Indexed";
    state.rendererSettings.addKeyValue("deferredRenderingMode", "Draw Indexed");
    states.push_back(state);

    state.name = "Deferred Draw Indirect";
    state.rendererSettings.addKeyValue("deferredRenderingMode", "Draw Indirect");
    states.push_back(state);

    if (device->getPhysicalDeviceMeshShaderFeaturesNV().meshShader) {
        state.name = "Deferred Mesh Shader (NV)";
        state.rendererSettings.addKeyValue("deferredRenderingMode", "Task/Mesh Shader");
        state.rendererSettings.addKeyValue("useMeshShaderNV", "true");
        states.push_back(state);
    }

#ifdef VK_EXT_mesh_shader
    if (device->getPhysicalDeviceMeshShaderFeaturesEXT().meshShader) {
        state.name = "Deferred Mesh Shader (EXT)";
        state.rendererSettings.addKeyValue("deferredRenderingMode", "Task/Mesh Shader");
        state.rendererSettings.addKeyValue("useMeshShaderNV", "false");
        states.push_back(state);
    }
#endif

    if (device->getPhysicalDeviceMeshShaderFeaturesNV().meshShader) {
        state.name = "Deferred BVH Mesh Shader (NV)";
        state.rendererSettings.addKeyValue("deferredRenderingMode", "BVH Mesh Shader");
        state.rendererSettings.addKeyValue("useMeshShaderNV", "true");
        states.push_back(state);
    }

#ifdef VK_EXT_mesh_shader
    if (device->getPhysicalDeviceMeshShaderFeaturesEXT().meshShader) {
        state.name = "Deferred BVH Mesh Shader (EXT)";
        state.rendererSettings.addKeyValue("deferredRenderingMode", "BVH Mesh Shader");
        state.rendererSettings.addKeyValue("useMeshShaderNV", "false");
        states.push_back(state);
    }
#endif

    state.name = "Deferred BVH Draw Indirect";
    state.rendererSettings.addKeyValue("deferredRenderingMode", "BVH Draw Indirect");
    states.push_back(state);

    state.name = "Deferred BVH Draw Indirect (Subgroup Ops)";
    state.rendererSettings.addKeyValue("useSubgroupOps", "true");
    states.push_back(state);
    state.rendererSettings.addKeyValue("useSubgroupOps", "false");

    for (uint32_t i = 8; i <= 256; i *= 2) {
        state.name = "Deferred BVH Draw Indirect (" + std::to_string(i) + ")";
        state.rendererSettings.addKeyValue("workgroupSizeBvh", std::to_string(i));
        states.push_back(state);
    }

    state.rendererSettings.addKeyValue("workgroupSizeBvh", std::to_string(32));
    for (uint32_t i = 8; i <= 256; i *= 2) {
        state.name = "Deferred BVH Draw Indirect (" + std::to_string(i) + " x 32)";
        state.rendererSettings.addKeyValue("numWorkgroupsBvh", std::to_string(i));
        states.push_back(state);
    }

    state.rendererSettings.addKeyValue("workgroupSizeBvh", std::to_string(128));
    for (uint32_t i = 8; i <= 256; i *= 2) {
        state.name = "Deferred BVH Draw Indirect (" + std::to_string(i) + " x 128)";
        state.rendererSettings.addKeyValue("numWorkgroupsBvh", std::to_string(i));
        states.push_back(state);
    }

    DevicePersistentThreadInfo persistentThreadInfo = getDevicePersistentThreadInfo(device);
    state.rendererSettings.addKeyValue(
            "numWorkgroupsBvh", std::to_string(persistentThreadInfo.optimalNumWorkgroups));
    state.rendererSettings.addKeyValue(
            "workgroupSizeBvh", std::to_string(persistentThreadInfo.optimalWorkgroupSize));

    for (int i = 0; i < IM_ARRAYSIZE(bvhBuildGeometryModeNames); i++) {
        for (int j = 0; j < IM_ARRAYSIZE(bvhBuildAlgorithmNames); j++) {
            state.name =
                    std::string() + "Deferred BVH Draw Indirect (" + bvhBuildAlgorithmNames[j]
                    + ", " + bvhBuildGeometryModeNames[i] + ")";
            state.rendererSettings.addKeyValue("bvhBuildGeometryMode", bvhBuildGeometryModeNames[i]);
            state.rendererSettings.addKeyValue("bvhBuildAlgorithm", bvhBuildAlgorithmNames[j]);
            states.push_back(state);
        }
    }
    state.rendererSettings.addKeyValue("bvhBuildGeometryMode", "Meshlets");
    state.rendererSettings.addKeyValue("bvhBuildAlgorithm", "Sweep SAH (CPU)");
}

void getTestModesDeferredRenderingImpl(std::vector<InternalState>& states, const InternalState& state) {
    getTestModesRasterizationTriMesh(states, state);
    getTestModesDeferred(states, state);
    //getTestModesVulkanRayTracing(states, state);
}

std::vector<InternalState> getTestModesDeferredRendering() {
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    std::vector<InternalState> states;
    std::vector<glm::ivec2> windowResolutions = {
            //glm::ivec2(1920, 1080),
            glm::ivec2(3840, 2160)
    };
    bool isIntegratedGpu =
            device->getDeviceDriverId() == VK_DRIVER_ID_MOLTENVK
            || ((device->getDeviceDriverId() == VK_DRIVER_ID_INTEL_PROPRIETARY_WINDOWS
                 || device->getDeviceDriverId() == VK_DRIVER_ID_INTEL_OPEN_SOURCE_MESA)
                && device->getDeviceType() == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU);
    if (isIntegratedGpu) {
        windowResolutions = { glm::ivec2(1920, 1080) };
    } else {
        windowResolutions = { glm::ivec2(3840, 2160) };
    }
    std::vector<DataSetDescriptor> dataSetDescriptors = {
            //DataSetDescriptor("Rings"),
            DataSetDescriptor("Aneurysm"),
            //DataSetDescriptor("Convection Rolls"),
            //DataSetDescriptor("Femur (Vis2021)"),
            //DataSetDescriptor("Bearing"),
            //DataSetDescriptor("Convection Rolls"),
            //DataSetDescriptor("Tangaroa (t=200)"),
    };
    std::vector<std::string> transferFunctionNames = {
            //"Standard.xml"
    };
    InternalState state;

    for (size_t i = 0; i < dataSetDescriptors.size(); i++) {
        state.dataSetDescriptor = dataSetDescriptors.at(i);
        for (size_t j = 0; j < windowResolutions.size(); j++) {
            state.windowResolution = windowResolutions.at(j);
            if (!transferFunctionNames.empty()) {
                state.transferFunctionName = transferFunctionNames.at(i);
            }
            getTestModesDeferredRenderingImpl(states, state);
        }
    }

    bool runStatesTwoTimesForErrorMeasure = true;
    if (runStatesTwoTimesForErrorMeasure) {
        std::vector<InternalState> oldStates = states;
        states.clear();
        for (size_t i = 0; i < oldStates.size(); i++) {
            InternalState state = oldStates.at(i);
            states.push_back(state);
            state.name += "(2)";
            states.push_back(state);
        }
    }

    for (InternalState& state : states) {
        state.nameRaw = state.name;
    }

    // Append model name to state name if more than one model is loaded
    if (dataSetDescriptors.size() > 1 || windowResolutions.size() > 1) {
        for (InternalState& state : states) {
            state.name =
                    sgl::toString(state.windowResolution.x)
                    + "x" + sgl::toString(state.windowResolution.y)
                    + " " + state.dataSetDescriptor.name + " " + state.name;
        }
    }

    return states;
}

std::vector<InternalState> getTestModes() {
    return getTestModesDeferredRendering();
}
