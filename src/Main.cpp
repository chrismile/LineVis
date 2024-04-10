/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020-2022, Christoph Neuhauser
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

#include <unordered_map>

#ifdef USE_PYTHON
#include <Utils/Python/PythonInit.hpp>
#endif

#ifdef USE_OSPRAY
#include <ospray/ospray.h>
#include "Renderers/Ospray/OsprayRenderer.hpp"
#endif

#include <Utils/File/FileUtils.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/AppSettings.hpp>
#include <Utils/AppLogic.hpp>
#include <Graphics/Window.hpp>
#include <Graphics/Vulkan/Utils/Device.hpp>
#include <Graphics/Vulkan/Utils/Swapchain.hpp>
#include <Graphics/Vulkan/Shader/ShaderManager.hpp>

#include "MainApp.hpp"

int main(int argc, char *argv[]) {
    // Initialize the filesystem utilities.
    sgl::FileUtils::get()->initialize("LineVis", argc, argv);

#ifdef DATA_PATH
    if (!sgl::FileUtils::get()->directoryExists("Data") && !sgl::FileUtils::get()->directoryExists("../Data")) {
        sgl::AppSettings::get()->setDataDirectory(DATA_PATH);
    }
#endif
    sgl::AppSettings::get()->initializeDataDirectory();

    std::string iconPath = sgl::AppSettings::get()->getDataDirectory() + "Fonts/icon_256.png";
    sgl::AppSettings::get()->setApplicationDescription("A visualization tool for rendering dense sets of 3D lines");
    sgl::AppSettings::get()->loadApplicationIconFromFile(iconPath);

    // Parse the arguments.
    bool usePerfMode = false;
    bool useCustomShaderCompilerBackend = false;
    sgl::vk::ShaderCompilerBackend shaderCompilerBackend = sgl::vk::ShaderCompilerBackend::SHADERC;
    for (int i = 1; i < argc; i++) {
        std::string command = argv[i];
        if (command == "--perf") {
            usePerfMode = true;
        } else if (command == "--shader-backend") {
            i++;
            if (i >= argc) {
                sgl::Logfile::get()->throwError(
                        "Error: Command line argument '--shader-backend' expects a backend name.");
            }
            useCustomShaderCompilerBackend = true;
            std::string backendName = argv[i];
            if (backendName == "shaderc") {
                shaderCompilerBackend = sgl::vk::ShaderCompilerBackend::SHADERC;
            } else if (backendName == "glslang") {
                shaderCompilerBackend = sgl::vk::ShaderCompilerBackend::GLSLANG;
            }
        }
    }

    // Load the file containing the app settings
    std::string settingsFile = sgl::FileUtils::get()->getConfigDirectory() + "settings.txt";
    sgl::AppSettings::get()->loadSettings(settingsFile.c_str());
    sgl::AppSettings::get()->getSettings().addKeyValue("window-multisamples", 0);
    sgl::AppSettings::get()->getSettings().addKeyValue("window-debugContext", true);
    if (usePerfMode) {
        sgl::AppSettings::get()->getSettings().addKeyValue("window-vSync", false);
    } else {
        sgl::AppSettings::get()->getSettings().addKeyValue("window-vSync", true);
    }
    sgl::AppSettings::get()->getSettings().addKeyValue("window-resizable", true);
    sgl::AppSettings::get()->getSettings().addKeyValue("window-savePosition", true);
    //sgl::AppSettings::get()->setVulkanDebugPrintfEnabled();

    ImVector<ImWchar> fontRanges;
    ImFontGlyphRangesBuilder builder;
    builder.AddChar(L'\u03BB'); // lambda
    builder.BuildRanges(&fontRanges);
    bool useMultiViewport = false;
    if (sgl::AppSettings::get()->getSettings().getValueOpt("useDockSpaceMode", useMultiViewport)) {
        useMultiViewport = !useMultiViewport;
    }
    sgl::AppSettings::get()->setLoadGUI(fontRanges.Data, true, useMultiViewport);

    sgl::AppSettings::get()->setRenderSystem(sgl::RenderSystem::VULKAN);
    sgl::Window* window = sgl::AppSettings::get()->createWindow();

    std::vector<const char*> optionalDeviceExtensions;
#ifdef SUPPORT_CUDA_INTEROP
    optionalDeviceExtensions = sgl::vk::Device::getCudaInteropDeviceExtensions();
#endif
    std::vector<const char*> raytracingDeviceExtensions = {
            VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
            VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME,
            VK_KHR_MAINTENANCE3_EXTENSION_NAME,
            VK_KHR_PIPELINE_LIBRARY_EXTENSION_NAME,
            VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
            VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME,
            VK_KHR_RAY_QUERY_EXTENSION_NAME
    };
    optionalDeviceExtensions.insert(
            optionalDeviceExtensions.end(),
            raytracingDeviceExtensions.begin(), raytracingDeviceExtensions.end());
    optionalDeviceExtensions.push_back(VK_KHR_TIMELINE_SEMAPHORE_EXTENSION_NAME);
    optionalDeviceExtensions.push_back(VK_EXT_FRAGMENT_SHADER_INTERLOCK_EXTENSION_NAME);
    optionalDeviceExtensions.push_back(VK_NV_FRAGMENT_SHADER_BARYCENTRIC_EXTENSION_NAME);
    optionalDeviceExtensions.push_back(VK_KHR_DRAW_INDIRECT_COUNT_EXTENSION_NAME);
    optionalDeviceExtensions.push_back(VK_KHR_SHADER_FLOAT16_INT8_EXTENSION_NAME);
    optionalDeviceExtensions.push_back(VK_KHR_8BIT_STORAGE_EXTENSION_NAME);
    //optionalDeviceExtensions.push_back(VK_KHR_SHADER_ATOMIC_INT64_EXTENSION_NAME);
    optionalDeviceExtensions.push_back(VK_EXT_SHADER_ATOMIC_FLOAT_EXTENSION_NAME);
    optionalDeviceExtensions.push_back(VK_NV_MESH_SHADER_EXTENSION_NAME);
#ifdef VK_EXT_mesh_shader
    optionalDeviceExtensions.push_back(VK_EXT_MESH_SHADER_EXTENSION_NAME);
#endif

    sgl::vk::Instance* instance = sgl::AppSettings::get()->getVulkanInstance();
    auto* device = new sgl::vk::Device;
    sgl::vk::DeviceFeatures requestedDeviceFeatures{};
    requestedDeviceFeatures.optionalPhysicalDeviceFeatures.geometryShader = VK_TRUE; // For a rasterizer mode.
    requestedDeviceFeatures.optionalPhysicalDeviceFeatures.sampleRateShading = VK_TRUE; // For OpaqueLineRenderer.
    requestedDeviceFeatures.optionalPhysicalDeviceFeatures.independentBlend = VK_TRUE; // For WBOITRenderer.
    requestedDeviceFeatures.optionalPhysicalDeviceFeatures.samplerAnisotropy = VK_TRUE; // For LineDataFlow textures.
    requestedDeviceFeatures.optionalPhysicalDeviceFeatures.multiDrawIndirect = VK_TRUE; // For DeferredRenderer.
    requestedDeviceFeatures.optionalPhysicalDeviceFeatures.shaderInt64 = VK_TRUE; // For AtomicLoop64Renderer.
    requestedDeviceFeatures.optionalEnableShaderDrawParametersFeatures = VK_TRUE; // For DeferredRenderer.
    requestedDeviceFeatures.optionalVulkan12Features.drawIndirectCount = VK_TRUE; // For DeferredRenderer.
    requestedDeviceFeatures.optionalVulkan12Features.shaderBufferInt64Atomics = VK_TRUE; // For AtomicLoop64Renderer.
    // For PerPixelLinkedListRenderer, OpacityOptimizationRenderer, DepthComplexityRenderer, ...
    requestedDeviceFeatures.requestedPhysicalDeviceFeatures.fragmentStoresAndAtomics = VK_TRUE;
    // For > 4GiB modes in per-pixel linked list renderer.
    requestedDeviceFeatures.optionalPhysicalDeviceFeatures.shaderStorageBufferArrayDynamicIndexing = VK_TRUE;
    requestedDeviceFeatures.optionalPhysicalDeviceFeatures.shaderSampledImageArrayDynamicIndexing = VK_TRUE;
    //requestedDeviceFeatures.optionalPhysicalDeviceFeatures.shaderInt64 = VK_TRUE;
    requestedDeviceFeatures.optionalVulkan12Features.descriptorIndexing = VK_TRUE;
    requestedDeviceFeatures.optionalVulkan12Features.descriptorBindingVariableDescriptorCount = VK_TRUE;
    requestedDeviceFeatures.optionalVulkan12Features.runtimeDescriptorArray = VK_TRUE;
    requestedDeviceFeatures.optionalVulkan12Features.shaderStorageBufferArrayNonUniformIndexing = VK_TRUE;
    device->createDeviceSwapchain(
            instance, window,
            {
                    VK_EXT_SCALAR_BLOCK_LAYOUT_EXTENSION_NAME
            },
            optionalDeviceExtensions, requestedDeviceFeatures);
    sgl::vk::Swapchain* swapchain = new sgl::vk::Swapchain(device);
    swapchain->create(window);
    sgl::AppSettings::get()->setPrimaryDevice(device);
    sgl::AppSettings::get()->setSwapchain(swapchain);
    sgl::AppSettings::get()->initializeSubsystems();
    if (useCustomShaderCompilerBackend) {
        sgl::vk::ShaderManager->setShaderCompilerBackend(shaderCompilerBackend);
    }

#ifdef USE_PYTHON
    sgl::pythonInit(argc, argv);
#endif

    auto app = new MainApp();
    app->run();
    delete app;

    sgl::AppSettings::get()->release();

#ifdef USE_PYTHON
    Py_Finalize();
#endif

#ifdef USE_OSPRAY
    if (OsprayRenderer::getIsOsprayInitialized()) {
        ospShutdown();
    }
#endif

    return 0;
}
