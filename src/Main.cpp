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

#ifdef USE_PYTHON
#if defined(PYTHONHOME_PATH) || defined(__APPLE__)
#include <cstdlib>
#endif
#include <Python.h>
#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif
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

#include "MainApp.hpp"

int main(int argc, char *argv[]) {
    // Initialize the filesystem utilities.
    sgl::FileUtils::get()->initialize("LineVis", argc, argv);

    // Load the file containing the app settings
    std::string settingsFile = sgl::FileUtils::get()->getConfigDirectory() + "settings.txt";
    sgl::AppSettings::get()->loadSettings(settingsFile.c_str());
    sgl::AppSettings::get()->getSettings().addKeyValue("window-multisamples", 0);
    sgl::AppSettings::get()->getSettings().addKeyValue("window-debugContext", true);
    if (argc > 1 && strcmp(sgl::FileUtils::get()->get_argv()[1], "--perf") == 0) {
        sgl::AppSettings::get()->getSettings().addKeyValue("window-vSync", false);
    } else {
        sgl::AppSettings::get()->getSettings().addKeyValue("window-vSync", true);
    }
    sgl::AppSettings::get()->getSettings().addKeyValue("window-resizable", true);
    sgl::AppSettings::get()->getSettings().addKeyValue("window-savePosition", true);
#ifdef DATA_PATH
    if (!sgl::FileUtils::get()->directoryExists("Data") && !sgl::FileUtils::get()->directoryExists("../Data")) {
        sgl::AppSettings::get()->setDataDirectory(DATA_PATH);
    }
#endif

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
    optionalDeviceExtensions.push_back(VK_NV_MESH_SHADER_EXTENSION_NAME);
    optionalDeviceExtensions.push_back(VK_NV_FRAGMENT_SHADER_BARYCENTRIC_EXTENSION_NAME);
    optionalDeviceExtensions.push_back(VK_KHR_DRAW_INDIRECT_COUNT_EXTENSION_NAME);

    sgl::vk::Instance* instance = sgl::AppSettings::get()->getVulkanInstance();
    sgl::vk::Device* device = new sgl::vk::Device;
    sgl::vk::DeviceFeatures requestedDeviceFeatures{};
    requestedDeviceFeatures.optionalPhysicalDeviceFeatures.geometryShader = VK_TRUE; // For a rasterizer mode.
    requestedDeviceFeatures.optionalPhysicalDeviceFeatures.sampleRateShading = VK_TRUE; // For OpaqueLineRenderer.
    requestedDeviceFeatures.optionalPhysicalDeviceFeatures.independentBlend = VK_TRUE; // For WBOITRenderer.
    requestedDeviceFeatures.optionalPhysicalDeviceFeatures.samplerAnisotropy = VK_TRUE; // For LineDataFlow textures.
    // For PerPixelLinkedListRenderer, OpacityOptimizationRenderer, DepthComplexityRenderer, ...
    requestedDeviceFeatures.requestedPhysicalDeviceFeatures.fragmentStoresAndAtomics = VK_TRUE;
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

#ifdef USE_PYTHON
#ifdef PYTHONHOME
    const char* pythonhomeEnvVar = getenv("PYTHONHOME");
    if (!pythonhomeEnvVar || strlen(pythonhomeEnvVar) == 0) {
#if !defined(__APPLE__) || !defined(PYTHONPATH)
        Py_SetPythonHome(PYTHONHOME);
#endif
        // As of 2022-01-25, "lib-dynload" is not automatically found when using MSYS2 together with MinGW.
#if (defined(__MINGW32__) || defined(__APPLE__)) && defined(PYTHONPATH)
#ifdef __MINGW32__
        Py_SetPath(PYTHONPATH ";" PYTHONPATH "/site-packages;" PYTHONPATH "/lib-dynload");
#else
        std::wstring pythonhomeWide = PYTHONHOME;
        std::string pythonhomeNormal(pythonhomeWide.size(), ' ');
        pythonhomeNormal.resize(std::wcstombs(
                &pythonhomeNormal[0], pythonhomeWide.c_str(), pythonhomeWide.size()));
        std::wstring pythonpathWide = PYTHONPATH;
        std::string pythonpathNormal(pythonpathWide.size(), ' ');
        pythonpathNormal.resize(std::wcstombs(
                &pythonpathNormal[0], pythonpathWide.c_str(), pythonpathWide.size()));
        if (!sgl::FileUtils::get()->exists(pythonhomeNormal)) {
            uint32_t pathBufferSize = 0;
            _NSGetExecutablePath(nullptr, &pathBufferSize);
            char* pathBuffer = new char[pathBufferSize];
            _NSGetExecutablePath(pathBuffer, &pathBufferSize);
            std::string executablePythonHome =
                    sgl::FileUtils::get()->getPathToFile(std::string() + pathBuffer) + "python3";
            if (sgl::FileUtils::get()->exists(executablePythonHome)) {
                std::wstring pythonHomeLocal(executablePythonHome.size(), L' ');
                pythonHomeLocal.resize(std::mbstowcs(
                        &pythonHomeLocal[0], executablePythonHome.c_str(), executablePythonHome.size()));
                std::string pythonVersionString = sgl::FileUtils::get()->getPathAsList(pythonpathNormal).back();
                 std::wstring pythonVersionStringWide(pythonVersionString.size(), L' ');
                pythonVersionStringWide.resize(std::mbstowcs(
                        &pythonVersionStringWide[0], pythonVersionString.c_str(), pythonVersionString.size()));
               std::wstring pythonPathLocal =
                        pythonHomeLocal + L"/lib/" + pythonVersionStringWide;
                std::wstring inputPath =
                        pythonPathLocal + L":"
                        + pythonPathLocal + L"/site-packages:"
                        + pythonPathLocal + L"/lib-dynload";
                Py_SetPythonHome(pythonHomeLocal.c_str());
                Py_SetPath(inputPath.c_str());
            } else {
                sgl::Logfile::get()->throwError("Fatal error: Couldn't find Python home.");
            }
            delete[] pathBuffer;
        } else {
            Py_SetPythonHome(PYTHONHOME);
            Py_SetPath(PYTHONPATH ":" PYTHONPATH "/site-packages:" PYTHONPATH "/lib-dynload");
        }
#endif
#endif
    }
#endif
    wchar_t** argvWidestr = (wchar_t**)PyMem_Malloc(sizeof(wchar_t*) * argc);
    for (int i = 0; i < argc; i++) {
        wchar_t* argWidestr = Py_DecodeLocale(argv[i], nullptr);
        argvWidestr[i] = argWidestr;
    }
    Py_Initialize();
    PySys_SetArgv(argc, argvWidestr);
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
