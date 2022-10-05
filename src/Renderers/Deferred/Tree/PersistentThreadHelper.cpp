/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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

#include <map>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Utils/Device.hpp>
#include "PersistentThreadHelper.hpp"

#ifdef SUPPORT_CUDA_INTEROP
#include <Graphics/Vulkan/Utils/InteropCuda.hpp>
#endif
#ifdef SUPPORT_OPENCL_INTEROP
#include <Graphics/Vulkan/Utils/InteropOpenCL.hpp>
#endif

static std::map<uint64_t, DevicePersistentThreadInfo> deviceMap;

DevicePersistentThreadInfo getDevicePersistentThreadInfo(sgl::vk::Device* device) {
    uint64_t deviceId = uint64_t(device->getDeviceId()) + (uint64_t(device->getVendorId()) << uint64_t(32));
    auto it = deviceMap.find(deviceId);
    if (it != deviceMap.end()) {
        return it->second;
    }

    DevicePersistentThreadInfo info{};
    info.optimalWorkgroupSize = device->getPhysicalDeviceSubgroupProperties().subgroupSize;
    info.optimalNumWorkgroups = 64;

#ifdef SUPPORT_CUDA_INTEROP
    if (device->getDeviceDriverId() == VK_DRIVER_ID_NVIDIA_PROPRIETARY
            && sgl::vk::getIsCudaDeviceApiFunctionTableInitialized()) {
        CUdevice cuDevice = 0;
        bool foundDevice = sgl::vk::getMatchingCudaDevice(device, &cuDevice);

        if (foundDevice) {
            /*
             * Only use one thread block per shader multiprocessor (SM) to improve chance of fair scheduling.
             * See, e.g.: https://stackoverflow.com/questions/33150040/doubling-buffering-in-cuda-so-the-cpu-can-operate-on-data-produced-by-a-persiste/33158954#33158954%5B/
             */
            CUresult cuResult;
            int numMultiprocessors = 16;
            cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuDeviceGetAttribute(
                    &numMultiprocessors, CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, cuDevice);
            sgl::vk::checkCUresult(cuResult, "Error in cuDeviceGetAttribute: ");

            /*
             * TODO: Use more threads than warp size. Factor 4 seems to make sense at least for RTX 3090.
             * For more details see: https://stackoverflow.com/questions/32530604/how-can-i-get-number-of-cores-in-cuda-device
             * Or: https://github.com/NVIDIA/cuda-samples/blob/master/Common/helper_cuda.h
             * https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#compute-capabilities
             * https://developer.nvidia.com/blog/inside-pascal/
             */
            /*int warpSize = 32;
            cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuDeviceGetAttribute(
                    &warpSize, CU_DEVICE_ATTRIBUTE_WARP_SIZE, cuDevice);
            sgl::vk::checkCUresult(cuResult, "Error in cuDeviceGetAttribute: ");

            int major = 0;
            cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuDeviceGetAttribute(
                    &major, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MAJOR, cuDevice);
            sgl::vk::checkCUresult(cuResult, "Error in cuDeviceGetAttribute: ");
            int minor = 0;
            cuResult = sgl::vk::g_cudaDeviceApiFunctionTable.cuDeviceGetAttribute(
                    &minor, CU_DEVICE_ATTRIBUTE_COMPUTE_CAPABILITY_MINOR, cuDevice);
            sgl::vk::checkCUresult(cuResult, "Error in cuDeviceGetAttribute: ");

            // Use warp size * 4 as fallback for unknown architectures.
            int numCoresPerMultiprocessor = warpSize * 4;

            if (major == 2) {
                if (minor == 1) {
                    numCoresPerMultiprocessor = 48;
                } else {
                    numCoresPerMultiprocessor = 32;
                }
            } else if (major == 3) {
                numCoresPerMultiprocessor = 192;
            } else if (major == 5) {
                numCoresPerMultiprocessor = 128;
            } else if (major == 6) {
                if (minor == 0) {
                    numCoresPerMultiprocessor = 64;
                } else {
                    numCoresPerMultiprocessor = 128;
                }
            } else if (major == 7) {
                numCoresPerMultiprocessor = 64;
            } else if (major == 8) {
                if (minor == 0) {
                    numCoresPerMultiprocessor = 64;
                } else {
                    numCoresPerMultiprocessor = 128;
                }
            } else if (major == 9) {
                numCoresPerMultiprocessor = 128;
            }
            info.optimalWorkgroupSize = numCoresPerMultiprocessor;*/

            info.optimalNumWorkgroups = numMultiprocessors;
        }
    }
#endif
#ifdef SUPPORT_OPENCL_INTEROP
#ifdef SUPPORT_CUDA_INTEROP
    else
#endif
    if (sgl::vk::getIsOpenCLFunctionTableInitialized()) {
        cl_int res;
        cl_device_id clDevice = getMatchingOpenCLDevice(device);
        if (clDevice != nullptr) {
            cl_uint maxComputeUnits = 0;
            res = sgl::vk::g_openclFunctionTable.clGetDeviceInfo(
                    clDevice, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(cl_uint), &maxComputeUnits, nullptr);
            sgl::vk::checkResultCL(res, "Error in clGetDeviceInfo[CL_DEVICE_MAX_COMPUTE_UNITS]: ");
            info.optimalNumWorkgroups = maxComputeUnits;
        }
    }
#endif

    sgl::Logfile::get()->write(
            "Persistent thread info: " + std::to_string(info.optimalNumWorkgroups)
            + " x " + std::to_string(info.optimalWorkgroupSize), sgl::BLUE);
    deviceMap.insert(std::make_pair(deviceId, info));

    return info;
}
