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

#ifndef STRESSLINEVIS_SYNCMODE_HPP
#define STRESSLINEVIS_SYNCMODE_HPP

namespace sgl { namespace vk {
class Device;
}}

struct SceneData;

enum SyncMode {
    NO_SYNC, SYNC_FRAGMENT_SHADER_INTERLOCK, SYNC_SPINLOCK
};

/**
 * @param device The Vulkan device to check the sync mode support for.
 * @return SYNC_FRAGMENT_SHADER_INTERLOCK if supported, SYNC_SPINLOCK otherwise.
 */
SyncMode getSupportedSyncMode(sgl::vk::Device* device);

/**
 * Checks whether the passed sync mode is supported and overwrites it with a supported mode if not.
 * @param sceneData The scene data used for enqueueing a non-blocking warning dialog.
 * @param device The Vulkan device to check the sync mode support for.
 * @param syncMode The current sync mode, which is overwritten if it is not supported by the device.
 */
void checkSyncModeSupported(SceneData* sceneData, sgl::vk::Device* device, SyncMode& syncMode);

#endif //STRESSLINEVIS_SYNCMODE_HPP
