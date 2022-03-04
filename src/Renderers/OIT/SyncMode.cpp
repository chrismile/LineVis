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

#include <Utils/Dialog.hpp>
#include <Utils/File/Logfile.hpp>
#include <Graphics/Vulkan/Utils/Device.hpp>

#include "Renderers/SceneData.hpp"
#include "SyncMode.hpp"

SyncMode getSupportedSyncMode(sgl::vk::Device* device) {
    if (!device->isDeviceExtensionSupported(VK_EXT_FRAGMENT_SHADER_INTERLOCK_EXTENSION_NAME)) {
        return SYNC_SPINLOCK;
    }
    return SYNC_FRAGMENT_SHADER_INTERLOCK;
}

void checkSyncModeSupported(SceneData* sceneData, sgl::vk::Device* device, SyncMode& syncMode) {
    if (syncMode == SYNC_FRAGMENT_SHADER_INTERLOCK
            && !device->isDeviceExtensionSupported(VK_EXT_FRAGMENT_SHADER_INTERLOCK_EXTENSION_NAME)) {
        std::string warningText =
                "Fragment shader interlock is not supported by the used GPU. "
                "Falling back to a supported rendering mode.";
        sgl::Logfile::get()->writeWarning(warningText, false);
        auto handle = sgl::dialog::openMessageBox(
                "Unsupported Synchronization Mode", warningText, sgl::dialog::Icon::WARNING);
        sceneData->nonBlockingMsgBoxHandles->push_back(handle);
        syncMode = getSupportedSyncMode(device);
    }
}
