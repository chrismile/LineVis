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

#include "OpenExrLoader.hpp"

#include <OpenEXR/OpenEXRConfig.h>
#include <OpenEXR/ImfRgbaFile.h>
#define COMBINED_OPENEXR_VERSION ((10000*OPENEXR_VERSION_MAJOR) + (100*OPENEXR_VERSION_MINOR) + OPENEXR_VERSION_PATCH)
#if COMBINED_OPENEXR_VERSION >= 20599
#include <Imath/ImathVec.h>
#include <Imath/half.h>
#else
#include <OpenEXR/ImathVec.h>
#include <OpenEXR/half.h>
#endif

bool loadOpenExrImageFile(const std::string& filename, OpenExrImageInfo& imageInfo) {
    Imf::RgbaInputFile file(filename.c_str());
    Imath::Box2i dw = file.dataWindow();

    imageInfo.width = dw.max.x - dw.min.x + 1;
    imageInfo.height = dw.max.y - dw.min.y + 1;

    imageInfo.pixelData = new uint16_t[imageInfo.width * imageInfo.height * 4];
    auto* rgbaData = reinterpret_cast<Imf::Rgba*>(imageInfo.pixelData);
    file.setFrameBuffer(rgbaData - dw.min.x - dw.min.y * imageInfo.width, 1, imageInfo.width);
    file.readPixels(dw.min.y, dw.max.y);

    return true;
}
