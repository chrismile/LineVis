/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Christoph Neuhauser
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

#ifndef LINEVIS_RBCBINFILELOADER_HPP
#define LINEVIS_RBCBINFILELOADER_HPP

#include <string>

class StreamlineTracingGrid;

/**
 * A loader for a custom binary format storing Rayleigh–Bénard convection data from the paper:
 * A. Frasson, M. Ender, S. Weiss, M. Kanzler, A. Pandey, J. Schumacher, R. Westermann.
 * Visual Exploration of Circulation Rolls in Convective Heat Flows. IEEE Pacific Visualization 2019.
 *
 * The file format stores 1024x32x1024 velocity and temperature values at the points of a Cartesian grid in the order
 * (Vx, Vy, Vz, T).
 */
class RbcBinFileLoader {
public:
    static void load(
            const std::string& dataSourceFilename, const GridDataSetMetaData& gridDataSetMetaData,
            StreamlineTracingGrid* grid);
};

#endif //LINEVIS_RBCBINFILELOADER_HPP
