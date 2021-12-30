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

#ifndef LINEVIS_STREAMLINETRACINGDEFINES_HPP
#define LINEVIS_STREAMLINETRACINGDEFINES_HPP

#include <memory>

enum class StreamlineTracingDataSource {
    // A .vtk file storing a STRUCTURED_GRID data set.
    VTK_STRUCTURED_GRID_FILE,
    // A Rayleigh-Bénard convection (RBC) data set storing a 1024x32x1024 grid of (Vx, Vy, Vz, T) values.
    RBC_BIN_FILE
};

enum class FlowPrimitives {
    STREAMLINES, STREAMRIBBONS
};
const char* const FLOW_PRIMITIVE_NAMES[] = {
        "Streamlines", "Streamribbons"
};

enum class StreamlineSeedingStrategy {
    // Seed the lines in a 3D box.
    VOLUME,
    // Seed the lines in a 2D plane.
    PLANE,
    // Seed the lines starting from cells with the highest helicity.
    MAX_HELICITY_FIRST
};
const char* const STREAMLINE_SEEDING_STRATEGY_NAMES[] = {
        "Volume", "Plane", "Max. Helicity First"
};

class StreamlineSeeder;
typedef std::shared_ptr<StreamlineSeeder> StreamlineSeederPtr;
class StreamlineSeeder;
class StreamlineTracingGrid;

#define IDXV(x,y,z,c) ((z)*xs*ys*3 + (y)*xs*3 + (x)*3 + (c))
#define IDXV4(x,y,z,c) ((z)*xs*ys*4 + (y)*xs*4 + (x)*4 + (c))
#define IDXS(x,y,z) ((z)*xs*ys + (y)*xs + (x))

struct StreamlineTracingSettings {
    std::string dataSourceFilename{};
    FlowPrimitives flowPrimitives = FlowPrimitives::STREAMLINES;
    int numPrimitives = 1024;
    StreamlineSeedingStrategy streamlineSeedingStrategy = StreamlineSeedingStrategy::VOLUME;
    StreamlineSeederPtr seeder = nullptr;
};

#endif //LINEVIS_STREAMLINETRACINGDEFINES_HPP
