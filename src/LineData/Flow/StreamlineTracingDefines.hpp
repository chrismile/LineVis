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

#include <string>
#include <memory>
#include <glm/vec3.hpp>
#include "Loader/AbcFlowGenerator.hpp"

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

enum class StreamlineIntegrationMethod {
    // Euler's method (1st order).
    EXPLICIT_EULER,
    // Backward Euler's method (implicit Euler).
    IMPLICIT_EULER,
    // Heun's method (2nd order).
    HEUN,
    // Midpoint method (Runge-Kutta of 2nd order).
    MIDPOINT,
    // Runge-Kutta of 4th order.
    RK4,
    // Runge-Kutta-Fehlberg.
    RKF45
};
const char* const STREAMLINE_INTEGRATION_METHOD_NAMES[] = {
        "Explicit Euler", "Implicit Euler", "Heun", "Midpoint",
        "Runge-Kutta 4th Order", "Runge-Kutta-Fehlberg"
};

enum class StreamlineIntegrationDirection {
    FORWARD, BACKWARD, BOTH
};
const char* const STREAMLINE_INTEGRATION_DIRECTION_NAMES[] = {
        "Forward", "Backward", "Forward & Backward"
};

enum class TerminationCheckType {
    NAIVE, //< Searches along all previously traced trajectories.
    GRID_BASED, //< Stores an occupancy grid.
    KD_TREE_BASED, //< Searches for nearest neighbors using a kd-tree.
    HASHED_GRID_BASED //< Searches for nearest neighbors using a hashed grid.
};
const char* const TERMINATION_CHECK_TYPE_NAMES[] = {
        "Naive (O(n^2))", "Grid-based", "k-d Tree-based", "Hashed Grid-based"
};

enum class LoopCheckMode {
    NONE, START_POINT, ALL_POINTS, GRID, CURVATURE
};
const char* const LOOP_CHECK_MODE_NAMES[] = {
        "None", "Start Point", "All Points", "Grid", "Curvature"
};

class StreamlineSeeder;
typedef std::shared_ptr<StreamlineSeeder> StreamlineSeederPtr;
class StreamlineSeeder;
class StreamlineTracingGrid;

#define IDXV(x,y,z,c) ((z)*xs*ys*3 + (y)*xs*3 + (x)*3 + (c))
#define IDXV4(x,y,z,c) ((z)*xs*ys*4 + (y)*xs*4 + (x)*4 + (c))
#define IDXS(x,y,z) ((z)*xs*ys + (y)*xs + (x))
#define IDXS_C(x,y,z) ((z)*(xs-1)*(ys-1) + (y)*(xs-1) + (x))

struct StreamlineTracingSettings {
    bool isAbcDataSet = false;
    std::string dataSourceFilename{};
    FlowPrimitives flowPrimitives = FlowPrimitives::STREAMRIBBONS;
    int numPrimitives = 1024;
    StreamlineSeedingStrategy streamlineSeedingStrategy = StreamlineSeedingStrategy::VOLUME;
    StreamlineSeederPtr seeder = nullptr;
    float timeStepScale = 1.0f;
    int maxNumIterations = 2000;
    float terminationDistance = 1.0f;
    float terminationDistanceSelf = 1.0f;
    float minimumLength = 0.7f;
    float minimumSeparationDistance = 0.08f;
    TerminationCheckType terminationCheckType = TerminationCheckType::GRID_BASED;
    bool showSimulationGridOutline = true;
    bool smoothedSimulationGridOutline = true;
    StreamlineIntegrationMethod integrationMethod = StreamlineIntegrationMethod::RK4;
    StreamlineIntegrationDirection integrationDirection = StreamlineIntegrationDirection::BOTH;
    int vectorFieldIndex = 0;
    AbcFlowGenerator abcFlowGenerator;
    LoopCheckMode loopCheckMode = LoopCheckMode::START_POINT;

    // For flowPrimitives == FlowPrimitives::STREAMRIBBONS.
    bool useHelicity = true;
    float maxHelicityTwist = 0.25f;
    glm::vec3 initialRibbonDirection = glm::vec3(0.0f, 1.0f, 0.0f);
    int gridSubsamplingFactor = 1;

    // For saving to the disk.
    bool exportToDisk = false;
    std::string exportPath;
};

#endif //LINEVIS_STREAMLINETRACINGDEFINES_HPP
