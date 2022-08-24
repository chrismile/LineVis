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

#include <cmath>
#include <ImGui/imgui_custom.h>
#include "../StreamlineTracingDefines.hpp"
#include "../StreamlineTracingGrid.hpp"
#include "GridLoader.hpp"
#include "AbcFlowGenerator.hpp"

AbcFlowGenerator::AbcFlowGenerator() {
    A = std::sqrt(3.0f);
    B = std::sqrt(2.0f);
    C = 1.0f;
}

void AbcFlowGenerator::generateAbcFlow(float* v) const {
#if _OPENMP >= 201107
    #pragma omp parallel for default(none) shared(v)
#endif
    for (int iz = 0; iz < zs; iz++) {
        for (int iy = 0; iy < ys; iy++) {
            for (int ix = 0; ix < xs; ix++) {
                float x = float(ix) / float(xs - 1) * resScale;
                float y = float(iy) / float(ys - 1) * resScale;
                float z = float(iz) / float(zs - 1) * resScale;
                v[(iz * xs * ys * 3 + iy * xs * 3 + ix * 3 + 0)] = A * std::sin(z) + C * std::cos(y);
                v[(iz * xs * ys * 3 + iy * xs * 3 + ix * 3 + 1)] = B * std::sin(x) + A * std::cos(z);
                v[(iz * xs * ys * 3 + iy * xs * 3 + ix * 3 + 2)] = C * std::sin(y) + B * std::cos(x);
            }
        }
    }
}

void AbcFlowGenerator::load(const GridDataSetMetaData& gridDataSetMetaData, StreamlineTracingGrid* grid) const {
    float maxDimension = float(std::max(xs - 1, std::max(ys - 1, zs - 1)));
    float cellStep = 1.0f / maxDimension;

    int vectorFieldNumEntries = xs * ys * zs * 3;
    int scalarFieldNumEntries = xs * ys * zs;

    auto* velocityField = new float[vectorFieldNumEntries];
    auto* velocityMagnitudeField = new float[scalarFieldNumEntries];
    auto* vorticityField = new float[vectorFieldNumEntries];
    auto* vorticityMagnitudeField = new float[scalarFieldNumEntries];
    auto* helicityField = new float[scalarFieldNumEntries];

    generateAbcFlow(velocityField);

    computeVectorMagnitudeField(velocityField, velocityMagnitudeField, xs, ys, zs);
    computeVorticityField(velocityField, vorticityField, xs, ys, zs, cellStep, cellStep, cellStep);
    computeVectorMagnitudeField(vorticityField, vorticityMagnitudeField, xs, ys, zs);
    computeHelicityFieldNormalized(
            velocityField, vorticityField, helicityField, xs, ys, zs,
            gridDataSetMetaData.useNormalizedVelocity,
            gridDataSetMetaData.useNormalizedVorticity);

    grid->setGridExtent(xs, ys, zs, cellStep, cellStep, cellStep);
    grid->addVectorField(velocityField, "Velocity");
    grid->addVectorField(vorticityField, "Vorticity");
    grid->addScalarField(helicityField, "Helicity");
    grid->addScalarField(velocityMagnitudeField, "Velocity Magnitude");
    grid->addScalarField(vorticityMagnitudeField, "Vorticity Magnitude");
}

bool AbcFlowGenerator::renderGui() {
    bool changed = false;
    if (ImGui::SliderInt3Edit("Grid Resolution", &xs, 4, 256) == ImGui::EditMode::INPUT_FINISHED) {
        changed = true;
    }
    if (ImGui::SliderFloatEdit(
            "Resolution Scale", &resScale, 1.0f, 16.0f) == ImGui::EditMode::INPUT_FINISHED) {
        changed = true;
    }
    if (ImGui::SliderFloat3Edit("A-B-C", &A, 0.1f, 10.0f) == ImGui::EditMode::INPUT_FINISHED) {
        changed = true;
    }
    return changed;
}

std::string AbcFlowGenerator::getSettingsString() const {
    return
            std::to_string(xs) + "x" + std::to_string(ys) + "x" + std::to_string(zs) + "_"
            + std::to_string(resScale) + "_"
            + std::to_string(A) + "_" + std::to_string(B) + "_" + std::to_string(C);
}
