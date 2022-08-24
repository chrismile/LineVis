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

#ifndef LINEVIS_ABCFLOWGENERATOR_HPP
#define LINEVIS_ABCFLOWGENERATOR_HPP

class StreamlineTracingGrid;
struct GridDataSetMetaData;

/**
 * Generator for Arnold-Beltrami-Childress (ABC) flow data.
 * For more details, please refer to: https://en.wikipedia.org/wiki/Arnold%E2%80%93Beltrami%E2%80%93Childress_flow
 * @param xs The grid size in x direction.
 * @param ys The grid size in y direction.
 * @param zs The grid size in z direction.
 * @param vectorField A float array of size xs * ys * zs * 3 storing the 3D velocity vector field.
 */
class AbcFlowGenerator {
public:
    AbcFlowGenerator();

    [[nodiscard]] inline int getGridSizeX() const { return xs; }
    [[nodiscard]] inline int getGridSizeY() const { return ys; }
    [[nodiscard]] inline int getGridSizeZ() const { return zs; }

    /**
     * @param xs The grid size in x direction.
     * @param ys The grid size in y direction.
     * @param zs The grid size in z direction.
     * @param vectorField A float array of size xs * ys * zs * 3 storing the 3D velocity vector field.
     */
    void generateAbcFlow(float *v) const;
    void load(const GridDataSetMetaData& gridDataSetMetaData, StreamlineTracingGrid* grid) const;

    bool renderGui();
    [[nodiscard]] std::string getSettingsString() const;

private:
    int xs = 64, ys = 64, zs = 64;
    float resScale = 6.0f;
    float A, B, C;
};

#endif //LINEVIS_ABCFLOWGENERATOR_HPP
