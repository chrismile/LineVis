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

#ifndef LINEVIS_STRUCTUREDGRIDVTKLOADER_HPP
#define LINEVIS_STRUCTUREDGRIDVTKLOADER_HPP

#include <string>

class StreamlineTracingGrid;

/**
 * A VTK file loader. Only structured mesh data is supported at the moment. It is assumed that all points have equal
 * distances (i.e., they form a Cartesian grid).
 */
class StructuredGridVtkLoader {
public:
    static void load(
            const std::string& dataSourceFilename, const GridDataSetMetaData& gridDataSetMetaData,
            StreamlineTracingGrid* grid);

private:
    enum class ReadMode {
        SCALAR, VECTOR, SKIP
    };
    static void _readLines(
            ReadMode readMode, int numObjects, float* fieldData,
            size_t& charPtr, size_t& length, const char* fileBuffer);
    static void _readFieldLine(
            std::string& arrayName, int& numComponents, int& numTuples, std::string& dataType,
            size_t& charPtr, size_t& length, const char* fileBuffer);
    static void _convertScalarFieldCellToPointMode(
            const float* scalarFieldCell, float* scalarFieldPoint, int xs, int ys, int zs);
};

#endif //LINEVIS_STRUCTUREDGRIDVTKLOADER_HPP
