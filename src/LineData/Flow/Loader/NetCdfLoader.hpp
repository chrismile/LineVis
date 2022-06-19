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

#ifndef LINEVIS_NETCDFLOADER_HPP
#define LINEVIS_NETCDFLOADER_HPP

#include <string>

class StreamlineTracingGrid;

/**
 * A loader for NetCDF volume data sets.
 */
class NetCdfLoader {
public:
    static void load(const std::string& dataSourceFilename, StreamlineTracingGrid* grid);

private:
    static bool getDimensionExists(int ncid, const std::string& dimensionName);
    static bool getVariableExists(int ncid, const std::string& variableName);

    /**
     * Loads a 1D floating point variable.
     * @param ncid The NetCDF file ID.
     * @param varname The name of the variable, e.g. "time".
     * @param len The dimension size queried by @ref getDim.
     * @param array A pointer to a float array where the variable data is to be stored. The function will automatically
     * allocate the memory. The caller needs to deallocate the allocated memory using "delete[]".
     */
    static void loadFloatArray1D(int ncid, const char* varname, size_t len, float*& array);

    /**
     * Loads a 1D floating point variable.
     * @param ncid The NetCDF file ID.
     * @param varid The ID of the variable.
     * @param len The dimension size queried by @ref getDim.
     * @param array A pointer to a float array where the variable data is to be stored. The function will automatically
     * allocate the memory. The caller needs to deallocate the allocated memory using "delete[]".
     */
    static void loadFloatArray1D(int ncid, int varid, size_t len, float*& array);

    /**
     * Loads a 3D floating point variable.
     * @param ncid The NetCDF file ID.
     * @param varid The ID of the variable.
     * @param zlen Dimension size queried by @ref getDim.
     * @param ylen Dimension size queried by @ref getDim.
     * @param xlen Dimension size queried by @ref getDim.
     * @param array A pointer to a float array where the variable data is to be stored. The function will automatically
     * allocate the memory. The caller needs to deallocate the allocated memory using "delete[]".
     */
    static void loadFloatArray3D(int ncid, int varid, size_t zlen, size_t ylen, size_t xlen, float*& array);

    /**
     * Loads a 3D floating point variable.
     * @param ncid The NetCDF file ID.
     * @param varid The ID of the variable.
     * @param time The time step to load.
     * @param zlen Dimension size queried by @ref getDim.
     * @param ylen Dimension size queried by @ref getDim.
     * @param xlen Dimension size queried by @ref getDim.
     * @param array A pointer to a float array where the variable data is to be stored. The function will automatically
     * allocate the memory. The caller needs to deallocate the allocated memory using "delete[]".
     */
    static void loadFloatArray3D(
            int ncid, int varid, size_t time, size_t zlen, size_t ylen, size_t xlen, float*& array);

    /**
     * Queries a string attribute of a variable.
     * @param ncid The NetCDF file ID.
     * @param varid The ID of the variable.
     * @param attname The name of the attribute to query.
     * @return The attribute string.
     */
    static std::string getStringAttribute(int ncid, int varid, const char* attname);
};

#endif //LINEVIS_NETCDFLOADER_HPP
