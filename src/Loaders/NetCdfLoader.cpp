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

#include <iostream>
#include <algorithm>
#include <cstring>
#include <vector>
#include <memory>
#include <fstream>
#include <iomanip>
#include <cassert>

#include <glm/glm.hpp>
#include <netcdf.h>

#include "NetCdfLoader.hpp"

#if defined(DEBUG) || !defined(NDEBUG)
#define myassert assert
#else
#define myassert(x)                                   \
	if (!(x))                                         \
	{                                                 \
		std::cerr << "assertion failed" << std::endl; \
		exit(1);                                      \
	}
#endif

/// Sets pointer to NULL after deletion.
#define SAFE_DELETE(x) (if (x != NULL) { delete x; x = NULL; })
#define SAFE_DELETE_ARRAY(x) if (x != NULL) { delete[] x; x = NULL; }

const float MISSING_VALUE = -999.E9;


/**
 * Queries a global string attribute.
 * @param ncid The NetCDF file ID.
 * @param varname The name of the global variable.
 * @return The content of the variable.
 */
std::string getGlobalStringAttribute(int ncid, const char* varname) {
    size_t stringLength = 0;
    myassert(nc_inq_attlen(ncid, NC_GLOBAL, varname, &stringLength) == NC_NOERR);
    char *stringVar = new char[stringLength + 1];
    nc_get_att_text(ncid, NC_GLOBAL, varname, stringVar);
    stringVar[stringLength] = '\0';
    std::string retString(stringVar);
    delete[] stringVar;
    return retString;
}

/**
 * Returns the size of a dimension as an integer.
 * @param ncid The NetCDF file ID.
 * @param dimname The name of the dimension, e.g. "time".
 * @return The dimension size.
 */
size_t getDim(int ncid, const char* dimname) {
    int dimid;
    size_t dimlen;
    myassert(nc_inq_dimid(ncid, dimname, &dimid) == NC_NOERR);
    myassert(nc_inq_dimlen(ncid, dimid, &dimlen) == NC_NOERR);
    return dimlen;
}

/**
 * Loads a 1D floating point variable.
 * @param ncid The NetCDF file ID.
 * @param varname The name of the variable, e.g. "time".
 * @param len The dimension size queried by @ref getDim.
 * @param array A pointer to a float array where the variable data is to be stored.
 *              The function will automatically allocate the memory.
 *              The caller needs to deallocate the allocated memory using "delete[]".
 */
void loadFloatArray1D(int ncid, const char* varname, size_t len, float** array) {
    int varid;
    myassert(nc_inq_varid(ncid, varname, &varid) == NC_NOERR);
    *array = new float[len];
    size_t startp[] = {0};
    size_t countp[] = {len};
    myassert(nc_get_vara_float(ncid, varid, startp, countp, *array) == NC_NOERR);
}

/**
 * Loads a 1D floating point variable.
 * @param ncid The NetCDF file ID.
 * @param varname The name of the variable, e.g. "time".
 * @param start Offset from start of file buffer.
 * @param len Number of values to read.
 * @param array A pointer to a float array where the variable data is to be stored.
 *              The function will automatically allocate the memory.
 *              The caller needs to deallocate the allocated memory using "delete[]".
 */
void loadFloatArray1D(int ncid, const char* varname, size_t start, size_t len, float **array) {
    int varid;
    myassert(nc_inq_varid(ncid, varname, &varid) == NC_NOERR);
    *array = new float[len];
    size_t startp[] = {start};
    size_t countp[] = {len};
    myassert(nc_get_vara_float(ncid, varid, startp, countp, *array) == NC_NOERR);
}

/**
 * Loads a 1D double-precision floating point variable.
 * @param ncid The NetCDF file ID.
 * @param varname The name of the variable, e.g. "time".
 * @param len The dimension size queried by @ref getDim.
 * @param array A pointer to a float array where the variable data is to be stored.
 *              The function will automatically allocate the memory.
 *              The caller needs to deallocate the allocated memory using "delete[]".
 */
void loadDoubleArray1D(int ncid, const char* varname, size_t len, double** array) {
    int varid;
    myassert(nc_inq_varid(ncid, varname, &varid) == NC_NOERR);
    *array = new double[len];
    size_t startp[] = {0};
    size_t countp[] = {len};
    myassert(nc_get_vara_double(ncid, varid, startp, countp, *array) == NC_NOERR);
}

/**
 * Loads a 2D double-precision floating point variable.
 * @param ncid The NetCDF file ID.
 * @param varname The name of the variable, e.g. "time".
 * @param ylen Dimension size queried by @ref getDim.
 * @param xlen Dimension size queried by @ref getDim.
 * @param array A pointer to a float array where the variable data is to be stored.
 *              The function will automatically allocate the memory.
 *              The caller needs to deallocate the allocated memory using "delete[]".
 */
void loadDoubleArray2D(int ncid, const char* varname, size_t ylen, size_t xlen, double** array) {
    int varid;
    myassert(nc_inq_varid(ncid, varname, &varid) == NC_NOERR);
    *array = new double[ylen * xlen];
    size_t startp[] = {0, 0};
    size_t countp[] = {ylen, xlen};
    myassert(nc_get_vara_double(ncid, varid, startp, countp, *array) == NC_NOERR);
}

/**
 * Loads a 2D floating point variable.
 * @param ncid The NetCDF file ID.
 * @param varname The name of the variable, e.g. "time".
 * @param ylen Dimension size queried by @ref getDim.
 * @param xlen Dimension size queried by @ref getDim.
 * @param array A pointer to a float array where the variable data is to be stored.
 *              The function will automatically allocate the memory.
 *              The caller needs to deallocate the allocated memory using "delete[]".
 */
void loadFloatArray2D(int ncid, const char* varname, size_t ylen, size_t xlen, float** array) {
    int varid;
    myassert(nc_inq_varid(ncid, varname, &varid) == NC_NOERR);
    *array = new float[ylen * xlen];
    size_t startp[] = {0, 0};
    size_t countp[] = {ylen, xlen};
    myassert(nc_get_vara_float(ncid, varid, startp, countp, *array) == NC_NOERR);
}

/**
 * Loads a 3D floating point variable.
 * @param ncid The NetCDF file ID.
 * @param varname The name of the variable, e.g. "time".
 * @param zlen Dimension size queried by @ref getDim.
 * @param ylen Dimension size queried by @ref getDim.
 * @param xlen Dimension size queried by @ref getDim.
 * @param array A pointer to a float array where the variable data is to be stored.
 *              The function will automatically allocate the memory.
 *              The caller needs to deallocate the allocated memory using "delete[]".
 */
void loadFloatArray3D(int ncid, const char* varname, size_t zlen, size_t ylen, size_t xlen, float** array) {
    int varid;
    myassert(nc_inq_varid(ncid, varname, &varid) == NC_NOERR);
    *array = new float[zlen * ylen * xlen];
    size_t startp[] = {0, 0, 0};
    size_t countp[] = {zlen, ylen, xlen};
    myassert(nc_get_vara_float(ncid, varid, startp, countp, *array) == NC_NOERR);
}

/**
 * Loads a 3D floating point variable.
 * @param ncid The NetCDF file ID.
 * @param varname The name of the variable, e.g. "time".
 * @param zstart Dimension size queried by @ref getDim.
 * @param ystart Dimension size queried by @ref getDim.
 * @param xstart Dimension size queried by @ref getDim.
 * @param zlen Dimension size queried by @ref getDim.
 * @param ylen Dimension size queried by @ref getDim.
 * @param xlen Dimension size queried by @ref getDim.
 * @param array A pointer to a float array where the variable data is to be stored.
 *              The function will automatically allocate the memory.
 *              The caller needs to deallocate the allocated memory using "delete[]".
 */
void loadFloatArray3D(
        int ncid, const char* varname, size_t zstart, size_t ystart, size_t xstart,
        size_t zlen, size_t ylen, size_t xlen, float** array) {
    int varid;
    myassert(nc_inq_varid(ncid, varname, &varid) == NC_NOERR);
    *array = new float[zlen * ylen * xlen];
    size_t startp[] = {zstart, ystart, xstart};
    size_t countp[] = {zlen, ylen, xlen};
    myassert(nc_get_vara_float(ncid, varid, startp, countp, *array) == NC_NOERR);
}

std::string getStringAttribute(int ncid, int varid, const char* attname) {
    nc_type type;
    size_t length = 0;
    myassert(nc_inq_att(ncid, varid, attname, &type, &length) == NC_NOERR);
    myassert(type == NC_CHAR);
    char* charArray = new char[length];
    nc_get_att_text(ncid, varid, attname, charArray);
    std::string attText = std::string(charArray, length);
    delete[] charArray;
    return attText;
}


Trajectories convertLatLonToCartesian(
        const float* lat, const float* lon, float* pressure, size_t trajectoryDim, size_t timeDim) {
    Trajectories trajectories;
    trajectories.reserve(timeDim);

    float minPressure = std::numeric_limits<float>::max();
    float maxPressure = std::numeric_limits<float>::lowest();
#if _OPENMP >= 201107
	#pragma omp parallel for shared(trajectoryDim, timeDim, pressure) \
    reduction(min:minPressure) reduction(max:maxPressure) default(none)
#endif
	for (size_t idx = 0; idx < trajectoryDim*timeDim; idx++) {
	    if (pressure[idx] > 0.0f) {
            minPressure = std::min(minPressure, pressure[idx]);
	    }
		maxPressure = std::max(maxPressure, pressure[idx]);
	}
	float logMinPressure = std::log(minPressure);
	float logMaxPressure = std::log(maxPressure);

    for (int trajectoryIndex = 0; trajectoryIndex < trajectoryDim; trajectoryIndex++) {
        Trajectory trajectory;
        //trajectory.attributes.resize(1);
        std::vector<glm::vec3> &cartesianCoords = trajectory.positions;
        //std::vector<float> &pressureAttr = trajectory.attributes.at(0);
        cartesianCoords.reserve(trajectoryDim);
        //pressureAttr.reserve(trajectoryDim);
        for (size_t i = 0; i < timeDim; i++) {
            size_t index = i + trajectoryIndex * timeDim;
            float pressureAtIdx = pressure[index];
            if (pressureAtIdx <= 0.0f) {
                continue;
            }
            float normalizedLogPressure = (std::log(pressureAtIdx) - logMaxPressure) / (logMinPressure - logMaxPressure);
            float x = lat[index];
            float y = normalizedLogPressure;
            float z = lon[index];

            glm::vec3 cartesianCoord = glm::vec3(x, y, z);

            cartesianCoords.push_back(cartesianCoord);
            //pressureAttr.push_back(pressureAtIdx);
        }

        //if (!trajectory.positions.empty()) {
        trajectories.push_back(trajectory);
        //}
    }
    return trajectories;
}

/**
 * Exports the passed trajectories to an .obj file. The normalized pressure is stored as a texture coordinate.
 * @param trajectories The trajectory paths to export.
 * @param filename The filename of the .obj file.
 */
void exportObjFile(Trajectories& trajectories, const std::string& filename) {
    std::ofstream outfile;
    outfile.open(filename.c_str());
    if (!outfile.is_open()) {
        std::cerr << "Error in exportObjFile: File \"" << filename << "\" couldn't be opened for writing!" << std::endl;
        exit(1);
        return;
    }

    // We want five digits in output file
    outfile << std::setprecision(5);

    // Index of the next point
    size_t objPointIndex = 1;

    size_t trajectoryFileIndex = 0;
    for (size_t trajectoryIndex = 0; trajectoryIndex < trajectories.size(); trajectoryIndex++) {
        Trajectory& trajectory = trajectories.at(trajectoryIndex);
        size_t trajectorySize = trajectory.positions.size();
        if (trajectorySize < 2) {
            continue;
        }

        for (size_t i = 0; i < trajectorySize; i++) {
            glm::vec3 &v = trajectory.positions.at(i);
            outfile << "v " << std::setprecision(5) << v.x << " " << v.y << " " << v.z << "\n";
            outfile << "vt " << std::setprecision(5) << trajectory.attributes.at(0).at(i) << "\n";
        }

        outfile << "g line" << trajectoryFileIndex << "\n";
        outfile << "l ";
        for (size_t i = 1; i < trajectorySize+1; i++) {
            outfile << objPointIndex << " ";
            objPointIndex++;
        }
        outfile << "\n\n";
        trajectoryFileIndex++;
    }
    outfile.close();
}

Trajectories loadTrajectoriesFromNetCdf(const std::string& filename, std::vector<std::string>& attributeNames) {
    Trajectories trajectories;

    // File handle
    int ncid;

    // Open the NetCDF file for reading
    int status = nc_open(filename.c_str(), NC_NOWRITE, &ncid);
    if (status != 0) {
        std::cerr << "Error in loadNetCdfFile: File \"" << filename << "\" couldn't be opened!" << std::endl;
        return trajectories;
    }

    // Load dimension data
    size_t timeDim = getDim(ncid, "time");
    size_t trajectoryDim = getDim(ncid, "trajectory");
    size_t ensembleDim = getDim(ncid, "ensemble");

    // Load data arrays
    double* time = nullptr;
    float* lon = nullptr, *lat = nullptr, *pressure = nullptr;
    loadDoubleArray1D(ncid, "time", timeDim, &time);
    loadFloatArray3D(ncid, "lon", 1, trajectoryDim, timeDim, &lon);
    loadFloatArray3D(ncid, "lat", 1, trajectoryDim, timeDim, &lat);
    loadFloatArray3D(ncid, "pressure", 1, trajectoryDim, timeDim, &pressure);

    trajectories = convertLatLonToCartesian(lat, lon, pressure, trajectoryDim, timeDim);

    // DIM: ensemble, trajectory, time
    std::vector<std::string> blacklistNames = {
            "time", "lon", "lat", "ensemble", "trajectory"
    };
    int nvarsp = 0;
    int dimids[NC_MAX_VAR_DIMS];
    char varname[NC_MAX_NAME];
    char attname[NC_MAX_NAME];
    myassert(nc_inq(ncid, nullptr, &nvarsp, nullptr, nullptr) == NC_NOERR);
    for (int varid = 0; varid < nvarsp; varid++) {
        nc_type type = NC_FLOAT;
        int ndims = 0;
        int natts = 0;
        nc_inq_var(ncid, varid, varname, &type, &ndims, dimids, &natts);
        if (type != NC_FLOAT || ndims != 3) {
            continue;
        }

        bool isAuxiliaryData = strcmp(varname, "pressure") == 0;
        std::string variableDisplayName = varname;
        for (int attnum = 0; attnum < natts; attnum++) {
            nc_inq_attname(ncid, varid, attnum, attname);
            if (strcmp(attname, "standard_name") == 0) {
                variableDisplayName = getStringAttribute(ncid, varid, "standard_name");
            }
            if (strcmp(attname, "auxiliary_data") == 0) {
                isAuxiliaryData = getStringAttribute(ncid, varid, "auxiliary_data") == "yes";
            }
        }
        attributeNames.push_back(variableDisplayName);

        float *varData = nullptr;
        loadFloatArray3D(ncid, varname, 1, trajectoryDim, timeDim, &varData);

        for (int trajectoryIndex = 0; trajectoryIndex < trajectoryDim; trajectoryIndex++) {
            Trajectory& trajectory = trajectories.at(trajectoryIndex);
            std::vector<float> attributeValues;
            attributeValues.reserve(timeDim);
            for (size_t i = 0; i < timeDim; i++) {
                size_t index = i + trajectoryIndex * timeDim;
                float pressureAtIdx = pressure[index];
                if (pressureAtIdx <= 0.0f) {
                    continue;
                }
                attributeValues.push_back(varData[index]);
            }
            trajectory.attributes.push_back(attributeValues);
        }

        SAFE_DELETE_ARRAY(varData);
    }

    //std::string outputFilename = filename.substr(0, filename.find_last_of(".")) + ".obj";
    //exportObjFile(trajectories, outputFilename);

    // Close the file
    myassert(nc_close(ncid) == NC_NOERR);

    SAFE_DELETE_ARRAY(time);
    SAFE_DELETE_ARRAY(lon);
    SAFE_DELETE_ARRAY(lat);
    SAFE_DELETE_ARRAY(pressure);

    return trajectories;
}
