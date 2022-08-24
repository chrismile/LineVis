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

#include <iostream>
#include <set>
#include <cassert>
#include <cstring>

#include <netcdf.h>

#include <Utils/File/Logfile.hpp>

#include "../StreamlineTracingDefines.hpp"
#include "../StreamlineTracingGrid.hpp"
#include "GridLoader.hpp"
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

bool NetCdfLoader::getDimensionExists(int ncid, const std::string& dimensionName) {
    int dimid = 0;
    int status = nc_inq_dimid(ncid, dimensionName.c_str(), &dimid);
    return status != NC_EBADDIM;
}

bool NetCdfLoader::getVariableExists(int ncid, const std::string& variableName) {
    int varid;
    int status = nc_inq_varid(ncid, variableName.c_str(), &varid);
    return status != NC_ENOTVAR;
}

void NetCdfLoader::loadFloatArray1D(int ncid, const char* varname, size_t len, float*& array) {
    int varid;
    myassert(nc_inq_varid(ncid, varname, &varid) == NC_NOERR);
    array = new float[len];
    size_t startp[] = { 0 };
    size_t countp[] = { len };
    myassert(nc_get_vara_float(ncid, varid, startp, countp, array) == NC_NOERR);
}

void NetCdfLoader::loadFloatArray1D(int ncid, int varid, size_t len, float*& array) {
    array = new float[len];
    size_t startp[] = { 0 };
    size_t countp[] = { len };
    myassert(nc_get_vara_float(ncid, varid, startp, countp, array) == NC_NOERR);
}

void NetCdfLoader::loadFloatArray3D(int ncid, int varid, size_t zlen, size_t ylen, size_t xlen, float*& array) {
    array = new float[zlen * ylen * xlen];
    size_t startp[] = { 0, 0, 0 };
    size_t countp[] = { zlen, ylen, xlen };
    myassert(nc_get_vara_float(ncid, varid, startp, countp, array) == NC_NOERR);
}

void NetCdfLoader::loadFloatArray3D(
        int ncid, int varid, size_t time, size_t zlen, size_t ylen, size_t xlen, float*& array) {
    array = new float[zlen * ylen * xlen];
    size_t startp[] = { time, 0, 0, 0 };
    size_t countp[] = { 1, zlen, ylen, xlen };
    myassert(nc_get_vara_float(ncid, varid, startp, countp, array) == NC_NOERR);
}

std::string NetCdfLoader::getStringAttribute(int ncid, int varid, const char* attname) {
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


void NetCdfLoader::load(
        const std::string& dataSourceFilename, const GridDataSetMetaData& gridDataSetMetaData,
        StreamlineTracingGrid* grid) {
    int ncid;

    int status = nc_open(dataSourceFilename.c_str(), NC_NOWRITE, &ncid);
    if (status != 0) {
        sgl::Logfile::get()->throwError(
                "Error in NetCdfLoader::load: File \"" + dataSourceFilename + "\" couldn't be opened.");
    }

    int timeIdx = gridDataSetMetaData.time;

    bool uLowerCaseVariableExists = getVariableExists(ncid, "u");
    bool vLowerCaseVariableExists = getVariableExists(ncid, "v");
    bool wLowerCaseVariableExists = getVariableExists(ncid, "w");
    bool uUpperCaseVariableExists = getVariableExists(ncid, "U");
    bool vUpperCaseVariableExists = getVariableExists(ncid, "V");
    bool wUpperCaseVariableExists = getVariableExists(ncid, "W");

    // Get the wind speed variable IDs.
    int varIdU = -1, varIdV = -1, varIdW = -1;
    if (uLowerCaseVariableExists && vLowerCaseVariableExists && wLowerCaseVariableExists) {
        myassert(nc_inq_varid(ncid, "u", &varIdU) == NC_NOERR);
        myassert(nc_inq_varid(ncid, "v", &varIdV) == NC_NOERR);
        myassert(nc_inq_varid(ncid, "w", &varIdW) == NC_NOERR);
    } else if (uUpperCaseVariableExists && vUpperCaseVariableExists && wUpperCaseVariableExists) {
        myassert(nc_inq_varid(ncid, "U", &varIdU) == NC_NOERR);
        myassert(nc_inq_varid(ncid, "V", &varIdV) == NC_NOERR);
        myassert(nc_inq_varid(ncid, "W", &varIdW) == NC_NOERR);
    } else {
        sgl::Logfile::get()->throwError(
                "Error in NetCdfLoader::load: Could not find u, v, w (or U, V, W) wind speeds in file \""
                + dataSourceFilename + "\".");
    }

    size_t ts = 0, zs = 0, ys = 0, xs = 0;
    float* zCoords = nullptr;
    float* yCoords = nullptr;
    float* xCoords = nullptr;
    float* uField = nullptr;
    float* vField = nullptr;
    float* wField = nullptr;

    // Get dimensions of the wind speed variables.
    // Examples: (time, level, rlat, rlon)
    // U(time, altitude, rlat, srlon), V(time, altitude, srlat, rlon), W(time, altitude, rlat, rlon), T(time, level, rlat, rlon), ...
    // U(time, level, rlat, srlon), V(time, level, srlat, rlon), W(time, level1, rlat, rlon), T(time, level, rlat, rlon), ...
    // u(zdim, ydim, xdim), v(zdim, ydim, xdim), w(zdim, ydim, xdim)
    int numDims = 0;
    myassert(nc_inq_varndims(ncid, varIdU, &numDims) == NC_NOERR);
    bool isLatLonData = false;
    if (numDims == 3) {
        // Assuming (zdim, ydim, xdim).
        int dimensionIds[3];
        char dimNameZ[NC_MAX_NAME + 1];
        char dimNameY[NC_MAX_NAME + 1];
        char dimNameX[NC_MAX_NAME + 1];
        myassert(nc_inq_vardimid(ncid, varIdU, dimensionIds) == NC_NOERR);
        myassert(nc_inq_dim(ncid, dimensionIds[0], dimNameZ, &zs) == NC_NOERR);
        myassert(nc_inq_dim(ncid, dimensionIds[1], dimNameY, &ys) == NC_NOERR);
        myassert(nc_inq_dim(ncid, dimensionIds[2], dimNameX, &xs) == NC_NOERR);
        zCoords = new float[zs];
        yCoords = new float[ys];
        xCoords = new float[xs];
        uField = new float[zs * ys * xs];
        vField = new float[zs * ys * xs];
        wField = new float[zs * ys * xs];
        int varid;
        myassert(nc_inq_varid(ncid, dimNameZ, &varid) == NC_NOERR);
        loadFloatArray1D(ncid, dimNameZ, zs, zCoords);
        loadFloatArray1D(ncid, dimNameY, ys, yCoords);
        loadFloatArray1D(ncid, dimNameX, xs, xCoords);
        loadFloatArray3D(ncid, varIdU, zs, ys, xs, uField);
        loadFloatArray3D(ncid, varIdV, zs, ys, xs, vField);
        loadFloatArray3D(ncid, varIdW, zs, ys, xs, wField);
    } else if (numDims == 4) {
        int dimensionIdsU[4];
        int dimensionIdsV[4];
        int dimensionIdsW[4];
        char dimNameTime[NC_MAX_NAME + 1];
        char dimNameZ[NC_MAX_NAME + 1];
        char dimNameY[NC_MAX_NAME + 1];
        char dimNameX[NC_MAX_NAME + 1];
        myassert(nc_inq_vardimid(ncid, varIdU, dimensionIdsU) == NC_NOERR);
        myassert(nc_inq_vardimid(ncid, varIdV, dimensionIdsV) == NC_NOERR);
        myassert(nc_inq_vardimid(ncid, varIdW, dimensionIdsW) == NC_NOERR);
        zCoords = new float[zs];
        yCoords = new float[ys];
        xCoords = new float[xs];
        uField = new float[zs * ys * xs];
        vField = new float[zs * ys * xs];
        wField = new float[zs * ys * xs];
        // Ignoring staggered grids for now by not querying i-th dimension using i-th variable.
        myassert(nc_inq_dim(ncid, dimensionIdsU[0], dimNameTime, &ts) == NC_NOERR);
        myassert(nc_inq_dim(ncid, dimensionIdsU[1], dimNameZ, &zs) == NC_NOERR);
        myassert(nc_inq_dim(ncid, dimensionIdsW[2], dimNameY, &ys) == NC_NOERR);
        myassert(nc_inq_dim(ncid, dimensionIdsW[3], dimNameX, &xs) == NC_NOERR);
        std::string stringDimNameX = dimNameX;
        std::string stringDimNameY = dimNameY;
        //std::string stringDimNameZ = dimNameZ;
        if (stringDimNameX.find("lon") != std::string::npos || stringDimNameY.find("lon") != std::string::npos
                || stringDimNameX.find("lat") != std::string::npos || stringDimNameY.find("lat") != std::string::npos) {
            isLatLonData = true;
        }
        if (!getVariableExists(ncid, dimNameZ) && getVariableExists(ncid, "vcoord")) {
            loadFloatArray1D(ncid, "vcoord", zs, zCoords);
        } else {
            loadFloatArray1D(ncid, dimNameZ, zs, zCoords);
        }
        loadFloatArray1D(ncid, dimNameY, ys, yCoords);
        loadFloatArray1D(ncid, dimNameX, xs, xCoords);
        loadFloatArray3D(ncid, varIdU, timeIdx, zs, ys, xs, uField);
        loadFloatArray3D(ncid, varIdV, timeIdx, zs, ys, xs, vField);
        loadFloatArray3D(ncid, varIdW, timeIdx, zs, ys, xs, wField);
    } else {
        sgl::Logfile::get()->throwError(
                "Error in NetCdfLoader::load: Invalid number of dimensions in file \""
                + dataSourceFilename + "\".");
    }

    // TODO: Use coords also for lat-lon-pressure?
    float dxCoords = 1.0f;
    float dyCoords = 1.0f;
    float dzCoords = 1.0f;
    if (!isLatLonData) {
        // Assume regular grid.
        dzCoords = (zCoords[zs - 1] - zCoords[0]) / float(zs - 1);
        dyCoords = (yCoords[ys - 1] - yCoords[0]) / float(ys - 1);
        dxCoords = (xCoords[xs - 1] - xCoords[0]) / float(xs - 1);
    }
    float maxDeltaCoords = std::max(dxCoords, std::max(dyCoords, dzCoords));

    float maxDimension = float(std::max(xs - 1, std::max(ys - 1, zs - 1)));
    float cellStep = 1.0f / maxDimension;
    float dx = cellStep * gridDataSetMetaData.scale[0] * dxCoords / maxDeltaCoords;
    float dy = cellStep * gridDataSetMetaData.scale[1] * dyCoords / maxDeltaCoords;
    float dz = cellStep * gridDataSetMetaData.scale[2] * dzCoords / maxDeltaCoords;
    auto numPoints = int(xs * ys * zs);

    grid->setGridExtent(int(xs), int(ys), int(zs), dx, dy, dz);

    auto* velocityField = new float[3 * numPoints];
    for (int ptIdx = 0; ptIdx < numPoints; ptIdx++) {
        velocityField[3 * ptIdx + 0] = uField[ptIdx];
        velocityField[3 * ptIdx + 1] = vField[ptIdx];
        velocityField[3 * ptIdx + 2] = wField[ptIdx];
    }

    auto* velocityMagnitudeField = new float[numPoints];
    computeVectorMagnitudeField(
            velocityField, velocityMagnitudeField, int(xs), int(ys), int(zs));

    auto* vorticityField = new float[numPoints * 3];
    computeVorticityField(
            velocityField, vorticityField, int(xs), int(ys), int(zs), dx, dy, dz);

    auto* vorticityMagnitudeField = new float[numPoints];
    computeVectorMagnitudeField(
            vorticityField, vorticityMagnitudeField, int(xs), int(ys), int(zs));

    auto* helicityField = new float[numPoints];
    computeHelicityFieldNormalized(
            velocityField, vorticityField, helicityField, int(xs), int(ys), int(zs),
            gridDataSetMetaData.useNormalizedVelocity,
            gridDataSetMetaData.useNormalizedVorticity);

    grid->addVectorField(velocityField, "Velocity");
    grid->addScalarField(velocityMagnitudeField, "Velocity Magnitude");
    grid->addVectorField(vorticityField, "Vorticity");
    grid->addScalarField(vorticityMagnitudeField, "Vorticity Magnitude");
    grid->addScalarField(helicityField, "Helicity");

    grid->addScalarField(uField, uUpperCaseVariableExists ? "U" : "u");
    grid->addScalarField(vField, vUpperCaseVariableExists ? "V" : "v");
    grid->addScalarField(wField, wUpperCaseVariableExists ? "W" : "w");

    std::set<std::string> blacklistNames = {
            "u", "v", "w", "U", "V", "W"
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
        if (type != NC_FLOAT || ndims != numDims || blacklistNames.find(attname) != blacklistNames.end()) {
            continue;
        }

        std::string variableDisplayName = varname;
        for (int attnum = 0; attnum < natts; attnum++) {
            nc_inq_attname(ncid, varid, attnum, attname);
            if (strcmp(attname, "standard_name") == 0) {
                variableDisplayName = getStringAttribute(ncid, varid, "standard_name");
            }
        }

        auto* scalarData = new float[numPoints];
        if (numDims == 3) {
            loadFloatArray3D(ncid, varid, zs, ys, xs, scalarData);
        } else {
            loadFloatArray3D(ncid, varid, 0, zs, ys, xs, scalarData);
        }

        grid->addScalarField(scalarData, variableDisplayName);
    }

    delete[] zCoords;
    delete[] yCoords;
    delete[] xCoords;

    if (nc_close(ncid) != NC_NOERR) {
        sgl::Logfile::get()->throwError(
                "Error in NetCdfLoader::load: nc_close failed for file \"" + dataSourceFilename + "\".");
    }
}
