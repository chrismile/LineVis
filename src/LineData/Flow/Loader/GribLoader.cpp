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

#define _FILE_OFFSET_BITS 64
#define __USE_FILE_OFFSET64

#include <iostream>
#include <set>
#include <unordered_set>
#include <unordered_map>

#include <eccodes.h>

#include <Utils/File/Logfile.hpp>

#include "../StreamlineTracingDefines.hpp"
#include "../StreamlineTracingGrid.hpp"
#include "GridLoader.hpp"
#include "GribLoader.hpp"

std::string GribLoader::getString(codes_handle* handle, const std::string& key) {
    size_t length = 0;
    codes_get_length(handle, key.c_str(), &length);
    char* value = new char[length + 1];
    int errorCode = codes_get_string(handle, key.c_str(), value, &length);
    if (errorCode != 0) {
        sgl::Logfile::get()->throwError(
                "Error in GribLoader::getString: Cannot access value for key \"" + key
                + "\". ecCodes error message: " + codes_get_error_message(errorCode));
    }
    value[length] = '\0';
    std::string valueString = value;
    delete[] value;
    return valueString;
}

double GribLoader::getDouble(codes_handle* handle, const std::string& key) {
    double value = 0.0;
    int errorCode = codes_get_double(handle, key.c_str(), &value);
    if (errorCode != 0) {
        sgl::Logfile::get()->throwError(
                "Error in GribLoader::getDouble: Cannot access value for key \"" + key
                + "\". ecCodes error message: " + codes_get_error_message(errorCode));
    }
    return value;
}

long GribLoader::getLong(codes_handle* handle, const std::string& key) {
    long value = 0;
    int errorCode = codes_get_long(handle, key.c_str(), &value);
    if (errorCode != 0) {
        sgl::Logfile::get()->throwError(
                "Error in GribLoader::getDouble: Cannot access value for key \"" + key
                + "\". ecCodes error message: " + codes_get_error_message(errorCode));
    }
    return value;
}

void GribLoader::load(
        const std::string& dataSourceFilename, const GridDataSetMetaData& gridDataSetMetaData,
        StreamlineTracingGrid* grid) {
#if defined(__linux__) || defined(__MINGW32__) // __GNUC__? Does GCC generally work on non-POSIX systems?
    FILE* file = fopen64(dataSourceFilename.c_str(), "rb");
#else
    FILE* file = fopen(dataSourceFilename.c_str(), "rb");
#endif

    if (!file) {
        sgl::Logfile::get()->throwError(
                std::string() + "Error in GribLoader::load: File \""
                + dataSourceFilename + "\" could not be opened.");
    }

    int errorCode = 0;

#if defined(_WIN32) && !defined(__MINGW32__)
    _fseeki64(file, 0, SEEK_END);
    auto fileSize = _ftelli64(file);
    _fseeki64(file, 0, SEEK_SET);
#else
    fseeko(file, 0, SEEK_END);
    auto fileSize = ftello(file);
    fseeko(file, 0, SEEK_SET);
#endif

    std::vector<std::string> variableNames;
    std::unordered_map<std::string, size_t> encounteredVariableNamesMap;
    std::unordered_set<long> encounteredLevelValuesSet;
    std::vector<std::vector<float*>> variableSliceArrays;
    variableSliceArrays.reserve(100);

    double* lonValues = nullptr;
    double* latValues = nullptr;
    std::vector<long> levelValues;

    long numberOfPointsGlobal = 0;
    long numLonsGlobal = 0;
    long numLatsGlobal = 0;

    //long dataDateLoad = 20161002; // 2016-10-02
    //long dataTimeLoad = 600; // 6:00 o'clock
    long dataDateLoad = gridDataSetMetaData.date;
    long dataTimeLoad = gridDataSetMetaData.time;

    while(true) {
#if defined(_WIN32) && !defined(__MINGW32__)
        auto bufferSize = _ftelli64(file); // __int64
#else
        auto filePosition = ftello(file); // __off64_t
#endif

        if (filePosition == fileSize) {
            break;
        }

        codes_handle* handle = codes_handle_new_from_file(nullptr, file, PRODUCT_GRIB, &errorCode);
        if (!handle) {
            sgl::Logfile::get()->throwError(
                    "Error in GribLoader::load: File \"" + dataSourceFilename + "\" couldn't be opened.");
        }

        long dataDate = getLong(handle, "dataDate");
        long dataTime = getLong(handle, "dataTime");

        // Only load data for one single time step.
        if (dataDate != dataDateLoad || dataTime != dataTimeLoad) {
            CODES_CHECK(codes_handle_delete(handle), 0);
            if (dataDate < dataDateLoad || (dataDate == dataDateLoad && dataTime < dataTimeLoad)) {
                continue;
            } else {
                break;
            }
        }

        long numberOfPoints = getLong(handle, "numberOfPoints");
        long numLons = getLong(handle, "Ni");
        long numLats = getLong(handle, "Nj");

        if (numLons * numLats != numberOfPoints) {
            sgl::Logfile::get()->throwError(
                    "Error in GribLoader::load: File \"" + dataSourceFilename + "\" has contradictory values for "
                    "numberOfPoints, Ni and Nj.");
        }

        long level = getLong(handle, "level");
        if (levelValues.empty() || level != levelValues.back()) {
            if (encounteredLevelValuesSet.find(level) != encounteredLevelValuesSet.end()) {
                sgl::Logfile::get()->throwError(
                        "Error in GribLoader::load: File \"" + dataSourceFilename + "\" levels do not lie "
                        "contiguously in memory, which is currently not supported.");
            }
            encounteredLevelValuesSet.insert(level);
            levelValues.push_back(level);
        }

        double lonMin = getDouble(handle, "longitudeOfFirstGridPointInDegrees");
        double lonMax = getDouble(handle, "longitudeOfLastGridPointInDegrees");
        double latMin = getDouble(handle, "latitudeOfFirstGridPointInDegrees");
        double latMax = getDouble(handle, "latitudeOfLastGridPointInDegrees");
        double lonInc = getDouble(handle, "iDirectionIncrementInDegrees");
        double latInc = getDouble(handle, "jDirectionIncrementInDegrees");
        (void)lonMax;
        (void)latMax;

        std::string typeOfLevel = getString(handle, "typeOfLevel");
        if (typeOfLevel != "isobaricInhPa") {
            sgl::Logfile::get()->throwError(
                    "Error in GribLoader::load: The loader currently only supports the level type isobaricInhPa. "
                    "However, a different level type is used in the file \"" + dataSourceFilename + "\".");
        }

        std::string gridType = getString(handle, "gridType");
        if (gridType != "regular_ll") {
            sgl::Logfile::get()->throwError(
                    "Error in GribLoader::load: The loader currently only supports the grid type regular_ll. "
                    "However, a different grid type is used in the file \"" + dataSourceFilename + "\".");
        }

        std::string parameterShortName = getString(handle, "shortName");

        /*std::cout << "N: " << numberOfPoints << std::endl;
        std::cout << "Ni: " << numLons << std::endl;
        std::cout << "Nj: " << numLats << std::endl;
        std::cout << "lonMin: " << lonMin << std::endl;
        std::cout << "lonMax: " << lonMax << std::endl;
        std::cout << "latMin: " << latMin << std::endl;
        std::cout << "latMax: " << latMax << std::endl;
        std::cout << "lonInc: " << lonInc << std::endl;
        std::cout << "latInc: " << latInc << std::endl;
        std::cout << "shortName: " << parameterShortName << std::endl;
        std::cout << "level: " << level << std::endl;
        std::cout << "typeOfLevel: " << typeOfLevel << std::endl;
        std::cout << "dataDate: " << dataDate << std::endl;
        std::cout << "dataTime: " << dataTime << std::endl;
        std::cout << std::endl;*/

        // First variable data read?
        if (variableNames.empty()) {
            numberOfPointsGlobal = numberOfPoints;
            numLonsGlobal = numLons;
            numLatsGlobal = numLats;
            lonValues = new double[numLons];
            latValues = new double[numLats];
            for (int i = 0; i < numLons; i ++) {
                lonValues[i] = lonMin + i * lonInc;
            }
            for (int j = 0; j < numLats; j ++) {
                latValues[j] = latMin + j * latInc;
            }
        } else {
            if (numberOfPoints != numberOfPointsGlobal || numLons != numLonsGlobal || numLats != numLatsGlobal) {
                sgl::Logfile::get()->throwError(
                        "Error in GribLoader::load: File \"" + dataSourceFilename + "\" has inconsistent values "
                        "for numberOfPoints, Ni or Nj.");
            }
        }

        if (encounteredVariableNamesMap.find(parameterShortName) == encounteredVariableNamesMap.end()) {
            // Encountered the variable for the first time.
            encounteredVariableNamesMap.insert(std::make_pair(parameterShortName, variableNames.size()));
            variableNames.push_back(parameterShortName);
            variableSliceArrays.emplace_back();
        } else {
            // Encountered the variable already previously.
        }
        size_t variableIdx = encounteredVariableNamesMap.find(parameterShortName)->second;

        if (levelValues.size() - 1 != variableSliceArrays.at(variableIdx).size()) {
            sgl::Logfile::get()->throwError(
                    "Error in GribLoader::load: Expected variable slices to lie contiguously in memory for file \""
                    + dataSourceFilename + "\".");
        }

        size_t valuesLength = 0;
        CODES_CHECK(codes_get_size(handle, "values", &valuesLength), 0);
        auto* valuesArrayDouble = new double[valuesLength];
        auto* valuesArrayFloat = new float[valuesLength];
        if (valuesLength != size_t(numberOfPoints)) {
            sgl::Logfile::get()->throwError(
                    "Error in GribLoader::load: The values array size and numberOfPoints do not match in file \""
                    + dataSourceFilename + "\".");
        }
        CODES_CHECK(codes_get_double_array(handle, "values", valuesArrayDouble, &valuesLength), 0);
        for (size_t i = 0; i < valuesLength; i++) {
            valuesArrayFloat[i] = static_cast<float>(valuesArrayDouble[i]);
        }
        variableSliceArrays.at(variableIdx).push_back(valuesArrayFloat);
        delete[] valuesArrayDouble;

        CODES_CHECK(codes_handle_delete(handle), 0);
    }

    size_t numLevels = levelValues.size();

    for (size_t varIdx = 0; varIdx < variableSliceArrays.size(); varIdx++) {
        auto& variableSliceArray = variableSliceArrays.at(varIdx);
        while (variableSliceArray.size() < numLevels) {
            variableSliceArray.push_back(nullptr);
        }
    }

    // Merge the variable slice arrays.
    std::vector<float*> variableArrays;
    variableArrays.resize(variableSliceArrays.size());
    for (size_t varIdx = 0; varIdx < variableSliceArrays.size(); varIdx++) {
        std::vector<float*>& variableSliceArray = variableSliceArrays.at(varIdx);
        float*& variableArray = variableArrays.at(varIdx);
        variableArray = new float[numLevels * size_t(numLatsGlobal) * size_t(numLonsGlobal)];
        for (size_t level = 0; level < variableSliceArrays.size(); level++) {
            const float* variableSlice = variableSliceArray.at(level);
            if (variableSlice == nullptr) {
                for (long latIdx = 0; latIdx < numLatsGlobal; latIdx++) {
                    for (long lonIdx = 0; lonIdx < numLonsGlobal; lonIdx++) {
                        variableArray[(level * size_t(numLonsGlobal * numLatsGlobal) + size_t(numLonsGlobal) * latIdx + lonIdx)] = 0.0f;
                    }
                }
            } else {
                for (long latIdx = 0; latIdx < numLatsGlobal; latIdx++) {
                    for (long lonIdx = 0; lonIdx < numLonsGlobal; lonIdx++) {
                        variableArray[(level * size_t(numLonsGlobal * numLatsGlobal) + size_t(numLonsGlobal) * latIdx + lonIdx)] =
                                variableSlice[size_t(numLonsGlobal) * latIdx + lonIdx];
                    }
                }
                delete[] variableSlice;
            }
        }
        variableSliceArray.clear();
    }

    // Use u, v, w (or upper case) to get velocity. Set rest as scalar data.
    auto itU = encounteredVariableNamesMap.find("u");
    if (itU == encounteredVariableNamesMap.end()) {
        itU = encounteredVariableNamesMap.find("U");
        if (itU == encounteredVariableNamesMap.end()) {
            sgl::Logfile::get()->throwError(
                    "Error in GribLoader::load: Couldn't find u wind speed variable in file \""
                    + dataSourceFilename + "\".");
        }
    }
    auto itV = encounteredVariableNamesMap.find("v");
    if (itV == encounteredVariableNamesMap.end()) {
        itV = encounteredVariableNamesMap.find("V");
        if (itV == encounteredVariableNamesMap.end()) {
            sgl::Logfile::get()->throwError(
                    "Error in GribLoader::load: Couldn't find v wind speed variable in file \""
                    + dataSourceFilename + "\".");
        }
    }
    auto itW = encounteredVariableNamesMap.find("w");
    if (itW == encounteredVariableNamesMap.end()) {
        itW = encounteredVariableNamesMap.find("W");
        if (itW == encounteredVariableNamesMap.end()) {
            sgl::Logfile::get()->throwError(
                    "Error in GribLoader::load: Couldn't find w wind speed variable in file \""
                    + dataSourceFilename + "\".");
        }
    }
    size_t varIdxU = itU->second;
    size_t varIdxV = itV->second;
    size_t varIdxW = itW->second;
    float* uField = variableArrays.at(varIdxU);
    float* vField = variableArrays.at(varIdxV);
    float* wField = variableArrays.at(varIdxW);

    auto xs = int(numLonsGlobal);
    auto ys = int(numLatsGlobal);
    auto zs = int(numLevels);
    float maxDimension = float(std::max(xs - 1, std::max(ys - 1, zs - 1)));
    float cellStep = 1.0f / maxDimension;
    // TODO: Use lon, lat, level/pressure (pv).
    float dx = cellStep * gridDataSetMetaData.scale[0];
    float dy = cellStep * gridDataSetMetaData.scale[1];
    float dz = cellStep * gridDataSetMetaData.scale[2];
    auto numPoints = int(xs * ys * zs);

    grid->setGridExtent(int(xs), int(ys), int(zs), dx, dy, dz);

    auto* velocityField = new float[3 * numPoints];
    for (int ptIdx = 0; ptIdx < numPoints; ptIdx++) {
        velocityField[3 * ptIdx + 0] = uField[ptIdx] * gridDataSetMetaData.scale[0];
        velocityField[3 * ptIdx + 1] = vField[ptIdx] * gridDataSetMetaData.scale[1];
        velocityField[3 * ptIdx + 2] = wField[ptIdx] * gridDataSetMetaData.scale[2];
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

    for (size_t varIdx = 0; varIdx < variableArrays.size(); varIdx++) {
        grid->addScalarField(variableArrays.at(varIdx), variableNames.at(varIdx));
    }

    if (lonValues) {
        delete[] lonValues;
    }
    if (latValues) {
        delete[] latValues;
    }

    fclose(file);
}
