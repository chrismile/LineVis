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

#include <algorithm>
#include <cstdint>

#include <boost/filesystem.hpp>
#include <tracy/Tracy.hpp>

#include <Utils/AppSettings.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Utils/Events/Stream/Stream.hpp>
#include <Graphics/Vulkan/Render/Renderer.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/imgui_stdlib.h>

#include <IsosurfaceCpp/src/MarchingCubes.hpp>
#include <IsosurfaceCpp/src/SnapMC.hpp>

#include "Utils/TriangleNormals.hpp"
#include "Utils/MeshSmoothing.hpp"
#include "Utils/IndexMesh.hpp"
#include "LineDataScattering.hpp"
#include "ScatteringLineTracingRequester.hpp"
#include "Texture3d.hpp"
#include "DtPathTrace.hpp"

ScatteringLineTracingRequester::ScatteringLineTracingRequester(
        sgl::TransferFunctionWindow& transferFunctionWindow, sgl::vk::Renderer* rendererMainThread)
        : transferFunctionWindow(transferFunctionWindow)
{
    sgl::vk::Device* device = rendererMainThread->getDevice();
    rendererVk = new sgl::vk::Renderer(device, 100);
    lineDensityFieldSmoothingPass = std::make_shared<LineDensityFieldSmoothingPass>(rendererVk);

    // Turn off multi-threaded loading if no dedicated worker thread queue is available.
    if (!device->getWorkerThreadGraphicsQueue()
            || device->getGraphicsQueue() == device->getWorkerThreadGraphicsQueue()) {
        supportsMultiThreadedLoading = false;
    }

    lineDataSetsDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";
    loadGridDataSetList();
    if (supportsMultiThreadedLoading) {
        requesterThread = std::thread(&ScatteringLineTracingRequester::mainLoop, this);
    }
}

ScatteringLineTracingRequester::~ScatteringLineTracingRequester() {
    if (supportsMultiThreadedLoading) {
        join();
    }
    cachedGrid.delete_maybe();

    requestLineData = {};
    replyLineData = {};

    if (rendererVk) {
        lineDensityFieldSmoothingPass = {};
        cachedScalarFieldTexture = {};
        delete rendererVk;
        rendererVk = nullptr;
    }
}

void ScatteringLineTracingRequester::loadGridDataSetList() {
    gridDataSetNames.clear();
    gridDataSetFilenames.clear();
    gridDataSetNames.emplace_back("Local file...");

    std::string filename = lineDataSetsDirectory + "scattering_grids.json";
    if (sgl::FileUtils::get()->exists(filename)) {
        // Parse the passed JSON file.
        std::ifstream jsonFileStream(filename.c_str());
        Json::CharReaderBuilder builder;
        JSONCPP_STRING errorString;
        Json::Value root;
        if (!parseFromStream(builder, jsonFileStream, &root, &errorString)) {
            sgl::Logfile::get()->writeError(errorString);
            return;
        }
        jsonFileStream.close();

        Json::Value sources = root["grids"];
        for (Json::Value::const_iterator sourceIt = sources.begin(); sourceIt != sources.end(); ++sourceIt) {
            DataSetInformation dataSetInformation;
            Json::Value source = *sourceIt;

            gridDataSetNames.push_back(source["name"].asString());
            gridDataSetFilenames.push_back(source["filename"].asString());
        }
    }
}

void ScatteringLineTracingRequester::renderGui() {
    if (!showWindow) {
        return;
    }

#ifndef NDEBUG
    int windowHeight = 645;
#else
    int windowHeight = 600;
#endif
    sgl::ImGuiWrapper::get()->setNextWindowStandardPosSizeLocation(
            sgl::LOCATION_RIGHT | sgl::LOCATION_BOTTOM, 22, 22, 760, windowHeight);
    if (ImGui::Begin("Scattering Line Tracer", &showWindow)) {
        bool changed = false;
        if (ImGui::Combo(
                "Data Set", &selectedGridDataSetIndex, gridDataSetNames.data(),
                int(gridDataSetNames.size()))) {
            if (selectedGridDataSetIndex >= 1) {
                const std::string pathString = gridDataSetFilenames.at(selectedGridDataSetIndex - 1);
#ifdef _WIN32
                bool isAbsolutePath =
                    (pathString.size() > 1 && pathString.at(1) == ':')
                    || boost::starts_with(pathString, "/") || boost::starts_with(pathString, "\\");
#else
                bool isAbsolutePath =
                        boost::starts_with(pathString, "/");
#endif
                if (isAbsolutePath) {
                    gridDataSetFilename = pathString;
                } else {
                    gridDataSetFilename = lineDataSetsDirectory + pathString;
                }
                changed = true;
            }
        }
        if (selectedGridDataSetIndex == 0) {
            ImGui::InputText("##griddatasetfilenamelabel", &gridDataSetFilename);
            ImGui::SameLine();
            if (ImGui::Button("Load File")) {
                changed = true;
            }
        }
#ifndef NDEBUG
        ImGui::Checkbox("Use Isosurface", &guiTracingSettings.show_iso_surface);
#endif

        changed |= ImGui::SliderFloatEdit(
                "Camera FOV", &guiTracingSettings.camera_fov_deg, 5.0f, 90.0f) == ImGui::EditMode::INPUT_FINISHED;
        changed |= ImGui::SliderFloat3Edit(
                "Camera Position", &guiTracingSettings.camera_position.x, -1, 1) == ImGui::EditMode::INPUT_FINISHED;
        changed |= ImGui::SliderFloat3Edit(
                "Camera Look At", &guiTracingSettings.camera_look_at.x, -1, 1) == ImGui::EditMode::INPUT_FINISHED;


        changed |= ImGui::InputInt("Res X", (int*)&guiTracingSettings.res_x);
        changed |= ImGui::InputInt("Res Y", (int*)&guiTracingSettings.res_y);
        changed |= ImGui::InputInt(
                "Samples per Pixel", (int*)&guiTracingSettings.samples_per_pixel);

        // NOTE(Felix): res_x, res_y and samples should not be smaller than 1
        guiTracingSettings.res_x = std::max(guiTracingSettings.res_x, 1u);
        guiTracingSettings.res_y = std::max(guiTracingSettings.res_y, 1u);
        guiTracingSettings.samples_per_pixel
            = std::max(guiTracingSettings.samples_per_pixel, 1u);


        changed |= ImGui::SliderFloat3Edit(
                "Extinction", &guiTracingSettings.extinction.x, 0.0f, 3000.0f) == ImGui::EditMode::INPUT_FINISHED;
        changed |= ImGui::SliderFloat3Edit(
                "Scattering Albedo", &guiTracingSettings.scattering_albedo.x, 0.0f, 1.0f) == ImGui::EditMode::INPUT_FINISHED;
        changed |= ImGui::SliderFloatEdit(
                "G", &guiTracingSettings.g, 0.0f, 1.0f) == ImGui::EditMode::INPUT_FINISHED;

        if (changed) {
            requestNewData();
        }
    }
    ImGui::End();
}

void ScatteringLineTracingRequester::setLineTracerSettings(const SettingsMap& settings) {
    bool changed = false;

    std::string datasetName;
    if (settings.getValueOpt("dataset", datasetName)) {
        for (int i = 0; i < int(gridDataSetNames.size()); i++) {
            if (datasetName == gridDataSetNames.at(i)) {
                selectedGridDataSetIndex = i + 1;
                const std::string pathString = gridDataSetFilenames.at(selectedGridDataSetIndex - 1);
#ifdef _WIN32
                bool isAbsolutePath =
                    (pathString.size() > 1 && pathString.at(1) == ':')
                    || boost::starts_with(pathString, "/") || boost::starts_with(pathString, "\\");
#else
                bool isAbsolutePath =
                        boost::starts_with(pathString, "/");
#endif
                if (isAbsolutePath) {
                    gridDataSetFilename = pathString;
                } else {
                    gridDataSetFilename = lineDataSetsDirectory + pathString;
                }
                changed = true;
                break;
            }
        }
    }

    changed |= settings.getValueOpt("use_isosurface", guiTracingSettings.show_iso_surface);
    changed |= settings.getValueOpt("camera_fov", guiTracingSettings.camera_fov_deg);
    changed |= settings.getValueOpt("camera_position", guiTracingSettings.camera_position);
    changed |= settings.getValueOpt("camera_look_at", guiTracingSettings.camera_look_at);
    changed |= settings.getValueOpt("res_x", guiTracingSettings.res_x);
    changed |= settings.getValueOpt("res_y", guiTracingSettings.res_y);
    changed |= settings.getValueOpt("samples_per_pixel", guiTracingSettings.samples_per_pixel);
    changed |= settings.getValueOpt("extinction", guiTracingSettings.extinction);
    changed |= settings.getValueOpt("scattering_albedo", guiTracingSettings.scattering_albedo);
    changed |= settings.getValueOpt("g", guiTracingSettings.g);

    if (changed) {
        requestNewData();
    }
}

void ScatteringLineTracingRequester::setDatasetFilename(const std::string& newDatasetFilename) {
    bool isDataSetInList = false;
    for (int i = 0; i < int(gridDataSetFilenames.size()); i++) {
        auto newDataSetPath = boost::filesystem::absolute(newDatasetFilename);
        auto currentDataSetPath = boost::filesystem::absolute(lineDataSetsDirectory + gridDataSetFilenames.at(i));
        if (boost::filesystem::equivalent(newDataSetPath, currentDataSetPath)) {
            selectedGridDataSetIndex = i + 1;
            const std::string pathString = gridDataSetFilenames.at(selectedGridDataSetIndex - 1);
#ifdef _WIN32
            bool isAbsolutePath =
                    (pathString.size() > 1 && pathString.at(1) == ':')
                    || boost::starts_with(pathString, "/") || boost::starts_with(pathString, "\\");
#else
            bool isAbsolutePath =
                    boost::starts_with(pathString, "/");
#endif
            if (isAbsolutePath) {
                gridDataSetFilename = pathString;
            } else {
                gridDataSetFilename = lineDataSetsDirectory + pathString;
            }
            isDataSetInList = true;
            break;
        }
    }

    if (!isDataSetInList) {
        selectedGridDataSetIndex = 0;
        gridDataSetFilename = newDatasetFilename;
    }

    requestNewData();
}

void ScatteringLineTracingRequester::requestNewData() {
    if (gridDataSetFilename.empty()) {
        return;
    }

    ScatteringTracingSettings request = guiTracingSettings;
    request.dataset_filename = boost::filesystem::absolute(gridDataSetFilename).generic_string();

    queueRequestStruct(request);
    if (!supportsMultiThreadedLoading) {
        mainLoop();
    }
}

bool ScatteringLineTracingRequester::getHasNewData(DataSetInformation& dataSetInformation, LineDataPtr& lineData) {
    if (getReply(lineData)) {
        return true;
    }
    return false;
}

void ScatteringLineTracingRequester::queueRequestStruct(const ScatteringTracingSettings& request) {
    {
        std::lock_guard<std::mutex> lock(requestMutex);
        workerTracingSettings = request;
        requestLineData = LineDataPtr(new LineDataScattering(
                transferFunctionWindow, rendererVk
        ));
        hasRequest = true;
    }
    hasRequestConditionVariable.notify_all();
}

bool ScatteringLineTracingRequester::getReply(LineDataPtr& lineData) {
    bool hasReply;
    {
        std::lock_guard<std::mutex> lock(replyMutex);
        hasReply = this->hasReply;
        if (hasReply) {
            lineData = replyLineData;
        }

        // Now, new requests can be worked on.
        this->hasReply = false;
        // this->replyMessage.clear();
        this->replyLineData = LineDataPtr();
    }
    hasReplyConditionVariable.notify_all();
    return hasReply;
}

void ScatteringLineTracingRequester::join() {
    if (!programIsFinished) {
        {
            std::lock_guard<std::mutex> lockRequest(requestMutex);
            programIsFinished = true;
            hasRequest = true;

            {
                std::lock_guard<std::mutex> lockReply(replyMutex);
                this->hasReply = false;
                // this->replyMessage.clear();
            }
            hasReplyConditionVariable.notify_all();
        }
        hasRequestConditionVariable.notify_all();
        if (requesterThread.joinable()) {
            requesterThread.join();
        }
    }
}

void ScatteringLineTracingRequester::mainLoop() {
#ifdef TRACY_ENABLE
    tracy::SetThreadName("ScatteringLineTracingRequester");
#endif

    while (true) {
        std::unique_lock<std::mutex> requestLock(requestMutex);
        hasRequestConditionVariable.wait(requestLock, [this] { return hasRequest; });

        if (programIsFinished) {
            break;
        }

        if (hasRequest) {
            ScatteringTracingSettings request = workerTracingSettings;
            std::shared_ptr<LineDataScattering> lineData =
                    std::static_pointer_cast<LineDataScattering>(requestLineData);
            hasRequest = false;
            isProcessingRequest = true;
            requestLock.unlock();

            traceLines(request, lineData);

            std::lock_guard<std::mutex> replyLock(replyMutex);
            hasReply = true;

            replyLineData = lineData;
            isProcessingRequest = false;
        }

        // Only one operation if this function is not run in a separate thread.
        if (!supportsMultiThreadedLoading) {
            break;
        }
    }
}

void ScatteringLineTracingRequester::traceLines(
        const ScatteringTracingSettings& request, std::shared_ptr<LineDataScattering>& lineData)
{
    std::string data_set_filename = request.dataset_filename;
    bool use_iso_surface = request.show_iso_surface;

    // if the user changed the file
    if (data_set_filename != cachedGridFileName) {
        cachedGrid.delete_maybe();
        cachedGridFileName = data_set_filename;
        cachedGrid = load_xyz_file(cachedGridFileName);

        if (use_iso_surface) {
            createScalarFieldTexture();
            createIsosurface();
        }
    }

    PathInfo pi {};
    pi.camera_pos    = request.camera_position;
    pi.ray_direction = glm::normalize(request.camera_look_at - request.camera_position);

    VolumeInfo vi {};
    vi.grid              = cachedGrid;
    vi.extinction        = request.extinction;
    vi.scattering_albedo = request.scattering_albedo;
    vi.g                 = request.g;

    float camera_fov_deg       = request.camera_fov_deg;
    uint32_t res_x             = request.res_x;
    uint32_t res_y             = request.res_y;
    uint32_t samples_per_pixel = request.samples_per_pixel;


    Trajectories trajectories;
    Exit_Directions exit_directions;
    pi.pass_number = 0;
    Random::init(request.seed);

    { // NEW

        glm::vec3 Y = { 0, -1, 0 };
        glm::vec3 X = glm::cross(pi.ray_direction, Y);
        Y = glm::cross(X, pi.ray_direction);

        float focal_length        = 1;  // how far away from the camera the grid will be
        float camera_fov_rad = glm::radians(camera_fov_deg);
        float grid_width     = tan(camera_fov_rad / 2) * 2 * focal_length;
        float grid_height    = res_y * (grid_width / res_x);

        glm::vec3 P0 =
            pi.camera_pos +
            pi.ray_direction * focal_length -
            0.5f * Y * grid_height -
            0.5f * X * grid_width;

        // Pixel loop
        for (uint32_t y = 0; y < res_y; ++y) {
            for (uint32_t x = 0; x < res_x; ++x) {

                // NOTE(Felix): these percentage values tell us how far along
                //   the x and y axis we are, while rendering the pixels. This
                //   is used to find out the direction onto which we send the
                //   ray. If the resolution in a dimension is 1, we send the ray
                //   in the middle, the calculation would otherwise lead to a
                //   divison-by-zero. Maybe this is too complicated. Is there a
                //   simper way?
                float x_percentage;
                float y_percentage;
                if (res_x < 2) {
                    x_percentage = 0.5f;
                } else {
                    x_percentage = (x*1.0f/(res_x-1));
                }

                if (res_y < 2) {
                    y_percentage = 0.5f;
                } else {
                    y_percentage = (y*1.0f/(res_y-1));
                }

                glm::vec3 P =
                    P0 +
                    X * x_percentage * grid_width +
                    Y * y_percentage * grid_height;

                P = glm::normalize(P - pi.camera_pos);
                pi.ray_direction = P;

                // Sample loop
                for (uint32_t i = 0; i < samples_per_pixel; ++i) {
                    pi.pass_number = i;
                    dt_path_trace(pi, vi, &trajectories, &exit_directions);
                }
            }
        }
    }

    lineData->setExitDirections(exit_directions);

    lineData->setDataSetInformation(gridDataSetFilename, { "Attribute #1" });
    lineData->setTrajectoryData(trajectories);
    lineData->setGridData(
            cachedScalarFieldTexture,
            outlineTriangleIndices, outlineVertexPositions, outlineVertexNormals,
            cachedGrid.data, cachedGrid.size_x, cachedGrid.size_y, cachedGrid.size_z,
            cachedGrid.voxel_size_x, cachedGrid.voxel_size_y, cachedGrid.voxel_size_z);

}

void ScatteringLineTracingRequester::createScalarFieldTexture() {
    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = cachedGrid.size_x;
    imageSettings.height = cachedGrid.size_y;
    imageSettings.depth = cachedGrid.size_z;
    imageSettings.imageType = VK_IMAGE_TYPE_3D;
    imageSettings.format = VK_FORMAT_R32_SFLOAT;

    sgl::vk::ImageSamplerSettings samplerSettings;
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    cachedScalarFieldTexture = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, samplerSettings);
    cachedScalarFieldTexture->getImage()->uploadData(
            cachedGrid.size_x * cachedGrid.size_y * cachedGrid.size_z * sizeof(float), cachedGrid.data);
}

void ScatteringLineTracingRequester::createIsosurface() {
    ZoneScoped;

    outlineTriangleIndices.clear();
    outlineVertexPositions.clear();
    outlineVertexNormals.clear();

    sgl::AABB3 gridAabb;
    gridAabb.min = glm::vec3(0.0f, 0.0f, 0.0f);
    gridAabb.max = glm::vec3(cachedGrid.size_x, cachedGrid.size_y, cachedGrid.size_z);

    std::vector<glm::vec3> isosurfaceVertexPositions;
    std::vector<glm::vec3> isosurfaceVertexNormals;

    const float gamma = 0.3f;
    bool shallSmoothScalarField = true;
    if (!shallSmoothScalarField) {
        polygonizeSnapMC(
                cachedGrid.data, int(cachedGrid.size_x), int(cachedGrid.size_y), int(cachedGrid.size_z),
                1e-4f, gamma, isosurfaceVertexPositions, isosurfaceVertexNormals);
    } else {
        int padding = 4;
        int smoothedGridSizeX = int(cachedGrid.size_x) + 2 * padding;
        int smoothedGridSizeY = int(cachedGrid.size_y) + 2 * padding;
        int smoothedGridSizeZ = int(cachedGrid.size_z) + 2 * padding;
        gridAabb.min = glm::vec3(padding, padding, padding);
        gridAabb.max = glm::vec3(cachedGrid.size_x, cachedGrid.size_y, cachedGrid.size_z);
        float* scalarFieldSmoothed = lineDensityFieldSmoothingPass->smoothScalarFieldCpu(
                cachedScalarFieldTexture, padding);
        polygonizeSnapMC(
                scalarFieldSmoothed, int(smoothedGridSizeX), int(smoothedGridSizeY), int(smoothedGridSizeZ),
                0.01f, gamma, isosurfaceVertexPositions, isosurfaceVertexNormals);
        delete[] scalarFieldSmoothed;
    }

    if (!isosurfaceVertexPositions.empty()) {
        computeSharedIndexRepresentation(
                isosurfaceVertexPositions, isosurfaceVertexNormals,
                outlineTriangleIndices, outlineVertexPositions, outlineVertexNormals);
        normalizeVertexPositions(outlineVertexPositions, gridAabb, nullptr);

        laplacianSmoothing(outlineTriangleIndices, outlineVertexPositions);
    }
}
