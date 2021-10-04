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

#include <Utils/AppSettings.hpp>
#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileUtils.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Utils/Events/Stream/Stream.hpp>
#include <ImGui/ImGuiWrapper.hpp>
#include <ImGui/imgui_custom.h>
#include <ImGui/imgui_stdlib.h>
#ifdef USE_VULKAN_INTEROP
#include <Graphics/Vulkan/Render/Renderer.hpp>
#endif

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
        sgl::TransferFunctionWindow& transferFunctionWindow
#ifdef USE_VULKAN_INTEROP
        , sgl::vk::Renderer* rendererMainThread
#endif
) : transferFunctionWindow(transferFunctionWindow)
        , rendererVk(rendererVk)
{
#ifdef USE_VULKAN_INTEROP
    rendererVk = new sgl::vk::Renderer(rendererMainThread->getDevice(), 100);
#endif

    loadGridDataSetList();
    lineDensityFieldSmoothingPass = std::make_shared<LineDensityFieldSmoothingPass>(rendererVk);
    requesterThread = std::thread(&ScatteringLineTracingRequester::mainLoop, this);
}

ScatteringLineTracingRequester::~ScatteringLineTracingRequester() {
    join();
    cached_grid.delete_maybe();

#ifdef USE_VULKAN_INTEROP
    lineDensityFieldSmoothingPass = {};
    cachedScalarFieldTexture = {};
    delete rendererVk;
#endif
}

void ScatteringLineTracingRequester::loadGridDataSetList() {
    gridDataSetNames.clear();
    gridDataSetFilenames.clear();
    gridDataSetNames.emplace_back("Local file...");

    const std::string lineDataSetsDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";
    std::string filename = lineDataSetsDirectory + "grids.json";
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
    sgl::ImGuiWrapper::get()->setNextWindowStandardPosSize(3072, 1146, 760, 628);
    if (ImGui::Begin("Scattering Line Tracer", &showWindow)) {
        bool changed = false;
        if (ImGui::Combo(
                "Data Set", &selectedGridDataSetIndex, gridDataSetNames.data(),
                int(gridDataSetNames.size()))) {
            if (selectedGridDataSetIndex >= 1) {
                gridDataSetFilename = gridDataSetFilenames.at(selectedGridDataSetIndex - 1);
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
        ImGui::Checkbox("Use Isosurface", &gui_tracing_settings.show_iso_surface);
#endif

        changed |= ImGui::SliderFloat("Camera FOV",
                                      &gui_tracing_settings.camera_fov_deg, 5.0f, 90.0f);
        changed |= ImGui::SliderFloat3("Camera Position",
                                       &gui_tracing_settings.camera_position.x, -1, 1);
        changed |= ImGui::SliderFloat3("Camera Look At",
                                       &gui_tracing_settings.camera_look_at.x, -1, 1);


        changed |= ImGui::InputInt("Res X", (int*)&gui_tracing_settings.res_x);
        changed |= ImGui::InputInt("Res Y", (int*)&gui_tracing_settings.res_y);
        changed |= ImGui::InputInt("Samples per Pixel",
                                   (int*)&gui_tracing_settings.samples_per_pixel);

        // NOTE(Felix): res_x, res_y and samples should not be smaller than 1
        gui_tracing_settings.res_x = std::max(gui_tracing_settings.res_x, 1u);
        gui_tracing_settings.res_y = std::max(gui_tracing_settings.res_y, 1u);
        gui_tracing_settings.samples_per_pixel
            = std::max(gui_tracing_settings.samples_per_pixel, 1u);


        changed |= ImGui::SliderFloat3("Extinction",
                                      &gui_tracing_settings.extinction.x, 0.0f, 100.0f);
        changed |= ImGui::SliderFloat3("Scattering Albedo",
                                      &gui_tracing_settings.scattering_albedo.x, 0.0f, 1.0f);
        changed |= ImGui::SliderFloat3("G",
                                      &gui_tracing_settings.g.x, 0.0f, 1.0f);

        if (changed) {
            requestNewData();
        }
    }
    ImGui::End();
}

void ScatteringLineTracingRequester::requestNewData() {
    if (gridDataSetFilename.empty()) {
        return;
    }

    const std::string lineDataSetsDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";

    Tracing_Settings request = gui_tracing_settings;
    request.dataset_filename = boost::filesystem::absolute(
        lineDataSetsDirectory + gridDataSetFilename).generic_string();


    queueRequestStruct(request);
    isProcessingRequest = true;
}

bool ScatteringLineTracingRequester::getHasNewData(DataSetInformation& dataSetInformation, LineDataPtr& lineData) {
    if (getReply(lineData)) {
        isProcessingRequest = false;
        return true;
    }
    return false;
}

void ScatteringLineTracingRequester::queueRequestStruct(const Tracing_Settings request) {
    {
        std::lock_guard<std::mutex> lock(replyMutex);
        worker_tracing_settings = request;
        requestLineData = LineDataPtr(new LineDataScattering(
                transferFunctionWindow
#ifdef USE_VULKAN_INTEROP
                , rendererVk
#endif
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
            std::lock_guard<std::mutex> lock(requestMutex);
            programIsFinished = true;
            hasRequest = true;

            {
                std::lock_guard<std::mutex> lock(replyMutex);
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
    while (true) {
        std::unique_lock<std::mutex> requestLock(requestMutex);
        hasRequestConditionVariable.wait(requestLock, [this] { return hasRequest; });

        if (programIsFinished) {
            break;
        }

        if (hasRequest) {
            Tracing_Settings request = worker_tracing_settings;
            std::shared_ptr<LineDataScattering> lineData = std::static_pointer_cast<LineDataScattering>(requestLineData);
            hasRequest = false;
            isProcessingRequest = true;
            requestLock.unlock();

            traceLines(request, lineData);

            std::lock_guard<std::mutex> replyLock(replyMutex);
            hasReply = true;

            replyLineData = lineData;
            isProcessingRequest = false;
        }
    }
}

void ScatteringLineTracingRequester::traceLines(
        Tracing_Settings request, std::shared_ptr<LineDataScattering>& lineData)
{
    std::string data_set_filename = request.dataset_filename;
    bool use_iso_surface = request.show_iso_surface;

    // if the user changed the file
    if (data_set_filename != cached_grid_file_name) {
        cached_grid.delete_maybe();
        cached_grid_file_name = data_set_filename;
        cached_grid = load_xyz_file(cached_grid_file_name);

        if (use_iso_surface) {
            createScalarFieldTexture();
            createIsosurface();
        }
    }

    PathInfo pi {};
    pi.camera_pos    = request.camera_position;
    pi.ray_direction = glm::normalize(request.camera_look_at - request.camera_position);

    VolumeInfo vi {};
    vi.grid              = cached_grid;
    vi.extinction        = request.extinction;
    vi.scattering_albedo = request.scattering_albedo;
    vi.g                 = request.g;

    float camera_fov_deg       = request.camera_fov_deg;
    uint32_t res_x             = request.res_x;
    uint32_t res_y             = request.res_y;
    uint32_t samples_per_pixel = request.samples_per_pixel;


    Trajectories trajectories;
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
        for (int y = 0; y < res_y; ++y) {
            for (int x = 0; x < res_x; ++x) {

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
                for (int i = 0; i < samples_per_pixel; ++i) {
                    pi.pass_number = i;
                    dt_path_trace(pi, vi, &trajectories);
                }
            }
        }
    }

    // TODO: This function normalizes the vertex positions of the trajectories;
    //   should we also normalize the grid size?
    //normalizeTrajectoriesVertexPositions(trajectories, nullptr);

    lineData->setDataSetInformation(gridDataSetFilename, { "Attribute #1" });
    lineData->setGridData(
#ifdef USE_VULKAN_INTEROP
            cachedScalarFieldTexture,
#endif
            outlineTriangleIndices, outlineVertexPositions, outlineVertexNormals,
            cached_grid.size_x, cached_grid.size_y, cached_grid.size_z,
            cached_grid.voxel_size_x, cached_grid.voxel_size_y, cached_grid.voxel_size_z);
    lineData->setTrajectoryData(trajectories);

}

void ScatteringLineTracingRequester::createScalarFieldTexture() {
    sgl::vk::ImageSettings imageSettings;
    imageSettings.width = cached_grid.size_x;
    imageSettings.height = cached_grid.size_y;
    imageSettings.depth = cached_grid.size_z;
    imageSettings.imageType = VK_IMAGE_TYPE_3D;
    imageSettings.format = VK_FORMAT_R32_SFLOAT;

    sgl::vk::ImageSamplerSettings samplerSettings;
    sgl::vk::Device* device = sgl::AppSettings::get()->getPrimaryDevice();
    imageSettings.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    cachedScalarFieldTexture = std::make_shared<sgl::vk::Texture>(
            device, imageSettings, samplerSettings);
    cachedScalarFieldTexture->getImage()->uploadData(
            cached_grid.size_x * cached_grid.size_y * cached_grid.size_z * sizeof(float), cached_grid.data);
}

void ScatteringLineTracingRequester::createIsosurface() {
    outlineTriangleIndices.clear();
    outlineVertexPositions.clear();
    outlineVertexNormals.clear();

    uint32_t maxDimSize = std::max(cached_grid.size_x, std::max(cached_grid.size_y, cached_grid.size_z));
    sgl::AABB3 gridAabb;
    gridAabb.min = glm::vec3(0.0f, 0.0f, 0.0f);
    gridAabb.max = glm::vec3(cached_grid.size_x, cached_grid.size_y, cached_grid.size_z);
    sgl::AABB3 gridAabbResized;
    gridAabbResized.max = glm::vec3(cached_grid.size_x, cached_grid.size_y, cached_grid.size_z) * 0.5f / float(maxDimSize);
    gridAabbResized.min = -gridAabbResized.max;

    std::vector<glm::vec3> isosurfaceVertexPositions;
    std::vector<glm::vec3> isosurfaceVertexNormals;

    const float gamma = 0.3f;
    bool shallSmoothScalarField = true;
#ifndef USE_VULKAN_INTEROP
    shallSmoothScalarField = false;
#endif
    if (!shallSmoothScalarField) {
        polygonizeSnapMC(
                cached_grid.data, int(cached_grid.size_x), int(cached_grid.size_y), int(cached_grid.size_z),
                1e-4f, gamma, isosurfaceVertexPositions, isosurfaceVertexNormals);

    }
#ifdef USE_VULKAN_INTEROP
    else {
        int padding = 4;
        int smoothedGridSizeX = int(cached_grid.size_x) + 2 * padding;
        int smoothedGridSizeY = int(cached_grid.size_y) + 2 * padding;
        int smoothedGridSizeZ = int(cached_grid.size_z) + 2 * padding;
        gridAabb.min = glm::vec3(padding, padding, padding);
        gridAabb.max = glm::vec3(cached_grid.size_x, cached_grid.size_y, cached_grid.size_z);
        float* scalarFieldSmoothed = lineDensityFieldSmoothingPass->smoothScalarFieldCpu(
                cachedScalarFieldTexture, padding);
        polygonizeSnapMC(
                scalarFieldSmoothed, int(smoothedGridSizeX), int(smoothedGridSizeY), int(smoothedGridSizeZ),
                1e-4f, gamma, isosurfaceVertexPositions, isosurfaceVertexNormals);
        delete[] scalarFieldSmoothed;
    }
#endif

    if (!isosurfaceVertexPositions.empty()) {
        computeSharedIndexRepresentation(
                isosurfaceVertexPositions, isosurfaceVertexNormals,
                outlineTriangleIndices, outlineVertexPositions, outlineVertexNormals);
        normalizeVertexPositions(outlineVertexPositions, gridAabb, nullptr);

        laplacianSmoothing(outlineTriangleIndices, outlineVertexPositions);
    }
}
