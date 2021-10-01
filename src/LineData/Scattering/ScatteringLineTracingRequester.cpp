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
        ImGui::Checkbox("Use Isosurface", &tracing_settings.useIsosurface);
#endif

        changed |= ImGui::SliderFloat("Camera FOV",
                                      &tracing_settings.camera_fov_deg, 5.0f, 90.0f);
        changed |= ImGui::SliderFloat3("Camera Position",
                                       &tracing_settings.camera_position.x, -1, 1);
        changed |= ImGui::SliderFloat3("Camera Look At",
                                       &tracing_settings.camera_look_at.x, -1, 1);

        // TODO(Felix): What if user enters negative numbers?
        changed |= ImGui::InputInt("Res X", (int*)&tracing_settings.res_x);
        changed |= ImGui::InputInt("Res Y", (int*)&tracing_settings.res_y);
        changed |= ImGui::InputInt("Samples per Pixel",
                                   (int*)&tracing_settings.samples_per_pixel);

        changed |= ImGui::SliderFloat3("Extinction",
                                      &tracing_settings.extinction.x, 0.0f, 100.0f);
        changed |= ImGui::SliderFloat3("Scattering Albedo",
                                      &tracing_settings.scattering_albedo.x, 0.0f, 1.0f);
        changed |= ImGui::SliderFloat3("G",
                                      &tracing_settings.g.x, 0.0f, 1.0f);

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

    Json::Value request;
    const std::string lineDataSetsDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";

    request["gridDataSetFilename"] = boost::filesystem::absolute(
            lineDataSetsDirectory + gridDataSetFilename).generic_string();

#define add_to_request(thing) request[#thing] = tracing_settings.thing
    add_to_request(useIsosurface);
    add_to_request(camera_fov_deg);
    add_to_request(camera_position.x);
    add_to_request(camera_position.y);
    add_to_request(camera_position.z);

    add_to_request(camera_look_at.x);
    add_to_request(camera_look_at.y);
    add_to_request(camera_look_at.z);

    add_to_request(res_x);
    add_to_request(res_y);

    add_to_request(samples_per_pixel);

    add_to_request(extinction.x);
    add_to_request(extinction.y);
    add_to_request(extinction.z);

    add_to_request(scattering_albedo.x);
    add_to_request(scattering_albedo.y);
    add_to_request(scattering_albedo.z);

    add_to_request(g.x);
    add_to_request(g.y);
    add_to_request(g.z);
#undef add_to_request

    queueRequestJson(request);
    isProcessingRequest = true;
}

bool ScatteringLineTracingRequester::getHasNewData(DataSetInformation& dataSetInformation, LineDataPtr& lineData) {
    Json::Value reply;
    if (getReplyJson(reply, lineData)) {
        isProcessingRequest = false;
        return true;
    }
    return false;
}

void ScatteringLineTracingRequester::queueRequestJson(const Json::Value& request) {
    {
        std::lock_guard<std::mutex> lock(replyMutex);
        requestMessage = request;
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

bool ScatteringLineTracingRequester::getReplyJson(Json::Value& reply, LineDataPtr& lineData) {
    bool hasReply;
    {
        std::lock_guard<std::mutex> lock(replyMutex);
        hasReply = this->hasReply;
        if (hasReply) {
            reply = replyMessage;
            lineData = replyLineData;
        }

        // Now, new requests can be worked on.
        this->hasReply = false;
        this->replyMessage.clear();
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
                this->replyMessage.clear();
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
            Json::Value request = requestMessage;
            std::shared_ptr<LineDataScattering> lineData = std::static_pointer_cast<LineDataScattering>(requestLineData);
            hasRequest = false;
            isProcessingRequest = true;
            requestLock.unlock();

            Json::Value reply = traceLines(request, lineData);

            std::lock_guard<std::mutex> replyLock(replyMutex);
            hasReply = true;
            replyMessage = reply;
            replyLineData = lineData;
            isProcessingRequest = false;
        }
    }
}

Json::Value ScatteringLineTracingRequester::traceLines(
        const Json::Value& request, std::shared_ptr<LineDataScattering>& lineData)
{
    std::string gridDataSetFilename = request["gridDataSetFilename"].asString();
    bool useIsosurface = request["useIsosurface"].asBool();

    // if the user changed the file
    if (gridDataSetFilename != cached_grid_file_name) {
        cached_grid.delete_maybe();
        cached_grid_file_name = gridDataSetFilename;
        cached_grid = load_xyz_file(cached_grid_file_name);

        if (useIsosurface) {
            createScalarFieldTexture();
            createIsosurface();
        }
    }

    PathInfo pi {};
    pi.camera_pos = {
        request["camera_position.x"].asFloat(),
        request["camera_position.y"].asFloat(),
        request["camera_position.z"].asFloat(),
    };
    // TODO(Felix): use look at here
    pi.ray_direction = glm::normalize(glm::vec3{1.0f,1.0f,1.0f});

    VolumeInfo vi {};
    vi.grid = cached_grid;
    vi.extinction = {
        request["extinction.x"].asFloat(),
        request["extinction.y"].asFloat(),
        request["extinction.z"].asFloat()
    };
    vi.scattering_albedo = {
        request["scattering_albedo.x"].asFloat(),
        request["scattering_albedo.y"].asFloat(),
        request["scattering_albedo.z"].asFloat()
    };
    vi.g = {
        request["g.x"].asFloat(),
        request["g.y"].asFloat(),
        request["g.z"].asFloat()
    };


    float camera_fov_deg = request["camera_fov_deg"].asFloat();
    uint32_t res_x = request["res_x"].asUInt();
    uint32_t res_y = request["res_y"].asUInt();
    uint32_t samples_per_pixel = request["samples_per_pixel"].asUInt();


    Trajectories trajectories;
    pi.pass_number = 0;
    Random::init(tracing_settings.seed);

    { // NEW

        glm::vec3 Y = { 0, 0, -1 };
        glm::vec3 X = glm::cross(pi.ray_direction, Y);
        Y = glm::cross(X, pi.ray_direction);

        float camera_fov_rad = glm::radians(camera_fov_deg);
        float grid_width = tan(camera_fov_rad / 2) * 2 * tracing_settings.focal_length;
        float grid_height = res_y * 1.0 / res_x * grid_width;

        glm::vec3 P0 =
            pi.camera_pos +
            pi.ray_direction * tracing_settings.focal_length -
            0.5f * Y * grid_height -
            0.5f * X * grid_width;

        // Pixel loop
        for (int y = 0; y < res_y; ++y) {
            for (int x = 0; x < res_x; ++x) {
                glm::vec3 P =
                    P0 +
                    X * (x*1.0f/(res_x-1)) * grid_width +
                    Y * (y*1.0f/(res_y-1)) * grid_height;

                P = glm::normalize(P - pi.camera_pos);

                pi.ray_direction = P;
                // Sample loop

                for (int i = 0; i < samples_per_pixel; ++i) {
                    pi.pass_number = i;
                    // DTPathtrace(PI, S.VolInfo, &RayPack);
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

    return {};
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
