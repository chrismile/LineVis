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

#ifndef LINEVIS_SCATTERINGLINETRACINGREQUESTER_HPP
#define LINEVIS_SCATTERINGLINETRACINGREQUESTER_HPP

#include <thread>
#include <condition_variable>
#include <json/json.h>

#include "Loaders/DataSetList.hpp"
#include "LineDataScattering.hpp"
#include "Texture3d.hpp"

struct ScatteringTracingSettings {
    bool show_iso_surface         = true;
    std::string dataset_filename;

    uint32_t seed;
    // camera:
    float camera_fov_deg      = 10;
    glm::vec3 camera_position = {-0.5f,-0.5f,-0.5f};
    glm::vec3 camera_look_at  = { 0 ,0, 0};

    uint32_t res_x = 1;
    uint32_t res_y = 1;
    uint32_t samples_per_pixel = 10;

    // volume:
    glm::vec3 extinction        = { 1024, 1024, 1024};
    glm::vec3 scattering_albedo = {    1,    1,    1};
    float g                     = 0.2f;
};

/**
 * Traces lines inside a scalar field while simulation scattering.
 * A LineDataVolume object is generated containing the scalar field and the lines.
 */
class ScatteringLineTracingRequester {
public:
    ScatteringLineTracingRequester(
            sgl::TransferFunctionWindow& transferFunctionWindow
#ifdef USE_VULKAN_INTEROP
            , sgl::vk::Renderer* rendererMainThread
#endif
    );
    ~ScatteringLineTracingRequester();

    void renderGui();
    void setLineTracerSettings(const SettingsMap& settings);
    void setDatasetFilename(const std::string& newDatasetFilename);
    bool getHasNewData(DataSetInformation& dataSetInformation, LineDataPtr& lineData);

    /**
     * @return Whether a request is currently processed (for UI progress spinner).
     */
    [[nodiscard]] inline bool getIsProcessingRequest() const { return isProcessingRequest; }

private:
    void loadGridDataSetList();
    void requestNewData();

    /// The main loop of the requester thread.
    void mainLoop();
    /// Waits for the requester thread to terminate.
    void join();

    /**
     * Queues the request for tracing .
     * @param request The message to queue.
     */
    void queueRequestStruct(const ScatteringTracingSettings& request);
    /**
     * Checks if a reply was received to a request.
     * @return Whether a reply was received.
     */
    bool getReply(LineDataPtr& lineData);

    /**
     * @param request Information for the requested tracing of lines scattered in the grid.
     */
    void traceLines(const ScatteringTracingSettings& request, std::shared_ptr<LineDataScattering>& lineData);

    sgl::TransferFunctionWindow& transferFunctionWindow;
#ifdef USE_VULKAN_INTEROP
    sgl::vk::Renderer* rendererVk = nullptr;
#endif

    bool supportsMultiThreadedLoading = true;
    std::thread requesterThread;
    std::condition_variable hasRequestConditionVariable;
    std::condition_variable hasReplyConditionVariable;
    std::mutex requestMutex;
    std::mutex replyMutex;

    bool programIsFinished = false;
    bool hasRequest = false;
    bool hasReply = false;
    bool isProcessingRequest = false;

    ScatteringTracingSettings workerTracingSettings;
    LineDataPtr requestLineData;
    LineDataPtr replyLineData;

    // Line tracing settings.
    ScatteringTracingSettings guiTracingSettings;

    // Cache.
    std::string cachedGridFileName;
    Texture3D   cachedGrid = {};

    std::vector<uint32_t> outlineTriangleIndices;
    std::vector<glm::vec3> outlineVertexPositions;
    std::vector<glm::vec3> outlineVertexNormals;

    void createIsosurface();
#ifdef USE_VULKAN_INTEROP
    void createScalarFieldTexture();
    std::shared_ptr<LineDensityFieldSmoothingPass> lineDensityFieldSmoothingPass;
    sgl::vk::TexturePtr cachedScalarFieldTexture;
#endif

    // GUI data.
    bool showWindow = true;
    std::string lineDataSetsDirectory;
    std::string gridDataSetFilename;
    std::vector<std::string> gridDataSetNames;
    std::vector<std::string> gridDataSetFilenames;
    int selectedGridDataSetIndex = 0;
};

#endif //LINEVIS_SCATTERINGLINETRACINGREQUESTER_HPP
