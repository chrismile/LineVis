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

#include <gtest/gtest.h>
#include <json/json.h>

#include <Utils/File/FileUtils.hpp>
#include <Utils/AppSettings.hpp>
#include <Graphics/Texture/Bitmap.hpp>
#include <Graphics/Vulkan/Utils/Instance.hpp>
#include <Graphics/Vulkan/Utils/Device.hpp>

#include "VolumetricPathTracingTestData.hpp"
#include "VolumetricPathTracingTestRenderer.hpp"

class VolumetricPathTracingTest : public ::testing::TestWithParam<std::string> {
protected:
    void SetUp() override {
        // Parse the passed JSON settings string.
        std::string settingsJsonString = GetParam();
        Json::Reader reader;
        Json::Value root;
        ASSERT_EQ(reader.parse(settingsJsonString, root), true);

        std::string mode0 = root["mode0"].asString();
        std::string mode1 = root["mode1"].asString();
        if (root.isMember("numSamples")) {
            numSamples = root["numSamples"].asInt();
        }
        if (root.isMember("blockSize")) {
            blockSize = root["blockSize"].asInt();
        }
        if (root.isMember("renderingResolution")) {
            blockSize = root["renderingResolution"].asInt();
        }

        CloudDataPtr cloudData = createCloudBlock(blockSize, blockSize, blockSize, 1.0f);
        renderer = new sgl::vk::Renderer(sgl::AppSettings::get()->getPrimaryDevice());

        vptRenderer0 = std::make_shared<VolumetricPathTracingTestRenderer>(renderer);
        vptRenderer0->setCloudData(cloudData);
        vptRenderer0->setRenderingResolution(renderingResolution, renderingResolution);
        vptRenderer0->setVptModeFromString(mode0);

        vptRenderer1 = std::make_shared<VolumetricPathTracingTestRenderer>(renderer);
        vptRenderer1->setCloudData(cloudData);
        vptRenderer1->setRenderingResolution(renderingResolution, renderingResolution);
        vptRenderer1->setVptModeFromString(mode1);
    }

    void TearDown() override {
        vptRenderer0 = {};
        vptRenderer1 = {};
        if (renderer) {
            delete renderer;
            renderer = nullptr;
        }
    }

    static void debugOutputImage(const std::string& filename, const float* frameData, uint32_t width, uint32_t height) {
        sgl::BitmapPtr bitmap(new sgl::Bitmap(int(width), int(height), 32));
        uint8_t* bitmapData = bitmap->getPixels();
        for (uint32_t y = 0; y < height; y++) {
            for (uint32_t x = 0; x < width; x++) {
                for (uint32_t c = 0; c < 3; c++) {
                    float value = frameData[(x + y * width) * 3 + c];
                    bitmapData[(x + y * width) * 4 + c] = uint8_t(std::clamp(value * 255.0f, 0.0f, 255.0f));
                }
                bitmapData[(x + y * width) * 4 + 3] = 255;
            }
        }
        bitmap->savePNG(filename.c_str(), false);
    }

    sgl::vk::Renderer* renderer = nullptr;
    int numSamples = 16;
    int renderingResolution = 256;
    int blockSize = 1;
    std::shared_ptr<VolumetricPathTracingTestRenderer> vptRenderer0;
    std::shared_ptr<VolumetricPathTracingTestRenderer> vptRenderer1;
};

/**
 * Test whether different volumetric path tracing renderers produce the same image mean when rendering a cube with
 * constant density across the whole volume domain.
 */
TEST_P(VolumetricPathTracingTest, EqualMeanTest) {
    uint32_t width = vptRenderer0->getFrameWidth();
    uint32_t height = vptRenderer0->getFrameHeight();
    float* frameData0 = vptRenderer0->renderFrame(numSamples);
    float* frameData1 = vptRenderer1->renderFrame(numSamples);

    float numPixelsFlt = float(width * height);
    float mean0[] = { 0.0f, 0.0f, 0.0f };
    float mean1[] = { 0.0f, 0.0f, 0.0f };
    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            for (uint32_t c = 0; c < 3; c++) {
                mean0[c] += frameData0[(x + y * width) * 3 + c] / numPixelsFlt;
                mean1[c] += frameData1[(x + y * width) * 3 + c] / numPixelsFlt;
            }
        }
    }

    for (uint32_t c = 0; c < 3; c++) {
        if (std::abs(mean0[c] - mean1[c]) > 1e-5f) {
            debugOutputImage("out0.png", frameData0, width, height);
            debugOutputImage("out1.png", frameData1, width, height);
        }
        ASSERT_NEAR(mean0[c], mean1[c], 1e-5);
    }
}

INSTANTIATE_TEST_SUITE_P(
        EqualityTestSuite, VolumetricPathTracingTest, ::testing::Values(
        "{ \"mode0\": \"Delta Tracking\", \"mode1\": \"Ratio Tracking\" }"/*,
        "{ \"mode0\": \"Delta Tracking\", \"mode1\": \"Decomposition Tracking\" }"*/));

void vulkanErrorCallback() {
    std::cerr << "Application callback" << std::endl;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);

    // Initialize the filesystem utilities.
    sgl::FileUtils::get()->initialize("LineVis", argc, argv);

    // Load the file containing the app settings.
    std::string settingsFile = sgl::FileUtils::get()->getConfigDirectory() + "settings.txt";
    sgl::AppSettings::get()->setSaveSettings(false);
    sgl::AppSettings::get()->getSettings().addKeyValue("window-debugContext", true);

#ifdef DATA_PATH
    if (!sgl::FileUtils::get()->directoryExists("Data") && !sgl::FileUtils::get()->directoryExists("../Data")) {
        sgl::AppSettings::get()->setDataDirectory(DATA_PATH);
    }
#endif

    sgl::AppSettings::get()->setRenderSystem(sgl::RenderSystem::VULKAN);
    sgl::AppSettings::get()->createHeadless();

    sgl::vk::Instance* instance = sgl::AppSettings::get()->getVulkanInstance();
    instance->setDebugCallback(&vulkanErrorCallback);
    sgl::vk::Device* device = new sgl::vk::Device;
    device->createDeviceHeadless(instance);
    sgl::AppSettings::get()->setPrimaryDevice(device);
    sgl::AppSettings::get()->initializeSubsystems();

    int returnValue = RUN_ALL_TESTS();

    sgl::AppSettings::get()->release();
    return returnValue;
}
