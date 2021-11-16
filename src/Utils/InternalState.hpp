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

#ifndef HEXVOLUMERENDERER_INTERNALSTATE_HPP
#define HEXVOLUMERENDERER_INTERNALSTATE_HPP

#include <string>
#include <map>
#include <glm/glm.hpp>

#include <Utils/AppSettings.hpp>
#include <Utils/Convert.hpp>

#include "Utils/VecStringConversion.hpp"
#include "Loaders/DataSetList.hpp"
#include "Renderers/RenderingModes.hpp"

class SettingsMap {
public:
    SettingsMap() = default;
    SettingsMap(const std::map<std::string, std::string>& stringMap) : settings(stringMap) {}
    inline std::string getValue(const char *key) const { auto it = settings.find(key); return it == settings.end() ? "" : it->second; }
    inline int getIntValue(const char *key) const { return sgl::fromString<int>(getValue(key)); }
    inline float getFloatValue(const char *key) const { return sgl::fromString<float>(getValue(key)); }
    inline bool getBoolValue(const char *key) const { std::string val = getValue(key); if (val == "false" || val == "0") return false; return val.length() > 0; }
    inline void addKeyValue(const std::string &key, const std::string &value) { settings[key] = value; }
    template<typename T> inline void addKeyValue(const std::string &key, const T &value) { settings[key] = toString(value); }
    inline void clear() { settings.clear(); }

    bool getValueOpt(const char *key, std::string &toset) const {
        auto it = settings.find(key);
        if (it != settings.end()) {
            toset = it->second;
            return true;
        }
        return false;
    }
    bool getValueOpt(const char *key, bool &toset) const {
        auto it = settings.find(key);
        if (it != settings.end()) {
            toset = (it->second == "true") || (it->second == "1");
            return true;
        }
        return false;
    }
    bool getValueOpt(const char *key, glm::vec2 &toset) const {
        auto it = settings.find(key);
        if (it != settings.end()) {
            toset = stringToVec2(it->second);
            return true;
        }
        return false;
    }
    bool getValueOpt(const char *key, glm::vec3 &toset) const {
        auto it = settings.find(key);
        if (it != settings.end()) {
            toset = stringToVec3(it->second);
            return true;
        }
        return false;
    }
    bool getValueOpt(const char *key, glm::vec4 &toset) const {
        auto it = settings.find(key);
        if (it != settings.end()) {
            toset = stringToVec4(it->second);
            return true;
        }
        return false;
    }
    template<typename T> bool getValueOpt(const char *key, T &toset) const {
        auto it = settings.find(key);
        if (it != settings.end()) {
            toset = sgl::fromString<T>(it->second);
            return true;
        }
        return false;
    }

    void set(const std::map<std::string, std::string>& stringMap) {
        settings = stringMap;
    }

    [[nodiscard]] const std::map<std::string, std::string>& getMap() const {
        return settings;
    }

    bool operator==(const SettingsMap& rhs) const {
        return this->settings == rhs.settings;
    }
    bool operator!=(const SettingsMap& rhs) const {
        return !(*this == rhs);
    }

    //template<typename T>
    //unsigned const T &operator[](const std::string &key) const { return getValue(key); }
    //unsigned T &operator[](const std::string &key) { return getValue(key); }

private:
    std::map<std::string, std::string> settings;
};

struct DataSetDescriptor {
    DataSetDescriptor() {}
    DataSetDescriptor(DataSetType type, const std::string& name, const std::string& filename)
            : type(type), name(name) {
        const std::string lineDataSetsDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";
        this->filenames.push_back(lineDataSetsDirectory + filename);
        enabledFileIndices.push_back(true);
    }
    DataSetDescriptor(DataSetType type, const std::string& name, const std::vector<std::string>& filenames)
            : type(type), name(name) {
        const std::string lineDataSetsDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";
        for (const std::string& filename : filenames) {
            this->filenames.push_back(lineDataSetsDirectory + filename);
            enabledFileIndices.push_back(true);
        }
    }
    DataSetDescriptor(
            DataSetType type, const std::string& name, const std::vector<std::string>& filenames,
            const std::vector<bool>& enabledFileIndices)
            : type(type), name(name), enabledFileIndices(enabledFileIndices) {
        const std::string lineDataSetsDirectory = sgl::AppSettings::get()->getDataDirectory() + "LineDataSets/";
        for (const std::string& filename : filenames) {
            this->filenames.push_back(lineDataSetsDirectory + filename);
        }
    }
    /// Assumes 'name' is a valid entry in datasets.json. Everything else is inferred from there.
    DataSetDescriptor(const std::string& name)
            : type(DATA_SET_TYPE_FLOW_LINES), name(name) {}

    bool operator==(const DataSetDescriptor& rhs) const {
        return this->type == rhs.type && this->name == rhs.name && this->filenames == rhs.filenames
                && this->enabledFileIndices == rhs.enabledFileIndices;
    }
    bool operator!=(const DataSetDescriptor& rhs) const {
        return !(*this == rhs);
    }

    DataSetType type;
    std::string name;
    std::vector<std::string> filenames;
    std::vector<bool> enabledFileIndices;
};

struct InternalState {
    bool operator==(const InternalState &rhs) const {
        return this->dataSetDescriptor == rhs.dataSetDescriptor && this->name == rhs.name
               && this->renderingMode == rhs.renderingMode
               && this->rendererSettings == rhs.rendererSettings
               && this->filterSettings == rhs.filterSettings
               && this->tilingWidth == rhs.tilingWidth && this->tilingHeight == rhs.tilingHeight
               && this->useMortonCodeForTiling == rhs.useMortonCodeForTiling
               && this->transferFunctionName == rhs.transferFunctionName
               && this->windowResolution == rhs.windowResolution;
    }

    bool operator!=(const InternalState& rhs) const {
        return !(*this == rhs);
    }

    DataSetDescriptor dataSetDescriptor;
    std::string name;
    RenderingMode renderingMode;
    SettingsMap rendererSettings;
    std::vector<SettingsMap> filterSettings;
    int tilingWidth = 2;
    int tilingHeight = 8;
    bool useMortonCodeForTiling = false;
    std::string transferFunctionName;
    glm::ivec2 windowResolution = glm::ivec2(0, 0);
};

std::vector<InternalState> getTestModesPaper();

#endif //HEXVOLUMERENDERER_INTERNALSTATE_HPP
