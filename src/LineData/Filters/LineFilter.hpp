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

#ifndef LINEVIS_LINEFILTER_HPP
#define LINEVIS_LINEFILTER_HPP

#include <memory>

namespace sgl {
class PropertyEditor;
}

class LineData;
typedef std::shared_ptr<LineData> LineDataPtr;

struct InternalState;
class SettingsMap;

class LineFilter {
public:
    virtual ~LineFilter() = default;

    // Returns if the filter is active and should be applied on the input data.
    [[nodiscard]] inline bool isEnabled() const { return enabled; }
    // Returns if the visualization mapping needs to be re-generated.
    [[nodiscard]] inline bool isDirty() const { return dirty; }

    // Called when a new data set is loaded from a file.
    virtual void onDataLoaded(LineDataPtr lineDataIn) {}
    virtual void filterData(LineDataPtr lineDataIn)=0;

    /// Renders the entries in the property editor.
    virtual void renderGuiPropertyEditorNodes(sgl::PropertyEditor& propertyEditor)=0;

    /// For changing performance measurement modes.
    virtual void setNewState(const InternalState& newState) {}
    virtual void setNewSettings(const SettingsMap& settings) {}

protected:
    bool enabled = true;
    bool dirty = true;
    bool showFilterWindow = true;
    bool canUseLiveUpdate = true;
};

#endif //LINEVIS_LINEFILTER_HPP
