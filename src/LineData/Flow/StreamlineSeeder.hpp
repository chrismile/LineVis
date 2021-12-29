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

#ifndef LINEVIS_STREAMLINESEEDER_HPP
#define LINEVIS_STREAMLINESEEDER_HPP

#include <memory>
#include <glm/vec3.hpp>

class StreamlineTracingGrid;

class StreamlineSeeder {
public:
    virtual ~StreamlineSeeder() = default;

    /**
     * @return A copy of the streamline seeder that can be passed to the worker thread.
     * NOTE: The returned pointer needs to be destroyed using "delete[]".
     */
    virtual StreamlineSeeder* copy() = 0;

    /**
     * Resets the internal state of the streamline tracer.
     * @param newGrid The grid used for tracing.
     */
    virtual void reset(StreamlineTracingGrid* newGrid) = 0;

    /**
     * @return The next seed point.
     * NOTE: @see reset needs to be called before calling this function.
     */
    virtual glm::vec3 getNextPoint() = 0;

    /**
     * Renders the GUI for changing the internal settings using ImGui.
     */
    virtual bool renderGui() = 0;
};

typedef std::shared_ptr<StreamlineSeeder> StreamlineSeederPtr;

class StreamlinePlaneSeeder : public StreamlineSeeder {
public:
    ~StreamlinePlaneSeeder() override = default;
    StreamlineSeeder* copy() override;
    void reset(StreamlineTracingGrid* newGrid) override;
    glm::vec3 getNextPoint() override;
    bool renderGui() override;

private:
    StreamlineTracingGrid* grid = nullptr;
    float planeDistance = 0.0f;
    glm::vec3 planeNormal = glm::vec3(0.0f, 1.0f, 0.0f);
};

#endif //LINEVIS_STREAMLINESEEDER_HPP
