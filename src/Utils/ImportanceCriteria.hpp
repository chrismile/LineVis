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

#ifndef LINEDENSITYCONTROL_IMPORTANCECRITERIA_HPP
#define LINEDENSITYCONTROL_IMPORTANCECRITERIA_HPP

#include <vector>
#include <cstdint>
#include <cfloat>

/// https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/packUnorm.xhtml
void packUnorm16Array(const std::vector<float>& floatVector, std::vector<uint16_t>& unormVector);

/// https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/packUnorm.xhtml
void packUnorm16ArrayOfArrays(
        const std::vector<std::vector<float>>& floatVector,
        std::vector<std::vector<uint16_t>>& unormVector);

/// https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/unpackUnorm.xhtml
void unpackUnorm16Array(uint16_t* unormVector, size_t vectorSize, std::vector<float> &floatVector);

#endif //LINEDENSITYCONTROL_IMPORTANCECRITERIA_HPP
