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

#define _FILE_OFFSET_BITS 64

#include <cstdio>

#include <Utils/File/Logfile.hpp>

#include "LineReader.hpp"

LineReader::LineReader(const std::string& filename)
        : userManagedBuffer(false), bufferData(nullptr), bufferSize(0) {
    FILE* file = fopen64(filename.c_str(), "r");
    if (!file) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in LineReader::LineReader: File \""
                + filename + "\" does not exist.");
        return;
    }
#if defined(_WIN32) && !defined(__MINGW32__)
    _fseeki64(file, 0, SEEK_END);
        size_t length = _ftelli64(file);
        _fseeki64(file, 0, SEEK_SET);
#else
    fseeko(file, 0, SEEK_END);
    size_t length = ftello(file);
    fseeko(file, 0, SEEK_SET);
#endif

    char* fileBuffer = new char[length];
    if (fileBuffer == nullptr) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in LineReader::LineReader: Couldn't reserve sufficient "
                + "memory for reading the file \"" + filename + "\".");
        return;
    }
    size_t result = fread(fileBuffer, 1, length, file);
    fclose(file);
    if (result != length) {
        sgl::Logfile::get()->writeError(
                std::string() + "Error in LineReader::LineReader: Invalid return value when "
                + "reading the file \"" + filename + "\".");
        delete[] fileBuffer;
        return;
    }

    bufferData = fileBuffer;
    bufferSize = length;
    fillLineBuffer();
}

LineReader::LineReader(const char* bufferData, const size_t bufferSize)
        : userManagedBuffer(true), bufferData(bufferData), bufferSize(bufferSize) {
    fillLineBuffer();
}

LineReader::~LineReader() {
    if (!userManagedBuffer && bufferData) {
        delete[] bufferData;
    }
    bufferData = nullptr;
    bufferSize = 0;
}


void LineReader::fillLineBuffer() {
    lineBuffer.clear();
    while (bufferOffset < bufferSize) {
        lineBuffer.clear();

        while (bufferOffset < bufferSize) {
            char currentChar = bufferData[bufferOffset];
            if (currentChar == '\n' || currentChar == '\r') {
                bufferOffset++;
                break;
            }
            lineBuffer.push_back(currentChar);
            bufferOffset++;
        }

        if (lineBuffer.size() == 0) {
            continue;
        } else {
            break;
        }
    }
}
