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

#ifndef STRESSLINEVIS_LINEREADER_HPP
#define STRESSLINEVIS_LINEREADER_HPP

#include <vector>
#include <string>
#include <sstream>
#include <Utils/File/Logfile.hpp>
#include <Utils/Convert.hpp>

/**
 * For now, this reader uses no buffering and reads everything at once.
 * Performance-wise, this is better than std::ifstream, which causes an overhead, but using a medium-sized buffer might
 * be better for a future version.
 */
class LineReader {
public:
    LineReader(const std::string& filename);
    LineReader(const char* bufferData, const size_t bufferSize);
    ~LineReader();

    inline bool isLineLeft() {
        return !lineBuffer.empty();
    }

    void fillLineBuffer();

    template<typename T>
    T readScalarLine() {
        if (!isLineLeft()) {
            sgl::Logfile::get()->writeError("ERROR in LineReader::readVectorLine: No lines left.");
        }

        T value = sgl::fromString<T>(lineBuffer);
        fillLineBuffer();
        return value;
    }

    template<typename T>
    std::vector<T> readVectorLine() {
        if (!isLineLeft()) {
            sgl::Logfile::get()->writeError("ERROR in LineReader::readVectorLine: No lines left.");
        }

        std::string tokenString;
        std::vector<T> vec;

        for (size_t linePtr = 0; linePtr < lineBuffer.size(); linePtr++) {
            char currentChar = lineBuffer.at(linePtr);
            bool isWhitespace = currentChar == ' ' || currentChar == '\t';
            if (isWhitespace && tokenString.size() != 0) {
                vec.push_back(sgl::fromString<T>(tokenString.c_str()));
                tokenString.clear();
            } else if (!isWhitespace) {
                tokenString.push_back(currentChar);
            }
        }
        if (tokenString.size() != 0) {
            vec.push_back(sgl::fromString<T>(tokenString.c_str()));
            tokenString.clear();
        }

        fillLineBuffer();
        return vec;
    }

    template<typename T>
    std::vector<T> readVectorLine(size_t knownVectorSize) {
        if (!isLineLeft()) {
            sgl::Logfile::get()->writeError("ERROR in LineReader::readVectorLine: No lines left.");
        }

        std::string tokenString;
        std::vector<T> vec;
        vec.reserve(knownVectorSize);

        for (size_t linePtr = 0; linePtr < lineBuffer.size(); linePtr++) {
            char currentChar = lineBuffer.at(linePtr);
            bool isWhitespace = currentChar == ' ' || currentChar == '\t';
            if (isWhitespace && tokenString.size() != 0) {
                vec.push_back(sgl::fromString<T>(tokenString.c_str()));
                tokenString.clear();
            } else if (!isWhitespace) {
                tokenString.push_back(currentChar);
            }
        }
        if (tokenString.size() != 0) {
            vec.push_back(sgl::fromString<T>(tokenString.c_str()));
            tokenString.clear();
        }

        if (vec.size() != knownVectorSize) {
            sgl::Logfile::get()->writeError(
                    "WARNING in LineReader::readVectorLine: Expected and real size don't match.");
        }

        fillLineBuffer();
        return vec;
    }

private:
    bool userManagedBuffer;
    const char* bufferData = nullptr;
    size_t bufferSize = 0;
    size_t bufferOffset = 0;

    // For buffering read lines.
    std::string lineBuffer;
};


#endif //STRESSLINEVIS_LINEREADER_HPP
