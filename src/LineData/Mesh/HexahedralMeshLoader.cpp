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

#include "HexahedralMeshLoader.hpp"

#include <Utils/File/Logfile.hpp>
#include <Utils/File/FileLoader.hpp>
#include <Utils/Convert.hpp>

/**
 * Splits a string into tokens with the whitespace characters as delimiters (space, tabulator).
 * No empty tokens are generated.
 * @param inputString The string to split.
 * @param splitString A list of non-empty tokens.
 */
void splitStringByWhitespaceChars(const std::string& inputString, std::vector<std::string>& splitString) {
    const size_t stringSize = inputString.size();
    std::string currentStringPart = "";
    splitString.clear();

    for (size_t i = 0; i < stringSize; i++) {
        char currentChar = inputString.at(i);
        if (currentChar == ' ' || currentChar == '\t') {
            if (currentStringPart.length() > 0) {
                splitString.push_back(currentStringPart);
                currentStringPart.clear();
            }
        } else {
            currentStringPart.push_back(currentChar);
        }
    }

    if (currentStringPart.length() > 0) {
        splitString.push_back(currentStringPart);
    }
}

/**
 * Reads a text file line by line.
 * @param filename The name of the text file.
 * @param readLineCallback A function that is called when a new (non-empty) line was read. It is passed the line string
 * and the tokenized words (seperated by whitespace characters) and is expected to return false if an error occurred
 * while reading the line and true otherwise.
 * @return Whether loading has succeeded.
 */
bool readFileLineByLine(
        const std::string& filename,
        std::function<bool(const std::string&, const std::vector<std::string>&)> readLineCallback) {
    uint8_t* buffer = nullptr;
    size_t length = 0;
    bool loaded = sgl::loadFileFromSource(filename, buffer, length, false);
    if (!loaded) {
        return false;
    }
    char* fileBuffer = reinterpret_cast<char*>(buffer);

    std::string lineBuffer;
    std::string numberString;
    std::vector<std::string> lineWords;
    size_t lineNumber = 0;

    for (size_t charPtr = 0; charPtr < length; ) {
        while (charPtr < length) {
            char currentChar = fileBuffer[charPtr];
            if (currentChar == '\n' || currentChar == '\r') {
                charPtr++;
                break;
            }
            lineBuffer.push_back(currentChar);
            charPtr++;
        }
        lineNumber++;

        /*sgl::stringTrimCopy_if(lineBuffer, boost::is_any_of("\t "));
        boost::algorithm::split(
                lineWords, lineBuffer, boost::is_any_of("\t "), boost::token_compress_on);*/
        splitStringByWhitespaceChars(lineBuffer, lineWords);
        if (lineWords.size() == 0) {
            continue;
        }

        if (!readLineCallback(lineBuffer, lineWords)) {
            sgl::Logfile::get()->writeError(
                    std::string() + "Error in readFileLineByLine: An error occured at line "
                    + sgl::toString(lineNumber) + ". Content of the line:");
            sgl::Logfile::get()->writeError(lineBuffer);
            return false;
        }

        lineBuffer.clear();
        lineWords.clear();
    }

    delete[] buffer;
    buffer = nullptr;

    return true;
}
