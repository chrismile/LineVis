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

#include "CsvParser.hpp"

RowMap parseCsv(const std::string& filename, bool filterComments) {
    RowMap rows;

    // Open the file and load its content
    std::ifstream file(filename, std::ios::in);
    if (!file.is_open()) {
        std::cerr << "ERROR in parseCsv: File " << filename << "doesn't exist!" << std::endl;
        exit(1);
    }
    std::string fileContent = std::string(
            (std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));
    file.close();
    size_t fileLength = fileContent.size();

    std::vector<std::string> row;
    std::string currCell;
    bool stringMode = false;
    for (size_t i = 0; i < fileLength; ++i) {
        // Load data from string
        char cCurr = fileContent.at(i);
        char cNext = '\0';
        if (i < fileLength-1) {
            cNext = fileContent.at(i+1);
        }

        if (filterComments && cCurr == '#') {
            while (fileContent.at(i) != '\n') {
                ++i;
            }
            continue;
        }

        // Check for (") in string
        if (cCurr == '\"') {
            if (cNext == '\"' && stringMode) {
                currCell += cCurr;
                ++i;
            } else {
                stringMode = !stringMode;
            }
            continue;
        }

        // String mode: Just add character
        if (stringMode) {
            currCell += cCurr;
        } else {
            // New cell
            if (cCurr == ',') {
                row.push_back(currCell);
                currCell = "";
            }

                // New line/row
            else if (cCurr == '\n') {
                if (currCell.size() != 0) {
                    row.push_back(currCell);
                    currCell = "";
                }
                rows.push_back(row);
                row.clear();
            }

                // Normal character
            else {
                currCell += cCurr;
            }
        }
    }

    // Any data left?
    if (currCell.size() != 0) {
        row.push_back(currCell);
        currCell = "";
    }
    if (row.size() != 0) {
        rows.push_back(row);
        row.clear();
    }

    return rows;
}

// Test main method
/*int main() {
    RowMap rows = parseCsv("test.csv");

    for (auto& row : rows) {
        for (string& cell : row) {
            cout << cell << endl;
        }
        cout << "--- NEWROW ---" << endl;
    }
    cout << "--- END OF FILE ---" << endl;

    return 0;
}*/
