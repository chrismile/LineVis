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

#ifndef HEXVOLUMERENDERER_CSVWRITER_HPP
#define HEXVOLUMERENDERER_CSVWRITER_HPP

#include <string>
#include <vector>
#include <fstream>

class CsvWriter {
public:
    CsvWriter();
    CsvWriter(const std::string& filename);
    ~CsvWriter();

    bool open(const std::string& filename);
    void close();

    // Note: All writing functions use "escapeString" to convert strings to a format valid for CSV.
    // For more details see: https://en.wikipedia.org/wiki/Comma-separated_values

    /**
     * Writes a row of string to the CSV file. The row is escaped internally if necessary.
     * NOTE: After a call to "writeCell", "writeRow" can only be called after calling "newRow" to end the row.
     */
    void writeRow(const std::vector<std::string>& row);

    /**
     * Writes a single cell string to the CSV file. The string is escaped internally if necessary.
     */
    void writeCell(const std::string& cell);
    void newRow();

private:
    std::string escapeString(const std::string& s);
    bool isOpen = false;
    bool writingRow = false;
    std::ofstream file;
};

#endif //HEXVOLUMERENDERER_CSVWRITER_HPP
