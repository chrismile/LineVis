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

#ifndef STRESSLINEVIS_TOKENS_HPP
#define STRESSLINEVIS_TOKENS_HPP

#include <string>
#include <vector>

/**
 * Regex string for token parsing of CSS/SVG numbers.
 * First part: Integer number (e.g. -10)
 * Second part: Floating point (e.g. 12.34 or -.12)
 * Third part: Exponential notation (e.g. 10e-2).
 */
const std::string NUMBER_REGEX_STRING =
        "([\\+-]?(([[:digit:]]+)|(([[:digit:]]*)?\\.([[:digit:]]+)?)|([[:digit:]]+[eE][\\+-]?[[:digit:]]+)))";

/**
 * Regex string for token parsing of CSS/SVG numbers, with additional support for values with a degree sign added.
 * First part: Integer number (e.g. -10)
 * Second part: Floating point (e.g. 12.34 or -.12)
 * Third part: Exponential notation (e.g. 10e-2).
 */
const std::string NUMBER_AND_DEGREES_REGEX_STRING =
        "([\\+-]?(([[:digit:]]+)|(([[:digit:]]*)?\\.([[:digit:]]+)?)|([[:digit:]]+[eE][\\+-]?[[:digit:]]+))(°)?)";

/**
 * Returns a list containing all tokens in 'str' specified by 'exprStr'.
 * @param str: The string to parse.
 * @param exprStr: A regular expression string describing the structure of a token.
 */
std::vector<std::string> getTokenList(const std::string &str, const std::string &exprStr);

/**
 * Tests whether the string 'str' marches the regex string 'exprStr'.
 * @param str: The string to test.
 * @param exprStr: The regular expression.
 */
bool regexMatches(const std::string &str, const std::string &exprStr);

#endif //STRESSLINEVIS_TOKENS_HPP
