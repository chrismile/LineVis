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

#ifndef STRESSLINEVIS_PPLL_HPP
#define STRESSLINEVIS_PPLL_HPP

const char* const SORTING_MODE_NAMES[] = {
        "Priority Queue", "Bubble Sort", "Insertion Sort", "Shell Sort", "Max Heap", "Bitonic Sort",
        "Quicksort", "Quicksort Hybrid"
};

const int NUM_SORTING_MODES = ((int)(sizeof(SORTING_MODE_NAMES) / sizeof(*SORTING_MODE_NAMES)));

enum SortingAlgorithmMode {
    SORTING_ALGORITHM_MODE_PRIORITY_QUEUE,
    SORTING_ALGORITHM_MODE_BUBBLE_SORT,
    SORTING_ALGORITHM_MODE_INSERTION_SORT,
    SORTING_ALGORITHM_MODE_SHELL_SORT,
    SORTING_ALGORITHM_MODE_MAX_HEAP,
    SORTING_ALGORITHM_MODE_BITONIC_SORT,
    SORTING_ALGORITHM_MODE_QUICKSORT,
    SORTING_ALGORITHM_MODE_QUICKSORT_HYBRID
};

#endif //STRESSLINEVIS_PPLL_HPP
