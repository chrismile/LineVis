/**
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
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

//const int STACK_SIZE = 18; // ceil(log2(MAX_NUM_FRAGS) / 2) * 2 * 2
int stackMemory[STACK_SIZE];
int stackCounter = 0;

void stackPush(int value) {
    //assert(stackCounter < STACK_SIZE);
    if (stackCounter < STACK_SIZE) {
        stackMemory[stackCounter] = value;
        stackCounter++;
    }
}

int stackPop() {
    //assert(stackCounter > 0);
    if (stackCounter > 0) {
        stackCounter--;
        return stackMemory[stackCounter];
    } else {
        return 0;
    }
}

bool stackEmpty() {
    return stackCounter == 0;
}

int partitionQuicksortLomuto(int low, int high) {
    DEPTH_TYPE pivotElement = depthList[high];
    int i = low;
    for (int j = low; j <= high; j++) {
        if (depthList[j] < pivotElement) {
            swapFragments(i, j);
            i++;
        }
    }
    swapFragments(i, high);
    return i;
}

int partitionQuicksortHoare(int low, int high) {
    // Take first, middle, and last element. Then, use the median as the pivot element.
    DEPTH_TYPE e0 = depthList[low];
    DEPTH_TYPE e1 = depthList[(low + high) / 2];
    DEPTH_TYPE e2 = depthList[high];
    DEPTH_TYPE pivotElement = e0 < e1 ? (e2 < e0 ? e0 : min(e1, e2)) : (e2 < e1 ? e1 : min(e0, e2));

    int i = low - 1;
    int j = high + 1;
    while (true) {
        do {
            i = i + 1;
        } while (depthList[i] < pivotElement);
        do {
            j = j - 1;
        } while (depthList[j] > pivotElement);
        if (i >= j) {
            return j;
        }
        swapFragments(i, j);
    }
    return 0;
}

vec4 quicksort(uint fragsCount) {
    stackPush(0);
    stackPush(int(fragsCount) - 1);

    while (!stackEmpty()) {
        int high = stackPop();
        int low = stackPop();

        int pivot = partitionQuicksortLomuto(low, high);

        if (low < pivot - 1) {
            stackPush(low);
            stackPush(pivot - 1);
        }

        if (pivot + 1 < high) {
            stackPush(pivot + 1);
            stackPush(high);
        }
    }

    return blendFTB(fragsCount);
}

vec4 quicksortHybrid(uint fragsCount) {
    stackPush(0);
    stackPush(int(fragsCount) - 1);

    if (fragsCount > 16) {
        while (!stackEmpty()) {
            int high = stackPop();
            int low = stackPop();

            int pivot = partitionQuicksortHoare(low, high);

            if (low + 16 < pivot) {
                stackPush(low);
                stackPush(pivot - 1);
            }

            if (pivot + 16 < high) {
                stackPush(pivot + 1);
                stackPush(high);
            }
        }
    }
    insertionSort(fragsCount);

    return blendFTB(fragsCount);
}
