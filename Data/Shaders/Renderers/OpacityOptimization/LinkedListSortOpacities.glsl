/**
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Maximilian Bandle, Christoph Neuhauser
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

#ifdef DEPTH_TYPE_UINT
#define DEPTH_TYPE uint
#else
#define DEPTH_TYPE float
#endif

// Swap two Frags in color and depth Array => Avoid bacause expensive
void swapFragments(uint i, uint j) {
    uint cTemp = colorList[i];
    colorList[i] = colorList[j];
    colorList[j] = cTemp;
    DEPTH_TYPE dTemp = depthList[i];
    depthList[i] = depthList[j];
    depthList[j] = dTemp;
}


void bubbleSort(uint fragsCount) {
    bool changed; // Has anything changed yet
    do {
        changed = false; // Nothing changed yet
        for (uint i = 0; i < fragsCount - 1; ++i) {
            // Go through all
            if(depthList[i] > depthList[i+1]) {
                // Order not correct => Swap
                swapFragments(i, i+1);
                changed = true; // Something has changed
            }
        }
    } while (changed); // Nothing changed => sorted
}


void insertionSort(uint fragsCount) {
    // Temporary fragment storage
    uint fragColor;
    DEPTH_TYPE fragDepth;

    uint i, j;
    for (i = 1; i < fragsCount; ++i) {
        // Get the fragment
        fragColor = colorList[i];
        fragDepth = depthList[i];

        j = i; // Store its position
        while (j >= 1 && depthList[j-1] > fragDepth) {
            // Shift the fragments through the list until place is found
            colorList[j] = colorList[j-1];
            depthList[j] = depthList[j-1];
            --j;
        }

        // Insert it at the right place
        colorList[j] = fragColor;
        depthList[j] = fragDepth;
    }
}


void shellSort(uint fragsCount) {
    // Temporary fragment storage
    uint fragColor;
    DEPTH_TYPE fragDepth;

    // Optimal gap sequence for 128 elements from [Ciu01, table 1]
    uint i, j, gap;
    uvec4 gaps = uvec4(24, 9, 4, 1);
    for(uint g = 0; g < 4; g++) {
        // For every gap
        gap = gaps[g]; // Current Cap
        for(i = gap; i < fragsCount; ++i) {
            // Get the fragment
            fragColor = colorList[i];
            fragDepth = depthList[i];
            j = i;

            // Shift earlier until correct
            while (j >= gap && depthList[j-gap] > fragDepth) {
                // Shift the fragments through the list until place is found
                colorList[j] = colorList[j-gap];
                depthList[j] = depthList[j-gap];
                j-=gap;
            }

            // Insert it at the right place
            colorList[j] = fragColor;
            depthList[j] = fragDepth;
        }
    }
}


void maxHeapSink(uint x, uint fragsCount) {
    uint c; // Child
    while((c = 2 * x + 1) < fragsCount) {
        // While children exist
        if(c + 1 < fragsCount && depthList[c] < depthList[c+1]) {
            // Find the biggest of both
            ++c;
        }

        if(depthList[x] >= depthList[c]) {
            // Does it have to sink
            return;
        } else {
            swapFragments(x, c);
            x = c; // Swap and sink again
        }
    }
}

void heapSort(uint fragsCount) {
    uint i;
    for (i = (fragsCount + 1)/2 ; i > 0 ; --i) {
        // Bring it to heap structure
        maxHeapSink(i-1, fragsCount); // Sink all inner nodes
    }
    // Heap => Sorted List
    for (i=1;i<fragsCount;++i) {
        swapFragments(0, fragsCount-i); // Swap max to List End
        maxHeapSink(0, fragsCount-i); // Sink the max to obtain correct heap
    }
}


// PQ doesn't make sense for opacity pass, so just use code from @see heapSort.
void frontToBackPQ(uint fragsCount) {
    uint i;
    for (i = (fragsCount + 1)/2 ; i > 0 ; --i) {
        // Bring it to heap structure
        maxHeapSink(i-1, fragsCount); // Sink all inner nodes
    }
    // Heap => Sorted List
    for (i=1;i<fragsCount;++i) {
        swapFragments(0, fragsCount-i); // Swap max to List End
        maxHeapSink(0, fragsCount-i); // Sink the max to obtain correct heap
    }
}

