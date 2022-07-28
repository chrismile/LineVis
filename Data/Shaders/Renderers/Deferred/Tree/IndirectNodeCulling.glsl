/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2022, Christoph Neuhauser
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

-- Compute

#version 450 core

#define WORKGROUP_SIZE 256

layout(local_size_x = WORKGROUP_SIZE) in;

struct VkDrawIndexedIndirectCommand {
    uint indexCount;
    uint instanceCount;
    uint firstIndex;
    int vertexOffset;
    uint firstInstance;
} VkDrawIndexedIndirectCommand;

struct Node {
    vec3 worldSpaceAabbMin;
    uint childIdx0;
    vec3 worldSpaceAabbMax;
    uint childIdx1;
};

layout(std430, binding = 0) readonly buffer NodeBuffer {
    Node nodes[];
};

layout(std430, binding = 1) buffer QueueBuffer {
    uint nodeIndicesQueue[];
};

layout(std430, binding = 2) coherent buffer AtomicCounterBuffer {
    int queueLock;
    int numWorkPending;
    uint queueReadIdx;
    uint queueWriteIdx;
};

layout(binding = 3) uniform NodeCullingUniformBuffer {
    mat4 modelViewProjectionMatrix;
    ivec2 viewportSize;
};


void main0() {
    if (gl_LocalInvocationID.x == 0) {
        Node rootNode = nodes[rootNodeIdx];
        if (visibilityCulling(rootNode.worldSpaceAabbMin, rootNode.worldSpaceAabbMin)) {
            atomicExchange(numWorkPending, 1);
            atomicExchange(queueReadIdx, 0);
            atomicExchange(queueWriteIdx, 1);
            nodeIndicesQueue[0] = rootNodeIdx;
        }
    }
}


void main1() {
    /*
     * Use persistent thread model.
     */
    uint workIdx;
    while (true) {
        bool isItemValid = false;
        if (atomicCompSwap(queueLock, 0, 1) == 0) {
            if (queueReadIdx < queueWriteIdx) {
                workIdx = atomicAdd(queueWriteIdx, 1u);
                isItemValid = true;
            }
            //memoryBarrierBuffer();
            atomicExchange(queueLock, 0);
        }
        if (!isItemValid) {
            if (numWorkPending == 0) {
                return;
            }
            continue;
        }

        // Get the node.
        uint nodeIdx = nodeIndicesQueue[workIdx];
        Node node = nodes[nodeIdx];

        if (visibilityCulling(node.worldSpaceAabbMin, node.worldSpaceAabbMin)) {
            uint numWorkQueuedLocal = 0u;
            uint childIdx0 = 0xFFFFFFFFu, childIdx1 = 0xFFFFFFFFu;
            if (node.childIdx0 != 0xFFFFFFFFu) {
                numWorkQueuedLocal++;
                childIdx0 = node.childIdx0;
            }
            if (node.childIdx1 != 0xFFFFFFFFu) {
                numWorkQueuedLocal++;
                if (node.childIdx0 == 0xFFFFFFFFu) {
                    childIdx1 = node.childIdx1;
                } else {
                    childIdx0 = node.childIdx1;
                }
            }

            if (numWorkQueuedLocal == 0u) {
                // This is a leaf node. Set the visibility to true.
                meshletVisibilityArray[nodeIdx] = 1u;
            } else {
                // This is an inner node. Enqueue all children.
                while (true) {
                    if (atomicCompSwap(queueLock, 0, 1) == 0) {
                        uint queueIdx = atomicAdd(queueWriteIdx, numWorkQueuedLocal);
                        if (childIdx0 != 0xFFFFFFFFu) {
                            nodeIndicesQueue[queueIdx] = childIdx0;
                        }
                        if (childIdx1 != 0xFFFFFFFFu) {
                            nodeIndicesQueue[queueIdx + 1] = childIdx1;
                        }
                        atomicAdd(numWorkPending, numWorkQueuedLocal);

                        atomicExchange(queueLock, 0);
                        break;
                    }
                }
            }
        }

        atomicAdd(numWorkPending, -1);
    }
}
