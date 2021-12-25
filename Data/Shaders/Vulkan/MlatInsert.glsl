/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2021, Christoph Neuhauser
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

/*
 * Any hit shader for multi-layer alpha tracing (MLAT). For more details on MLAT, please refer to:
 * F. Brüll and T. Grosch. Multi-Layer Alpha Tracing. In J. Krüger, M. Niessner, and J. Stückler, editors, Vision,
 * Modeling, and Visualization. The Eurographics Association, 2020.
 */

MlatNode merge(MlatNode a, MlatNode b, inout float depth2, bool isFirst) {
    MlatNode r;
    r.transmittance = a.transmittance * b.transmittance;

    float fa = 1.0;
    float fb = a.transmittance;

    r.depth = a.depth;
    depth2 = max(depth2, b.depth);

    // Is node b in node a?
    if (b.depth < depth2 && !isFirst) {
        float d = (b.depth - a.depth);
        d /= (depth2 - a.depth);
        float a_pow_d = pow (a.transmittance, d);
        fa = (a_pow_d - 1.0);
        fa += (a.transmittance - a_pow_d) * b.transmittance;
        fa /= (a.transmittance - 1.0);
        fb = a_pow_d;
    }

    r.color = fa * a.color + fb * b.color;
    return r;
}

void swap(inout MlatNode a, inout MlatNode b) {
    MlatNode temp = a;
    a = b;
    b = temp;
}

void insertNodeMlat(vec4 color) {
#ifdef MISS_SHADER
    float depth = 1e7;
#else
    float depth = gl_HitTEXT;
#endif

    MlatNode newNode;
    newNode.depth = depth;
    float alpha = color.a;
#ifndef MISS_SHADER
    if (alpha == 0.0) {
        ignoreIntersectionEXT;
    }
#endif
    newNode.transmittance = 1.0 - alpha;
    newNode.color = vec4(alpha * color.rgb, color.a);

#ifdef NODES_UNROLLED
#if NUM_NODES >= 4
    if (newNode.depth > payload.node3.depth) {
        swap(newNode, payload.node3);
    }
#endif
#if NUM_NODES >= 3
    if (newNode.depth > payload.node2.depth) {
        swap(newNode, payload.node2);
    }
#endif
#if NUM_NODES >= 2
    if (newNode.depth > payload.node1.depth) {
        swap(newNode, payload.node1);
    }
#endif
    if (newNode.depth > payload.node0.depth) {
        swap(newNode, payload.node0);
    }
#else
    for (int i = NUM_NODES - 1; i >= 0; --i) {
        if (newNode.depth > payload.nodes[i].depth) {
            swap(newNode, payload.nodes[i]);
        }
    }
#endif

    // Merge the first two nodes. Here, MLAT differs from MLAB, where the last two nodes are merged.
#ifdef NODES_UNROLLED
    if (newNode.depth > 0.0) {
        payload.node0 = merge(newNode, payload.node0, payload.depth2, newNode.depth == depth);
    }
#else
    if (newNode.depth > 0.0) {
        payload.nodes[0] = merge(newNode, payload.nodes[0], payload.depth2, newNode.depth == depth);
    }
#endif

    // Can we do early ray termination because this is an opaque color?
    if (alpha == 1.0) {
        return;
    }

    // Has enough absorption accumulated to do early ray termination?
    float transmittance = 1.0;
#ifdef NODES_UNROLLED
    transmittance *= payload.node0.transmittance;
#if NUM_NODES >= 2
    transmittance *= payload.node1.transmittance;
#endif
#if NUM_NODES >= 3
    transmittance *= payload.node2.transmittance;
#endif
#if NUM_NODES >= 4
    transmittance *= payload.node3.transmittance;
#endif
#else
    for (int i = 0; i < NUM_NODES; ++i) {
        transmittance *= payload.nodes[i].transmittance;
    }
#endif

#ifdef NODES_UNROLLED
#if NUM_NODES == 1
    if (transmittance <= 0.001 && payload.node0.depth <= depth) {
        return;
    }
#elif NUM_NODES == 2
    if (transmittance <= 0.001 && payload.node1.depth <= depth) {
        return;
    }
#elif NUM_NODES == 3
    if (transmittance <= 0.001 && payload.node2.depth <= depth) {
        return;
    }
#elif NUM_NODES == 4
    if (transmittance <= 0.001 && payload.node3.depth <= depth) {
        return;
    }
#endif
#else
    if (transmittance <= 0.001 && payload.nodes[NUM_NODES - 1].depth <= depth) {
        return;
    }
#endif

#ifndef MISS_SHADER
    ignoreIntersectionEXT;
#endif
}
