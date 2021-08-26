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

#ifndef LINEVIS_VOXELRAYCASTINGRENDERER_HPP
#define LINEVIS_VOXELRAYCASTINGRENDERER_HPP

#include "Renderers/LineRenderer.hpp"

/**
 * A voxel ray caster (VRC) for line rendering based on the work by:
 *
 * M. Kanzler, M. Rautenhaus, R. Westermann. A Voxel-based Rendering Pipeline for Large 3D Line Sets.
 * IEEE Transactions on Visualization and Computer Graphics 2018.
 * https://www.in.tum.de/cg/research/publications/2018/a-voxel-based-rendering-pipeline-for-large-3d-line-sets/
 */
class VoxelRayCastingRenderer : public LineRenderer {
public:
    VoxelRayCastingRenderer(
            SceneData& sceneData, sgl::TransferFunctionWindow& transferFunctionWindow);
};

#endif //LINEVIS_VOXELRAYCASTINGRENDERER_HPP
