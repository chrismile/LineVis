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

#include <random>
#include <glm/glm.hpp>
#include "gtest/gtest.h"
#include "LineData/SearchStructures/NearestNeighborNaive.hpp"
#include "LineData/SearchStructures/KdTree.hpp"

class KdTreeNearestNeighborTest : public ::testing::TestWithParam<int> {
protected:
    void SetUp() override {
        N = GetParam();
        pointSetSearch.resize(N);
        pointSetFind.resize(N);
        distancesSlow.resize(N);
        distancesKdTree.resize(N);

        // Compute search point data.
        std::default_random_engine generator(12345);
        std::uniform_real_distribution<float> distribution(0.0f, 1.0f);
        for (int i = 0; i < N; i++) {
            pointSetSearch.at(i) = glm::vec3(
                    distribution(generator), distribution(generator), distribution(generator));
        }

        // Compute points to find nearest neighbors for.
        for (int i = 0; i < N; i++) {
            pointSetFind.at(i) = glm::vec3(
                    distribution(generator), distribution(generator), distribution(generator));
        }

        // Build the search structure on the points.
        KdTree<ptrdiff_t> kdTree;
        std::vector<ptrdiff_t> pointIndices;
        pointIndices.reserve(pointSetSearch.size());
        for (size_t i = 0; i < pointSetSearch.size(); i++) {
            pointIndices.push_back(ptrdiff_t(i));
        }
        kdTree.build(pointSetSearch);

        // Get the k-d-tree distances.
        for (int i = 0; i < N; i++) {
            auto nearestNeighbor = kdTree.findNearestNeighbor(pointSetFind.at(i));
            distancesKdTree.at(i) = glm::distance(pointSetFind.at(i), nearestNeighbor.value().first);
        }

        // Get the ground-truth distances.
        for (int i = 0; i < N; i++) {
            glm::vec3 nearestNeighbor = nearestNeighborNaive(pointSetFind.at(i), pointSetSearch);
            distancesSlow.at(i) = glm::distance(pointSetFind.at(i), nearestNeighbor);
        }
    }

    void TearDown() override {
    }

    int N = 0;
    KdTree<ptrdiff_t> kdTree;
    std::vector<glm::vec3> pointSetSearch;
    std::vector<glm::vec3> pointSetFind;
    std::vector<float> distancesSlow;
    std::vector<float> distancesKdTree;
};

TEST_P(KdTreeNearestNeighborTest, DistanceCorrect){
    for (int i = 0; i < N; i++) {
        EXPECT_FLOAT_EQ(distancesSlow.at(i), distancesKdTree.at(i));
    }
}

INSTANTIATE_TEST_SUITE_P(DistanceRangeTest, KdTreeNearestNeighborTest, ::testing::Values(
        2, 3, 4, 5, 6, 7, 8, 10, 128, 1024));
