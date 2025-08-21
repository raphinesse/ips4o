/******************************************************************************
 * include/ips4o/partitioning.hpp
 *
 * In-place Parallel Super Scalar Samplesort (IPS⁴o)
 *
 ******************************************************************************
 * BSD 2-Clause License
 *
 * Copyright © 2017, Michael Axtmann <michael.axtmann@gmail.com>
 * Copyright © 2017, Daniel Ferizovic <daniel.ferizovic@student.kit.edu>
 * Copyright © 2017, Sascha Witt <sascha.witt@kit.edu>
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
 *****************************************************************************/

#pragma once

#include <atomic>
#include <tuple>
#include <utility>

#include "ips4o_fwd.hpp"
#include "block_permutation.hpp"
#include "cleanup_margins.hpp"
#include "local_classification.hpp"
#include "memory.hpp"
#include "sampling.hpp"
#include "utils.hpp"

namespace ips4o {
namespace detail {

/**
 * Main partitioning function.
 */
template <class Cfg>
std::pair<int, bool> Sorter<Cfg>::partition(const iterator begin, const iterator end,
                                            diff_t* const bucket_start) {
#ifdef IPS4O_TIMER
    g_overhead.stop();
    g_sampling.start();
#endif

    // Sampling
    bool use_equal_buckets = false;
    std::tie(this->num_buckets_, use_equal_buckets) =
            buildClassifier(begin, end, local_.classifier);

    // Set parameters for this partitioning step
    // Must do this AFTER sampling, because sampling will recurse to sort splitters.
    this->classifier_ = &local_.classifier;
    this->bucket_pointers_ = local_.bucket_pointers;
    this->bucket_start_ = bucket_start;
    this->overflow_ = nullptr;
    this->begin_ = begin;
    this->end_ = end;

#ifdef IPS4O_TIMER
    g_sampling.stop();
    g_classification.start();
#endif

    // Local Classification
    sequentialClassification(use_equal_buckets);

#ifdef IPS4O_TIMER
    g_classification.stop(end - begin, "class");
    g_permutation.start();
#endif

    // Compute which bucket can cause overflow
    const int overflow_bucket = computeOverflowBucket();

    // Block Permutation
    if (use_equal_buckets)
        permuteBlocks<true>();
    else
        permuteBlocks<false>();

#ifdef IPS4O_TIMER
    g_permutation.stop(end - begin, "perm");
    g_cleanup.start();
#endif

    // Cleanup
    {
        // Save excess elements at right end of stripe
        auto in_swap_buffer =  std::pair<int, diff_t>(-1, 0);

        // Write remaining elements
        writeMargins(0, num_buckets_, overflow_bucket,
                     in_swap_buffer.first, in_swap_buffer.second);
    }

    local_.reset();

#ifdef IPS4O_TIMER
    g_cleanup.stop();
    g_overhead.start();
#endif

    return {this->num_buckets_, use_equal_buckets};
}

}  // namespace detail
}  // namespace ips4o
