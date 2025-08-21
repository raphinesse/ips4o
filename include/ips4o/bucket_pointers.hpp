/******************************************************************************
 * include/ips4o/bucket_pointers.hpp
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
#include <climits>
#include <cstdint>
#include <mutex>
#include <new>
#include <tuple>
#include <utility>

#include "ips4o_fwd.hpp"

namespace ips4o {
namespace detail {

template <class Cfg>
class Sorter<Cfg>::BucketPointers {
    using diff_t = typename Cfg::difference_type;

 public:
    /**
     * Sets write/read pointers.
     */
    void set(diff_t w, diff_t r) {
        read_ = r;
        write_ = w;
    }

    /**
     * Gets the write pointer.
     */
    diff_t getWrite() const {
        return write_;
    }

    /**
     * Gets write/read pointers and increases the write pointer.
     */
    std::pair<diff_t, diff_t> incWrite() {
        const auto tmp = write_;
        write_ += Cfg::kBlockSize;
        return {tmp, read_};
    }

    /**
     * Gets write/read pointers, decreases the read pointer, and increases the read
     * counter.
     */
    std::pair<diff_t, diff_t> decRead() {
        const auto tmp = read_;
        read_ -= Cfg::kBlockSize;
        return {write_, tmp};
    }

 private:
    diff_t write_, read_;
};

}  // namespace detail
}  // namespace ips4o
