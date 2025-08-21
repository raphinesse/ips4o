/******************************************************************************
 * include/ips4o.hpp
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

/******************************************************************************
 * include/ips4o/ips4o.hpp
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

#include <functional>
#include <iterator>
#include <type_traits>

/******************************************************************************
 * include/ips4o/ips4o_fwd.hpp
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

#include <iterator>
#include <memory>
#include <utility>
#include <vector>

/******************************************************************************
 * include/ips4o/config.hpp
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

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <type_traits>
#include <utility>

/******************************************************************************
 * include/ips4o/utils.hpp
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

#include <cassert>

#ifdef NDEBUG
#define IPS4OML_ASSUME_NOT(c) if (c) __builtin_unreachable()
#else
#define IPS4OML_ASSUME_NOT(c) assert(!(c))
#endif

#define IPS4OML_IS_NOT(c) assert(!(c))

#include <limits>

namespace ips4o {
namespace detail {

/**
 * Compute the logarithm to base 2, rounded down.
 */
inline constexpr unsigned long log2(unsigned long n) {
    return (std::numeric_limits<unsigned long>::digits - 1 - __builtin_clzl(n));
}

}  // namespace detail
}  // namespace ips4o

#ifndef IPS4OML_ALLOW_EQUAL_BUCKETS
#define IPS4OML_ALLOW_EQUAL_BUCKETS true
#endif

#ifndef IPS4OML_BASE_CASE_SIZE
#define IPS4OML_BASE_CASE_SIZE 16
#endif

#ifndef IPS4OML_BASE_CASE_MULTIPLIER
#define IPS4OML_BASE_CASE_MULTIPLIER 16
#endif

#ifndef IPS4OML_BLOCK_SIZE
#define IPS4OML_BLOCK_SIZE (2 << 10)
#endif

#ifndef IPS4OML_BUCKET_TYPE
#define IPS4OML_BUCKET_TYPE std::ptrdiff_t
#endif

#ifndef IPS4OML_DATA_ALIGNMENT
#define IPS4OML_DATA_ALIGNMENT (4 << 10)
#endif

#ifndef IPS4OML_EQUAL_BUCKETS_THRESHOLD
#define IPS4OML_EQUAL_BUCKETS_THRESHOLD 5
#endif

#ifndef IPS4OML_LOG_BUCKETS
#define IPS4OML_LOG_BUCKETS 8
#endif

#ifndef IPS4OML_MIN_PARALLEL_BLOCKS_PER_THREAD
#define IPS4OML_MIN_PARALLEL_BLOCKS_PER_THREAD 4
#endif

#ifndef IPS4OML_OVERSAMPLING_FACTOR_PERCENT
#define IPS4OML_OVERSAMPLING_FACTOR_PERCENT 20
#endif

#ifndef IPS4OML_UNROLL_CLASSIFIER
#define IPS4OML_UNROLL_CLASSIFIER 7
#endif

namespace ips4o {

template <bool AllowEqualBuckets_     = IPS4OML_ALLOW_EQUAL_BUCKETS
        , std::ptrdiff_t BaseCase_    = IPS4OML_BASE_CASE_SIZE
        , std::ptrdiff_t BaseCaseM_   = IPS4OML_BASE_CASE_MULTIPLIER
        , std::ptrdiff_t BlockSize_   = IPS4OML_BLOCK_SIZE
        , class BucketT_              = IPS4OML_BUCKET_TYPE
        , std::size_t DataAlign_      = IPS4OML_DATA_ALIGNMENT
        , std::ptrdiff_t EqualBuckTh_ = IPS4OML_EQUAL_BUCKETS_THRESHOLD
        , int LogBuckets_             = IPS4OML_LOG_BUCKETS
        , std::ptrdiff_t MinParBlks_  = IPS4OML_MIN_PARALLEL_BLOCKS_PER_THREAD
        , int OversampleF_            = IPS4OML_OVERSAMPLING_FACTOR_PERCENT
        , int UnrollClass_            = IPS4OML_UNROLL_CLASSIFIER
        >
struct Config {
    /**
     * The type used for bucket indices in the classifier.
     */
    using bucket_type = BucketT_;

    /**
     * Whether we are on 64 bit or 32 bit.
     */
    static constexpr const bool kIs64Bit = sizeof(std::uintptr_t) == 8;
    static_assert(kIs64Bit || sizeof(std::uintptr_t) == 4,
                  "Architecture must be 32 or 64 bit");

    /**
     * Whether equal buckets can be used.
     */
    static constexpr const bool kAllowEqualBuckets = AllowEqualBuckets_;
    /**
     * Desired base case size.
     */
    static constexpr const std::ptrdiff_t kBaseCaseSize = BaseCase_;
    /**
     * Multiplier for base case threshold.
     */
    static constexpr const int kBaseCaseMultiplier = BaseCaseM_;
    /**
     * Number of bytes in one block.
     */
    static constexpr const std::ptrdiff_t kBlockSizeInBytes = BlockSize_;
    /**
     * Alignment for shared and thread-local data.
     */
    static constexpr const std::size_t kDataAlignment = DataAlign_;
    /**
     * Number of splitters that must be equal before equality buckets are enabled.
     */
    static constexpr const std::ptrdiff_t kEqualBucketsThreshold = EqualBuckTh_;
    /**
     * Logarithm of the maximum number of buckets (excluding equality buckets).
     */
    static constexpr const int kLogBuckets = LogBuckets_;
    /**
     * Minimum number of blocks per thread for which parallelism is used.
     */
    static constexpr const std::ptrdiff_t kMinParallelBlocksPerThread = MinParBlks_;
    static_assert(kMinParallelBlocksPerThread > 0,
                  "Min. blocks per thread must be at least 1.");
    /**
     * How many times the classification loop is unrolled.
     */
    static constexpr const int kUnrollClassifier = UnrollClass_;

    static constexpr const std::ptrdiff_t kSingleLevelThreshold =
            kBaseCaseSize * (1ul << kLogBuckets);
    static constexpr const std::ptrdiff_t kTwoLevelThreshold =
            kSingleLevelThreshold * (1ul << kLogBuckets);

    /**
     * The oversampling factor to be used for input of size n.
     */
    static constexpr double oversamplingFactor(std::ptrdiff_t n) {
        const double f = OversampleF_ / 100.0 * detail::log2(n);
        return f < 1.0 ? 1.0 : f;
    }

    /**
    * Computes the logarithm of the number of buckets to use for input size n.
    */
    static int logBuckets(const std::ptrdiff_t n) {
        if (n <= kSingleLevelThreshold) {
            // Only one more level until  the base case, reduce the number of buckets
            return std::max(1ul, detail::log2(n / kBaseCaseSize));
        } else if (n <= kTwoLevelThreshold) {
            // Only two more levels until we reach the base case, split the buckets evenly
            return std::max(1ul, (detail::log2(n / kBaseCaseSize) + 1) / 2);
        } else {
            // Use the maximum number of buckets
            return kLogBuckets;
        }
    }

    /**
     * Returns the number of threads that should be used for the given input range.
     */
    template <class It>
    static constexpr int numThreadsFor(const It&, const It&, int) {
        return 1;
    }
};

template <class It_, class Comp_, class Cfg = Config<>
        >
struct ExtendedConfig : public Cfg {
    /**
     * Base config containing user-specified parameters.
     */
    using BaseConfig = Cfg;
    /**
     * The iterator type for the input data.
     */
    using iterator = It_;
    /**
     * The difference type for the iterator.
     */
    using difference_type = typename std::iterator_traits<iterator>::difference_type;
    /**
     * The value type of the input data.
     */
    using value_type = typename std::iterator_traits<iterator>::value_type;
    /**
     * The comparison operator.
     */
    using less = Comp_;

    struct Sync {
        constexpr void barrier() const {}
        template <class F>
        constexpr void single(F&&) const {}
    };

    /**
     * Dummy thread pool.
     */
    class SubThreadPool {
     public:
        explicit SubThreadPool(int) {}

        void join(int) {}

        void release_threads() {}

        template <class F>
        void operator()(F&&, int) {}

        Sync& sync() { return sync_; }

        int numThreads() const { return 1; }

     private:
        Sync sync_;
    };

    /**
     * Maximum number of buckets (including equality buckets).
     */
    static constexpr const int kMaxBuckets =
            1ul << (Cfg::kLogBuckets + Cfg::kAllowEqualBuckets);

    /**
     * Number of elements in one block.
     */
    static constexpr const difference_type kBlockSize =
            1ul << (detail::log2(
                    Cfg::kBlockSizeInBytes < sizeof(value_type)
                            ? 1
                            : (Cfg::kBlockSizeInBytes / sizeof(value_type))));

    // Redefine applicable constants as difference_type.
    static constexpr const difference_type kBaseCaseSize = Cfg::kBaseCaseSize;
    static constexpr const difference_type kEqualBucketsThreshold =
            Cfg::kEqualBucketsThreshold;

    // Cannot sort without random access.
    static_assert(std::is_same<typename std::iterator_traits<iterator>::iterator_category,
                               std::random_access_iterator_tag>::value,
                  "Iterator must be a random access iterator.");
    // Number of buckets is limited by switch in classifier
    static_assert(Cfg::kLogBuckets >=1, "Max. bucket count must be <= 512.");
    // The implementation of the block alignment limits the possible block sizes.
    static_assert((kBlockSize & (kBlockSize - 1)) == 0,
                  "Block size must be a power of two.");
    // The main classifier function assumes that the loop can be unrolled at least once.
    static_assert(Cfg::kUnrollClassifier <= kBaseCaseSize,
                  "Base case size must be larger than unroll factor.");

    /**
     * Aligns an offset to the next block boundary, upwards.
     */
    static constexpr difference_type alignToNextBlock(difference_type p) {
        return (p + kBlockSize - 1) & ~(kBlockSize - 1);
    }
};

#undef IPS4OML_ALLOW_EQUAL_BUCKETS
#undef IPS4OML_BASE_CASE_SIZE
#undef IPS4OML_BASE_CASE_MULTIPLIER
#undef IPS4OML_BLOCK_SIZE
#undef IPS4OML_BUCKET_TYPE
#undef IPS4OML_DATA_ALIGNMENT
#undef IPS4OML_EQUAL_BUCKETS_THRESHOLD
#undef IPS4OML_LOG_BUCKETS
#undef IPS4OML_MIN_PARALLEL_BLOCKS_PER_THREAD
#undef IPS4OML_OVERSAMPLING_FACTOR_PERCENT
#undef IPS4OML_UNROLL_CLASSIFIER

}  // namespace ips4o

/******************************************************************************
 * include/ips4o/scheduler.hpp
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

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <numeric>
#include <vector>

namespace ips4o {
namespace detail {

template <class T>
class PrivateQueue {
 public:
    PrivateQueue(size_t init_size = ((1ul << 12) + sizeof(T) - 1) / sizeof(T))
        : m_v(), m_off(0)
    {
        // Preallocate memory. By default, the vector covers at least one page.
        m_v.reserve(init_size);
    }

    template <class Iterator>
    void push(Iterator begin, Iterator end) {
        m_v.insert(m_v.end(), begin, end);
    }

    template <class T1>
    void push(const T1&& e) {
        m_v.emplace_back(std::forward(e));
    }

    template <typename... Args>
    void emplace(Args... args) {
        m_v.emplace_back(args...);
    }

    size_t size() const { return m_v.size() - m_off; }

    bool empty() const { return size() == 0; }

    T popBack() {
        assert(m_v.size() > m_off);

        const T e = m_v.back();
        m_v.pop_back();

        if (m_v.size() == m_off) {
            clear();
        }

        return e;
    }

    T popFront() {
        assert(m_v.size() > m_off);

        const T e = m_v[m_off];
        ++m_off;

        if (m_v.size() == m_off) {
            clear();
        }

        return e;
    }

    void clear() {
        m_off = 0;
        m_v.clear();
    }

 protected:
    std::vector<T> m_v;
    size_t m_off;
};

}  // namespace detail
}  // namespace ips4o

/******************************************************************************
 * include/ips4o/task.hpp
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

#include <cstddef>

namespace ips4o {
namespace detail {

/**
 * A subtask in the parallel algorithm.
 * Uses indices instead of iterators to avoid unnecessary template instantiations.
 */
struct Task {
    Task() {}
    Task(std::ptrdiff_t begin, std::ptrdiff_t end) : begin(begin), end(end) {}

    std::ptrdiff_t begin;
    std::ptrdiff_t end;
};

}  // namespace detail
}  // namespace ips4o

namespace ips4o {

template <class Cfg>
class SequentialSorter;

namespace detail {

template <class It, class Comp>
inline void baseCaseSort(It begin, It end, Comp&& comp);

inline constexpr unsigned long log2(unsigned long n);

template <class It, class RandomGen>
inline void selectSample(It begin, It end,
                         typename std::iterator_traits<It>::difference_type num_samples,
                         RandomGen&& gen);

template <class Cfg> class Classifier;

template <class Cfg>
class Sorter {
 public:
    using iterator = typename Cfg::iterator;
    using diff_t = typename Cfg::difference_type;
    using value_type = typename Cfg::value_type;
    using SubThreadPool = typename Cfg::SubThreadPool;

    class BufferStorage;
    class Block;
    class Buffers;
    class BucketPointers;
    struct LocalData;
    struct SharedData;
    explicit Sorter(LocalData& local) : local_(local) {}

    void sequential(iterator begin, iterator end);

    void sequential(const iterator begin, const Task& task, PrivateQueue<Task>& queue);

    void sequential_rec(iterator begin, iterator end);

 private:
    using Classifier = ::ips4o::detail::Classifier<Cfg>;

    LocalData& local_;
    SharedData* shared_;
    Classifier* classifier_;

    diff_t* bucket_start_;
    BucketPointers* bucket_pointers_;
    Block* overflow_;

    iterator begin_;
    iterator end_;
    int num_buckets_;
    int my_id_;
    int num_threads_;

    static inline int computeLogBuckets(diff_t n);

    std::pair<int, bool> buildClassifier(iterator begin, iterator end,
                                         Classifier& classifier);

    template <bool kEqualBuckets>
    __attribute__((flatten)) diff_t classifyLocally(iterator my_begin, iterator my_end);

    inline void parallelClassification(bool use_equal_buckets);

    inline void sequentialClassification(bool use_equal_buckets);

    void moveEmptyBlocks(diff_t my_begin, diff_t my_end, diff_t my_first_empty_block);

    inline int computeOverflowBucket();

    template <bool kEqualBuckets, bool kIsParallel>
    inline int classifyAndReadBlock(int read_bucket);

    template <bool kEqualBuckets, bool kIsParallel>
    inline int swapBlock(diff_t max_off, int dest_bucket, bool current_swap);

    template <bool kEqualBuckets, bool kIsParallel>
    void permuteBlocks();

    template <bool kIsParallel>
    void writeMargins(int first_bucket, int last_bucket, int overflow_bucket,
                      int swap_bucket, diff_t in_swap_buffer);

    template <bool kIsParallel>
    std::pair<int, bool> partition(iterator begin, iterator end, diff_t* bucket_start,
                                   int my_id, int num_threads);

    void processSmallTasks(iterator begin);

    void processBigTasks(const iterator begin, const diff_t stripe, const int my_id,
                         BufferStorage& buffer_storage,
                         std::vector<std::shared_ptr<SubThreadPool>>& tp_trash);

    void processBigTaskPrimary(const iterator begin, const diff_t stripe, const int my_id,
                               BufferStorage& buffer_storage,
                               std::vector<std::shared_ptr<SubThreadPool>>& tp_trash);
    void processBigTasksSecondary(const int my_id);

    void queueTasks(const diff_t stripe, const int id, const int num_threads,
                    const diff_t parent_task_size, const diff_t offset,
                    const diff_t* bucket_start, int num_buckets, bool equal_buckets);
};

}  // namespace detail

template <class Cfg>
class ParallelSorter;

template <class It, class Comp>
inline void sort(It begin, It end, Comp comp);

template <class It>
inline void sort(It begin, It end);

}  // namespace ips4o

/******************************************************************************
 * include/ips4o/base_case.hpp
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

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

namespace ips4o {
namespace detail {

/**
 * Insertion sort.
 */
template <class It, class Comp>
void insertionSort(const It begin, const It end, Comp comp) {
    IPS4OML_ASSUME_NOT(begin >= end);

    for (It it = begin + 1; it < end; ++it) {
        typename std::iterator_traits<It>::value_type val = std::move(*it);
        if (comp(val, *begin)) {
            std::move_backward(begin, it, it + 1);
            *begin = std::move(val);
        } else {
            auto cur = it;
            for (auto next = it - 1; comp(val, *next); --next) {
                *cur = std::move(*next);
                cur = next;
            }
            *cur = std::move(val);
        }
    }
}

/**
 * Wrapper for base case sorter, for easier swapping.
 */
template <class It, class Comp>
inline void baseCaseSort(It begin, It end, Comp&& comp) {
    if (begin == end) return;
    detail::insertionSort(std::move(begin), std::move(end), std::forward<Comp>(comp));
}

template <class It, class Comp, class ThreadPool>
inline bool isSorted(It begin, It end, Comp&& comp, ThreadPool& thread_pool) {
    // Do nothing if input is already sorted.
    std::vector<bool> is_sorted(thread_pool.numThreads());
    thread_pool(
            [begin, end, &is_sorted, &comp](int my_id, int num_threads) {
                const auto size = end - begin;
                const auto stripe = (size + num_threads - 1) / num_threads;
                const auto my_begin = begin + std::min(stripe * my_id, size);
                const auto my_end = begin + std::min(stripe * (my_id + 1) + 1, size);
                is_sorted[my_id] = std::is_sorted(my_begin, my_end, comp);
            },
            thread_pool.numThreads());

    return std::all_of(is_sorted.begin(), is_sorted.end(), [](bool res) { return res; });
}

template <class It, class Comp>
inline bool sortSimpleCases(It begin, It end, Comp&& comp) {
    if (begin == end) {
        return true;
    }

    // If last element is not smaller than first element,
    // test if input is sorted (input is not reverse sorted).
    if (!comp(*(end - 1), *begin)) {
        if (std::is_sorted(begin, end, comp)) {
            return true;
        }
    } else {
        // Check whether the input is reverse sorted.
        for (It it = begin; (it + 1) != end; ++it) {
            if (comp(*it, *(it + 1))) {
                return false;
            }
        }
        std::reverse(begin, end);
        return true;
    }

    return false;
}

}  // namespace detail
}  // namespace ips4o

/******************************************************************************
 * include/ips4o/memory.hpp
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

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <random>
#include <utility>
#include <vector>

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

#include <atomic>
#include <climits>
#include <cstdint>
#include <mutex>
#include <new>
#include <tuple>
#include <utility>

namespace ips4o {
namespace detail {

template <class Cfg>
class Sorter<Cfg>::BucketPointers {
    using diff_t = typename Cfg::difference_type;

#if UINTPTR_MAX == UINT32_MAX || defined(__SIZEOF_INT128__)

#if UINTPTR_MAX == UINT32_MAX

    using atomic_type = std::uint64_t;

#elif defined(__SIZEOF_INT128__)

    using atomic_type = __uint128_t;

#endif  // defined( __SIZEOF_INT128__)

    // alignas(std::hardware_destructive_interference_size)
    class Uint128 {
     public:
        inline void set(diff_t l, diff_t m) {
            single_.m_ = m;
            single_.l_ = l;
        }

        inline diff_t getLeastSignificant() const { return single_.l_; }

        template <bool kAtomic>
        inline std::pair<diff_t, diff_t> fetchSubMostSignificant(diff_t m) {
            if (kAtomic) {
                const atomic_type atom_m = static_cast<atomic_type>(m) << kShift;
                const auto p = __atomic_fetch_sub(&all_, atom_m, __ATOMIC_RELAXED);
                return {p & kMask, p >> kShift};
            } else {
                const auto tmp = single_.m_;
                single_.m_ -= m;
                return {single_.l_, tmp};
            }
        }

        template <bool kAtomic>
        inline std::pair<diff_t, diff_t> fetchAddLeastSignificant(diff_t l) {
            if (kAtomic) {
                const auto p = __atomic_fetch_add(&all_, l, __ATOMIC_RELAXED);
                return {p & kMask, p >> kShift};
            } else {
                const auto tmp = single_.l_;
                single_.l_ += l;
                return {tmp, single_.m_};
            }
        }

     private:
        static constexpr const int kShift = sizeof(atomic_type) * CHAR_BIT / 2;
        static constexpr const atomic_type kMask =
                (static_cast<atomic_type>(1) << kShift) - 1;

        struct Pointers {
            diff_t l_, m_;
        };
        union {
            atomic_type all_;
            Pointers single_;
        };
    };

#else

    class Uint128 {
     public:
        inline void set(diff_t l, diff_t m) {
            m_ = m;
            l_ = l;
        }

        inline diff_t getLeastSignificant() const { return l_; }

        template <bool kAtomic>
        inline std::pair<diff_t, diff_t> fetchSubMostSignificant(diff_t m) {
            if (kAtomic) {
                std::lock_guard<std::mutex> lock(mtx_);
                std::pair<diff_t, diff_t> p{l_, m_};
                m_ -= m;
                return p;
            } else {
                const auto tmp = m_;
                m_ -= m;
                return {l_, tmp};
            }
        }

        template <bool kAtomic>
        inline std::pair<diff_t, diff_t> fetchAddLeastSignificant(diff_t l) {
            if (kAtomic) {
                std::lock_guard<std::mutex> lock(mtx_);
                std::pair<diff_t, diff_t> p{l_, m_};
                l_ += l;
                return p;
            } else {
                const auto tmp = l_;
                l_ += l;
                return {tmp, m_};
            }
        }

     private:
        diff_t m_, l_;
        std::mutex mtx_;
    };

#endif

 public:
    /**
     * Sets write/read pointers.
     */
    void set(diff_t w, diff_t r) {
        ptr_.set(w, r);
        num_reading_.store(0, std::memory_order_relaxed);
    }

    /**
     * Gets the write pointer.
     */
    diff_t getWrite() const {
        return ptr_.getLeastSignificant();
    }

    /**
     * Gets write/read pointers and increases the write pointer.
     */
    template <bool kAtomic>
    std::pair<diff_t, diff_t> incWrite() {
        return ptr_.template fetchAddLeastSignificant<kAtomic>(Cfg::kBlockSize);
    }

    /**
     * Gets write/read pointers, decreases the read pointer, and increases the read
     * counter.
     */
    template <bool kAtomic>
    std::pair<diff_t, diff_t> decRead() {
        if (kAtomic) {
            // Must not be moved after the following fetch_sub, as that could lead to
            // another thread writing to our block, because isReading() returns false.
            num_reading_.fetch_add(1, std::memory_order_acquire);
            const auto p =
                    ptr_.template fetchSubMostSignificant<kAtomic>(Cfg::kBlockSize);
            return {p.first, p.second & ~(Cfg::kBlockSize - 1)};
        } else {
            return ptr_.template fetchSubMostSignificant<kAtomic>(Cfg::kBlockSize);
        }
    }

    /**
     * Decreases the read counter.
     */
    void stopRead() {
        // Synchronizes with threads wanting to write to this bucket
        num_reading_.fetch_sub(1, std::memory_order_release);
    }

    /**
     * Returns true if any thread is currently reading from here.
     */
    bool isReading() {
        // Synchronizes with threads currently reading from this bucket
        return num_reading_.load(std::memory_order_acquire) != 0;
    }

 private:
    Uint128 ptr_;
    std::atomic_int num_reading_;
};

}  // namespace detail
}  // namespace ips4o

/******************************************************************************
 * include/ips4o/buffers.hpp
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

#include <algorithm>
#include <type_traits>
#include <utility>

namespace ips4o {
namespace detail {

/**
 * A single buffer block.
 */
template <class Cfg>
class Sorter<Cfg>::Block {
    using iterator = typename Cfg::iterator;
    using diff_t = typename Cfg::difference_type;
    using value_type = typename Cfg::value_type;

 public:
    static constexpr const bool kInitializedStorage =
            std::is_trivially_default_constructible<value_type>::value;
    static constexpr const bool kDestruct =
            !kInitializedStorage && !std::is_trivially_destructible<value_type>::value;

    /**
     * Pointer to data.
     */
    value_type* data() {
        return static_cast<value_type*>(static_cast<void*>(storage_));
    }

    /**
     * First element.
     */
    const value_type& head() { return *data(); }

    /**
     * Reads a full block from input.
     */
    void readFrom(iterator src) {
        if (kInitializedStorage) {
            std::move(src, src + Cfg::kBlockSize, data());
        } else {
            for (auto p = data(), end = p + Cfg::kBlockSize; p < end; ++p) {
                IPS4OML_ASSUME_NOT(p == nullptr);
                new (p) value_type(std::move(*src++));
            }
        }
    }

    /**
     * Reads a partial block from input.
     */
    void readFrom(iterator src, const diff_t n) {
        if (kInitializedStorage) {
            std::move(src, src + n, data());
        } else {
            for (auto p = data(), end = p + n; p < end; ++p) {
                IPS4OML_ASSUME_NOT(p == nullptr);
                new (p) value_type(std::move(*src++));
            }
        }
    }

    /**
     * Resets a partial block.
     */
    void reset(const diff_t n) {
        if (kDestruct)
            for (auto p = data(), end = p + n; p < end; ++p)
                p->~value_type();
    }

    /**
     * Writes a full block to other block.
     */
    void writeTo(Block& block) {
        if (kInitializedStorage) {
            std::move(data(), data() + Cfg::kBlockSize, block.data());
        } else {
            for (auto src = data(), dst = block.data(), end = src + Cfg::kBlockSize;
                 src < end; ++src, ++dst) {
                IPS4OML_ASSUME_NOT(dst == nullptr);
                new (dst) value_type(std::move(*src));
            }
        }
        if (kDestruct)
            for (auto p = data(), end = p + Cfg::kBlockSize; p < end; ++p)
                p->~value_type();
    }

    /**
     * Writes a full block to input.
     */
    void writeTo(iterator dest) {
        std::move(data(), data() + Cfg::kBlockSize, std::move(dest));
        if (kDestruct)
            for (auto p = data(), end = p + Cfg::kBlockSize; p < end; ++p)
                p->~value_type();
    }

 private:
    using storage_type = std::conditional_t<
            kInitializedStorage, value_type,
            std::aligned_storage_t<sizeof(value_type), alignof(value_type)>>;
    storage_type storage_[Cfg::kBlockSize];
};

/**
 * Per-thread buffers for each bucket.
 */
template <class Cfg>
class Sorter<Cfg>::Buffers {
    using diff_t = typename Cfg::difference_type;
    using value_type = typename Cfg::value_type;

 public:
    Buffers(char* storage) : storage_(static_cast<Block*>(static_cast<void*>(storage))) {
        for (diff_t i = 0; i < Cfg::kMaxBuckets; ++i) {
            resetBuffer(i);
            buffer_[i].end = buffer_[i].ptr + Cfg::kBlockSize;
        }
    }

    /**
     * Checks if buffer is full.
     */
    bool isFull(const int i) const {
        return buffer_[i].ptr == buffer_[i].end;
    }

    /**
     * Pointer to buffer data.
     */
    value_type* data(const int i) {
        return static_cast<value_type*>(static_cast<void*>(storage_))
               + i * Cfg::kBlockSize;
    }

    /**
     * Number of elements in buffer.
     */
    diff_t size(const int i) const {
        return Cfg::kBlockSize - (buffer_[i].end - buffer_[i].ptr);
    }

    /**
     * Resets buffer.
     */
    void reset(const int i) {
        if (Block::kDestruct)
            for (auto p = data(i), end = p + size(i); p < end; ++p)
                p->~value_type();
        resetBuffer(i);
    }

    /**
     * Pushes new element to buffer.
     */
    void push(const int i, value_type&& value) {
        if (Block::kInitializedStorage) {
            *buffer_[i].ptr++ = std::move(value);
        } else {
            IPS4OML_ASSUME_NOT(buffer_[i].ptr == nullptr);
            new (buffer_[i].ptr++) value_type(std::move(value));
        }
    }

    /**
     * Flushes buffer to input.
     */
    void writeTo(const int i, typename Cfg::iterator dest) {
        resetBuffer(i);
        auto ptr = buffer_[i].ptr;
        std::move(ptr, ptr + Cfg::kBlockSize, std::move(dest));

        if (Block::kDestruct)
            for (const auto end = buffer_[i].end; ptr < end; ++ptr)
                ptr->~value_type();
    }

 private:
    struct Info {
        value_type* ptr;
        const value_type* end;
    };

    void resetBuffer(const int i) {
        buffer_[i].ptr = static_cast<value_type*>(static_cast<void*>(storage_))
                         + i * Cfg::kBlockSize;
    }

    Info buffer_[Cfg::kMaxBuckets];
    Block* storage_;
    // Blocks should have no extra elements or padding
    static_assert(sizeof(Block) == sizeof(typename Cfg::value_type) * Cfg::kBlockSize,
                  "Block size mismatch.");
    static_assert(std::is_trivially_default_constructible<Block>::value,
                  "Block must be trivially default constructible.");
    static_assert(std::is_trivially_destructible<Block>::value,
                  "Block must be trivially destructible.");
};

}  // namespace detail
}  // namespace ips4o

/******************************************************************************
 * include/ips4o/classifier.hpp
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

#include <type_traits>
#include <utility>

namespace ips4o {
namespace detail {

/**
 * Branch-free classifier.
 */
template <class Cfg>
class Classifier {
    using value_type = typename Cfg::value_type;
    using bucket_type = typename Cfg::bucket_type;
    using less = typename Cfg::less;

 public:
    Classifier(less comp) : comp_(std::move(comp)) {}

    ~Classifier() {
        if (log_buckets_) cleanup();
    }

    /**
     * Calls destructors on splitter elements.
     */
    void reset() {
        if (log_buckets_) cleanup();
    }

    /**
     * The sorted array of splitters, to be filled externally.
     */
    value_type* getSortedSplitters() {
        return static_cast<value_type*>(static_cast<void*>(sorted_storage_));
    }

    /**
     * The comparison operator.
     */
    less getComparator() const { return comp_; }

    /**
     * Builds the tree from the sorted splitters.
     */
    void build(int log_buckets) {
        log_buckets_ = log_buckets;
        num_buckets_ = 1 << log_buckets;
        const auto num_splitters = (1 << log_buckets) - 1;
        IPS4OML_ASSUME_NOT(getSortedSplitters() + num_splitters == nullptr);
        new (getSortedSplitters() + num_splitters)
                value_type(getSortedSplitters()[num_splitters - 1]);
        build(getSortedSplitters(), getSortedSplitters() + num_splitters, 1);
    }

    /**
     * Classifies a single element.
     */
    template <bool kEqualBuckets>
    bucket_type classify(const value_type& value) const {
        const int log_buckets = log_buckets_;
        const bucket_type num_buckets = num_buckets_;
        IPS4OML_ASSUME_NOT(log_buckets < 1);
        IPS4OML_ASSUME_NOT(log_buckets > Cfg::kLogBuckets + 1);

        bucket_type b = 1;
        for (int l = 0; l < log_buckets; ++l)
            b = 2 * b + comp_(splitter(b), value);
        if (kEqualBuckets)
            b = 2 * b + !comp_(value, sortedSplitter(b - num_buckets));
        return b - (kEqualBuckets ? 2 * num_buckets : num_buckets);
    }

    /**
     * Classifies all elements using a callback.
     */
    template <bool kEqualBuckets, class Iterator, class Yield>
    void classify(Iterator begin, Iterator end, Yield&& yield) const {
        classifySwitch<kEqualBuckets>(begin, end, std::forward<Yield>(yield),
            std::make_integer_sequence<int, Cfg::kLogBuckets + 1>{});
    }

    /**
     * Classifies all elements using a callback.
     */
  template <bool kEqualBuckets, class Iterator, class Yield, int...Args>
  void classifySwitch(Iterator begin, Iterator end, Yield&& yield,
		      std::integer_sequence<int, Args...>) const {
    IPS4OML_ASSUME_NOT(log_buckets_ <= 0 && log_buckets_ >= static_cast<int>(sizeof...(Args)));
    ((Args == log_buckets_ &&
      classifyUnrolled<kEqualBuckets, Args>(begin, end, std::forward<Yield>(yield)))
     || ...);
    }

    /**
     * Classifies all elements using a callback.
     */
    template <bool kEqualBuckets, int kLogBuckets, class Iterator, class Yield>
    bool classifyUnrolled(Iterator begin, const Iterator end, Yield&& yield) const {

        constexpr const bucket_type kNumBuckets = 1l << (kLogBuckets + kEqualBuckets);
        constexpr const int kUnroll = Cfg::kUnrollClassifier;
        IPS4OML_ASSUME_NOT(begin >= end);
        IPS4OML_ASSUME_NOT(begin > (end - kUnroll));

        bucket_type b[kUnroll];
        for (auto cutoff = end - kUnroll; begin <= cutoff; begin += kUnroll) {
            for (int i = 0; i < kUnroll; ++i)
                b[i] = 1;

            for (int l = 0; l < kLogBuckets; ++l)
                for (int i = 0; i < kUnroll; ++i)
                    b[i] = 2 * b[i] + comp_(splitter(b[i]), begin[i]);

            if (kEqualBuckets)
                for (int i = 0; i < kUnroll; ++i)
                    b[i] = 2 * b[i]
                           + !comp_(begin[i], sortedSplitter(b[i] - kNumBuckets / 2));

            for (int i = 0; i < kUnroll; ++i)
                yield(b[i] - kNumBuckets, begin + i);
        }

        IPS4OML_ASSUME_NOT(begin > end);
        for (; begin != end; ++begin) {
            bucket_type b = 1;
            for (int l = 0; l < kLogBuckets; ++l)
                b = 2 * b + comp_(splitter(b), *begin);
            if (kEqualBuckets)
                b = 2 * b + !comp_(*begin, sortedSplitter(b - kNumBuckets / 2));
            yield(b - kNumBuckets, begin);
        }
	return true;
    }

 private:
    const value_type& splitter(bucket_type i) const {
        return static_cast<const value_type*>(static_cast<const void*>(storage_))[i];
    }

    const value_type& sortedSplitter(bucket_type i) const {
        return static_cast<const value_type*>(
                static_cast<const void*>(sorted_storage_))[i];
    }

    value_type* data() {
        return static_cast<value_type*>(static_cast<void*>(storage_));
    }

    /**
     * Recursively builds the tree.
     */
    void build(const value_type* const left, const value_type* const right,
               const bucket_type pos) {
        const auto mid = left + (right - left) / 2;
        IPS4OML_ASSUME_NOT(data() + pos == nullptr);
        new (data() + pos) value_type(*mid);
        if (2 * pos < num_buckets_) {
            build(left, mid, 2 * pos);
            build(mid, right, 2 * pos + 1);
        }
    }

    /**
     * Destructs splitters.
     */
    void cleanup() {
        auto p = data() + 1;
        auto q = getSortedSplitters();
        for (int i = num_buckets_ - 1; i; --i) {
            p++->~value_type();
            q++->~value_type();
        }
        q->~value_type();
        log_buckets_ = 0;
    }

    // Filled from 1 to num_buckets_
    std::aligned_storage_t<sizeof(value_type), alignof(value_type)>
            storage_[Cfg::kMaxBuckets / 2];
    // Filled from 0 to num_buckets_, last one is duplicated
    std::aligned_storage_t<sizeof(value_type), alignof(value_type)>
            sorted_storage_[Cfg::kMaxBuckets / 2];
    int log_buckets_ = 0;
    bucket_type num_buckets_ = 0;
    less comp_;
};

}  // namespace detail
}  // namespace ips4o

namespace ips4o {
namespace detail {

/**
 * Aligns a pointer.
 */
template <class T>
static T* alignPointer(T* ptr, std::size_t alignment) {
    uintptr_t v = reinterpret_cast<std::uintptr_t>(ptr);
    v = (v - 1 + alignment) & ~(alignment - 1);
    return reinterpret_cast<T*>(v);
}

/**
 * Constructs an object at the specified alignment.
 */
template <class T>
class AlignedPtr {
 public:
    AlignedPtr() {}

    template <class... Args>
    explicit AlignedPtr(std::size_t alignment, Args&&... args)
        : alloc_(new char[sizeof(T) + alignment])
        , value_(new (alignPointer(alloc_, alignment)) T(std::forward<Args>(args)...)) {}

    AlignedPtr(const AlignedPtr&) = delete;
    AlignedPtr& operator=(const AlignedPtr&) = delete;

    AlignedPtr(AlignedPtr&& rhs) : alloc_(rhs.alloc_), value_(rhs.value_) {
        rhs.alloc_ = nullptr;
    }
    AlignedPtr& operator=(AlignedPtr&& rhs) {
        std::swap(alloc_, rhs.alloc_);
        std::swap(value_, rhs.value_);
        return *this;
    }

    ~AlignedPtr() {
        if (alloc_) {
            value_->~T();
            delete[] alloc_;
        }
    }

    T& get() { return *value_; }

 private:
    char* alloc_ = nullptr;
    T* value_;
};

/**
 * Provides aligned storage without constructing an object.
 */
template <>
class AlignedPtr<void> {
 public:
    AlignedPtr() {}

    template <class... Args>
    explicit AlignedPtr(std::size_t alignment, std::size_t size)
        : alloc_(new char[size + alignment]), value_(alignPointer(alloc_, alignment)) {}

    AlignedPtr(const AlignedPtr&) = delete;
    AlignedPtr& operator=(const AlignedPtr&) = delete;

    AlignedPtr(AlignedPtr&& rhs) : alloc_(rhs.alloc_), value_(rhs.value_) {
        rhs.alloc_ = nullptr;
    }
    AlignedPtr& operator=(AlignedPtr&& rhs) {
        std::swap(alloc_, rhs.alloc_);
        std::swap(value_, rhs.value_);
        return *this;
    }

    ~AlignedPtr() {
        if (alloc_) {
            delete[] alloc_;
        }
    }

    char* get() { return value_; }

 private:
    char* alloc_ = nullptr;
    char* value_;
};

/**
 * Aligned storage for use in buffers.
 */
template <class Cfg>
class Sorter<Cfg>::BufferStorage : public AlignedPtr<void> {
 public:
    static constexpr const auto kPerThread =
            Cfg::kBlockSizeInBytes * Cfg::kMaxBuckets * (1 + Cfg::kAllowEqualBuckets);

    BufferStorage() {}

    explicit BufferStorage(int num_threads)
        : AlignedPtr<void>(Cfg::kDataAlignment, num_threads * kPerThread) {}

    char* forThread(int id) { return this->get() + id * kPerThread; }
};

/**
 * Data local to each thread.
 */
template <class Cfg>
struct Sorter<Cfg>::LocalData {
    using diff_t = typename Cfg::difference_type;
    // Buffers
    diff_t bucket_size[Cfg::kMaxBuckets];
    Buffers buffers;
    Block swap[2];
    Block overflow;

    PrivateQueue<Task> seq_task_queue;

    // Bucket information
    BucketPointers bucket_pointers[Cfg::kMaxBuckets];

    // Classifier
    Classifier classifier;

    // Information used during empty block movement
    diff_t first_block;
    diff_t first_empty_block;

    // Random bit generator for sampling
    // LCG using constants by Knuth (for 64 bit) or Numerical Recipes (for 32 bit)
    std::linear_congruential_engine<
            std::uintptr_t, Cfg::kIs64Bit ? 6364136223846793005u : 1664525u,
            Cfg::kIs64Bit ? 1442695040888963407u : 1013904223u, 0u>
            random_generator;

    LocalData(typename Cfg::less comp, char* buffer_storage)
        : buffers(buffer_storage), classifier(std::move(comp)) {
        std::random_device rdev;
        std::ptrdiff_t seed = rdev();
        if (Cfg::kIs64Bit) seed = (seed << (Cfg::kIs64Bit * 32)) | rdev();
        random_generator.seed(seed);
        reset();
    }

    /**
     * Resets local data after partitioning is done.
     */
    void reset() {
        classifier.reset();
        std::fill_n(bucket_size, Cfg::kMaxBuckets, 0);
    }
};

/**
 * Data describing a parallel task and the corresponding threads.
 */
struct BigTask {
    BigTask() : has_task{false} {}
    // TODO or Cfg::iterator???
    std::ptrdiff_t begin;
    std::ptrdiff_t end;
    // My thread id of this task.
    int task_thread_id;
    // Index of the thread owning the thread pool used by this task.
    int root_thread;
    // Indicates whether this is a task or not
    bool has_task;
};

}  // namespace detail
}  // namespace ips4o

/******************************************************************************
 * include/ips4o/parallel.hpp
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

/******************************************************************************
 * include/ips4o/sequential.hpp
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

#include <utility>

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

#include <atomic>
#include <tuple>
#include <utility>

/******************************************************************************
 * include/ips4o/block_permutation.hpp
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

#include <tuple>

namespace ips4o {
namespace detail {

/**
 * Computes which bucket can cause an overflow,
 * i.e., the last bucket that has more than one full block.
 */
template <class Cfg>
int Sorter<Cfg>::computeOverflowBucket() {
    int bucket = num_buckets_ - 1;
    while (bucket >= 0
           && (bucket_start_[bucket + 1] - bucket_start_[bucket]) <= Cfg::kBlockSize)
        --bucket;
    return bucket;
}

/**
 * Tries to read a block from read_bucket.
 */
template <class Cfg>
template <bool kEqualBuckets, bool kIsParallel>
int Sorter<Cfg>::classifyAndReadBlock(const int read_bucket) {
    auto& bp = bucket_pointers_[read_bucket];

    diff_t write, read;
    std::tie(write, read) = bp.template decRead<kIsParallel>();

    if (read < write) {
        // No more blocks in this bucket
        if (kIsParallel) bp.stopRead();
        return -1;
    }

    // Read block
    local_.swap[0].readFrom(begin_ + read);
    if (kIsParallel) bp.stopRead();

    return classifier_->template classify<kEqualBuckets>(local_.swap[0].head());
}

/**
 * Finds a slot for the block in the swap buffer. May or may not read another block.
 */
template <class Cfg>
template <bool kEqualBuckets, bool kIsParallel>
int Sorter<Cfg>::swapBlock(const diff_t max_off, const int dest_bucket,
                           const bool current_swap) {
    diff_t write, read;
    int new_dest_bucket;
    auto& bp = bucket_pointers_[dest_bucket];
    do {
        std::tie(write, read) = bp.template incWrite<kIsParallel>();
        if (write > read) {
            // Destination block is empty
            if (write >= max_off) {
                // Out-of-bounds; write to overflow buffer instead
                local_.swap[current_swap].writeTo(local_.overflow);
                overflow_ = &local_.overflow;
                return -1;
            }
            // Make sure no one is currently reading this block
            while (kIsParallel && bp.isReading()) {}
            // Write block
            local_.swap[current_swap].writeTo(begin_ + write);
            return -1;
        }
        // Check if block needs to be moved
        new_dest_bucket = classifier_->template classify<kEqualBuckets>(begin_[write]);
    } while (new_dest_bucket == dest_bucket);

    // Swap blocks
    local_.swap[!current_swap].readFrom(begin_ + write);
    local_.swap[current_swap].writeTo(begin_ + write);

    return new_dest_bucket;
}

/**
 * Block permutation phase.
 */
template <class Cfg>
template <bool kEqualBuckets, bool kIsParallel>
void Sorter<Cfg>::permuteBlocks() {
    const auto num_buckets = num_buckets_;
    // Distribute starting points of threads
    int read_bucket = (my_id_ * num_buckets / num_threads_) % num_buckets;
    // Not allowed to write to this offset, to avoid overflow
    const diff_t max_off = Cfg::alignToNextBlock(end_ - begin_ + 1) - Cfg::kBlockSize;

    // Go through all buckets
    for (int count = num_buckets; count; --count) {
        int dest_bucket;
        // Try to read a block ...
        while ((dest_bucket =
                        classifyAndReadBlock<kEqualBuckets, kIsParallel>(read_bucket))
               != -1) {
            bool current_swap = 0;
            // ... then write it to the correct bucket
            while ((dest_bucket = swapBlock<kEqualBuckets, kIsParallel>(
                            max_off, dest_bucket, current_swap))
                   != -1) {
                // Read another block, keep going
                current_swap = !current_swap;
            }
        }
        read_bucket = (read_bucket + 1) % num_buckets;
    }
}

}  // namespace detail
}  // namespace ips4o

/******************************************************************************
 * include/ips4o/cleanup_margins.hpp
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

#include <limits>
#include <utility>

namespace ips4o {
namespace detail {

/**
 * Saves margins at thread boundaries.
 */

/**
 * Fills margins from buffers.
 */
template <class Cfg>
template <bool kIsParallel>
void Sorter<Cfg>::writeMargins(const int first_bucket, const int last_bucket,
                               const int overflow_bucket, const int swap_bucket,
                               const diff_t in_swap_buffer) {
    const bool is_last_level = end_ - begin_ <= Cfg::kSingleLevelThreshold;
    const auto comp = classifier_->getComparator();

    for (int i = first_bucket; i < last_bucket; ++i) {
        // Get bucket information
        const auto bstart = bucket_start_[i];
        const auto bend = bucket_start_[i + 1];
        const auto bwrite = bucket_pointers_[i].getWrite();
        // Destination where elements can be written
        auto dst = begin_ + bstart;
        auto remaining = Cfg::alignToNextBlock(bstart) - bstart;

        if (i == overflow_bucket && overflow_) {
            // Is there overflow?

            // Overflow buffer has been written => write pointer must be at end of bucket
            IPS4OML_ASSUME_NOT(Cfg::alignToNextBlock(bend) != bwrite);

            auto src = overflow_->data();
            // There must be space for at least BlockSize elements
            IPS4OML_ASSUME_NOT((bend - (bwrite - Cfg::kBlockSize)) + remaining
                               < Cfg::kBlockSize);
            auto tail_size = Cfg::kBlockSize - remaining;

            // Fill head
            std::move(src, src + remaining, dst);
            src += remaining;
            remaining = std::numeric_limits<diff_t>::max();

            // Write remaining elements into tail
            dst = begin_ + (bwrite - Cfg::kBlockSize);
            dst = std::move(src, src + tail_size, dst);

            overflow_->reset(Cfg::kBlockSize);
        } else if (i == swap_bucket && in_swap_buffer) {
            // Did we save this in saveMargins?

            // Bucket of last block in this thread's area => write swap buffer
            auto src = local_.swap[0].data();
            // All elements from the buffer must fit
            IPS4OML_ASSUME_NOT(in_swap_buffer > remaining);

            // Write to head
            dst = std::move(src, src + in_swap_buffer, dst);
            remaining -= in_swap_buffer;

            local_.swap[0].reset(in_swap_buffer);
        } else if (bwrite > bend && bend - bstart > Cfg::kBlockSize) {
            // Final block has been written => move excess elements to head
            IPS4OML_ASSUME_NOT(Cfg::alignToNextBlock(bend) != bwrite);

            auto src = begin_ + bend;
            auto head_size = bwrite - bend;
            // Must fit, no other empty space left
            IPS4OML_ASSUME_NOT(head_size > remaining);

            // Write to head
            dst = std::move(src, src + head_size, dst);
            remaining -= head_size;
        }

        // Write elements from buffers
        for (int t = 0; t < num_threads_; ++t) {
            decltype(local_.buffers)*buffers_ptr = nullptr;
            if constexpr (kIsParallel) {
                buffers_ptr = &shared_->local[t]->buffers;
            } else {
                buffers_ptr = &local_.buffers;
            }
            auto& buffers = *buffers_ptr;
            auto src = buffers.data(i);
            auto count = buffers.size(i);

            if (count <= remaining) {
                dst = std::move(src, src + count, dst);
                remaining -= count;
            } else {
                std::move(src, src + remaining, dst);
                src += remaining;
                count -= remaining;
                remaining = std::numeric_limits<diff_t>::max();

                dst = begin_ + bwrite;
                dst = std::move(src, src + count, dst);
            }

            buffers.reset(i);
        }

        // Perform final base case sort here, while the data is still cached
        if (is_last_level
            || ((bend - bstart <= 2 * Cfg::kBaseCaseSize) && !kIsParallel)) {
#ifdef IPS4O_TIMER
            g_cleanup.stop();
            g_base_case.start();
#endif

            detail::baseCaseSort(begin_ + bstart, begin_ + bend, comp);

#ifdef IPS4O_TIMER
            g_base_case.stop();
            g_cleanup.start();
#endif
        }
    }
}

}  // namespace detail
}  // namespace ips4o

/******************************************************************************
 * include/ips4o/local_classification.hpp
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

/******************************************************************************
 * include/ips4o/empty_block_movement.hpp
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

#include <algorithm>

namespace ips4o {
namespace detail {

/**
 * Moves empty blocks to establish invariant:
 * All buckets must consist of full blocks followed by empty blocks.
 */
template <class Cfg>
void Sorter<Cfg>::moveEmptyBlocks(const diff_t my_begin, const diff_t my_end,
                                  const diff_t my_first_empty_block) {
    // Find range of buckets that start in this stripe
    const int bucket_range_start = [&](int i) {
        while (Cfg::alignToNextBlock(bucket_start_[i]) < my_begin) ++i;
        return i;
    }(0);
    const int bucket_range_end = [&](int i) {
        const auto num_buckets = num_buckets_;
        if (my_id_ == num_threads_ - 1) return num_buckets;
        while (i < num_buckets && Cfg::alignToNextBlock(bucket_start_[i]) < my_end) ++i;
        return i;
    }(bucket_range_start);

    /*
     * After classification, a stripe consists of full blocks followed by empty blocks.
     * This means that the invariant above already holds for all buckets except those that
     * cross stripe boundaries.
     *
     * The following cases exist:
     * 1)  The bucket is fully contained within one stripe.
     *     In this case, nothing needs to be done, just set the bucket pointers.
     *
     * 2)  The bucket starts in stripe i, and ends in stripe i+1.
     *     In this case, thread i moves full blocks from the end of the bucket (from the
     *     stripe of thread i+1) to fill the holes at the end of its stripe.
     *
     * 3)  The bucket starts in stripe i, crosses more than one stripe boundary, and ends
     *     in stripe i+k. This is an extension of case 2. In this case, multiple threads
     *     work on the same bucket. Each thread is responsible for filling the empty
     *     blocks in its stripe. The left-most thread will take the right-most blocks.
     *     Therefore, we count how many blocks are fetched by threads to our left before
     *     moving our own blocks.
     */

    // Check if last bucket overlaps the end of the stripe
    const auto bucket_end = Cfg::alignToNextBlock(bucket_start_[bucket_range_end]);
    const bool last_bucket_is_overlapping = bucket_end > my_end;

    // Case 1)
    for (int b = bucket_range_start; b < bucket_range_end - last_bucket_is_overlapping;
         ++b) {
        const auto start = Cfg::alignToNextBlock(bucket_start_[b]);
        const auto stop = Cfg::alignToNextBlock(bucket_start_[b + 1]);
        auto read = stop;
        if (my_first_empty_block <= start) {
            // Bucket is completely empty
            read = start;
        } else if (my_first_empty_block < stop) {
            // Bucket is partially empty
            read = my_first_empty_block;
        }
        bucket_pointers_[b].set(start, read - Cfg::kBlockSize);
    }

    // Cases 2) and 3)
    if (last_bucket_is_overlapping) {
        const int overlapping_bucket = bucket_range_end - 1;
        const auto bucket_start =
                Cfg::alignToNextBlock(bucket_start_[overlapping_bucket]);

        // If it is a very large bucket, other threads will also move blocks around in it
        // (case 3) Count how many filled blocks are in this bucket
        diff_t flushed_elements_in_bucket = 0;
        if (bucket_start < my_begin) {
            int prev_id = my_id_ - 1;
            // Iterate over stripes which are completely contained in this bucket
            while (bucket_start < shared_->local[prev_id]->first_block) {
                const auto eb = shared_->local[prev_id]->first_empty_block;
                flushed_elements_in_bucket += eb - shared_->local[prev_id]->first_block;
                --prev_id;
            }
            // Count blocks in stripe where bucket starts
            const auto eb = shared_->local[prev_id]->first_empty_block;
            // Check if there are any filled blocks in this bucket
            if (eb > bucket_start) flushed_elements_in_bucket += eb - bucket_start;
        }

        // Threads to our left will move this many blocks (0 if we are the left-most)
        diff_t elements_reserved = 0;
        if (my_begin > bucket_start) {
            // Threads to the left of us get priority
            elements_reserved = my_begin - bucket_start - flushed_elements_in_bucket;

            // Count how many elements we flushed into this bucket
            flushed_elements_in_bucket += my_first_empty_block - my_begin;
        } else if (my_first_empty_block > bucket_start) {
            // We are the left-most thread
            // Count how many elements we flushed into this bucket
            flushed_elements_in_bucket += my_first_empty_block - bucket_start;
        }

        // Find stripe which contains last block of this bucket (off by one)
        // Also continue counting how many filled blocks are in this bucket
        int read_from_thread = my_id_ + 1;
        while (read_from_thread < num_threads_
               && bucket_end > shared_->local[read_from_thread]->first_block) {
            const auto eb = std::min<diff_t>(
                    shared_->local[read_from_thread]->first_empty_block, bucket_end);
            flushed_elements_in_bucket +=
                    eb - shared_->local[read_from_thread]->first_block;
            ++read_from_thread;
        }

        // After moving blocks, this will be the first empty block in this bucket
        const auto first_empty_block_in_bucket =
                bucket_start + flushed_elements_in_bucket;

        // This is the range of blocks we want to fill
        auto write_ptr = begin_ + std::max(my_first_empty_block, bucket_start);
        const auto write_ptr_end = begin_ + std::min(first_empty_block_in_bucket, my_end);

        // Read from other stripes until we filled our blocks
        while (write_ptr < write_ptr_end) {
            --read_from_thread;
            // This is the range of blocks we can read from stripe 'read_from_thread'
            auto read_ptr = std::min(shared_->local[read_from_thread]->first_empty_block,
                                     bucket_end);
            auto read_range_size =
                    read_ptr - shared_->local[read_from_thread]->first_block;

            // Skip reserved blocks
            if (elements_reserved >= read_range_size) {
                elements_reserved -= read_range_size;
                continue;
            }
            read_ptr -= elements_reserved;
            read_range_size -= elements_reserved;
            elements_reserved = 0;

            // Move blocks
            const auto size = std::min(read_range_size, write_ptr_end - write_ptr);
            write_ptr = std::move(begin_ + read_ptr - size, begin_ + read_ptr, write_ptr);
        }

        // Set bucket pointers if the bucket starts in this stripe
        if (my_begin <= bucket_start) {
            bucket_pointers_[overlapping_bucket].set(
                    bucket_start, first_empty_block_in_bucket - Cfg::kBlockSize);
        }
    }
}

}  // namespace detail
}  // namespace ips4o

namespace ips4o {
namespace detail {

/**
 * Local classification phase.
 */
template <class Cfg>
template <bool kEqualBuckets>
typename Cfg::difference_type Sorter<Cfg>::classifyLocally(const iterator my_begin,
                                                           const iterator my_end) {
    auto write = my_begin;
    auto& buffers = local_.buffers;

    // Do the classification
    classifier_->template classify<kEqualBuckets>(
            my_begin, my_end, [&](typename Cfg::bucket_type bucket, iterator it) {
                // Only flush buffers on overflow
                if (buffers.isFull(bucket)) {
                    buffers.writeTo(bucket, write);
                    write += Cfg::kBlockSize;
                    local_.bucket_size[bucket] += Cfg::kBlockSize;
                }
                buffers.push(bucket, std::move(*it));
            });

    // Update bucket sizes to account for partially filled buckets
    for (int i = 0, end = num_buckets_; i < end; ++i)
        local_.bucket_size[i] += local_.buffers.size(i);

    return write - begin_;
}

/**
 * Local classification in the sequential case.
 */
template <class Cfg>
void Sorter<Cfg>::sequentialClassification(const bool use_equal_buckets) {
    const auto my_first_empty_block = use_equal_buckets
                                              ? classifyLocally<true>(begin_, end_)
                                              : classifyLocally<false>(begin_, end_);

    // Find bucket boundaries
    diff_t sum = 0;
    bucket_start_[0] = 0;
    for (int i = 0, end = num_buckets_; i < end; ++i) {
        sum += local_.bucket_size[i];
        bucket_start_[i + 1] = sum;
    }
    IPS4OML_ASSUME_NOT(bucket_start_[num_buckets_] != end_ - begin_);

    // Set write/read pointers for all buckets
    for (int bucket = 0, end = num_buckets_; bucket < end; ++bucket) {
        const auto start = Cfg::alignToNextBlock(bucket_start_[bucket]);
        const auto stop = Cfg::alignToNextBlock(bucket_start_[bucket + 1]);
        bucket_pointers_[bucket].set(
                start,
                (start >= my_first_empty_block
                         ? start
                         : (stop <= my_first_empty_block ? stop : my_first_empty_block))
                        - Cfg::kBlockSize);
    }
}

/**
 * Local classification in the parallel case.
 */
template <class Cfg>
void Sorter<Cfg>::parallelClassification(const bool use_equal_buckets) {
    // Compute stripe for each thread
    const auto elements_per_thread = static_cast<double>(end_ - begin_) / num_threads_;
    const auto my_begin =
            begin_ + Cfg::alignToNextBlock(my_id_ * elements_per_thread + 0.5);
    const auto my_end = [&] {
        const auto size = end_ - begin_;
        const auto e = Cfg::alignToNextBlock((my_id_ + 1) * elements_per_thread + 0.5);
        if (size < e) {
          return end_;
        }
        return begin_ + e;
    }();

    local_.first_block = my_begin - begin_;

    // Do classification
    if (my_begin >= my_end) {
        // Small input (less than two blocks per thread), wait for other threads to finish
        local_.first_empty_block = my_begin - begin_;
        shared_->sync.barrier();
        shared_->sync.barrier();
    } else {
        const auto my_first_empty_block =
                use_equal_buckets ? classifyLocally<true>(my_begin, my_end)
                                  : classifyLocally<false>(my_begin, my_end);

        // Find bucket boundaries
        diff_t sum = 0;
        for (int i = 0, end = num_buckets_; i < end; ++i) {
            sum += local_.bucket_size[i];
            __atomic_fetch_add(&bucket_start_[i + 1], sum, __ATOMIC_RELAXED);
        }

        local_.first_empty_block = my_first_empty_block;

        shared_->sync.barrier();

#ifdef IPS4O_TIMER
        g_classification.stop();
        g_empty_block.start();
#endif

        // Move empty blocks and set bucket write/read pointers
        moveEmptyBlocks(my_begin - begin_, my_end - begin_, my_first_empty_block);

        shared_->sync.barrier();

#ifdef IPS4O_TIMER
        g_empty_block.stop();
        g_classification.start();
#endif
    }
}

}  // namespace detail
}  // namespace ips4o

/******************************************************************************
 * include/ips4o/sampling.hpp
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

#include <iterator>
#include <random>
#include <utility>

namespace ips4o {
namespace detail {

/**
 * Selects a random sample in-place.
 */
template <class It, class RandomGen>
void selectSample(It begin, const It end,
                  typename std::iterator_traits<It>::difference_type num_samples,
                  RandomGen&& gen) {
    using std::swap;

    auto n = end - begin;
    while (num_samples--) {
        const auto i = std::uniform_int_distribution<
                typename std::iterator_traits<It>::difference_type>(0, --n)(gen);
        swap(*begin, begin[i]);
        ++begin;
    }
}

/**
 * Builds the classifer.
 * Number of used_buckets is a power of two and at least two.
 */
template <class Cfg>
std::pair<int, bool> Sorter<Cfg>::buildClassifier(const iterator begin,
                                                  const iterator end,
                                                  Classifier& classifier) {
    const auto n = end - begin;
    int log_buckets = Cfg::logBuckets(n);
    int num_buckets = 1 << log_buckets;
    const auto step = std::max<diff_t>(1, Cfg::oversamplingFactor(n));
    const auto num_samples = std::min(step * num_buckets - 1, n / 2);

    // Select the sample
    detail::selectSample(begin, end, num_samples, local_.random_generator);

    // Sort the sample
    sequential(begin, begin + num_samples);
    auto splitter = begin + step - 1;
    auto sorted_splitters = classifier.getSortedSplitters();
    const auto comp = classifier.getComparator();

    // Choose the splitters
    IPS4OML_ASSUME_NOT(sorted_splitters == nullptr);
    new (sorted_splitters) typename Cfg::value_type(*splitter);
    for (int i = 2; i < num_buckets; ++i) {
        splitter += step;
        // Skip duplicates
        if (comp(*sorted_splitters, *splitter)) {
            IPS4OML_ASSUME_NOT(sorted_splitters + 1 == nullptr);
            new (++sorted_splitters) typename Cfg::value_type(*splitter);
        }
    }

    // Check for duplicate splitters
    const auto diff_splitters = sorted_splitters - classifier.getSortedSplitters() + 1;
    const bool use_equal_buckets = Cfg::kAllowEqualBuckets
            && num_buckets - 1 - diff_splitters >= Cfg::kEqualBucketsThreshold;

    // Fill the array to the next power of two
    log_buckets = log2(diff_splitters) + 1;
    num_buckets = 1 << log_buckets;
    for (int i = diff_splitters + 1; i < num_buckets; ++i) {
        IPS4OML_ASSUME_NOT(sorted_splitters + 1 == nullptr);
        new (++sorted_splitters) typename Cfg::value_type(*splitter);
    }

    // Build the tree
    classifier.build(log_buckets);
    this->classifier_ = &classifier;

    const int used_buckets = num_buckets * (1 + use_equal_buckets);
    return {used_buckets, use_equal_buckets};
}

}  // namespace detail
}  // namespace ips4o

namespace ips4o {
namespace detail {

/**
 * Main partitioning function.
 */
template <class Cfg>
template <bool kIsParallel>
std::pair<int, bool> Sorter<Cfg>::partition(const iterator begin, const iterator end,
                                            diff_t* const bucket_start, const int my_id,
                                            const int num_threads) {
#ifdef IPS4O_TIMER
    g_overhead.stop();
    g_sampling.start();
#endif

    // Sampling
    bool use_equal_buckets = false;
    {
        if constexpr (!kIsParallel) {
            std::tie(this->num_buckets_, use_equal_buckets) =
                    buildClassifier(begin, end, local_.classifier);
        } else {
            shared_->sync.single([&] {
                std::tie(this->num_buckets_, use_equal_buckets) =
                        buildClassifier(begin, end, shared_->classifier);
                shared_->num_buckets = this->num_buckets_;
                shared_->use_equal_buckets = use_equal_buckets;
            });
            this->num_buckets_ = shared_->num_buckets;
            use_equal_buckets = shared_->use_equal_buckets;
        }
    }

    // Set parameters for this partitioning step
    // Must do this AFTER sampling, because sampling will recurse to sort splitters.
    if constexpr (kIsParallel) {
        this->classifier_ = &shared_->classifier;
    this->bucket_pointers_ = shared_->bucket_pointers;
    } else {
        this->classifier_ = &local_.classifier;
        this->bucket_pointers_ = local_.bucket_pointers;
    }
    this->bucket_start_ = bucket_start;
    this->overflow_ = nullptr;
    this->begin_ = begin;
    this->end_ = end;
    this->my_id_ = my_id;
    this->num_threads_ = num_threads;

#ifdef IPS4O_TIMER
    g_sampling.stop();
    g_classification.start();
#endif

    // Local Classification
    if constexpr (kIsParallel)
        parallelClassification(use_equal_buckets);
    else
        sequentialClassification(use_equal_buckets);

#ifdef IPS4O_TIMER
    g_classification.stop(end - begin, "class");
    g_permutation.start();
#endif

    // Compute which bucket can cause overflow
    const int overflow_bucket = computeOverflowBucket();

    // Block Permutation
    if (use_equal_buckets)
        permuteBlocks<true, kIsParallel>();
    else
        permuteBlocks<false, kIsParallel>();

    if constexpr (kIsParallel && overflow_)
        shared_->overflow = &local_.overflow;

    if constexpr (kIsParallel) shared_->sync.barrier();

#ifdef IPS4O_TIMER
    g_permutation.stop(end - begin, "perm");
    g_cleanup.start();
#endif

    // Cleanup
    {
        if constexpr (kIsParallel) overflow_ = shared_->overflow;

        // Distribute buckets among threads
        const int num_buckets = num_buckets_;
        const int buckets_per_thread = (num_buckets + num_threads_ - 1) / num_threads_;
        int my_first_bucket = my_id_ * buckets_per_thread;
        int my_last_bucket = (my_id_ + 1) * buckets_per_thread;
        my_first_bucket = num_buckets < my_first_bucket ? num_buckets : my_first_bucket;
        my_last_bucket = num_buckets < my_last_bucket ? num_buckets : my_last_bucket;

        // Save excess elements at right end of stripe
        auto in_swap_buffer =  std::pair<int, diff_t>(-1, 0);
        if constexpr (kIsParallel) shared_->sync.barrier();

        // Write remaining elements
        writeMargins<kIsParallel>(my_first_bucket, my_last_bucket, overflow_bucket,
                                  in_swap_buffer.first, in_swap_buffer.second);
    }

    if constexpr (kIsParallel) shared_->sync.barrier();
    local_.reset();

#ifdef IPS4O_TIMER
    g_cleanup.stop();
    g_overhead.start();
#endif

    return {this->num_buckets_, use_equal_buckets};
}

}  // namespace detail
}  // namespace ips4o

namespace ips4o {
namespace detail {

/**
 * Recursive entry point for sequential algorithm.
 */
template <class Cfg>
void Sorter<Cfg>::sequential(const iterator begin, const iterator end) {
    // Check for base case
    const auto n = end - begin;
    if (n <= 2 * Cfg::kBaseCaseSize) {
#ifdef IPS4O_TIMER
        g_overhead.stop();
        g_base_case.start();
#endif

        detail::baseCaseSort(begin, end, local_.classifier.getComparator());

#ifdef IPS4O_TIMER
        g_base_case.stop();
        g_overhead.start();
#endif

        return;
    }

    sequential_rec(begin, end);
}

/**
 * Recursive entry point for sequential algorithm.
 */
template <class Cfg>
void Sorter<Cfg>::sequential_rec(const iterator begin, const iterator end) {
    // Check for base case
    const auto n = end - begin;
    IPS4OML_IS_NOT(n <= 2 * Cfg::kBaseCaseSize);

    diff_t bucket_start[Cfg::kMaxBuckets + 1];

    // Do the partitioning
    const auto res = partition<false>(begin, end, bucket_start, 0, 1);
    const int num_buckets = std::get<0>(res);
    const bool equal_buckets = std::get<1>(res);

    // Final base case is executed in cleanup step, so we're done here
    if (n <= Cfg::kSingleLevelThreshold) {
        return;
    }

#ifdef IPS4O_TIMER
    g_ips4o_level++;
#endif

    // Recurse
    for (int i = 0; i < num_buckets; i += 1 + equal_buckets) {
        const auto start = bucket_start[i];
        const auto stop = bucket_start[i + 1];
        if (stop - start > 2 * Cfg::kBaseCaseSize)
            sequential(begin + start, begin + stop);
    }
    if (equal_buckets) {
        const auto start = bucket_start[num_buckets - 1];
        const auto stop = bucket_start[num_buckets];
        if (stop - start > 2 * Cfg::kBaseCaseSize)
            sequential(begin + start, begin + stop);
    }

#ifdef IPS4O_TIMER
    g_ips4o_level--;
#endif

}

}  // namespace detail

/**
 * Reusable sequential sorter.
 */
template <class Cfg>
class SequentialSorter {
    using Sorter = detail::Sorter<Cfg>;
    using iterator = typename Cfg::iterator;

 public:
    explicit SequentialSorter(bool check_sorted, typename Cfg::less comp)
        : check_sorted_(check_sorted)
        , buffer_storage_(1)
        , local_ptr_(Cfg::kDataAlignment, std::move(comp), buffer_storage_.get()) {}

    explicit SequentialSorter(bool check_sorted, typename Cfg::less comp,
                              char* buffer_storage)
        : check_sorted_(check_sorted)
        , local_ptr_(Cfg::kDataAlignment, std::move(comp), buffer_storage) {}

    void operator()(iterator begin, iterator end) {
        if (check_sorted_) {
            const bool sorted = detail::sortSimpleCases(
                    begin, end, local_ptr_.get().classifier.getComparator());
            if (sorted) return;
        }

        Sorter(local_ptr_.get()).sequential(std::move(begin), std::move(end));
    }

 private:
    const bool check_sorted_;
    typename Sorter::BufferStorage buffer_storage_;
    detail::AlignedPtr<typename Sorter::LocalData> local_ptr_;
};

}  // namespace ips4o

namespace ips4o {

/**
 * Helper function for creating a reusable sequential sorter.
 */
template <class It, class Cfg = Config<>, class Comp = std::less<>>
SequentialSorter<ExtendedConfig<It, Comp, Cfg>> make_sorter(Comp comp = Comp()) {
  return SequentialSorter<ExtendedConfig<It, Comp, Cfg>>{true, std::move(comp)};
}

/**
 * Configurable interface.
 */
template <class Cfg, class It, class Comp = std::less<>>
void sort(It begin, It end, Comp comp = Comp()) {
#ifdef IPS4O_TIMER
    g_active_counters = -1;
    g_total.start();
    g_overhead.start();
#endif

    if (detail::sortSimpleCases(begin, end, comp)) {
#ifdef IPS4O_TIMER
        g_overhead.stop();
        g_total.stop();
#endif

        return;
    }

    if ((end - begin) <= Cfg::kBaseCaseMultiplier * Cfg::kBaseCaseSize) {
#ifdef IPS4O_TIMER
        g_overhead.stop();
        g_base_case.start();
#endif

        detail::baseCaseSort(std::move(begin), std::move(end), std::move(comp));

#ifdef IPS4O_TIMER
        g_base_case.stop();
        g_overhead.start();
#endif
    } else {
        ips4o::SequentialSorter<ips4o::ExtendedConfig<It, Comp, Cfg>> sorter{
                false, std::move(comp)};
        sorter(std::move(begin), std::move(end));
    }

#ifdef IPS4O_TIMER
    g_overhead.stop();
    g_total.stop();
#endif
}

/**
 * Standard interface.
 */
template <class It, class Comp>
void sort(It begin, It end, Comp comp) {
    ips4o::sort<Config<>>(std::move(begin), std::move(end), std::move(comp));
}

template <class It>
void sort(It begin, It end) {
    ips4o::sort<Config<>>(std::move(begin), std::move(end), std::less<>());
}

}  // namespace ips4o

