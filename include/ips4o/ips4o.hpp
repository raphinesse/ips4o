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

#pragma once

#include <functional>
#include <iterator>
#include <type_traits>

#include "ips4o_fwd.hpp"
#include "base_case.hpp"
#include "config.hpp"
#include "memory.hpp"
#include "sequential.hpp"

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
