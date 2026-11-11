#pragma once

#include <cstddef>
#include <vector>

namespace eng::cpu {

struct Affinity {
    // First entry leaves room for the thread that created the worker pool.
    // IDs are logical CPU numbers on Linux and CPU Set IDs on Windows.
    std::vector<unsigned> processors;
    std::size_t mask_bytes{};
};

Affinity discover();
bool select_current_thread(const Affinity& affinity,std::size_t processor);

} // namespace eng::cpu
