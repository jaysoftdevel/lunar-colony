#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace eng::lunar {

inline constexpr int page_grid=5;
inline constexpr int page_cells=page_grid*page_grid;
struct PageCoord {
    int x{},y{};
    bool operator==(const PageCoord&) const=default;
};
struct PageUpload { PageCoord coordinate; int layer; };
struct PagePlan {
    std::array<std::uint8_t,page_cells*4> table{};
    std::vector<PageUpload> uploads;
};
using PageRevisions=std::array<std::uint64_t,page_cells>;

// Physical texture layers hold the pages nearest the window's center. A
// revision change invalidates an entry immediately, even when upload is deferred.
class PageCache {
public:
    explicit PageCache(int layers);
    PagePlan plan(PageCoord origin,const PageRevisions& revisions,int upload_limit,int fade_step=255);
    void clear();

private:
    struct Slot { PageCoord coordinate{}; std::uint64_t revision{}; int opacity{}; };
    std::vector<Slot> slots_;
};

} // namespace eng::lunar
