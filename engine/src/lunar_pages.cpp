#include "lunar_pages.hpp"

#include <algorithm>
#include <numeric>

namespace eng::lunar {

PageCache::PageCache(int layers):slots_(static_cast<std::size_t>(layers)){}

void PageCache::clear(){ std::fill(slots_.begin(),slots_.end(),Slot{}); }

PagePlan PageCache::plan(PageCoord origin,const PageRevisions& revisions,int upload_limit){
    std::array<int,page_cells> indices{};
    std::iota(indices.begin(),indices.end(),0);
    const auto cost=[](int i){
        const int x=i%page_grid-page_grid/2,y=i/page_grid-page_grid/2;
        return x*x+y*y;
    };
    std::stable_sort(indices.begin(),indices.end(),[&](int a,int b){ return cost(a)<cost(b); });
    std::array<PageCoord,page_cells> wanted{};
    int count=0;
    for(int i:indices){
        if(revisions[i]==0) continue;
        if(count==static_cast<int>(slots_.size())) break;
        wanted[count++]={origin.x+i%page_grid,origin.y+i/page_grid};
    }
    const auto wanted_end=wanted.begin()+count;
    const auto needed=[&](PageCoord coord){ return std::find(wanted.begin(),wanted_end,coord)!=wanted_end; };

    PagePlan out;
    for(auto coord=wanted.begin();coord!=wanted_end;++coord){
        const auto index=(coord->y-origin.y)*page_grid+coord->x-origin.x;
        const auto revision=revisions[index];
        auto slot=std::find_if(slots_.begin(),slots_.end(),[&](const Slot& s){ return s.revision && s.coordinate==*coord; });
        if(slot==slots_.end() || slot->revision!=revision){
            if(static_cast<int>(out.uploads.size())>=upload_limit) continue;
            if(slot==slots_.end()){
                slot=std::find_if(slots_.begin(),slots_.end(),[&](const Slot& s){ return !s.revision || !needed(s.coordinate); });
            }
            if(slot==slots_.end()) continue;
            *slot={*coord,revision};
            out.uploads.push_back({*coord,static_cast<int>(slot-slots_.begin())});
        }
        out.table[index*4]=static_cast<std::uint8_t>(slot-slots_.begin()+1);
    }
    return out;
}

} // namespace eng::lunar
