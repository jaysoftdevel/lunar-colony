#if defined(__linux__) && !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif

#include "cpu_affinity.hpp"
#include <algorithm>
#include <limits>
#include <utility>

#if defined(__linux__)
#include <cerrno>
#include <memory>
#include <pthread.h>
#include <sched.h>
#elif defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <cstring>
#endif

namespace eng::cpu {
namespace {
void prefer_caller(Affinity& affinity,unsigned id){
    const auto current=std::find(affinity.processors.begin(),affinity.processors.end(),id);
    if(current!=affinity.processors.end())
        std::rotate(affinity.processors.begin(),current,current+1);
}

#if defined(__linux__)
struct MaskDeleter {
    void operator()(cpu_set_t* mask) const { CPU_FREE(mask); }
};
using Mask=std::unique_ptr<cpu_set_t,MaskDeleter>;
#endif
}

Affinity discover(){
    Affinity affinity;
#if defined(__linux__)
    // The kernel can use a larger mask than cpu_set_t, and CPU numbers can
    // have holes after cgroup, launcher, or hotplug restrictions.
    for(int bits=CPU_SETSIZE;;bits*=2){
        Mask mask(CPU_ALLOC(bits));
        if(!mask) return {};
        const auto bytes=CPU_ALLOC_SIZE(bits);
        CPU_ZERO_S(bytes,mask.get());
        if(sched_getaffinity(0,bytes,mask.get())==0){
            affinity.mask_bytes=bytes;
            affinity.processors.reserve(CPU_COUNT_S(bytes,mask.get()));
            for(int id=0;id<bits;++id)
                if(CPU_ISSET_S(id,bytes,mask.get())) affinity.processors.push_back(static_cast<unsigned>(id));
            const int caller=sched_getcpu();
            if(caller>=0) prefer_caller(affinity,static_cast<unsigned>(caller));
            break;
        }
        if(errno!=EINVAL || bits>std::numeric_limits<int>::max()/2) return {};
    }
#elif defined(_WIN32)
    const HANDLE process=GetCurrentProcess();
    ULONG default_count{};
    if(!GetProcessDefaultCpuSets(process,nullptr,0,&default_count) && GetLastError()!=ERROR_INSUFFICIENT_BUFFER)
        return {};
    std::vector<ULONG> defaults(default_count);
    if(default_count && !GetProcessDefaultCpuSets(process,defaults.data(),default_count,&default_count)) return {};
    defaults.resize(default_count);
    std::sort(defaults.begin(),defaults.end());

    std::vector<USHORT> groups(GetActiveProcessorGroupCount());
    USHORT group_count=static_cast<USHORT>(groups.size());
    if(!groups.empty() && GetProcessGroupAffinity(process,&group_count,groups.data()))
        groups.resize(group_count);
    else groups.clear();

    DWORD_PTR process_mask{},system_mask{};
    GROUP_AFFINITY thread_group{};
    const bool restricted=GetProcessAffinityMask(process,&process_mask,&system_mask) &&
        process_mask && process_mask!=system_mask && GetThreadGroupAffinity(GetCurrentThread(),&thread_group);
    PROCESSOR_NUMBER caller{};
    GetCurrentProcessorNumberEx(&caller);

    ULONG bytes{};
    GetSystemCpuSetInformation(nullptr,0,&bytes,process,0);
    if(!bytes) return {};
    std::vector<std::byte> records(bytes);
    if(!GetSystemCpuSetInformation(reinterpret_cast<SYSTEM_CPU_SET_INFORMATION*>(records.data()),
                                  bytes,&bytes,process,0)) return {};
    std::vector<std::pair<WORD,BYTE>> logical_cpus;
    for(std::size_t offset=0;offset+offsetof(SYSTEM_CPU_SET_INFORMATION,CpuSet)<=bytes;){
        SYSTEM_CPU_SET_INFORMATION record{};
        std::memcpy(&record,records.data()+offset,offsetof(SYSTEM_CPU_SET_INFORMATION,CpuSet));
        if(record.Size<offsetof(SYSTEM_CPU_SET_INFORMATION,CpuSet) || record.Size>bytes-offset) return {};
        const auto first=offset;
        offset+=record.Size;
        if(record.Type!=CpuSetInformation || record.Size<sizeof(record)) continue;
        std::memcpy(&record,records.data()+first,sizeof(record));
        const auto& set=record.CpuSet;
        if((set.Allocated || set.RealTime) && !set.AllocatedToTargetProcess) continue;
        if(!defaults.empty() && !std::binary_search(defaults.begin(),defaults.end(),set.Id)) continue;
        if(!groups.empty() && std::find(groups.begin(),groups.end(),set.Group)==groups.end()) continue;
        if(restricted && (set.Group!=thread_group.Group || set.LogicalProcessorIndex>=sizeof(process_mask)*8 ||
                          !(process_mask&(DWORD_PTR{1}<<set.LogicalProcessorIndex)))) continue;
        const auto logical=std::pair{set.Group,set.LogicalProcessorIndex};
        if(std::find(logical_cpus.begin(),logical_cpus.end(),logical)!=logical_cpus.end()) continue;
        logical_cpus.push_back(logical);
        affinity.processors.push_back(set.Id);
    }
    const auto current=std::find(logical_cpus.begin(),logical_cpus.end(),std::pair{caller.Group,caller.Number});
    if(current!=logical_cpus.end()) prefer_caller(affinity,affinity.processors[current-logical_cpus.begin()]);
#endif
    return affinity;
}

bool select_current_thread(const Affinity& affinity,std::size_t processor){
    if(processor>=affinity.processors.size()) return false;
    const auto id=affinity.processors[processor];
#if defined(__linux__)
    const auto bytes=affinity.mask_bytes;
    Mask mask(CPU_ALLOC(static_cast<int>(bytes*8)));
    if(!mask) return false;
    CPU_ZERO_S(bytes,mask.get());
    CPU_SET_S(static_cast<int>(id),bytes,mask.get());
    return pthread_setaffinity_np(pthread_self(),bytes,mask.get())==0;
#elif defined(_WIN32)
    const ULONG cpu_set=id;
    return SetThreadSelectedCpuSets(GetCurrentThread(),&cpu_set,1)!=FALSE;
#else
    (void)id;
    return false;
#endif
}

} // namespace eng::cpu
