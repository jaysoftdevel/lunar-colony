#include "engine/audio.hpp"

#if defined(__linux__) || defined(_WIN32)
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <dlfcn.h>
#endif
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace eng {
namespace {
using UInt=unsigned int;
using Int=int;
using Enum=int;
using Bool=char;
using Device=void;
using Context=void;

#ifdef _WIN32
using Library=HMODULE;
Library open_audio_library(){
    return LoadLibraryExW(L"OpenAL32.dll",nullptr,LOAD_LIBRARY_SEARCH_APPLICATION_DIR|LOAD_LIBRARY_SEARCH_SYSTEM32);
}
auto library_symbol(Library library,const char* name){ return GetProcAddress(library,name); }
void close_audio_library(Library library){ FreeLibrary(library); }
#else
using Library=void*;
Library open_audio_library(){ return dlopen("libopenal.so.1",RTLD_NOW|RTLD_LOCAL); }
auto library_symbol(Library library,const char* name){ return dlsym(library,name); }
void close_audio_library(Library library){ dlclose(library); }
#endif

constexpr Enum pcm16_mono=0x1101, pcm16_stereo=0x1103, source_state=0x1010, playing=0x1012, buffer=0x1009;
constexpr Enum looping=0x1007, relative=0x0202, position=0x1004, orientation=0x100F;
constexpr Enum gain=0x100A, pitch=0x1003, rolloff=0x1021;
constexpr Enum filter_type=0x8001, lowpass=0x0001, lowpass_gain=1, lowpass_hf=2, direct_filter=0x20005;
constexpr std::size_t voice_count=32;
constexpr std::size_t loop_limit=20;

#define AUDIO_FUNCTIONS(X) \
    X(Device*,alcOpenDevice,(const char*)) X(Bool,alcCloseDevice,(Device*)) \
    X(Context*,alcCreateContext,(Device*,const Int*)) X(void,alcDestroyContext,(Context*)) \
    X(Bool,alcMakeContextCurrent,(Context*)) X(Bool,alcIsExtensionPresent,(Device*,const char*)) \
    X(Enum,alGetError,()) X(void*,alGetProcAddress,(const char*)) \
    X(void,alGenSources,(Int,UInt*)) X(void,alDeleteSources,(Int,const UInt*)) \
    X(void,alSourcei,(UInt,Enum,Int)) X(void,alSourcef,(UInt,Enum,float)) X(void,alSource3f,(UInt,Enum,float,float,float)) \
    X(void,alSourcePlay,(UInt)) X(void,alSourceStop,(UInt)) X(void,alGetSourcei,(UInt,Enum,Int*)) \
    X(void,alGenBuffers,(Int,UInt*)) X(void,alDeleteBuffers,(Int,const UInt*)) X(void,alBufferData,(UInt,Enum,const void*,Int,Int)) \
    X(void,alListener3f,(Enum,float,float,float)) X(void,alListenerfv,(Enum,const float*)) X(void,alListenerf,(Enum,float))

struct Api {
#define DECLARE(type,name,args) type (*name) args{};
    AUDIO_FUNCTIONS(DECLARE)
#undef DECLARE
    void (*gen_filters)(Int,UInt*){};
    void (*delete_filters)(Int,const UInt*){};
    void (*filter_i)(UInt,Enum,Int){};
    void (*filter_f)(UInt,Enum,float){};

    bool load(Library library){
#define LOAD(type,name,args) name=reinterpret_cast<decltype(name)>(library_symbol(library,#name)); if(!name) return false;
        AUDIO_FUNCTIONS(LOAD)
#undef LOAD
        return true;
    }

    bool load_filters(){
        gen_filters=reinterpret_cast<decltype(gen_filters)>(alGetProcAddress("alGenFilters"));
        delete_filters=reinterpret_cast<decltype(delete_filters)>(alGetProcAddress("alDeleteFilters"));
        filter_i=reinterpret_cast<decltype(filter_i)>(alGetProcAddress("alFilteri"));
        filter_f=reinterpret_cast<decltype(filter_f)>(alGetProcAddress("alFilterf"));
        return gen_filters && delete_filters && filter_i && filter_f;
    }
};
#undef AUDIO_FUNCTIONS

struct Slot {
    UInt source{};
    Audio::Clip clip{};
    std::uint64_t key{};
    float level{};
    float target{};
    std::uint8_t priority{};
    bool touched{};
};

} // namespace

struct Audio::Impl {
    Api al;
    Library library{};
    Device* device{};
    Context* context{};
    UInt filter{};
    std::array<Slot,voice_count> voices{};
    std::vector<UInt> clips;
    Vec3 listener{};
    float master{-1};
    bool attempted{},ready{};

    ~Impl(){
        if(context && al.alcMakeContextCurrent(context)){
            for(const auto& voice:voices) if(voice.source) al.alDeleteSources(1,&voice.source);
            if(!clips.empty()) al.alDeleteBuffers(static_cast<Int>(clips.size()),clips.data());
            if(filter) al.delete_filters(1,&filter);
            al.alcMakeContextCurrent(nullptr);
        }
        if(context) al.alcDestroyContext(context);
        if(device) al.alcCloseDevice(device);
        if(library) close_audio_library(library);
    }

    bool open(){
        library=open_audio_library();
        if(!library || !al.load(library)) return false;
        device=al.alcOpenDevice(nullptr);
        if(!device) return false;
        context=al.alcCreateContext(device,nullptr);
        if(!context || !al.alcMakeContextCurrent(context)) return false;
        std::array<UInt,voice_count> sources{};
        al.alGenSources(static_cast<Int>(sources.size()),sources.data());
        if(al.alGetError()!=0) return false;
        for(std::size_t i=0;i<sources.size();++i){
            voices[i].source=sources[i];
            al.alSourcef(sources[i],rolloff,0.0f);
        }
        if(al.alcIsExtensionPresent(device,"ALC_EXT_EFX") && al.load_filters()){
            al.gen_filters(1,&filter);
            if(filter){
                al.filter_i(filter,filter_type,lowpass);
                al.filter_f(filter,lowpass_gain,1.0f);
                al.filter_f(filter,lowpass_hf,0.16f);
            }
            if(al.alGetError()!=0 && filter){
                al.delete_filters(1,&filter);
                filter=0;
            }
        }
        return true;
    }

    Clip add_clip(std::span<const std::int16_t> pcm,int sample_rate,Enum format){
        if(!ready || pcm.empty() || sample_rate<8000 ||
           pcm.size_bytes()>static_cast<std::size_t>(std::numeric_limits<Int>::max())) return 0;
        UInt id{};
        al.alGenBuffers(1,&id);
        if(!id) return 0;
        al.alBufferData(id,format,pcm.data(),static_cast<Int>(pcm.size_bytes()),sample_rate);
        if(al.alGetError()!=0){ al.alDeleteBuffers(1,&id); return 0; }
        clips.push_back(id);
        return id;
    }

    void recycle(Slot& slot){
        al.alSourceStop(slot.source);
        al.alSourcei(slot.source,buffer,0);
        const UInt source=slot.source;
        slot={};
        slot.source=source;
    }

    float strength(const SoundVoice& voice) const {
        if(voice.relative) return std::clamp(voice.gain,0.0f,1.0f);
        const Vec3 offset=voice.position-listener;
        const float radius=std::max(0.1f,voice.range);
        const float t=std::max(0.0f,1.0f-dot(offset,offset)/(radius*radius));
        return std::clamp(voice.gain*t*t,0.0f,1.0f);
    }

    void configure(Slot& slot,const SoundVoice& voice,float level){
        slot.target=level;
        slot.priority=voice.priority;
        al.alSourcei(slot.source,relative,voice.relative?1:0);
        const Vec3 p=voice.relative?Vec3{}:voice.position;
        al.alSource3f(slot.source,position,p.x,p.y,p.z);
        al.alSourcef(slot.source,pitch,std::clamp(voice.pitch,0.55f,2.0f));
        if(filter) al.alSourcei(slot.source,direct_filter,voice.filtered?static_cast<Int>(filter):0);
    }

    Slot* allocate(std::uint64_t key,float level,std::uint8_t priority){
        const auto loops=std::count_if(voices.begin(),voices.end(),[](const Slot& slot){ return slot.key!=0; });
        const bool reserve=key && loops>=static_cast<std::ptrdiff_t>(loop_limit);
        Slot* weakest=nullptr;
        float score=100;
        for(auto& slot:voices){
            if(!slot.clip && !reserve) return &slot;
            if(!slot.clip || (reserve && !slot.key) || slot.touched) continue;
            const float rank=slot.priority*0.20f+std::max(slot.target,slot.level);
            if(rank<score){ score=rank; weakest=&slot; }
        }
        if(weakest && priority*0.20f+level>score){
            recycle(*weakest);
            return weakest;
        }
        return nullptr;
    }

    void start(Slot& slot,Clip clip,std::uint64_t key,const SoundVoice& voice,float level){
        slot.clip=clip;
        slot.key=key;
        slot.touched=true;
        slot.level=key?0.0f:level;
        configure(slot,voice,level);
        al.alSourcei(slot.source,buffer,static_cast<Int>(clip));
        al.alSourcei(slot.source,looping,key?1:0);
        al.alSourcef(slot.source,gain,slot.level);
        al.alSourcePlay(slot.source);
    }
};

Audio::Audio():impl_(std::make_unique<Impl>()){}
Audio::~Audio()=default;

bool Audio::initialize(){
    if(!impl_->attempted){
        impl_->attempted=true;
        impl_->ready=impl_->open();
        if(!impl_->ready) std::cerr<<"OpenAL sound output unavailable. The game will run silently.\n";
    }
    return impl_->ready;
}

Audio::Clip Audio::add_mono(std::span<const std::int16_t> pcm,int sample_rate){
    return impl_->add_clip(pcm,sample_rate,pcm16_mono);
}

Audio::Clip Audio::add_stereo(std::span<const std::int16_t> pcm,int sample_rate){
    if(pcm.size()%2!=0) return 0;
    return impl_->add_clip(pcm,sample_rate,pcm16_stereo);
}

void Audio::begin(Vec3 eye,Vec3 direction,Vec3 up,float volume){
    if(!impl_->ready) return;
    auto& al=impl_->al;
    impl_->listener=eye;
    al.alListener3f(position,eye.x,eye.y,eye.z);
    const float frame[]{direction.x,direction.y,direction.z,up.x,up.y,up.z};
    al.alListenerfv(orientation,frame);
    volume=std::clamp(volume,0.0f,1.0f);
    if(volume!=impl_->master){
        al.alListenerf(gain,volume);
        impl_->master=volume;
    }
    for(auto& slot:impl_->voices){
        slot.touched=false;
        if(!slot.clip || slot.key) continue;
        Int state{};
        al.alGetSourcei(slot.source,source_state,&state);
        if(state!=playing) impl_->recycle(slot);
    }
}

void Audio::play(Clip clip,const SoundVoice& voice){
    if(!impl_->ready || !clip) return;
    const float level=impl_->strength(voice);
    if(level<0.004f) return;
    if(auto* slot=impl_->allocate(0,level,voice.priority)) impl_->start(*slot,clip,0,voice,level);
}

void Audio::loop(std::uint64_t key,Clip clip,const SoundVoice& voice){
    if(!impl_->ready || !key || !clip) return;
    const float level=impl_->strength(voice);
    if(level<0.003f) return;
    for(auto& slot:impl_->voices){
        if(slot.key!=key) continue;
        if(slot.clip==clip){
            slot.touched=true;
            impl_->configure(slot,voice,level);
            return;
        }
        impl_->recycle(slot);
        impl_->start(slot,clip,key,voice,level);
        return;
    }
    if(auto* slot=impl_->allocate(key,level,voice.priority)) impl_->start(*slot,clip,key,voice,level);
}

void Audio::stop_loop(std::uint64_t key){
    if(!impl_->ready || !key) return;
    for(auto& slot:impl_->voices){
        if(slot.key!=key) continue;
        impl_->recycle(slot);
        return;
    }
}

void Audio::finish(float dt){
    if(!impl_->ready) return;
    const float blend=1.0f-std::exp(-std::clamp(dt,0.0f,0.2f)*12.0f);
    for(auto& slot:impl_->voices){
        if(!slot.key) continue;
        if(!slot.touched) slot.target=0.0f;
        slot.level=std::lerp(slot.level,slot.target,blend);
        if(!slot.touched && slot.level<0.002f){ impl_->recycle(slot); continue; }
        impl_->al.alSourcef(slot.source,gain,slot.level);
    }
}

} // namespace eng

#else
namespace eng {
struct Audio::Impl {};
Audio::Audio():impl_(std::make_unique<Impl>()){}
Audio::~Audio()=default;
bool Audio::initialize(){ return false; }
Audio::Clip Audio::add_mono(std::span<const std::int16_t>,int){ return 0; }
Audio::Clip Audio::add_stereo(std::span<const std::int16_t>,int){ return 0; }
void Audio::begin(Vec3,Vec3,Vec3,float){}
void Audio::play(Clip,const SoundVoice&){}
void Audio::loop(std::uint64_t,Clip,const SoundVoice&){}
void Audio::stop_loop(std::uint64_t){}
void Audio::finish(float){}
}
#endif
