#include "game/runtime/audio.hpp"
#include <jsd/audio.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <tuple>
#include <unordered_map>

namespace game::runtime {
struct Audio::Impl {
    jsd::AudioMixer mixer;
    std::unique_ptr<jsd::AudioDevice> device;
    std::vector<std::shared_ptr<jsd::AudioClip>> clips{nullptr};
    std::map<std::tuple<Clip,int,bool>,std::shared_ptr<jsd::AudioClip>> prepared;
    struct Loop {
        jsd::Sound sound{};
        SoundVoice voice{};
        std::shared_ptr<jsd::AudioClip> source;
        double phase{};
        std::chrono::steady_clock::time_point started;
        std::uint64_t frame{};
        float gain{};
        Vec3 location{};
    };
    std::unordered_map<std::uint64_t,Loop> loops;
    std::uint64_t frame{};
    Vec3 listener{};
    float volume{};
    bool attempted{};
    std::shared_ptr<jsd::AudioClip> clip(Clip id,const SoundVoice& voice){
        if(!id || id>=clips.size())return {};
        const int pitch=int(std::lround(std::clamp(voice.pitch,.1f,4.f)*20));
        const auto key=std::tuple{id,pitch,voice.filtered};
        if(const auto found=prepared.find(key);found!=prepared.end())return found->second;
        auto result=std::make_shared<jsd::AudioClip>(*clips[id]);
        result->sampleRate=std::max(1u,unsigned(result->sampleRate*(pitch/20.f)));
        if(voice.filtered){
            float previous[2]{};
            for(std::size_t i=0;i<result->samples.size();++i){
                auto& filtered=previous[i%result->channels];filtered+=(result->samples[i]-filtered)*.22f;result->samples[i]=filtered;
            }
        }
        if(prepared.size()>256)prepared.clear();
        prepared.emplace(key,result);return result;
    }
    Vec3 position(const SoundVoice& voice)const{
        if(voice.relative)return listener;
        // JSD's native inverse-square mixer uses 5 m as its reference range.
        return listener+(voice.position-listener)*(5.f/std::max(1.f,voice.range));
    }
    Clip add(std::span<const std::int16_t> samples,int rate,unsigned channels){
        if(samples.empty() || rate<8000 || rate>192000 || samples.size()%channels)return 0;
        auto clip=std::make_shared<jsd::AudioClip>();clip->sampleRate=unsigned(rate);clip->channels=channels;clip->samples.reserve(samples.size());
        for(auto sample:samples)clip->samples.push_back(sample/32768.f);
        clips.push_back(std::move(clip));return static_cast<Clip>(clips.size()-1);
    }
    jsd::Sound play(std::shared_ptr<jsd::AudioClip> clip,Vec3 position,float gain,bool loop,bool spatial){
        try{return mixer.play(std::move(clip),position,gain,loop,spatial);}
        catch(const std::runtime_error&){return {};}// The engine's 128-voice budget drops excess effects.
    }
};
Audio::Audio():impl_(std::make_unique<Impl>()){}
Audio::~Audio()=default;
bool Audio::initialize(){
    auto& p=*impl_;if(p.attempted)return bool(p.device);p.attempted=true;
    try{p.device=std::make_unique<jsd::AudioDevice>(p.mixer);return true;}
    catch(const std::exception& e){std::cerr<<"JSD audio unavailable: "<<e.what()<<'\n';return false;}
}
Audio::Clip Audio::add_mono(std::span<const std::int16_t> pcm,int rate){return impl_->add(pcm,rate,1);}
Audio::Clip Audio::add_stereo(std::span<const std::int16_t> pcm,int rate){return impl_->add(pcm,rate,2);}
void Audio::begin(Vec3 position,Vec3 direction,Vec3 up,float volume){
    auto& p=*impl_;++p.frame;p.listener=position;p.volume=std::clamp(volume,0.f,1.f);
    auto right=normalize(cross(direction,up));if(length(right)<.001f)right={1,0,0};p.mixer.setListener(position,right);
}
void Audio::play(Clip id,const SoundVoice& voice){
    auto& p=*impl_;if(!p.device || p.volume<=0)return;
    if(auto clip=p.clip(id,voice))p.play(clip,p.position(voice),std::max(0.f,voice.gain)*p.volume,false,!voice.relative);
}
void Audio::loop(std::uint64_t key,Clip id,const SoundVoice& voice){
    auto& p=*impl_;if(!p.device)return;
    auto clip=p.clip(id,voice);if(!clip)return;
    auto& loop=p.loops[key];loop.frame=p.frame;
    const auto location=p.position(voice);const float gain=std::max(0.f,voice.gain)*p.volume;
    const bool changed=loop.source!=clip || length(location-loop.location)>.25f || std::abs(gain-loop.gain)>.02f || voice.relative!=loop.voice.relative;
    if(loop.sound.id && !changed)return;
    const auto now=std::chrono::steady_clock::now();
    if(loop.sound.id){
        loop.phase+=std::chrono::duration<double>(now-loop.started).count()*loop.source->sampleRate;
        p.mixer.stop(loop.sound);
    }
    const auto frames=clip->samples.size()/clip->channels;
    loop.phase=std::fmod(loop.phase,double(frames));
    // Rebase moving loops at their current PCM phase through the public JSD API.
    auto rebased=clip;
    const auto offset=std::size_t(loop.phase)*clip->channels;
    if(offset){
        rebased=std::make_shared<jsd::AudioClip>(*clip);
        std::rotate(rebased->samples.begin(),rebased->samples.begin()+offset,rebased->samples.end());
    }
    loop.sound=p.play(rebased,location,gain,true,!voice.relative);loop.source=clip;loop.started=now;
    loop.location=location;loop.gain=gain;loop.voice=voice;
}
void Audio::stop_loop(std::uint64_t key){
    auto& p=*impl_;if(auto it=p.loops.find(key);it!=p.loops.end()){p.mixer.stop(it->second.sound);p.loops.erase(it);}
}
void Audio::finish(float){
    auto& p=*impl_;for(auto it=p.loops.begin();it!=p.loops.end();){
        if(it->second.frame!=p.frame){p.mixer.stop(it->second.sound);it=p.loops.erase(it);}else ++it;
    }
}
}
