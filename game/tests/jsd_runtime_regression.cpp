#include "game/runtime/presentation.hpp"
#include "game/runtime/audio.hpp"
#include <jsd/audio.hpp>
#include <array>
#include <atomic>
#include <barrier>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <type_traits>
using namespace game::runtime;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){try{
    static_assert(std::is_same_v<Vec3,rf::V3> && std::is_same_v<Mat4,rf::Mat4>);
    auto& jobs=GameJobs::shared();const auto lanes=jobs.parallel_lanes();
    std::barrier occupied(static_cast<std::ptrdiff_t>(lanes));std::atomic<unsigned> arrived{};
    jobs.parallel_for(lanes,[&](std::size_t){++arrived;occupied.arrive_and_wait();});
    require(arrived==lanes,"JSD worker lane participation changed");
    std::array<std::atomic<unsigned>,1021> visits{};
    jobs.submit([&]{jobs.parallel_for_ranges(visits.size(),19,[&](auto first,auto last){for(auto i=first;i<last;++i)++visits[i];});}).get();
    for(auto& v:visits)require(v==1,"Nested JSD jobs lost or duplicated work");
    bool threw=false;try{jobs.parallel_for(64,[](auto i){if(i==7)throw std::logic_error("expected");});}catch(const std::logic_error&){threw=true;}
    require(threw && jobs.submit([]{return 60;}).get()==60,"JSD job exception recovery changed");
    jsd::AudioMixer mixer;auto clip=std::make_shared<jsd::AudioClip>();clip->samples.assign(2400,.25f);
    const auto sound=mixer.play(clip,{},1,true,false);std::array<float,512> samples;mixer.mix(samples);
    require(samples[4]>.1f,"External JSD mixer produced no PCM output");mixer.stop(sound);mixer.mix(samples);require(samples[4]==0,"External JSD stop retained audio");
    auto ramp=std::make_shared<jsd::AudioClip>();ramp->samples.resize(48000);
    for(std::size_t i=0;i<ramp->samples.size();++i)ramp->samples[i]=float(i)/48000;
    const auto moving=mixer.play(ramp,{},1,true,false);mixer.mix(samples);
    require(mixer.update(moving,{0,0,10},.5f,false),"Live voice update rejected active sound");mixer.mix(samples);
    require(samples[0]>.003f,"Live update restarted the PCM cursor");
    require(mixer.update(moving,{},0,false),"Live mute failed");
    require(samples[0]>0,"Voice update lost phase");for(int i=0;i<80;++i)mixer.mix(samples);
    require(std::abs(samples.back())<1e-6f,"Live gain did not settle to silence");
    bool invalid=false;try{mixer.update(moving,{},std::numeric_limits<float>::infinity());}catch(const std::invalid_argument&){invalid=true;}
    require(invalid,"Nonfinite live gain accepted");mixer.stop(moving);
    require(!mixer.update(moving,{},1),"Stopped sound accepted updates");
    Audio audio;require(audio.initialize(),"JSD dummy audio device did not open");
    std::array<std::int16_t,2400> pcm{};pcm.fill(4000);const auto id=audio.add_mono(pcm,48000);require(id!=0,"PCM cue did not register");
    for(int frame=0;frame<8;++frame){audio.begin({}, {0,0,-1},{0,1,0},.5f);audio.loop(1,id,{{float(frame),0,0},.4f,1,20,false,frame>3,1});audio.play(id,{});audio.finish(.016f);}
    audio.stop_loop(1);audio.begin({}, {0,0,-1},{0,1,0},0);audio.finish(.016f);
    std::cout<<"PASS external JSD math, worker scheduling, nested jobs, exceptions and audio device/mixer\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
