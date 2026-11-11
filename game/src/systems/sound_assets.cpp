#include "game/systems/sound_assets.hpp"
#include "engine/engine.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace game::sound {
namespace {

constexpr float tau=6.28318530718f;
struct Recipe { float seconds,peak; bool loop; std::size_t variants; };
constexpr std::array<Recipe,cue_count> recipes{{
    {4.8f,.66f,true,1}, {4.2f,.50f,true,1}, {4.0f,.64f,true,1}, {3.7f,.61f,true,1},
    {2.9f,.69f,true,1}, {3.7f,.70f,true,1}, {3.2f,.73f,true,1}, {3.6f,.74f,true,1},
    {2.9f,.80f,true,1}, {3.4f,.77f,true,1}, {2.7f,.69f,true,1}, {2.8f,.72f,true,2},
    {1.8f,.77f,true,1}, {4.1f,.86f,true,1},
    {.33f,.78f,false,4}, {.39f,.79f,false,4}, {.61f,.83f,false,2},
    {.61f,.76f,false,2}, {.46f,.76f,false,2}, {1.46f,.74f,true,2},
    {.49f,.73f,false,2}, {.53f,.83f,false,3}, {.32f,.70f,false,3},
    {1.19f,.86f,false,2}, {.61f,.77f,false,1}, {.49f,.69f,false,3},
    {.105f,.61f,false,3}, {.15f,.64f,false,1}, {.31f,.73f,false,2}, {.68f,.76f,false,1},
    {.49f,.73f,false,1}, {.27f,.72f,false,1}, {.76f,.76f,false,1}
}};

float sine(float frequency,float time){ return std::sin(tau*frequency*time); }
float attack(float time,float length){
    const float t=std::clamp(time/length,0.0f,1.0f);
    return t*t*(3-2*t);
}
float fall(float t,float decay){ return t<0?0.0f:std::exp(-decay*t); }
float burst(float t,float start,float onset,float decay){
    const float local=t-start;
    return attack(local,onset)*fall(local,decay);
}

struct Bands { float bass,body,mid,treble,air; };
class Noise {
public:
    explicit Noise(std::uint32_t seed):seed_(seed?seed:1u){}
    Bands sample(){
        const float white=random();
        for(std::size_t i=0;i<stages_.size();++i) stages_[i]+=coefficients_[i]*(white-stages_[i]);
        return {stages_[0]*4.2f,(stages_[1]-stages_[0])*3.4f,
            (stages_[2]-stages_[1])*2.5f,(stages_[3]-stages_[2])*1.7f,white-stages_[3]};
    }
    float random(){
        seed_^=seed_<<13;
        seed_^=seed_>>17;
        seed_^=seed_<<5;
        return float(seed_>>8)*(2.0f/16777215.0f)-1.0f;
    }
private:
    std::uint32_t seed_;
    std::array<float,4> stages_{};
    // One-pole coefficients for 75 Hz, 340 Hz, 1.35 kHz and 4.8 kHz at 48 kHz.
    static constexpr std::array<float,4> coefficients_{{.00977f,.04353f,.16198f,.46651f}};
};

float resonance(float t,float f,float decay){
    return t<0?0.0f:fall(t,decay)*(0.72f*sine(f,t)+0.21f*sine(f*1.53f,t)+0.09f*sine(f*2.37f,t));
}
float servo(float t,float carrier,float rate){
    const float phase=carrier*t+0.52f*std::sin(tau*rate*t);
    return std::sin(tau*phase)+0.26f*std::sin(2*tau*phase);
}
float grit(float t,float seed){
    const float hits=0.5f+0.5f*sine(73.2f,t+0.25f*seed)*sine(96.3f,t-0.1f*seed);
    return 0.32f+0.68f*hits*hits;
}
float chime(float t,float start,float frequency,float length){
    const float x=t-start;
    return attack(x,.008f)*(1.0f-attack(x-length*.66f,length*.34f))*
        (0.85f*sine(frequency,x)+0.11f*sine(frequency*2,x)+0.04f*sine(frequency*3.01f,x));
}

float synth(Cue cue,float t,float u,float variant,const Bands& n){
    const float slow=sine(.33f,t+variant*.37f);
    const float wobble=sine(2.1f,t);
    switch(cue){
        case Cue::Campus: {
            const float gust=.44f+.16f*slow+.11f*sine(.77f,t)+.07f*sine(1.4f,t);
            const float exterior=n.bass*.38f+n.body*.25f+n.mid*.12f+n.treble*.08f;
            const float equipment=.014f*sine(48,t)+.009f*sine(97.4f,t)+.006f*sine(143,t);
            const float bird=burst(t,1.22f,.015f,12)*.026f*sine(2510.0f+360.0f*sine(6,t),t);
            return exterior*gust+equipment+bird;
        }
        case Cue::Receiver:
            return (n.body*.20f+n.mid*.27f+n.treble*.09f)*(.7f+.13f*slow)+.009f*sine(62,t);
        case Cue::Cabin:
            return (n.bass*.23f+n.body*.22f+n.mid*.14f)*(.78f+.11f*slow)+
                .085f*sine(59.8f,t)+.043f*sine(119.6f,t)+.023f*servo(t,227,3.5f);
        case Cue::Suit:
            return (.32f*n.body+.15f*n.mid+.027f*n.treble)*(.85f+.09f*wobble)+
                .074f*servo(t,163.4f,7.1f)+.024f*sine(325,t);
        case Cue::Breath: {
            const float inhale=burst(t,.16f,.38f,1.8f);
            const float exhale=burst(t,1.21f,.18f,2.7f);
            return inhale*(n.mid*.58f+n.treble*.18f)+exhale*(n.body*.34f+n.mid*.37f+n.treble*.08f);
        }
        case Cue::Vent:
            return (.26f*n.body+.20f*n.mid+.07f*n.treble)*(.79f+.13f*sine(12.7f,t))+
                .061f*sine(82,t)+.057f*sine(167.2f,t);
        case Cue::Pump:
            return (.23f*n.body+.19f*n.mid)*(.66f+.26f*sine(5.8f,t))+
                .15f*servo(t,74.6f,5.8f)+.045f*sine(150.4f,t)+.031f*sine(224,t);
        case Cue::Chiller:
            return (.19f*n.bass+.20f*n.body+.05f*n.mid)*(.74f+.15f*slow)+
                .14f*servo(t,44.1f,2.3f)+.05f*sine(89,t)+.03f*sine(172,t);
        case Cue::Drill: {
            const float tooth=.50f+.44f*sine(17.7f,t+0.012f*slow);
            return (.33f*n.bass+.34f*n.body+.23f*n.mid+.055f*n.treble)*tooth+
                .14f*servo(t,53.2f,3.6f)+.064f*sine(161.2f,t);
        }
        case Cue::Mill:
            return (.24f*n.bass+.25f*n.body+.14f*n.mid+.08f*n.treble)*(.64f+.23f*sine(23.1f,t))+
                .13f*servo(t,92,7.3f)+.055f*sine(184.8f,t)+.038f*sine(377,t);
        case Cue::Belt: {
            const float roller=std::pow(.5f+.5f*sine(8.0f,t+0.005f*slow),7.0f);
            return (.15f*n.body+.16f*n.mid+.11f*n.treble)*(.52f+.51f*roller)+
                roller*.13f*servo(t,118.4f,8)+.06f*sine(68.7f,t);
        }
        case Cue::Motor: {
            const float f=99.0f+22.0f*variant;
            const float tread=.62f+.24f*sine(24.4f,t)+.10f*sine(15.8f,t);
            return (.29f*n.body+.22f*n.mid+.07f*n.treble)*tread+
                .19f*servo(t,f,6.5f)+.044f*sine(2.19f*f,t)+.045f*sine(43.1f,t);
        }
        case Cue::Weld: {
            const float sparks=std::pow(.5f+.5f*sine(42.7f,t)*sine(56.3f,t),5);
            return (.19f*n.mid+.20f*n.treble+.33f*n.air)*(.22f+.78f*sparks)+
                .075f*sine(120,t)+.036f*sine(361.3f,t);
        }
        case Cue::Exhaust: {
            const float buffeting=.74f+.19f*slow+.15f*sine(6.8f,t)+.07f*sine(17.8f,t);
            return (.61f*n.bass+.37f*n.body+.16f*n.mid+.10f*n.treble)*buffeting+
                .074f*servo(t,33,2.2f)+.042f*sine(62,t);
        }
        case Cue::FootDust: {
            const float crush=burst(t,.005f,.018f,13.8f)*grit(t,variant);
            const float scuff=burst(t,.076f+.024f*variant,.028f,20);
            return crush*(.37f*n.bass+.44f*n.body+.36f*n.mid+.25f*n.treble)+
                scuff*(.29f*n.mid+.29f*n.treble+.08f*n.air)+.22f*resonance(t-.010f,68+11*variant,24);
        }
        case Cue::FootDeck: {
            const float thump=burst(t,.009f,.007f,21);
            const float scrape=burst(t,.042f,.023f,28);
            return thump*(.47f*n.body+.19f*n.mid)+scrape*(.12f*n.mid+.19f*n.treble)+
                .36f*resonance(t-.009f,101+14*variant,24)+.15f*resonance(t-.021f,412-19*variant,32);
        }
        case Cue::BodyLand: {
            const float hit=burst(t,.008f,.009f,17);
            return hit*(.57f*n.bass+.29f*n.body+.16f*n.mid)+
                .40f*resonance(t-.012f,53+12*variant,15)+.20f*resonance(t-.037f,132,21);
        }
        case Cue::Hatch: {
            const float motion=attack(t,.036f)*(1.0f-attack(t-.36f,.12f));
            const float mechanical=motion*(.13f*servo(t,127+26*variant,7.3f)+.25f*n.body+.16f*n.mid);
            return mechanical+.22f*resonance(t-.019f,277,24)+.21f*resonance(t-.435f,173,30)+
                .25f*burst(t,.40f,.015f,19)*(n.mid+n.treble*.46f);
        }
        case Cue::Seal:
            return .51f*resonance(t-.012f,111+11*variant,22)+.15f*resonance(t-.024f,346,33)+
                burst(t,.004f,.012f,19)*(.43f*n.body+.21f*n.mid)+burst(t,.08f,.022f,15)*.29f*n.treble;
        case Cue::Equalize:
            return (.15f*n.body+.41f*n.mid+.31f*n.treble+.055f*n.air)*(.80f+.16f*wobble)+
                .072f*servo(t,variant>.2f?130.0f:91.0f,5.2f);
        case Cue::Lift: {
            const float moving=burst(t,.06f,.075f,5);
            return .28f*resonance(t-.014f,245,23)+moving*(.17f*n.mid+.12f*n.body+
                .21f*sine(111.0f+103.0f*t,t))+.15f*resonance(t-.324f,376,24);
        }
        case Cue::Crate:
            return .47f*resonance(t-.014f,68+10*variant,17)+.20f*resonance(t-.031f,209+33*variant,22)+
                burst(t,.013f,.009f,18)*(.50f*n.body+.32f*n.mid+.14f*n.treble)+
                .20f*resonance(t-.114f,164,25);
        case Cue::Ore:
            return burst(t,.012f,.011f,20)*(.43f*n.body+.28f*n.mid+.33f*n.treble)+
                burst(t,.08f,.010f,29)*(.23f*n.mid+.21f*n.treble)+
                .28f*resonance(t-.018f,145+32*variant,25);
        case Cue::Touchdown:
            return burst(t,.017f,.025f,8)*(.71f*n.bass+.35f*n.body+.15f*n.mid)+
                .40f*resonance(t-.020f,34+7*variant,7)+.19f*resonance(t-.118f,82,10)+
                .22f*burst(t,.21f,.045f,8)*n.body;
        case Cue::Latch:
            return .32f*resonance(t-.015f,189,18)+.27f*resonance(t-.155f,123,20)+
                burst(t,.145f,.011f,16)*(.38f*n.body+.30f*n.mid)+chime(t,.35f,821,.13f)*.22f;
        case Cue::Radio: {
            const float squelch=burst(t,.006f,.010f,28)+burst(t,.366f,.006f,33);
            const float carrier=chime(t,.075f,941+107*variant,.11f)+.71f*chime(t,.212f,615+128*variant,.12f);
            return .26f*carrier+squelch*(.30f*n.mid+.58f*n.treble+.15f*n.air);
        }
        case Cue::Pick:
            return .38f*resonance(t-.003f,631+83*variant,73)+.32f*resonance(t-.005f,1290+125*variant,100)+
                .23f*n.treble*burst(t,.004f,.002f,99);
        case Cue::Cancel: return .38f*chime(t,.008f,517,.065f)+.45f*chime(t,.046f,352,.085f);
        case Cue::Place:
            return .32f*resonance(t-.008f,142,30)+.35f*chime(t,.018f,582,.095f)+.29f*chime(t,.092f,881+59*variant,.14f);
        case Cue::Build:
            return .31f*resonance(t-.01f,154,15)+.35f*chime(t,.034f,472,.18f)+
                .31f*chime(t,.13f,711,.23f)+.31f*chime(t,.24f,950,.33f);
        case Cue::Success:
            return .37f*chime(t,.01f,532,.16f)+.38f*chime(t,.09f,798,.19f)+.34f*chime(t,.20f,1064,.25f);
        case Cue::Reject:
            return .50f*chime(t,.01f,219,.116f)+.36f*chime(t,.104f,169,.13f);
        case Cue::Alarm:
            return .41f*chime(t,.011f,731,.17f)+.40f*chime(t,.208f,731,.17f)+.37f*chime(t,.432f,489,.25f);
        case Cue::Count: break;
    }
    (void)u;
    return 0;
}

} // namespace

std::size_t variant_count(Cue cue){
    const auto index=static_cast<std::size_t>(cue);
    return index<recipes.size()?recipes[index].variants:0;
}

std::vector<std::int16_t> synthesize(Cue cue,std::uint32_t variant){
    const auto index=static_cast<std::size_t>(cue);
    if(index>=recipes.size()) return {};
    const auto& recipe=recipes[index];
    const auto frames=static_cast<std::size_t>(recipe.seconds*sample_rate);
    const std::size_t seam=recipe.loop?sample_rate*72/1000:0;
    Noise noise(static_cast<std::uint32_t>(index+1)*0x721dce81u^(variant+1)*0x8a7fb5a5u);
    const float variation=float(variant)/max_variants;
    std::vector<float> raw(frames+seam);
    for(std::size_t i=0;i<raw.size();++i){
        const float t=float(i)/sample_rate;
        raw[i]=synth(cue,t,t/recipe.seconds,variation,noise.sample());
    }

    const bool reflective=cue==Cue::FootDeck || cue==Cue::Hatch || cue==Cue::Seal || cue==Cue::Crate || cue==Cue::Touchdown;
    std::vector<float> mixed(frames);
    float peak=0;
    for(std::size_t i=0;i<frames;++i){
        float v=raw[i];
        if(i<seam) v=std::lerp(raw[frames+i],v,attack(float(i),float(seam)));
        if(!recipe.loop){
            const float t=float(i)/sample_rate;
            if(reflective){
                for(const auto delay:{336u,887u,1661u}) if(i>=delay) v+=raw[i-delay]*(delay<400?.15f:delay<1000?.09f:.046f);
            }
            v*=attack(t,.0025f)*(1.0f-attack(t-recipe.seconds+.034f,.034f));
        }
        v=std::tanh(v*1.12f);
        mixed[i]=v;
        peak=std::max(peak,std::abs(v));
    }

    std::vector<std::int16_t> pcm(frames);
    const float scale=32767.0f*recipe.peak/std::max(peak,.001f);
    for(std::size_t i=0;i<frames;++i) pcm[i]=static_cast<std::int16_t>(std::lround(mixed[i]*scale));
    return pcm;
}

std::vector<std::int16_t> synthesize_theme(){
    // Eight slow chords over a low pad and a sparse, soft upper motif.
    // The end of the final chord overlaps the beginning of the first one.
    constexpr std::array<std::array<int,6>,8> chords{{
        {38,50,53,57,60,64}, {34,46,53,57,60,62}, {41,48,53,57,60,64}, {36,48,55,60,62,67},
        {38,50,53,57,60,64}, {43,50,55,58,62,65}, {34,46,53,57,60,62}, {45,52,55,57,61,64}
    }};
    constexpr std::array<int,8> motif{69,72,76,74,69,70,65,64};
    constexpr std::array<float,6> levels{.19f,.11f,.10f,.085f,.064f,.055f};
    constexpr int rate=theme_sample_rate;
    constexpr int chord_frames=7*rate, overlap=rate*3/2;
    constexpr int frame_count=int(chords.size())*chord_frames;
    std::vector<std::array<float,2>> dry(frame_count);
    const auto frequency=[](int midi){ return 440.0*std::exp2((midi-69)/12.0); };
    const auto oscillator=[](double phase){
        constexpr double turn=6.283185307179586;
        return static_cast<float>(std::sin(turn*phase));
    };

    auto& pool=eng::TaskPool::shared();
    // Each job owns its samples, including the portions that wrap at the seam.
    // Mix chords in their original order within each range.
    pool.parallel_for_ranges(dry.size(),8192,[&](std::size_t first,std::size_t last){
        for(std::size_t chord=0;chord<chords.size();++chord){
            std::array<double,6> frequencies{};
            for(std::size_t i=0;i<frequencies.size();++i) frequencies[i]=frequency(chords[chord][i]);
            const double melody=frequency(motif[chord]);
            const int start=int(chord)*chord_frames;
            const auto add_sample=[&](std::array<float,2>& sample,int frame){
                const double t=static_cast<double>(frame)/rate;
                const float local=float(frame-start)/rate;
                const float weight=attack(local+1.5f,3.0f)*(1.0f-attack(local-5.5f,3.0f));
                const float drift=oscillator(0.071*t+0.17*chord);
                float center{},spread{};
                for(std::size_t i=0;i<frequencies.size();++i){
                    const double phase=frequencies[i]*t;
                    center+=levels[i]*(.79f*oscillator(phase)+
                        (.15f+.05f*drift)*oscillator(2*phase+0.08*drift)+.045f*oscillator(3*phase+0.13));
                    if(i>0)
                        spread+=levels[i]*.28f*oscillator(phase*1.0012+0.24*i+0.055*drift);
                }
                const float note_time=local-1.65f;
                const float note_weight=attack(note_time,1.25f)*fall(note_time,.52f)*
                    (1.0f-attack(local-5.1f,1.65f));
                const float air=note_weight*(.10f*oscillator(melody*t)+.036f*oscillator(melody*t*2.0+0.1));
                sample[0]+=weight*(center+spread)+air;
                sample[1]+=weight*(center-spread)+air*.83f;
            };
            for(int loop:{-frame_count,0,frame_count}){
                const int begin=std::max(int(first),start-overlap-loop);
                const int end=std::min(int(last),start+chord_frames+overlap-loop);
                for(int i=begin;i<end;++i) add_sample(dry[i],i+loop);
            }
        }
    });

    // Circular cross-channel delays give the pad depth without a seam.
    std::vector<std::int16_t> pcm(static_cast<std::size_t>(frame_count)*2);
    pool.parallel_for_ranges(dry.size(),8192,[&](std::size_t first,std::size_t last){
        for(std::size_t i=first;i<last;++i){
            const auto& early=dry[(i+frame_count-rate*231/1000)%frame_count];
            const auto& late=dry[(i+frame_count-rate*593/1000)%frame_count];
            const auto& tail=dry[(i+frame_count-rate*971/1000)%frame_count];
            for(std::size_t channel=0;channel<2;++channel){
                const float mixed=dry[i][channel]+.21f*early[1-channel]+.15f*late[channel]+.09f*tail[1-channel];
                pcm[i*2+channel]=static_cast<std::int16_t>(std::lround(24576.f*std::tanh(1.40f*mixed)));
            }
        }
    });
    return pcm;
}

} // namespace game::sound
