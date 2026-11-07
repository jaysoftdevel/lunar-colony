#pragma once

#include "engine/math.hpp"
#include <cstdint>
#include <memory>
#include <span>

namespace eng {

struct SoundVoice {
    Vec3 position{};
    float gain{1.0f};
    float pitch{1.0f};
    float range{20.0f};
    bool relative{};
    bool filtered{};
    std::uint8_t priority{1};
};

// Spatial PCM playback. Clip zero denotes an unavailable sound. Loops use
// stable keys and fade out when they are no longer submitted by the game.
class Audio {
public:
    using Clip=unsigned int;

    Audio();
    ~Audio();
    bool initialize();
    Clip add_mono(std::span<const std::int16_t> pcm,int sample_rate);
    Clip add_stereo(std::span<const std::int16_t> pcm,int sample_rate);
    void begin(Vec3 position,Vec3 direction,Vec3 up,float volume);
    void play(Clip clip,const SoundVoice& voice);
    void loop(std::uint64_t key,Clip clip,const SoundVoice& voice);
    void stop_loop(std::uint64_t key);
    void finish(float dt);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace eng
