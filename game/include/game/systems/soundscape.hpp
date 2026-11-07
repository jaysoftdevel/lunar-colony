#pragma once

#include "engine/audio.hpp"
#include "game/systems/sound_assets.hpp"
#include <array>
#include <future>
#include <optional>
#include <unordered_map>

namespace game::sound {

class Soundscape {
public:
    Soundscape();
    float volume() const { return volume_; }
    void set_volume(float volume);
    bool music_enabled() const { return music_enabled_; }
    void set_music_enabled(bool enabled);
    bool begin(eng::Vec3 listener,eng::Vec3 direction,eng::Vec3 up);
    void finish(float dt);
    void play(Cue cue,const eng::SoundVoice& voice);
    void loop(std::uint64_t key,Cue cue,const eng::SoundVoice& voice,std::size_t variant=0);
    std::optional<std::uint64_t> observe(std::uint64_t key,std::uint64_t value);

    eng::Vec3 feet{};
    float step_distance{};
    bool tracking_feet{},grounded{};
    float warning_cooldown{};
    float completion_cooldown{};
    float launch_seconds{};

private:
    struct Observation { std::uint64_t frame{},value{}; };
    eng::Audio::Clip clip(Cue cue,std::size_t variant);
    void update_music();
    eng::Audio audio_;
    std::future<std::vector<std::int16_t>> theme_job_;
    eng::Audio::Clip theme_clip_{};
    bool music_enabled_{true};
    bool theme_requested_{};
    std::array<std::array<eng::Audio::Clip,max_variants>,cue_count> clips_{};
    std::array<std::array<bool,max_variants>,cue_count> attempted_{};
    std::unordered_map<std::uint64_t,Observation> observations_;
    std::uint64_t frame_{};
    std::uint32_t variant_counter_{};
    float volume_{.72f};
    bool initialized_{};
};

} // namespace game::sound
