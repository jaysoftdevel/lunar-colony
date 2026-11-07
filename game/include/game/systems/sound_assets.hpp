#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace game::sound {

enum class Cue : std::uint8_t {
    Campus, Receiver, Cabin, Suit, Breath, Vent, Pump, Chiller, Drill, Mill, Belt, Motor, Weld,
    Exhaust, FootDust, FootDeck, BodyLand, Hatch, Seal, Equalize, Lift, Crate, Ore, Touchdown,
    Latch, Radio, Pick, Cancel, Place, Build, Success, Reject, Alarm, Count
};
inline constexpr std::size_t cue_count=static_cast<std::size_t>(Cue::Count);
inline constexpr int sample_rate=48000;
inline constexpr int theme_sample_rate=24000;
inline constexpr std::size_t max_variants=4;

std::size_t variant_count(Cue cue);
std::vector<std::int16_t> synthesize(Cue cue,std::uint32_t variant);
std::vector<std::int16_t> synthesize_theme();

} // namespace game::sound
