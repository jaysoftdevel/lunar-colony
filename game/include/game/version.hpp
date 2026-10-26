#pragma once
#include <string_view>

#ifndef GAME_VERSION
#define GAME_VERSION "0.0.0-dev"
#endif

namespace game::version {
inline constexpr std::string_view current = GAME_VERSION;
}
