#pragma once

namespace game::build {
#if defined(GAME_DEBUG_FEATURES) && GAME_DEBUG_FEATURES
inline constexpr bool debug_features=true;
#else
inline constexpr bool debug_features=false;
#endif
}
