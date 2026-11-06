#pragma once
#include "engine/engine.hpp"
#include <array>
#include <cstdint>
#include <functional>
#include <vector>
namespace game {
inline constexpr std::size_t kTradeGoodCount=10;
inline constexpr std::int64_t kTraderCapacityKg=20000;
enum class TradePhase:std::uint8_t {Idle,Approach,Docked,Loading,Departing,Complete};
enum class TradeGood:std::uint8_t {Iron,Copper,Silicon,Plastic,Water,Food,Medical,Aluminum,Oxygen,Hydrogen};
using TradeManifest=std::array<std::int64_t,kTradeGoodCount>;
struct TradeLot {TradeGood good{TradeGood::Iron};std::int64_t remaining{0};eng::Vec3 position{};};
struct TradeHaul {
    std::uint8_t robot{0},phase{0}; // 0 collect, 1 deliver, 2 clear receiving dock
    std::uint32_t lot{0};std::int64_t cargo{0};
    eng::Vec3 service{},exit{};bool service_valid{false};
};
struct OxygenExportStock {std::uint64_t source_sequence{0};double kg{0};};
struct TraderState {
    TradePhase phase{TradePhase::Idle};float elapsed{0};
    std::uint64_t pad_sequence{0};eng::Vec3 dock{};
    TradeManifest requested{},loaded{};
    std::int64_t sale_value_eur{0},last_payment_eur{0};
    bool oxygen_packaging{false};
    std::vector<OxygenExportStock> oxygen{};
    std::vector<TradeLot> lots{};std::vector<TradeHaul> hauls{};
    bool active()const{return phase!=TradePhase::Idle&&phase!=TradePhase::Complete;}
};
const char* trade_good_name(TradeGood good);
std::int64_t trade_price_eur(TradeGood good);
const char* trade_phase_name(TradePhase phase);
}
