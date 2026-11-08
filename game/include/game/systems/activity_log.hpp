#pragma once

#include <compare>
#include <cstdint>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace game {

enum class LogEvent { Started, Stopped, Warning, Resolved, Note, Incident };

const char* log_event_name(LogEvent event);
std::string log_timestamp(double seconds,bool precise);

struct ActivityLogEntry {
    double seconds{};
    LogEvent event{};
    std::string message;
};

class ActivityLog {
public:
    enum class Channel {
        Session, Lander, LanderSalvage, Trader, TraderPacking, TradeHaul, Mission, Contract,
        RoverTask, RoverAction, RoverTravel, RoverCharge, RoverProblem, RoverTraffic, RoverBattery,
        CrewTask, CrewSuit, CrewDoor, CrewAir, CrewNeeds, CrewHealth,
        HabitatBuild, ResourceBuild, CorridorBuild, UtilityBuild, WalkwayBuild,
        HabitatSupply, ResourceSupply, Production, Belt, Airlock
    };
    struct Key {
        Channel channel{};
        std::uint64_t id{};
        unsigned slot{};
        auto operator<=>(const Key&) const = default;
    };

    bool active(Key key) const { return active_.contains(key); }
    const std::vector<ActivityLogEntry>& entries() const { return entries_; }
    void reset(double now,std::string message);
    void begin_sample(double now);
    void end_sample();
    void record(double now,LogEvent event,std::string message);

    template<class Describe>
    void observe(Key key,std::uint64_t state,bool warning,const Describe& describe){
        auto [it,inserted]=active_.try_emplace(key);
        auto& active=it->second;
        if(!inserted && active.state==state){
            active.sample=sample_;
            return;
        }
        if(!inserted)stop(active);
        active={state,sample_,warning,describe()};
        entries_.push_back({elapsed_,warning?LogEvent::Warning:LogEvent::Started,active.message});
    }

private:
    struct Active {
        std::uint64_t state{},sample{};
        bool warning{};
        std::string message;
    };
    void stop(const Active& active);
    double origin_{},elapsed_{};
    std::uint64_t sample_{};
    std::map<Key,Active> active_;
    std::vector<ActivityLogEntry> entries_;
};
} // namespace game
