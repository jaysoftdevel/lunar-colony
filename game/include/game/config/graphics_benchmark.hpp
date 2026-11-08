#pragma once
#include "engine/graphics_settings.hpp"
#include <array>
#include <cstddef>
#include <span>

namespace game {

enum class BenchmarkProfile { High, Balanced, Performance, Classic };
enum class BenchmarkScene { Orbit, Surface, Campus };
enum class BenchmarkStatus { Idle, Running, Complete, Cancelled, Resized, Failed };

eng::GraphicsSettings graphics_profile_settings(BenchmarkProfile profile,const eng::GraphicsSettings& current,
                                                const eng::GraphicsCapabilities& caps);

struct BenchmarkStats {
    double mean_ms{};
    double p90_ms{};
};

struct BenchmarkTrial {
    BenchmarkProfile profile{};
    eng::GraphicsSettings settings{};
    std::array<BenchmarkStats,3> scenes{};
    bool available{true};
    double worst_p90() const;
};

class GraphicsBenchmark {
public:
    void start(const eng::GraphicsSettings& current,const eng::GraphicsCapabilities& caps,int width,int height,int target);
    void record(double milliseconds,bool rendered_profile);
    void stop(BenchmarkStatus reason);
    bool running() const { return status_==BenchmarkStatus::Running; }
    BenchmarkStatus status() const { return status_; }
    bool same_size(int width,int height) const { return width==width_ && height==height_; }
    int target_fps() const { return target_fps_; }
    void set_target_fps(int target);
    int scene_count() const { return scene_count_; }
    BenchmarkScene scene() const { return static_cast<BenchmarkScene>(scene_); }
    const BenchmarkTrial& current() const { return trials_[trial_]; }
    std::span<const BenchmarkTrial> trials() const { return {trials_.data(),trial_count_}; }
    const BenchmarkTrial* recommendation() const;
    float progress() const;

private:
    void advance();
    std::array<BenchmarkTrial,4> trials_{};
    std::array<double,24> samples_{};
    std::size_t trial_count_{},trial_{},sample_count_{};
    int scene_{},scene_count_{2},width_{},height_{},target_fps_{60};
    int warm_frames_{},invalid_frames_{};
    double warm_ms_{},sample_ms_{};
    BenchmarkStatus status_{BenchmarkStatus::Idle};
};

} // namespace game
