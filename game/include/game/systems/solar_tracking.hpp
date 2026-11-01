#pragma once
#include "engine/engine.hpp"
#include <algorithm>
#include <cmath>

namespace game::solar_tracking {

// The game clock uses a 24 h solar day. For a believable sky path without
// introducing seasons/geographic configuration yet, model an equinox-like sun
// at 30 degrees north latitude. X is east/west, +Y is up and Z is north/south.
inline constexpr float kReferenceLatitudeDegrees=30.0f;
inline constexpr float kHorizonTrackingRampDegrees=2.0f;
inline constexpr float kStarReferenceDistanceMetres=1500.0f;

struct SolarPose {
    eng::Vec3 direction{};       // unit vector from the playfield toward the star
    eng::Vec3 position{};        // future render position relative to playfield centre
    float azimuth_degrees{0.0f}; // 0=N, 90=E, 180=S, 270=W
    float elevation_degrees{-90.0f};
    float irradiance_fraction{0.0f};
    bool above_horizon{false};
};

inline SolarPose pose(float hour){
    constexpr float pi=3.14159265358979323846f;
    const float wrapped=std::fmod(std::fmod(hour,24.0f)+24.0f,24.0f);
    const float latitude=kReferenceLatitudeDegrees*pi/180.0f;
    const float hour_angle=(wrapped-12.0f)*15.0f*pi/180.0f;

    // Equinox declination = 0. This yields sunrise near 06:00, solar noon at
    // 12:00 and sunset near 18:00 while still providing a true 3D sky vector.
    eng::Vec3 d{
        -std::sin(hour_angle),
        std::cos(latitude)*std::cos(hour_angle),
        -std::sin(latitude)*std::cos(hour_angle)
    };
    d=eng::normalize(d);

    SolarPose out{};
    out.direction=d;
    out.position=d*kStarReferenceDistanceMetres;
    out.elevation_degrees=std::asin(std::clamp(d.y,-1.0f,1.0f))*180.0f/pi;
    out.azimuth_degrees=std::fmod(std::atan2(d.x,d.z)*180.0f/pi+360.0f,360.0f);
    out.above_horizon=d.y>0.0f;

    // A two-axis tracker keeps panel incidence at zero. In the airless lunar
    // setting irradiance is therefore essentially flat while the star is well
    // above the horizon; only a narrow horizon ramp represents local terrain,
    // the finite star disc and tracker travel limits.
    if(out.above_horizon){
        out.irradiance_fraction=std::clamp(out.elevation_degrees/kHorizonTrackingRampDegrees,0.0f,1.0f);
    }
    return out;
}

} // namespace game::solar_tracking
