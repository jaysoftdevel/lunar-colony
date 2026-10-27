#pragma once

namespace game {

// Shared behavioural contract for anything that participates in the structural
// access network. Geometry is provided by the concrete module, while access,
// pressure and POV code consume this common profile.
struct StructureAccessProfile {
    float walkable_floor_y{0.20f};
    bool crew_accessible{true};
    bool pressurized{true};
    bool exterior_gateway{false};
};

class StructureModule {
public:
    virtual ~StructureModule() = default;
    virtual const char* module_id() const = 0;
    virtual StructureAccessProfile access_profile() const = 0;
};

} // namespace game
