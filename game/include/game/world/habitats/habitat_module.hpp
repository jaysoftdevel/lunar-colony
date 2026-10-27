#pragma once
#include "game/world/structure_module.hpp"
#include "game/world/habitat_builder.hpp"
#include <array>

namespace game::habitats {

using HabitatBuildFn = void(*)(Builder&);

// All habitats use this one implementation. Concrete habitats differ only by
// their registered outer/inner builders and access profile.
class HabitatModule final : public StructureModule {
public:
    constexpr HabitatModule(HabitatKind kind,const char* id,StructureAccessProfile access,
                            HabitatBuildFn outer,HabitatBuildFn inner)
        : kind_(kind),id_(id),access_(access),outer_(outer),inner_(inner) {}

    HabitatKind kind() const { return kind_; }
    const char* module_id() const override { return id_; }
    StructureAccessProfile access_profile() const override { return access_; }
    HabitatMesh instantiate() const;

private:
    HabitatKind kind_;
    const char* id_;
    StructureAccessProfile access_;
    HabitatBuildFn outer_{};
    HabitatBuildFn inner_{};
};

const HabitatModule& module(HabitatKind kind);
const std::array<const HabitatModule*,(std::size_t)HabitatKind::Count>& modules();

// Outer builders: pressure hull, foundation, exterior detail, entrances.
void build_airlock_outer(Builder&);
void build_oxygen_dome_outer(Builder&);
void build_cantine_dome_outer(Builder&);
void build_bio_dome_outer(Builder&);
void build_crew_quarters_outer(Builder&);
void build_processing_facility_outer(Builder&);
void build_factory_outer(Builder&);
void build_manufactory_outer(Builder&);
void build_sick_bay_outer(Builder&);
void build_laboratory_outer(Builder&);

// Inner builders: deck/floor and all interior equipment/furniture.
void build_airlock_inner(Builder&);
void build_oxygen_dome_inner(Builder&);
void build_cantine_dome_inner(Builder&);
void build_bio_dome_inner(Builder&);
void build_crew_quarters_inner(Builder&);
void build_processing_facility_inner(Builder&);
void build_factory_inner(Builder&);
void build_manufactory_inner(Builder&);
void build_sick_bay_inner(Builder&);
void build_laboratory_inner(Builder&);

} // namespace game::habitats
