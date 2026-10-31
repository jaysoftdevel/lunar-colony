#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace game {

enum class MaterialResource : std::uint8_t {
    Iron = 0,
    Copper,
    Silicon,
    Plastic,
    Water,
    Food,
    Medical,
    Aluminum,
    Count
};

inline constexpr std::size_t kMaterialResourceCount = static_cast<std::size_t>(MaterialResource::Count);
inline constexpr std::size_t kMaxConstructionResourceSteps = 10;

inline constexpr std::array<MaterialResource,5> kConstructionMaterialResources{{
    MaterialResource::Iron,
    MaterialResource::Copper,
    MaterialResource::Silicon,
    MaterialResource::Plastic,
    MaterialResource::Aluminum
}};

struct ResourceBundle {
    std::int64_t iron{0};
    std::int64_t copper{0};
    std::int64_t silicon{0};
    std::int64_t plastic{0};
    std::int64_t water{0};
    std::int64_t food{0};
    std::int64_t medical{0};
    std::int64_t aluminum{0};

    std::int64_t amount(MaterialResource resource) const;
    std::int64_t& amount(MaterialResource resource);
    std::int64_t total_amount() const;
    bool empty() const;
};

struct ConstructionResourcePlan {
    std::uint8_t total_steps{1};
    std::array<ResourceBundle,kMaxConstructionResourceSteps> steps{};

    const ResourceBundle& next_step_cost(std::uint8_t completed_steps) const;
    ResourceBundle total() const;
    ResourceBundle consumed(std::uint8_t completed_steps) const;
    ResourceBundle remaining(std::uint8_t completed_steps) const;
};

class ResourceManagement {
public:
    void reset();
    std::int64_t amount(MaterialResource resource) const { return resources_.amount(resource); }
    const ResourceBundle& amounts() const { return resources_; }

    bool can_consume(const ResourceBundle& cost) const;
    bool consume(const ResourceBundle& cost,bool allow_negative);
    void add(const ResourceBundle& amount);
    void restore(const ResourceBundle& amount){ resources_=amount; }

private:
    ResourceBundle resources_{};
};

const char* material_resource_name(MaterialResource resource);
const char* material_resource_symbol(MaterialResource resource);

// Logistics package limits. Resource amounts are kilograms. The current
// testing balance applies a 10x carry/storage-per-box multiplier; visible box
// dimensions still scale with each package's fill fraction.
std::int64_t resource_package_capacity_kg(MaterialResource resource);
float resource_package_half_width(MaterialResource resource,std::int64_t amount_kg);
float resource_package_height(MaterialResource resource,std::int64_t amount_kg);

} // namespace game
