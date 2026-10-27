# Architecture

## Engine layer
`engine/` owns Linux/X11 windowing, raw input, GPU rendering, math primitives and immediate UI drawing. Game code uses only the public interfaces in `engine/include/engine`.

## Game layer
`game/` owns gameplay, simulation, world geometry and interaction. `GameApp` routes the high-level screen/state flow while focused systems implement placement, construction, networks, POV traversal and rendering state.

## Structural module hierarchy
All crew-accessible pressure structures share `StructureModule` and its `StructureAccessProfile`. The profile is the authoritative source for walkable deck height, crew accessibility, pressure behavior and exterior-gateway permission.

- `StructureModule`
  - `habitats::HabitatModule`
    - one registered instance per `HabitatKind`
    - each habitat has a separate `<name>.cpp` outer module and `<name>_inner.cpp` inner module
    - the outer module owns pressure hull/foundation/glazing/exterior details/entrances
    - the inner module owns the usable deck plus furniture/equipment
  - `corridor_detail::CorridorModule`
    - outer implementation builds the pressure shell and structural ribs
    - inner implementation builds the continuous walkable corridor deck

Habitat types do not duplicate common access/navigation behavior. Their registry entry supplies the concrete outer/inner builders and access profile, then `HabitatModule::instantiate()` creates the complete structure mesh.

## POV floor and access contract
`HabitatMesh::walkable_floor_y` is copied directly from the structure module and is never inferred from arbitrary triangles. The same value is used by POV floor resolution and corridor docking stairs, so the visual basement/deck and character elevation stay synchronized.

POV structure containment uses the projected union of structural shell, glazing and foundation meshes. This is important for mixed-envelope structures such as the Bio Dome. Structural borders remain solid except at legal connected doorway portals. Corridors use the deck height declared by `CorridorModule`.

## Extension direction
Add a new habitat by creating its outer and inner builder files and registering one `HabitatModule` entry. Common access, floor, connection and POV behavior should remain in the shared structural system rather than being reimplemented per habitat.
