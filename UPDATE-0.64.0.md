# Lunar Colony 0.64.0 — sealed corridor connections

Requires game 0.63.0 and external JSD source 0.5.14. Engine source is unchanged.

The pressure envelope is the boundary of closed convex corridor cells. Neighboring
cells remove only their shared interior surfaces by polygon clipping. Whole
pressure triangles are no longer discarded because they touch a branch opening.
Miter rings meet through bends; junction walls, floors, roofs and docking liners
share the same volume boundary. Dead ends and short extensions remain capped.

Commissioned and unfinished cells are isolated. A blueprint cannot open a completed
host hull. Completion refreshes the boundary once, and demolition reseals surviving
hosts. A cached construction revision avoids recomputing pressure geometry each
frame. Loading older saves regenerates current pressure skins without changing
the save format. Hull visibility options affect rendering only.

Habitat and storage-dome ports sample their actual pressure hull, including
all storage sizes and free-angle connections.

Full-height rounded hatch profiles keep the clear doorway inside its pressure
liner. Curved/faceted hull contact samples determine how far the liner penetrates
the source wall. Existing crew navigation and sliding doors are retained.

Intact pressure hulls have zero passive oxygen leakage. Crew/process consumption
and deliberate air-lock operation still consume oxygen; connecting a new empty
volume still needs filling. No oxygen is manufactured by these changes.

Linux and Windows share all sources. See VALIDATION.md for performed checks.
