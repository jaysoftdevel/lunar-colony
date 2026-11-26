# Lunar Base v0.90.0

Baseline: v0.89.0. JSD engine 0.5.22 and save schema 56 unchanged.

- Restore the exact v0.87.0 habitat model generator and structural reinforcement. All ten original oval/elliptic habitat hulls, foundations, floors and airlock partitions return. The storage dome remains unchanged.
- Restore matching interior equipment alignment and placement, FPV enclosure mathematics, and airlock hull contact. Preserve newer corridor crown and frame geometry, deck/stair alignment, Earth, ground reticle, lander rotation and boarding support.
- During save load, reproject generated docking interfaces and attempt to fit installed machines back into the restored geometry for v0.88.x/v0.89.x saves. If no legal bay is found, existing unit data is retained rather than deleted. Back up saves before loading.
- Enable habitat and facility blueprint placement once cargo is unloaded and the ramp opens. The placement catalog, size selection, rotation and shift-repeat controls remain available during robot deployment. Landing and stash selection retain exclusive input. Existing landing apron, stash and rover escape checks still apply.
- To avoid obstruction of incoming robots, corridor/utility/walkway placement and construction work wait until normal deployment is finished. No changes to unloading animation or robot release timing.

Validation: targeted C++ syntax checks passed; standalone checks passed for all ten elliptical habitat shapes and 25 deployment policy cases. Added linked UI regression for the early catalog, placement rotation, committing a blueprint and keeping the apron clear. Full native compile/link, that UI regression, Windows execution, save migration replay and interactive graphics were not run.
