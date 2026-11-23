# Lunar Colony 0.67.1 — restore lunar background

Requires game 0.67.0 with external JSD engine source 0.5.16.

Classic graphics skipped lunar scene initialization, which also owned the
world-oriented star sky and terrain surrounding the buildable square. A saved
Classic setting could therefore leave only the flat playfield visible.

All full-graphics paths now initialize the lunar world. Classic keeps its low
cost lighting/effect settings; stars and the surrounding surface remain present.
With terrain relief off, a simple textured plane extends to the normal horizon.
Higher relief settings retain the surrounding crater terrain. Cached terrain
meshes and shader programs remain in use. Debug Meshed stays a wire view.

The same source change applies to Linux and Windows. Save format, preferences,
FPV building, faded hulls and rover recovery are unchanged.
