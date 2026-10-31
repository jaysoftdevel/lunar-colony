add_library(rvo2 STATIC
    src/Agent.cc src/KdTree.cc src/Line.cc src/Obstacle.cc
    src/RVOSimulator.cc src/Vector2.cc)
target_include_directories(rvo2 PUBLIC src)
target_compile_definitions(rvo2 PUBLIC RVO_STATIC_DEFINE)
