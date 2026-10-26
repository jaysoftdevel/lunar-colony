#pragma once
#include "engine/engine.hpp"
#include <vector>

namespace game::mesh_wireframe {

inline void append_triangle_edges(std::vector<eng::Vec3>& out,const std::vector<eng::Vec3>& triangles){
    out.reserve(out.size()+(triangles.size()/3)*6);
    for(std::size_t i=0;i+2<triangles.size();i+=3){
        const eng::Vec3& a=triangles[i];
        const eng::Vec3& b=triangles[i+1];
        const eng::Vec3& c=triangles[i+2];
        out.push_back(a); out.push_back(b);
        out.push_back(b); out.push_back(c);
        out.push_back(c); out.push_back(a);
    }
}

inline std::vector<eng::Vec3> triangle_edges(const std::vector<eng::Vec3>& triangles){
    std::vector<eng::Vec3> out;
    append_triangle_edges(out,triangles);
    return out;
}

inline void draw_layer(eng::Renderer& r,const std::vector<eng::Vec3>& triangles,eng::Color color,float width=1.0f){
    if(triangles.empty()) return;
    r.draw_lines_3d(triangle_edges(triangles),color,width);
}

} // namespace game::mesh_wireframe
