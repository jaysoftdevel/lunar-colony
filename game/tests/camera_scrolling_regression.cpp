#include "game/systems/camera_controller.hpp"
#include "game/util/mesh_wireframe.hpp"
#include <iostream>
#include <stdexcept>
using namespace game;using namespace game::runtime;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){try{
    for(float yaw:{0.f,45.f,90.f,225.f})for(float elevation:{35.f,55.f,90.f}){
        CameraController camera;camera.restore({22,elevation,yaw,{0,0,0}});
        const Rect area{0,50,1600,750};InputState in;in.mouse_x=1599;in.mouse_y=400;
        bool initial_fits=true;for(int x:{0,1600})for(int y:{50,800}){Vec3 hit;initial_fits=initial_fits && camera.intersect_playfield(x,y,1600,900,128,hit);}
        auto start=camera.target();camera.scroll_at_edge(in,128,1600,900,area,1.f/60);
        require(length(camera.target()-start)>.01f,"edge scrolling did not move");
        for(int f=0;f<1500;++f)camera.scroll_at_edge(in,128,1600,900,area,1.f/60);
        start=camera.target();camera.scroll_at_edge(in,128,1600,900,area,1.f/60);
        require(length(camera.target()-start)<.0001f,"scroll did not stop at visible map boundary");
        if(initial_fits)for(int x:{0,1600})for(int y:{50,800}){Vec3 hit;if(!camera.intersect_playfield(x,y,1600,900,128,hit))throw std::runtime_error("visible ground footprint escaped map yaw="+std::to_string(yaw)+" elev="+std::to_string(elevation)+" hit="+std::to_string(hit.x)+","+std::to_string(hit.z));}
    }
    CameraController camera;camera.restore({22,90,0,{}});InputState in;in.mouse_x=1599;in.mouse_y=400;
    const auto still=camera.target();in.pointer_inside_window=false;camera.scroll_at_edge(in,128,1600,900,{0,50,1600,750},.05f);
    require(length(camera.target()-still)<.0001f,"unfocused/outside pointer scrolled");
    in.pointer_inside_window=true;in.left_down=true;camera.scroll_at_edge(in,128,1600,900,{0,50,1600,750},.05f);
    require(length(camera.target()-still)<.0001f,"marquee drag scrolled");
    camera.restore({300,90,0,{}});in.left_down=false;camera.scroll_at_edge(in,128,1600,900,{0,0,1600,900},.05f);
    require(length(camera.target())<.0001f,"fully visible playfield scrolled off screen");
    const std::vector<Vec3> square{{0,0,0},{1,0,0},{1,0,1},{0,0,0},{1,0,1},{0,0,1}};
    require(mesh_wireframe::unique_triangle_edges(square).size()==10,"wireframe duplicates shared edge");
    auto changed=square;changed[1].x=2;const auto first=mesh_wireframe::unique_triangle_edges(changed);
    require(first.size()==10 && mesh_wireframe::unique_triangle_edges(square).size()==10,"wire cache reused mutable source");
    std::cout<<"Visible camera bounds, edge input guards and retained unique wireframe passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
