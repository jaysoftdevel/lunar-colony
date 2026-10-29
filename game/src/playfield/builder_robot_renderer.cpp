#include "game/game_app.hpp"
#include "game/util/world_math.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace game {
namespace {
using eng::Color;
using eng::Renderer;
using eng::Vec3;

struct MeshBits { std::vector<Vec3> tris; std::vector<Vec3> edges; };

void add_box(MeshBits& m,Vec3 c,Vec3 s){
    const Vec3 h=s*0.5f;
    Vec3 p[8]={{c.x-h.x,c.y-h.y,c.z-h.z},{c.x+h.x,c.y-h.y,c.z-h.z},{c.x+h.x,c.y+h.y,c.z-h.z},{c.x-h.x,c.y+h.y,c.z-h.z},
               {c.x-h.x,c.y-h.y,c.z+h.z},{c.x+h.x,c.y-h.y,c.z+h.z},{c.x+h.x,c.y+h.y,c.z+h.z},{c.x-h.x,c.y+h.y,c.z+h.z}};
    const int f[6][4]={{0,1,2,3},{5,4,7,6},{4,0,3,7},{1,5,6,2},{3,2,6,7},{4,5,1,0}};
    for(const auto& q:f){ m.tris.insert(m.tris.end(),{p[q[0]],p[q[1]],p[q[2]],p[q[0]],p[q[2]],p[q[3]]}); }
    const int e[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
    for(const auto& q:e){ m.edges.push_back(p[q[0]]); m.edges.push_back(p[q[1]]); }
}

void add_cylinder_x(MeshBits& m,Vec3 c,float length,float radius,int segments=10){
    for(int i=0;i<segments;++i){
        const float a0=6.28318530718f*(float)i/(float)segments;
        const float a1=6.28318530718f*(float)(i+1)/(float)segments;
        Vec3 p0{c.x-length*0.5f,c.y+std::cos(a0)*radius,c.z+std::sin(a0)*radius};
        Vec3 p1{c.x-length*0.5f,c.y+std::cos(a1)*radius,c.z+std::sin(a1)*radius};
        Vec3 q0{c.x+length*0.5f,c.y+std::cos(a0)*radius,c.z+std::sin(a0)*radius};
        Vec3 q1{c.x+length*0.5f,c.y+std::cos(a1)*radius,c.z+std::sin(a1)*radius};
        m.tris.insert(m.tris.end(),{p0,q0,q1,p0,q1,p1});
        m.edges.insert(m.edges.end(),{p0,p1,q0,q1});
        if(i%2==0) m.edges.insert(m.edges.end(),{p0,q0});
    }
}

void add_segment_prism(MeshBits& m,Vec3 a,Vec3 b,float width,float depth){
    // Arm members primarily move in the Y/Z plane. Build a rectangular prism
    // around the segment with X as the lateral axis.
    Vec3 d=b-a;
    const float len=std::sqrt(d.y*d.y+d.z*d.z);
    if(len<0.001f) return;
    Vec3 n{0.0f,-d.z/len,d.y/len};
    Vec3 x{1,0,0};
    const Vec3 hx=x*(width*0.5f), hn=n*(depth*0.5f);
    Vec3 p[8]={a-hx-hn,a+hx-hn,a+hx+hn,a-hx+hn,b-hx-hn,b+hx-hn,b+hx+hn,b-hx+hn};
    const int f[6][4]={{0,1,2,3},{4,7,6,5},{0,4,5,1},{3,2,6,7},{1,5,6,2},{0,3,7,4}};
    for(const auto& q:f) m.tris.insert(m.tris.end(),{p[q[0]],p[q[1]],p[q[2]],p[q[0]],p[q[2]],p[q[3]]});
    const int e[12][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
    for(const auto& q:e){ m.edges.push_back(p[q[0]]); m.edges.push_back(p[q[1]]); }
}

std::vector<Vec3> transform(const std::vector<Vec3>& in,Vec3 position,float heading){
    std::vector<Vec3> out; out.reserve(in.size());
    for(const auto& v:in) out.push_back(world_math::rotate_y(v,heading)+position);
    return out;
}

float action_fraction(const BuilderRobotState& robot){
    if(robot.action==BuilderRobotAction::None || robot.action_duration<=0.001f) return 0.0f;
    return std::clamp(robot.action_elapsed/robot.action_duration,0.0f,1.0f);
}
float smooth(float t){ t=std::clamp(t,0.0f,1.0f); return t*t*(3.0f-2.0f*t); }
float segment(float t,float a,float b){ return b<=a?1.0f:smooth((t-a)/(b-a)); }
Vec3 mix(Vec3 a,Vec3 b,float t){ return a+(b-a)*smooth(t); }

Color resource_color(MaterialResource resource){
    switch(resource){
        case MaterialResource::Iron: return {0.48f,0.52f,0.56f,1.0f};
        case MaterialResource::Copper: return {0.74f,0.42f,0.20f,1.0f};
        case MaterialResource::Silicon: return {0.28f,0.58f,0.76f,1.0f};
        case MaterialResource::Plastic: return {0.72f,0.70f,0.58f,1.0f};
        case MaterialResource::Water: return {0.22f,0.58f,0.86f,1.0f};
        case MaterialResource::Food: return {0.48f,0.72f,0.28f,1.0f};
        case MaterialResource::Medical: return {0.88f,0.42f,0.48f,1.0f};
        case MaterialResource::Count: break;
    }
    return {0.62f,0.62f,0.62f,1.0f};
}

struct RobotArmPose {
    Vec3 root{0,1.00f,0.08f};
    Vec3 elbow{0,1.34f,-0.05f};
    Vec3 tool{0,1.12f,-0.42f};
    float gripper_open{0.16f};
    float tool_extension{0.0f};
};

RobotArmPose robot_arm_pose(const BuilderRobotState& robot){
    RobotArmPose folded{};
    const float t=action_fraction(robot);
    RobotArmPose pose=folded;
    switch(robot.action){
        case BuilderRobotAction::Pickup:{
            const Vec3 ready_tool{0,0.74f,0.62f};
            const Vec3 ready_elbow{0,1.12f,0.34f};
            const Vec3 low_tool{0,0.24f,0.88f};
            const Vec3 low_elbow{0,0.82f,0.50f};
            const Vec3 carry_tool{0,0.94f,0.28f};
            const Vec3 carry_elbow{0,1.23f,0.18f};
            if(t<0.18f){
                const float u=segment(t,0.0f,0.18f); pose.tool=mix(folded.tool,ready_tool,u); pose.elbow=mix(folded.elbow,ready_elbow,u);
            }else if(t<0.50f){
                const float u=segment(t,0.18f,0.50f); pose.tool=mix(ready_tool,low_tool,u); pose.elbow=mix(ready_elbow,low_elbow,u);
            }else if(t<0.66f){
                pose.tool=low_tool; pose.elbow=low_elbow;
                pose.gripper_open=0.16f*(1.0f-segment(t,0.50f,0.66f))+0.035f*segment(t,0.50f,0.66f);
            }else if(t<0.90f){
                const float u=segment(t,0.66f,0.90f); pose.tool=mix(low_tool,carry_tool,u); pose.elbow=mix(low_elbow,carry_elbow,u); pose.gripper_open=0.035f;
            }else{
                const float u=segment(t,0.90f,1.0f); pose.tool=mix(carry_tool,folded.tool,u); pose.elbow=mix(carry_elbow,folded.elbow,u); pose.gripper_open=0.035f;
            }
            break;
        }
        case BuilderRobotAction::PutDown:{
            const Vec3 carry_tool{0,0.94f,0.28f};
            const Vec3 carry_elbow{0,1.23f,0.18f};
            const Vec3 low_tool{0,0.25f,0.90f};
            const Vec3 low_elbow{0,0.83f,0.50f};
            if(t<0.18f){ const float u=segment(t,0,0.18f); pose.tool=mix(folded.tool,carry_tool,u); pose.elbow=mix(folded.elbow,carry_elbow,u); pose.gripper_open=0.035f; }
            else if(t<0.56f){ const float u=segment(t,0.18f,0.56f); pose.tool=mix(carry_tool,low_tool,u); pose.elbow=mix(carry_elbow,low_elbow,u); pose.gripper_open=0.035f; }
            else if(t<0.72f){ pose.tool=low_tool; pose.elbow=low_elbow; pose.gripper_open=0.035f+0.125f*segment(t,0.56f,0.72f); }
            else{ const float u=segment(t,0.72f,1.0f); pose.tool=mix(low_tool,folded.tool,u); pose.elbow=mix(low_elbow,folded.elbow,u); pose.gripper_open=0.16f; }
            break;
        }
        case BuilderRobotAction::ApplyResources:{
            const float deploy=segment(t,0.0f,0.18f);
            const float retract=segment(t,0.82f,1.0f);
            const float sweep=std::sin(segment(t,0.18f,0.82f)*3.14159265359f*3.0f);
            const Vec3 work_tool{sweep*0.18f,0.50f,0.92f};
            const Vec3 work_elbow{sweep*0.05f,1.02f,0.50f};
            pose.tool=mix(folded.tool,work_tool,deploy); pose.elbow=mix(folded.elbow,work_elbow,deploy);
            if(t>0.82f){ pose.tool=mix(work_tool,folded.tool,retract); pose.elbow=mix(work_elbow,folded.elbow,retract); }
            pose.gripper_open=0.055f; pose.tool_extension=0.12f;
            break;
        }
        case BuilderRobotAction::Welding:{
            const float deploy=segment(t,0.0f,0.14f);
            const float retract=segment(t,0.86f,1.0f);
            const float work=segment(t,0.14f,0.86f);
            const float raster=std::sin(work*3.14159265359f*5.0f);
            const float lift=std::sin(work*3.14159265359f*2.0f);
            const Vec3 work_tool{raster*0.16f,0.62f+lift*0.10f,0.94f};
            const Vec3 work_elbow{raster*0.05f,1.07f,0.52f};
            pose.tool=mix(folded.tool,work_tool,deploy); pose.elbow=mix(folded.elbow,work_elbow,deploy);
            if(t>0.86f){ pose.tool=mix(work_tool,folded.tool,retract); pose.elbow=mix(work_elbow,folded.elbow,retract); }
            pose.gripper_open=0.045f; pose.tool_extension=0.18f;
            break;
        }
        case BuilderRobotAction::BuildUp:{
            const Vec3 pickup_tool{0,0.34f,0.82f};
            const Vec3 pickup_elbow{0,0.90f,0.48f};
            const Vec3 raised_tool{0,1.48f,0.78f};
            const Vec3 raised_elbow{0,1.56f,0.38f};
            if(t<0.22f){ const float u=segment(t,0,0.22f); pose.tool=mix(folded.tool,pickup_tool,u); pose.elbow=mix(folded.elbow,pickup_elbow,u); pose.gripper_open=0.13f-0.09f*u; }
            else if(t<0.72f){ const float u=segment(t,0.22f,0.72f); pose.tool=mix(pickup_tool,raised_tool,u); pose.elbow=mix(pickup_elbow,raised_elbow,u); pose.gripper_open=0.04f; }
            else{ const float u=segment(t,0.72f,1.0f); pose.tool=mix(raised_tool,folded.tool,u); pose.elbow=mix(raised_elbow,folded.elbow,u); pose.gripper_open=0.04f+0.12f*u; }
            break;
        }
        case BuilderRobotAction::None: break;
    }
    return pose;
}

} // namespace

void GameApp::render_builder_robot(Renderer& r){
    // Starter logistics pad remains as a visual depot even after the initial
    // reserve is moved away. Six constructor bays and six carrier bays are marked beside it.
    const Vec3 c=starter_stash_center_;
    const float y=0.025f;
    // The depot outline covers the complete starter-pallet row. It uses the
    // same relative center as reset_builder_robot_and_starter_stash(), so the
    // rendered stash and physical resources stay aligned on every map size.
    const float hx=2.75f;
    const float hz=6.65f;
    std::vector<Vec3> pad{
        {c.x-hx,y,c.z-hz},{c.x+hx,y,c.z-hz},{c.x+hx,y,c.z-hz},{c.x+hx,y,c.z+hz},
        {c.x+hx,y,c.z+hz},{c.x-hx,y,c.z+hz},{c.x-hx,y,c.z+hz},{c.x-hx,y,c.z-hz},
        {c.x-hx,y,c.z},{c.x+hx,y,c.z},{c.x,y,c.z-hz},{c.x,y,c.z+hz}};
    r.draw_lines_3d(pad,{0.72f,0.52f,0.16f,0.85f},1.5f);
    for(int column=0;column<3;++column){
        for(int row=0;row<2;++row){
            const Vec3 bay{c.x+3.20f+(float)column*2.20f,y,c.z-1.20f+(float)row*2.40f};
            const float bx=0.58f,bz=0.72f;
            r.draw_lines_3d({{bay.x-bx,y,bay.z-bz},{bay.x+bx,y,bay.z-bz},
                             {bay.x+bx,y,bay.z-bz},{bay.x+bx,y,bay.z+bz},
                             {bay.x+bx,y,bay.z+bz},{bay.x-bx,y,bay.z+bz},
                             {bay.x-bx,y,bay.z+bz},{bay.x-bx,y,bay.z-bz}},
                            {0.46f,0.50f,0.48f,0.55f},1.0f);
        }
    }
    for(int column=0;column<3;++column){
        for(int row=0;row<2;++row){
            const Vec3 carrier_bay{c.x+5.10f+(float)column*1.70f,y,c.z+4.00f+(float)row*2.20f};
            r.draw_lines_3d({{carrier_bay.x-0.58f,y,carrier_bay.z-0.76f},{carrier_bay.x+0.58f,y,carrier_bay.z-0.76f},
                             {carrier_bay.x+0.58f,y,carrier_bay.z-0.76f},{carrier_bay.x+0.58f,y,carrier_bay.z+0.76f},
                             {carrier_bay.x+0.58f,y,carrier_bay.z+0.76f},{carrier_bay.x-0.58f,y,carrier_bay.z+0.76f},
                             {carrier_bay.x-0.58f,y,carrier_bay.z+0.76f},{carrier_bay.x-0.58f,y,carrier_bay.z-0.76f}},
                            {0.22f,0.62f,0.82f,0.72f},1.35f);
        }
    }

    MeshBits body,carrier_body,wheels,suspension,sensors,carrier_sensors;
    add_box(body,{0,0.58f,0},{0.82f,0.48f,1.34f});
    add_box(body,{0,0.79f,-0.18f},{0.68f,0.22f,0.70f});
    add_box(body,{0,0.60f,0.61f},{0.70f,0.34f,0.22f});
    // CR-01 uses the same wheel/suspension family but a low flat logistics
    // deck, protected cargo rails and a smaller sensor mast.
    add_box(carrier_body,{0,0.50f,0},{0.84f,0.34f,1.48f});
    add_box(carrier_body,{0,0.70f,-0.47f},{0.70f,0.24f,0.38f});
    add_box(carrier_body,{-0.39f,0.83f,0.18f},{0.07f,0.34f,0.86f});
    add_box(carrier_body,{ 0.39f,0.83f,0.18f},{0.07f,0.34f,0.86f});
    add_box(carrier_body,{0,0.69f,0.57f},{0.66f,0.08f,0.20f});
    for(float z:{-0.48f,0.0f,0.48f}){
        add_cylinder_x(wheels,{-0.49f,0.28f,z},0.20f,0.22f,10);
        add_cylinder_x(wheels,{ 0.49f,0.28f,z},0.20f,0.22f,10);
        add_box(suspension,{-0.35f,0.42f,z},{0.18f,0.10f,0.12f});
        add_box(suspension,{ 0.35f,0.42f,z},{0.18f,0.10f,0.12f});
    }
    add_box(sensors,{0,1.02f,0.52f},{0.10f,0.42f,0.10f});
    add_box(sensors,{0,1.27f,0.52f},{0.52f,0.16f,0.18f});
    for(float x:{-0.17f,0.0f,0.17f}) add_box(sensors,{x,1.28f,0.42f},{0.08f,0.07f,0.04f});
    add_box(carrier_sensors,{0,0.86f,-0.56f},{0.09f,0.28f,0.09f});
    add_box(carrier_sensors,{0,1.04f,-0.56f},{0.42f,0.12f,0.14f});

    // Delivered construction resources remain visibly staged inside the active
    // blueprint after PUT-DOWN. The rover handoff point stays outside so a
    // partial wall can never trap the logistics vehicle. These crates render the authoritative shared
    // site buffer, not loose ground resources, so they cannot be collected a
    // second time. As a construction step consumes material the corresponding
    // boxes disappear from the staging area.
    auto render_site_buffer=[&](const ConstructionTargetRef& target,const ResourceBundle& buffer){
        if(buffer.empty()) return;
        ResourceBundle visible_buffer=buffer;
        const std::size_t owner_index=construction_site_owner_index(target);
        if(owner_index<builder_robots_.size() && builder_robots_[owner_index].stage_materials_picked){
            if(const auto* plan=construction_target_plan(target)){
                const ResourceBundle picked=plan->next_step_cost(construction_target_step(target));
                for(std::size_t ri=0;ri<kMaterialResourceCount;++ri){
                    const auto material=(MaterialResource)ri;
                    visible_buffer.amount(material)=std::max<std::int64_t>(0,visible_buffer.amount(material)-picked.amount(material));
                }
            }
        }
        int visual_slot=0;
        constexpr int max_visual_packages=36;
        for(std::size_t mi=0;mi<kMaterialResourceCount && visual_slot<max_visual_packages;++mi){
            const auto material=(MaterialResource)mi;
            std::int64_t remaining=visible_buffer.amount(material);
            const std::int64_t capacity=std::max<std::int64_t>(1,resource_package_capacity_kg(material));
            while(remaining>0 && visual_slot<max_visual_packages){
                const std::int64_t package_amount=std::min(remaining,capacity);
                const int layer=visual_slot/18;
                const float half=resource_package_half_width(material,package_amount);
                const float height=resource_package_height(material,package_amount);
                MeshBits box;
                Vec3 base=construction_drop_package_position(target,visual_slot);
                base.y=first_person_floor_height(base)+0.04f+(float)layer*0.38f;
                add_box(box,base+Vec3{0,height*0.5f,0},{half*2.0f,height,half*2.0f});
                r.draw_triangles_3d(box.tris,resource_color(material));
                r.draw_lines_3d(box.edges,{0.07f,0.09f,0.10f,0.95f},0.9f);
                remaining-=package_amount;
                ++visual_slot;
            }
        }
    };
    for(std::size_t i=0;i<placed_habitats_.size();++i)
        render_site_buffer({ConstructionTargetKind::Habitat,i},placed_habitats_[i].site_buffer);
    for(std::size_t i=0;i<placed_resources_.size();++i)
        render_site_buffer({ConstructionTargetKind::Resource,i},placed_resources_[i].site_buffer);
    for(std::size_t i=0;i<corridor_network_.connections().size();++i)
        render_site_buffer({ConstructionTargetKind::Corridor,i},corridor_network_.connections()[i].site_buffer);
    for(std::size_t i=0;i<utility_network_.connections().size();++i)
        render_site_buffer({ConstructionTargetKind::Utility,i},utility_network_.connections()[i].site_buffer);
    for(std::size_t i=0;i<walkway_network_.connections().size();++i)
        render_site_buffer({ConstructionTargetKind::Walkway,i},walkway_network_.connections()[i].site_buffer);

    // Every moving task exposes its complete precomputed continuous trajectory
    // on the ground. Cyan is the planned lane; amber means the reciprocal speed
    // governor is currently yielding right-of-way. The lane itself never jitters
    // left/right because traffic negotiation changes speed or triggers a complete
    // lane replan instead of reactive steering.
    for(std::size_t robot_index=0;robot_index<builder_robots_.size();++robot_index){
        const auto& robot=builder_robots_[robot_index];
        if(!robot.route_active || robot.route_cursor>=robot.planned_route.size()) continue;
        std::vector<Vec3> route_lines;
        Vec3 previous=robot.position;
        previous.y=first_person_floor_height(previous)+0.055f;
        for(std::size_t i=robot.route_cursor;i<robot.planned_route.size();++i){
            Vec3 next=robot.planned_route[i];
            next.y=first_person_floor_height(next)+0.055f;
            route_lines.push_back(previous);
            route_lines.push_back(next);
            previous=next;
        }
        const Color route_color=robot.route_waiting_for_clearance?
            Color{1.00f,0.66f,0.18f,0.92f}:Color{0.18f,0.82f,1.00f,0.90f};
        if(!route_lines.empty()) r.draw_lines_3d(route_lines,route_color,robot.route_waiting_for_clearance?2.6f:2.15f);
        for(std::size_t i=robot.route_cursor;i<robot.planned_route.size();i+=4){
            Vec3 p=robot.planned_route[i];
            p.y=first_person_floor_height(p)+0.058f;
            const float d=0.10f;
            r.draw_lines_3d({{p.x-d,p.y,p.z},{p.x+d,p.y,p.z},{p.x,p.y,p.z-d},{p.x,p.y,p.z+d}},route_color,1.15f);
        }
        if(!robot.planned_route.empty()){
            Vec3 p=robot.planned_route.back();
            p.y=first_person_floor_height(p)+0.058f;
            const float d=0.14f;
            r.draw_lines_3d({{p.x-d,p.y,p.z},{p.x+d,p.y,p.z},{p.x,p.y,p.z-d},{p.x,p.y,p.z+d}},route_color,1.35f);
        }
    }

    for(std::size_t robot_index=0;robot_index<builder_robots_.size();++robot_index){
        const auto& robot=builder_robots_[robot_index];
        const float phase=action_fraction(robot);
        MeshBits cargo,arm,gripper;
        const bool cargo_in_bay=robot.cargo.total_amount()>0 &&
            !(robot.action==BuilderRobotAction::PutDown && phase>0.18f);
        if(cargo_in_bay){
            const float fill=std::clamp((float)robot.cargo.total_amount()/(float)robot.payload_capacity_kg,0.12f,1.0f);
            if(robot.role==RobotRole::Carrier)
                add_box(cargo,{0,0.85f,0.18f},{0.68f,0.20f+0.24f*fill,0.78f});
            else
                add_box(cargo,{0,0.92f,0.16f},{0.58f,0.16f+0.20f*fill,0.42f});
        }

        // The chassis follows its smoothed driving heading, while the arm sits
        // on a rotating turret and can keep facing the resource/build focus.
        const RobotArmPose pose=robot_arm_pose(robot);
        add_box(arm,pose.root,{0.46f,0.15f,0.40f});
        add_segment_prism(arm,pose.root,pose.elbow,0.22f,0.18f);
        add_segment_prism(arm,pose.elbow,pose.tool,0.18f,0.15f);
        add_box(arm,pose.tool,{0.22f,0.18f,0.24f});
        if(pose.tool_extension>0.001f)
            add_box(arm,pose.tool+Vec3{0,0,pose.tool_extension*0.5f},{0.07f,0.07f,pose.tool_extension});
        const float jaw=pose.gripper_open;
        add_box(gripper,pose.tool+Vec3{-0.09f-jaw*0.5f,-0.08f,0.11f},{0.07f,0.16f,0.20f});
        add_box(gripper,pose.tool+Vec3{ 0.09f+jaw*0.5f,-0.08f,0.11f},{0.07f,0.16f,0.20f});

        const Vec3 pos=robot.visual_initialized?robot.visual_position:robot.position;
        const float heading=robot.visual_initialized?robot.visual_heading_degrees:robot.heading_degrees;
        float arm_heading=heading;
        if(robot.action!=BuilderRobotAction::None){
            const float fx=robot.action_focus_position.x-pos.x;
            const float fz=robot.action_focus_position.z-pos.z;
            if(fx*fx+fz*fz>0.04f) arm_heading=std::atan2(fx,fz)*180.0f/3.14159265358979323846f;
        }

        const bool carrier=robot.role==RobotRole::Carrier;
        const auto& chassis=carrier?carrier_body:body;
        const auto& sensor_mesh=carrier?carrier_sensors:sensors;
        r.draw_triangles_3d(transform(chassis.tris,pos,heading),carrier?Color{0.34f,0.48f,0.56f,1.0f}:Color{0.58f,0.61f,0.60f,1.0f});
        r.draw_triangles_3d(transform(wheels.tris,pos,heading),{0.08f,0.10f,0.11f,1.0f});
        r.draw_triangles_3d(transform(suspension.tris,pos,heading),{0.35f,0.37f,0.36f,1.0f});
        r.draw_triangles_3d(transform(arm.tris,pos,arm_heading),carrier?Color{0.28f,0.56f,0.68f,1.0f}:Color{0.72f,0.46f,0.14f,1.0f});
        r.draw_triangles_3d(transform(gripper.tris,pos,arm_heading),{0.23f,0.25f,0.24f,1.0f});
        r.draw_triangles_3d(transform(sensor_mesh.tris,pos,heading),carrier?Color{0.18f,0.50f,0.62f,1.0f}:Color{0.30f,0.40f,0.43f,1.0f});
        if(!cargo.tris.empty()) r.draw_triangles_3d(transform(cargo.tris,pos,heading),{0.28f,0.48f,0.56f,0.95f});
        const Color edge{0.04f,0.06f,0.07f,1.0f};
        r.draw_lines_3d(transform(chassis.edges,pos,heading),edge,1.15f);
        r.draw_lines_3d(transform(wheels.edges,pos,heading),{0.30f,0.34f,0.34f,1.0f},1.0f);
        r.draw_lines_3d(transform(suspension.edges,pos,heading),edge,1.0f);
        r.draw_lines_3d(transform(arm.edges,pos,arm_heading),edge,1.10f);
        r.draw_lines_3d(transform(gripper.edges,pos,arm_heading),edge,0.95f);
        r.draw_lines_3d(transform(sensor_mesh.edges,pos,heading),carrier?Color{0.24f,0.84f,0.96f,1.0f}:Color{0.18f,0.76f,0.92f,1.0f},1.25f);
        if(!cargo.edges.empty()) r.draw_lines_3d(transform(cargo.edges,pos,heading),{0.12f,0.22f,0.24f,1.0f},1.0f);

        const Vec3 tool_world=world_math::rotate_y(pose.tool,arm_heading)+pos;
        const bool carrying_handled=
            (robot.action==BuilderRobotAction::Pickup && phase>=0.56f && phase<0.97f) ||
            (robot.action==BuilderRobotAction::PutDown && phase<0.70f);
        if(carrying_handled && robot.action_amount>0){
            MeshBits handled;
            const float half=resource_package_half_width(robot.action_material,robot.action_amount)*0.72f;
            const float height=resource_package_height(robot.action_material,robot.action_amount)*0.72f;
            add_box(handled,pose.tool+Vec3{0,-height*0.58f,0.10f},{half*2.0f,height,half*2.0f});
            r.draw_triangles_3d(transform(handled.tris,pos,arm_heading),resource_color(robot.action_material));
            r.draw_lines_3d(transform(handled.edges,pos,arm_heading),edge,0.9f);
        }
        if(robot.action==BuilderRobotAction::ApplyResources){
            const Vec3 nozzle=tool_world+world_math::rotate_y({0,0,pose.tool_extension},arm_heading);
            const Vec3 end=nozzle+world_math::rotate_y({0,-0.24f,0.38f},arm_heading);
            r.draw_lines_3d({nozzle,end},{0.30f,0.82f,0.95f,0.95f},2.0f);
            std::vector<Vec3> feed;
            for(int i=0;i<4;++i){
                const float o=((float)i-1.5f)*0.045f;
                feed.push_back(nozzle+world_math::rotate_y({o,0,0},arm_heading));
                feed.push_back(end+world_math::rotate_y({o,-0.05f,0},arm_heading));
            }
            r.draw_lines_3d(feed,{0.74f,0.72f,0.48f,0.70f},1.1f);
        }
        if(robot.action==BuilderRobotAction::Welding){
            const float active=segment(phase,0.14f,0.86f)*(1.0f-segment(phase,0.86f,1.0f));
            if(active>0.05f){
                std::vector<Vec3> sparks;
                for(int i=0;i<9;++i){
                    const float a=phase*23.0f+(float)i*0.77f;
                    const float len=0.10f+0.20f*(float)((i%4)+1)/4.0f;
                    const Vec3 tip=tool_world+Vec3{std::sin(a)*len,-0.05f-0.04f*(float)(i%3),std::cos(a)*len};
                    sparks.push_back(tool_world); sparks.push_back(tip);
                }
                r.draw_lines_3d(sparks,{1.0f,0.70f,0.22f,0.95f},1.8f);
            }
        }
        if(robot.action==BuilderRobotAction::BuildUp){
            // A standardized panel/beam is visibly gripped, lifted and aligned
            // with the emerging structure before the stage is committed.
            MeshBits lifted;
            if(phase>0.12f && phase<0.82f){
                add_box(lifted,pose.tool+Vec3{0,-0.10f,0.12f},{0.52f,0.12f,0.38f});
                r.draw_triangles_3d(transform(lifted.tris,pos,arm_heading),{0.52f,0.58f,0.57f,0.96f});
                r.draw_lines_3d(transform(lifted.edges,pos,arm_heading),edge,1.0f);
            }
            const Vec3 guide_a=tool_world+world_math::rotate_y({-0.20f,-0.38f,0},arm_heading);
            const Vec3 guide_b=tool_world+world_math::rotate_y({0.20f,-0.38f,0},arm_heading);
            r.draw_lines_3d({guide_a,tool_world,guide_b,tool_world},{0.38f,0.78f,0.88f,0.82f},1.5f);
        }

        if(selection_kind_==SelectionKind::Robot && selected_robot_==(int)robot_index){
            const float rr=robot_body_radius(robot.role)+0.26f;
            const float yy=pos.y+0.04f;
            const Color sel=robot.role==RobotRole::Carrier?Color{0.24f,0.86f,1.0f,0.95f}:Color{1.0f,0.66f,0.18f,0.95f};
            r.draw_lines_3d({{pos.x-rr,yy,pos.z-rr},{pos.x+rr,yy,pos.z-rr},
                             {pos.x+rr,yy,pos.z-rr},{pos.x+rr,yy,pos.z+rr},
                             {pos.x+rr,yy,pos.z+rr},{pos.x-rr,yy,pos.z+rr},
                             {pos.x-rr,yy,pos.z+rr},{pos.x-rr,yy,pos.z-rr}},sel,2.2f);
        }

        if(debug_frame_enabled_ && robot.target.valid()){
            const Vec3 target=construction_target_position(robot.target);
            r.draw_lines_3d({pos+Vec3{0,1.35f,0},target+Vec3{0,0.08f,0}},
                            {0.94f,0.68f,0.18f,0.75f},1.2f);
        }
    }
}


} // namespace game
