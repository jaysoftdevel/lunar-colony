#include "jsd_bridge.hpp"
#include "jsd_lunar_terrain.hpp"
#include "ui_batch.hpp"
#include <frontier/detail/gl.hpp>
#include <chrono>
#include <cctype>
#include <cstring>
#include <iostream>
#include <map>
#include <tuple>

namespace game::runtime::jsd_bridge {
namespace {
namespace gfx=rf::gfx;
struct Key {
    int kind{},x{},z{},alpha{},emission{},mapped{};
    unsigned seed{};
    auto operator<=>(const Key&) const = default;
};
struct Batch {
    gfx::MeshId mesh{};
    std::vector<gfx::Vertex> vertices;
    gfx::Material material;
    std::uint64_t last_frame{};
    std::vector<gfx::Vertex> previous_vertices;
    std::size_t uploaded{};
};
class JsdRenderer final: public SceneRenderer {
    rf::Platform& platform_;
    GraphicsSettings graphics_=graphics_settings_from_environment();
    gfx::Renderer renderer_;
    vl::GL gl_;
    gfx::GraphicsSettings effective_{};
    View view_{};
    gfx::Environment environment_{};
    std::vector<gfx::Light> lights_;
    std::vector<gfx::LineSegment> lines_;
    std::vector<gfx::ParticleSprite> particles_;
    std::map<Key,Batch> batches_;
    ui::Batch ui_;
    Color clear_{};
    bool world_{},lunar_{},benchmarking_{},hardware_{},vsync_supported_{},ui_mode_{};
    float ui_scale_=1,previous_seconds_{};
    std::uint64_t frame_{};
    int width_=1,height_=1;
    std::chrono::steady_clock::time_point begin_;
    double benchmark_ms_{};
    unsigned ui_program_{},ui_vao_{},ui_vbo_{},ui_atlas_{};
    std::string vendor_;
    float terrain_half_=-1;
    int terrain_quality_=-1,terrain_clutter_=-1,terrain_range_=-1;
    std::vector<gfx::Draw> terrain_;

    unsigned shader(unsigned type,const char* source){
        const auto id=gl_.CreateShader(type);
        gl_.ShaderSource(id,1,&source,nullptr);gl_.CompileShader(id);
        int ok=0;gl_.GetShaderiv(id,vl::GL_COMPILE_STATUS,&ok);
        if(!ok){char log[4096]{};gl_.GetShaderInfoLog(id,4095,nullptr,log);gl_.DeleteShader(id);throw std::runtime_error(log);}
        return id;
    }
    void initialize_ui(){
        const auto vs=shader(vl::GL_VERTEX_SHADER,R"GLSL(#version 330 core
layout(location=0) in vec2 position;layout(location=1) in vec2 uv;layout(location=2) in vec4 color;
uniform vec2 viewport;out vec2 vUV;out vec4 vColor;
void main(){vUV=uv;vColor=color;gl_Position=vec4(position.x/viewport.x*2.-1.,1.-position.y/viewport.y*2.,0,1);})GLSL");
        const auto fs=shader(vl::GL_FRAGMENT_SHADER,R"GLSL(#version 330 core
uniform sampler2D fontAtlas;in vec2 vUV;in vec4 vColor;out vec4 color;
void main(){float coverage=1.;if(vUV.x>=0.){float d=texture(fontAtlas,vUV).r;
float edge=clamp(fwidth(d)*.55,.017,.28);coverage=smoothstep(.49-edge,.49+edge,d);}
color=vec4(vColor.rgb,vColor.a*coverage);})GLSL");
        ui_program_=gl_.CreateProgram();gl_.AttachShader(ui_program_,vs);gl_.AttachShader(ui_program_,fs);gl_.LinkProgram(ui_program_);
        gl_.DeleteShader(vs);gl_.DeleteShader(fs);
        int ok=0;gl_.GetProgramiv(ui_program_,vl::GL_LINK_STATUS,&ok);
        if(!ok)throw std::runtime_error("Could not link the colony UI shaders");
        gl_.GenVertexArrays(1,&ui_vao_);gl_.GenBuffers(1,&ui_vbo_);gl_.BindVertexArray(ui_vao_);gl_.BindBuffer(vl::GL_ARRAY_BUFFER,ui_vbo_);
        for(unsigned a:{0u,1u,2u})gl_.EnableVertexAttribArray(a);
        gl_.VertexAttribPointer(0,2,vl::GL_FLOAT,0,sizeof(ui::Vertex),nullptr);
        gl_.VertexAttribPointer(1,2,vl::GL_FLOAT,0,sizeof(ui::Vertex),reinterpret_cast<void*>(offsetof(ui::Vertex,uv)));
        gl_.VertexAttribPointer(2,4,vl::GL_UNSIGNED_BYTE,1,sizeof(ui::Vertex),reinterpret_cast<void*>(offsetof(ui::Vertex,color)));
        gl_.GenTextures(1,&ui_atlas_);gl_.BindTexture(vl::GL_TEXTURE_2D,ui_atlas_);
        gl_.TexParameteri(vl::GL_TEXTURE_2D,vl::GL_TEXTURE_MIN_FILTER,vl::GL_LINEAR);
        gl_.TexParameteri(vl::GL_TEXTURE_2D,vl::GL_TEXTURE_MAG_FILTER,vl::GL_LINEAR);
        gl_.TexImage2D(vl::GL_TEXTURE_2D,0,0x8229,ui::atlas_width,ui::atlas_height,0,0x1903,vl::GL_UNSIGNED_BYTE,ui::font_atlas().data());
        gl_.BindVertexArray(0);
    }
    void flush_ui(){
        if(ui_.vertices.empty())return;
        gl_.BindFramebuffer(vl::GL_FRAMEBUFFER,0);gl_.Viewport(0,0,width_,height_);
        gl_.Disable(0x0B71);gl_.Disable(0x0B44);gl_.Disable(vl::GL_SCISSOR_TEST);gl_.Disable(0x8DB9);
        gl_.Enable(vl::GL_BLEND);gl_.BlendFunc(vl::GL_SRC_ALPHA,vl::GL_ONE_MINUS_SRC_ALPHA);
        gl_.UseProgram(ui_program_);gl_.Uniform2f(gl_.GetUniformLocation(ui_program_,"viewport"),float(width_),float(height_));
        gl_.Uniform1i(gl_.GetUniformLocation(ui_program_,"fontAtlas"),0);gl_.ActiveTexture(vl::GL_TEXTURE0);
        gl_.BindSampler(0,0);gl_.BindTexture(vl::GL_TEXTURE_2D,ui_atlas_);gl_.BindVertexArray(ui_vao_);gl_.BindBuffer(vl::GL_ARRAY_BUFFER,ui_vbo_);
        gl_.BufferData(vl::GL_ARRAY_BUFFER,static_cast<vl::GLsizeiptr>(ui_.vertices.size()*sizeof(ui::Vertex)),ui_.vertices.data(),vl::GL_STREAM_DRAW);
        gl_.DrawArrays(vl::GL_TRIANGLES,0,static_cast<int>(ui_.vertices.size()));gl_.BindVertexArray(0);
    }
    gfx::Material material(int kind,Color tint){
        gfx::Material m;
        m.doubleSided=true;m.projection=gfx::TextureProjection::UV;
        m.surface=gfx::SurfaceKind::HullPanels;m.roughness=.58f;m.clearcoat=.12f;
        switch(static_cast<LunarMaterial>(kind)){
        case LunarMaterial::Alloy:m.surface=gfx::SurfaceKind::BareMetal;m.metallic=.82f;m.roughness=.31f;break;
        case LunarMaterial::Regolith:
            m.surface=gfx::SurfaceKind::Regolith;m.roughness=.96f;m.clearcoat=0;m.normalScale=.65f;m.castShadow=false;break;
        case LunarMaterial::Hull:m.surface=gfx::SurfaceKind::HullPanels;m.roughness=.54f;break;
        case LunarMaterial::Interior:m.surface=gfx::SurfaceKind::InteriorPanels;m.roughness=.68f;m.clearcoat=.04f;m.castShadow=false;break;
        case LunarMaterial::Deck:m.surface=gfx::SurfaceKind::DeckPlating;m.metallic=.24f;m.roughness=.84f;m.clearcoat=0;m.castShadow=false;break;
        case LunarMaterial::RobotPaint:m.surface=gfx::SurfaceKind::RobotArmor;m.roughness=.40f;m.metallic=.15f;break;
        case LunarMaterial::Textile:m.surface=gfx::SurfaceKind::FabricWeave;m.roughness=.92f;m.clearcoat=0;m.sheen=.15f;break;
        case LunarMaterial::Display:m.surface=gfx::SurfaceKind::Console;m.emissive={.10f,.75f,.92f,1};m.castShadow=false;m.temporalReactive=true;break;
        case LunarMaterial::Rubber:m.surface=gfx::SurfaceKind::Rubber;m.roughness=.92f;m.clearcoat=0;break;
        case LunarMaterial::Solar:m.metallic=.48f;m.roughness=.22f;m.clearcoat=.55f;break;
        case LunarMaterial::Glass:m.surface=gfx::SurfaceKind::Textured;m.normalScale=0;m.metallic=.05f;m.roughness=.11f;m.clearcoat=.85f;m.castShadow=false;break;
        case LunarMaterial::Fabric:case LunarMaterial::Hair:m.surface=gfx::SurfaceKind::FabricWeave;m.roughness=.89f;m.clearcoat=0;break;
        case LunarMaterial::Pad:m.surface=gfx::SurfaceKind::Concrete;m.roughness=.85f;m.clearcoat=0;break;
        case LunarMaterial::Emissive:m.surface=gfx::SurfaceKind::Textured;m.emissive={tint.r*2.8f,tint.g*2.8f,tint.b*2.8f,1};m.castShadow=false;break;
        case LunarMaterial::Marking:m.surface=gfx::SurfaceKind::Textured;m.castShadow=false;m.roughness=.77f;break;
        case LunarMaterial::Skin:m.surface=gfx::SurfaceKind::Textured;m.roughness=.67f;m.sheen=.1f;break;
        default:break;
        }
        m.normalScale*=graphics_.quality(GraphicsFeature::MaterialDetail)/3.f;
        if(tint.a<.995f){m.alphaMode=gfx::AlphaMode::Blend;m.temporalReactive=true;m.castShadow=false;}
        return m;
    }
    void queue(const std::vector<Vec3>& vertices,int kind,Color tint,const LunarSurfaceMapping* mapping=nullptr){
        if(tint.a<=0)return;
        world_=true;
        for(std::size_t i=0;i+2<vertices.size();i+=3){
            const Vec3 center=(vertices[i]+vertices[i+1]+vertices[i+2])*(1.f/3);
            const Key key{kind,int(std::floor(center.x/24)),int(std::floor(center.z/24)),tint.a<.995f,
                kind==int(LunarMaterial::Emissive)?(int(tint.r*255)<<16)|(int(tint.g*255)<<8)|int(tint.b*255):0,
                mapping!=nullptr,mapping?mapping->seed:0};
            auto& batch=batches_[key];
            if(batch.last_frame!=frame_){batch.previous_vertices.swap(batch.vertices);batch.vertices.clear();batch.last_frame=frame_;batch.material=material(kind,tint);batch.material.surfaceUV=mapping!=nullptr;
                batch.material.surfaceSeed=(key.seed?key.seed:unsigned(kind*8191+17))&0x7fffffffu;}
            const auto face=cross(vertices[i+1]-vertices[i],vertices[i+2]-vertices[i]);
            const float area=length(face);
            if(area<1e-9f)continue; // Dome poles and collapsed preview faces have no surface.
            const auto n=face*(1/area);
            const Vec3 local_n=mapping?Vec3{dot(n,mapping->x),dot(n,mapping->y),dot(n,mapping->z)}:n;
            for(std::size_t j=i;j<i+3;++j){
                auto p=vertices[j];auto uv=mapping?mapping->coordinate(p):p;
                float u=uv.x,v=uv.z;
                if(std::abs(local_n.y)<.6f){u=std::abs(local_n.x)>std::abs(local_n.z)?uv.z:uv.x;v=uv.y;}
                batch.vertices.push_back({vector(p),vector(n),color(tint),u,v});
            }
        }
    }
    void prepare_terrain(float half){
        const int q=graphics_.quality(GraphicsFeature::TerrainRelief),clutter=graphics_.ground_clutter,range=graphics_.clutter_range;
        if(half==terrain_half_ && q==terrain_quality_ && clutter==terrain_clutter_ && range==terrain_range_)return;
        for(const auto& draw:terrain_)renderer_.destroyMesh(draw.mesh);
        terrain_.clear();terrain_half_=half;terrain_quality_=q;terrain_clutter_=clutter;terrain_range_=range;
        // Retain the colony's flat build plane, crater landscape and boulders.
        const auto landscape=jsd_terrain::terrain_mesh(half,q,clutter,range);
        std::map<std::tuple<int,int,bool>,std::vector<gfx::Vertex>> cells;
        for(std::size_t i=0;i+2<landscape.size();i+=3){
            const auto p=landscape[i].position;
            auto& cell=cells[{int(std::floor(p.x/64)),int(std::floor(p.z/64)),i<6}];
            for(std::size_t j=i;j<i+3;++j){
                const auto& v=landscape[j];
                cell.push_back({vector(v.position),vector(v.normal),{.40f,.39f,.37f,1},v.position.x,v.position.z});
            }
        }
        for(auto& [cell,vertices]:cells){
            gfx::Draw draw;draw.mesh=renderer_.createMesh(vertices);draw.material=material(int(LunarMaterial::Regolith),{1,1,1,1});
            draw.material.castShadow=!std::get<2>(cell);
            draw.objectId=0x80000000u+terrain_.size();terrain_.push_back(draw);
        }
    }
public:
    explicit JsdRenderer(rf::Platform& platform,std::function<void(const gfx::ShaderBuildProgress&)> progress):platform_(platform),renderer_(platform,jsd_bridge::settings(graphics_),gfx::RendererStartup{true,{},std::move(progress)}){
        width_=std::max(1,platform.width);height_=std::max(1,platform.height);
        effective_=jsd_bridge::settings(graphics_);
        gl_.load([&](const char* n){return platform.resolve(n);});
        const auto* vendor=gl_.GetString(0x1F00);vendor_=vendor?reinterpret_cast<const char*>(vendor):"";
        auto name=renderer_.capabilities().renderer;
        std::transform(name.begin(),name.end(),name.begin(),[](unsigned char c){return char(std::tolower(c));});
        hardware_=name.find("llvmpipe")==std::string::npos && name.find("softpipe")==std::string::npos && name.find("swrast")==std::string::npos;
        vsync_supported_=platform_.setVSync(graphics_.vertical_sync);
        initialize_ui();
        const auto& cache=renderer_.shaderCacheStatistics();
        std::cout<<"Shader cache: "<<cache.loaded<<" restored, "<<cache.compiled<<" compiled, "<<cache.stored<<" stored"<<(cache.supported?"":" (driver binaries unavailable)")<<'\n';
        std::cout<<"JSD 0.5.16 colony renderer: "<<gpu_renderer()<<" | "<<gpu_version()<<'\n';
    }
    ~JsdRenderer() override {
        gl_.DeleteProgram(ui_program_);gl_.DeleteVertexArrays(1,&ui_vao_);gl_.DeleteBuffers(1,&ui_vbo_);gl_.DeleteTextures(1,&ui_atlas_);
    }
    bool initialize(void*,std::uintptr_t,int w,int h) override{resize(w,h);return true;}
    void resize(int w,int h) override{width_=std::max(1,w);height_=std::max(1,h);}
    void begin_frame(Color clear) override{
        ++frame_;clear_=clear;world_=false;lunar_=false;ui_mode_=false;ui_scale_=1;ui_.vertices.clear();lights_.clear();particles_.clear();lines_.clear();
        environment_={};environment_.fogDensity=0;
        begin_=std::chrono::steady_clock::now();
    }
    void end_frame() override{
        const auto submitted=std::chrono::steady_clock::now();
        std::size_t vertex_count=0;for(const auto& [key,b]:batches_)if(b.last_frame==frame_)vertex_count+=b.vertices.size();
        if(world_){
            renderer_.begin(view_.camera,environment_);
            if(lunar_)for(auto draw:terrain_){draw.modelMatrix=view_.correction;renderer_.submit(draw);}
            for(auto light:lights_){light.position=rf::transformPoint(view_.correction,light.position);renderer_.light(light);}
            for(auto& p:particles_)p.position=rf::transformPoint(view_.correction,p.position);
            if(!particles_.empty())renderer_.particles(particles_);
            for(auto it=batches_.begin();it!=batches_.end();){
                auto& b=it->second;
                if(b.last_frame+120<frame_){if(b.mesh.value)renderer_.destroyMesh(b.mesh);it=batches_.erase(it);continue;}
                if(b.last_frame==frame_ && !b.vertices.empty()){
                    if(!b.mesh.value)b.mesh=renderer_.createMesh(b.vertices);
                    else if(b.previous_vertices.size()!=b.vertices.size() || std::memcmp(b.previous_vertices.data(),b.vertices.data(),b.vertices.size()*sizeof(gfx::Vertex)))renderer_.updateMesh(b.mesh,b.vertices);
                    b.uploaded=b.vertices.size();
                    gfx::Draw draw;draw.mesh=b.mesh;draw.material=b.material;draw.objectId=b.mesh.value;draw.modelMatrix=view_.correction;renderer_.submit(draw);
                }
                ++it;
            }
            renderer_.lines(lines_,view_.correction);
            const auto prepared=std::chrono::steady_clock::now();
            renderer_.render();
            if(std::getenv("GAME_RENDER_PROFILE")){
                const auto& stats=renderer_.statistics();
                std::cout<<"render profile submit="<<std::chrono::duration<double,std::milli>(submitted-begin_).count()
                    <<" upload="<<std::chrono::duration<double,std::milli>(prepared-submitted).count()
                    <<" engine="<<stats.cpuMilliseconds<<" gpu="<<stats.gpuMilliseconds
                    <<" lines="<<stats.lineSegments<<" line_bytes="<<stats.lineUploadBytes<<" vertices="<<vertex_count<<" batches="<<batches_.size()<<std::endl;
            }
        }else{
            renderer_.resetHistory();gl_.BindFramebuffer(vl::GL_FRAMEBUFFER,0);gl_.Viewport(0,0,width_,height_);
            gl_.Disable(vl::GL_SCISSOR_TEST);gl_.ClearColor(clear_.r,clear_.g,clear_.b,clear_.a);gl_.Clear(vl::GL_COLOR_BUFFER_BIT);
        }
        flush_ui();
        if(benchmarking_)gl_.Finish();
        benchmark_ms_=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin_).count();
        if(gl_.GetError()!=vl::GL_NO_ERROR)throw std::runtime_error("JSD colony renderer reported an OpenGL error");
        platform_.present();
    }
    void set_3d(const Mat4& matrix) override{view_=decode_view(matrix);ui_mode_=false;}
    void set_2d(float scale=1) override{ui_scale_=std::max(.01f,scale);ui_mode_=true;}
    void set_graphics_settings(const GraphicsSettings& settings) override{
        const bool sync_changed=settings.vertical_sync!=graphics_.vertical_sync;
        graphics_=settings;graphics_.normalize();const auto next=jsd_bridge::settings(graphics_);
        if(next!=effective_){effective_=next;renderer_.setSettings(next);}
        if(sync_changed)vsync_supported_=platform_.setVSync(graphics_.vertical_sync && !benchmarking_);
    }
    GraphicsCapabilities graphics_capabilities() override{
        GraphicsCapabilities caps;caps.enhanced=true;caps.external_jsd=true;caps.vertical_sync=vsync_supported_;
        caps.supported_features=0;
        for(auto f:{GraphicsFeature::TerrainRelief,GraphicsFeature::MaterialDetail,GraphicsFeature::Decals,
                    GraphicsFeature::LocalLights,GraphicsFeature::ScreenOcclusion,GraphicsFeature::ScreenIndirect,
                    GraphicsFeature::ScreenReflections,GraphicsFeature::Filmic,GraphicsFeature::Vignette,
                    GraphicsFeature::Clouds,GraphicsFeature::Haze,GraphicsFeature::EarthAtmosphere,
                    GraphicsFeature::VolumetricPlumes,GraphicsFeature::VolumetricDust})caps.supported_features|=graphics_bit(f);
        caps.texture_anisotropy=int(renderer_.capabilities().maxAnisotropy);return caps;
    }
    bool supports_graphics_benchmark() const override{return true;}
    void set_graphics_benchmark(bool enabled) override{if(enabled==benchmarking_)return;benchmarking_=enabled;platform_.setVSync(graphics_.vertical_sync && !enabled);}
    double benchmark_frame_ms() const override{return benchmark_ms_;}
    bool begin_lunar_scene(const LunarScene& scene) override{
        lunar_=true;world_=true;view_=decode_view(scene.view_projection);
        environment_.sunDirection=rf::normalized(rf::transformVector(view_.correction,vector(scene.sun_direction)));
        environment_.sunColor={1,.96f,.90f,1};environment_.sunIntensity=2.4f*std::clamp(scene.daylight,0.f,1.f);
        environment_.skyColor={.0003f,.0005f,.0008f,1};environment_.groundColor={.10f,.095f,.09f,1};
        environment_.ambientIntensity=.22f;environment_.fogDensity=0;
        environment_.atmosphere.enabled=false;environment_.stars.enabled=true;
        environment_.stars.intensity=2.0f;environment_.stars.orientation=rf::inverse(view_.correction);
        environment_.shadowDistance=scene.first_person?48.f:std::clamp(scene.eye.y*1.8f,60.f,400.f);
        environment_.atmosphere.time=scene.seconds;
        environment_.deltaSeconds=std::clamp(scene.seconds-previous_seconds_,.001f,.1f);previous_seconds_=scene.seconds;
        for(std::size_t i=0;i<std::min(scene.light_count,scene.lights.size());++i){
            const auto& source=scene.lights[i];gfx::Light light;
            light.position=vector(source.position);light.color=color(source.color);light.radius=source.radius;light.intensity=source.power;
            lights_.push_back(light);
        }
        prepare_terrain(scene.half_extent);return true;
    }
    bool lunar_scene_active() const override{return lunar_;}
    bool draw_campus_atmosphere(const Mat4& vp,Vec3,float seconds) override{
        set_3d(vp);world_=true;environment_={};environment_.fogDensity=.06f;
        environment_.atmosphere.enabled=graphics_.enabled(GraphicsFeature::EarthAtmosphere);environment_.atmosphere.time=seconds;
        environment_.stars.enabled=environment_.atmosphere.enabled;environment_.stars.orientation=rf::inverse(view_.correction);
        environment_.atmosphere.clouds=graphics_.enabled(GraphicsFeature::Clouds)?gfx::Quality::Low:gfx::Quality::Off;
        environment_.fogDensity=graphics_.enabled(GraphicsFeature::Haze)?.06f:0;
        return true;
    }
    bool draw_lunar_volume(const LunarVolume& v) override{
        if(!lunar_)return false;
        const auto axis=normalize(v.axis);
        for(int i=0;i<20;++i){
            const float t=(i+.5f)/20;
            gfx::ParticleSprite p;p.position=vector(v.origin+axis*(v.length*t));
            p.size=std::max(.03f,v.radius*(v.cone?(.2f+.8f*t):1));
            p.color=color(v.tint);p.color.a*=std::clamp(v.density*.16f,0.f,.22f)*(1-t);
            p.style=v.emission>0?gfx::ParticleStyle::Spark:gfx::ParticleStyle::Dust;p.softness=.25f;
            particles_.push_back(p);
        }
        return true;
    }
    void draw_lunar_surface(const std::vector<Vec3>& v,LunarMaterial m,Color c) override{queue(v,int(m),c);}
    void draw_mapped_lunar_surface(const std::vector<Vec3>& v,LunarMaterial m,Color c,const LunarSurfaceMapping& map) override{queue(v,int(m),c,&map);}
    void draw_triangles_3d(const std::vector<Vec3>& v,Color c) override{
        if(ui_mode_){
            const auto byte=[](float x){return static_cast<std::uint8_t>(std::clamp(x,0.f,1.f)*255+.5f);};
            const std::array<std::uint8_t,4> rgba{byte(c.r),byte(c.g),byte(c.b),byte(c.a)};
            for(const auto& p:v)ui_.vertices.push_back({{p.x*ui_scale_,p.y*ui_scale_},{-1,-1},rgba});
        }else queue(v,int(LunarMaterial::Paint),c);
    }
    void draw_material_triangles_3d(const std::vector<Vec3>& v,int m,Color c) override{
        const auto kind=m<2?LunarMaterial::Regolith:m==3?LunarMaterial::Pad:m==4?LunarMaterial::Glass:LunarMaterial::Paint;
        queue(v,int(kind),c);
    }
    void draw_lines_3d(const std::vector<Vec3>& v,Color c,float pixels=1) override{
        if(ui_mode_){
            for(std::size_t i=0;i+1<v.size();i+=2)draw_line({v[i].x,v[i].y},{v[i+1].x,v[i+1].y},c,pixels);
            return;
        }
        if(c.a<=0 || pixels<=0)return;
        world_=true;
        for(std::size_t i=0;i+1<v.size();i+=2){
            if(length(v[i+1]-v[i])<1.e-7f)continue;
            lines_.push_back({vector(v[i]),vector(v[i+1]),color(c),std::clamp(pixels,.25f,128.f)});
        }
    }
    void draw_rect(Rect r,Color c) override{draw_gradient_rect(r,c,c);}
    void draw_gradient_rect(Rect r,Color top,Color bottom) override{ui_.quad({r.x*ui_scale_,r.y*ui_scale_,r.w*ui_scale_,r.h*ui_scale_},top,bottom);}
    void draw_line(Vec2 a,Vec2 b,Color c,float width=1) override{ui_.stroke({a.x*ui_scale_,a.y*ui_scale_},{b.x*ui_scale_,b.y*ui_scale_},c,width*ui_scale_);}
    void draw_text(Vec2 p,const std::string& text,float scale,Color c) override{ui_.text({p.x*ui_scale_,p.y*ui_scale_},text,scale*ui_scale_,c);}
    int width() const override{return width_;}int height() const override{return height_;}
    bool hardware_accelerated() const override{return hardware_;}
    std::string gpu_vendor() const override{return vendor_;}
    std::string gpu_renderer() const override{return renderer_.capabilities().renderer;}
    std::string gpu_version() const override{return renderer_.capabilities().version;}
};
}
std::unique_ptr<SceneRenderer> create_renderer(rf::Platform& platform,std::function<void(const rf::gfx::ShaderBuildProgress&)> progress){return std::make_unique<JsdRenderer>(platform,std::move(progress));}
}
