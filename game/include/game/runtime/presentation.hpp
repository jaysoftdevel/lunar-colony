#pragma once
#include "game/runtime/types.hpp"
#include "game/runtime/jobs.hpp"
#include "game/runtime/graphics_preferences.hpp"
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace game::runtime {
struct Color { float r{},g{},b{},a{1.0f}; };
enum class LunarMaterial { Paint, Alloy, Regolith, Solar, Glass, Fabric, Pad, Emissive, Skin, Marking, Hair, Hull, Interior, Deck, RobotPaint, Textile, Display, Rubber };

struct LunarSurfaceMapping {
    Vec3 origin{};
    Vec3 x{1,0,0},y{0,1,0},z{0,0,1};
    std::uint32_t seed{};
    Vec3 coordinate(Vec3 position) const {
        const auto relative=position-origin;
        return {dot(relative,x),dot(relative,y),dot(relative,z)};
    }
};

struct LunarLight {
    Vec3 position{};
    Color color{};
    float radius{};
    float power{};
};

struct LunarVolume {
    Vec3 origin{};
    Vec3 axis{0,1,0};
    float length{};
    float radius{};
    Color tint{};
    float density{};
    float emission{};
    bool cone{};
    float nozzle_radius{};
    float plume_length{}; // Unclipped reach when the ground shortens length.
};

struct LunarScene {
    Mat4 view_projection{};
    Vec3 eye{};
    Vec3 focus{};
    Vec3 sun_direction{};
    float daylight{};
    float half_extent{};
    std::array<LunarLight,64> lights{};
    std::size_t light_count{};
    float seconds{};
    bool first_person{};
};

struct Rect { float x{},y{},w{},h{}; bool contains(float px,float py) const { return px>=x&&px<=x+w&&py>=y&&py<=y+h; } };
struct DisplayMode {
    int width{};
    int height{};
    int refresh_millihz{};
    bool operator==(const DisplayMode& r) const { return width==r.width && height==r.height; }
};

enum class MouseButton { Left, Middle, Right };
struct InputState {
    int mouse_x{}, mouse_y{};
    int mouse_dx{}, mouse_dy{};
    float wheel{};
    bool left_down{}, middle_down{}, right_down{};
    bool left_pressed{}, middle_pressed{}, right_pressed{};
    bool escape_pressed{};
    bool key_n_pressed{};
    bool key_m_pressed{};
    bool key_v_pressed{}, key_v_down{};
    bool key_w_down{}, key_a_down{}, key_s_down{}, key_d_down{};
    bool key_shift_down{};
    bool key_space_pressed{}, key_space_down{};
    bool key_j_pressed{}, key_j_down{};
    std::uint8_t key_number_pressed{};
    bool delete_pressed{};
    bool backspace_pressed{}, enter_pressed{};
    std::string text_input{};
};

class SceneRenderer {
public:
    virtual ~SceneRenderer() = default;
    // native_display is platform specific (X11 Display* on Linux, unused on
    // Win32). native_window is an integer-sized native window handle so this
    // remains safe on 64-bit Windows where HWND is pointer-sized but unsigned
    // long is only 32 bits.
    virtual bool initialize(void* native_display, std::uintptr_t native_window, int width, int height) = 0;
    virtual void resize(int width,int height)=0;
    virtual void begin_frame(Color clear)=0;
    virtual void end_frame()=0;
    virtual void set_3d(const Mat4& view_proj)=0;
    virtual void set_2d(float scale=1.0f)=0;
    virtual void draw_lines_3d(const std::vector<Vec3>& vertices, Color color, float width=1.0f)=0;
    virtual void draw_triangles_3d(const std::vector<Vec3>& vertices, Color color)=0;
    // Optional high-frequency world-space material shading. The base fallback
    // works on every backend and keeps tests/headless renderers compatible.
    // 0 grass, 1 sand, 2 tarmac, 3 concrete, 4 water, 5 industrial roof.
    virtual void draw_material_triangles_3d(const std::vector<Vec3>& vertices, int material, Color tint){
        (void)material;
        draw_triangles_3d(vertices,tint);
    }
    virtual void set_graphics_settings(const GraphicsSettings&) {}
    virtual GraphicsCapabilities graphics_capabilities() { return {}; }
    // Benchmark samples cover the completed rendering work before presentation.
    virtual bool supports_graphics_benchmark() const { return false; }
    virtual void set_graphics_benchmark(bool) {}
    virtual double benchmark_frame_ms() const { return 0.0; }
    // Optional lunar scene pass. Other backends retain the standard mesh view.
    virtual bool begin_lunar_scene(const LunarScene&) { return false; }
    virtual bool lunar_scene_active() const { return false; }
    virtual bool draw_lunar_volume(const LunarVolume&) { return false; }
    virtual bool draw_campus_atmosphere(const Mat4&,Vec3,float){ return false; }
    virtual void draw_lunar_surface(const std::vector<Vec3>& vertices,LunarMaterial,Color tint){
        draw_triangles_3d(vertices,tint);
    }
    virtual void draw_mapped_lunar_surface(const std::vector<Vec3>& vertices,LunarMaterial material,Color tint,
                                          const LunarSurfaceMapping&){
        draw_lunar_surface(vertices,material,tint);
    }
    virtual void draw_rect(Rect r, Color color)=0;
    virtual void draw_gradient_rect(Rect r,Color top,Color){ draw_rect(r,top); }
    virtual void draw_line(Vec2 a, Vec2 b, Color color, float width=1.0f)=0;
    virtual void draw_text(Vec2 p, const std::string& text, float scale, Color color)=0;
    virtual int width() const=0;
    virtual int height() const=0;
    virtual bool hardware_accelerated() const=0;
    virtual std::string gpu_vendor() const=0;
    virtual std::string gpu_renderer() const=0;
    virtual std::string gpu_version() const=0;
};

class GameLoop {
public:
    virtual ~GameLoop()=default;
    virtual void on_start() {}
    virtual void on_resize(int,int) {}
    virtual void update(float dt,const InputState& input)=0;
    virtual void render(SceneRenderer& r)=0;
};

struct WindowConfig {
    int width=1600;
    int height=900;
    std::string title="Game";
    bool fullscreen=false;
};

class GameHost {
public:
    GameHost();
    ~GameHost();
    int run(const WindowConfig& config, std::unique_ptr<GameLoop> game);
    void request_quit();
    void set_fullscreen(bool enabled);
    bool fullscreen() const;
    std::vector<DisplayMode> display_modes() const;
    DisplayMode current_display_mode() const;
    bool set_display_mode(int width,int height);
    void set_cursor_visible(bool visible);
    bool cursor_visible() const;
    void set_relative_mouse_mode(bool enabled);
    bool relative_mouse_mode() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

namespace ui {
struct Theme {
    Color panel{0.047f,0.065f,0.086f,0.96f};
    Color panel2{0.085f,0.12f,0.15f,0.96f};
    Color text{0.92f,0.96f,0.98f,1.0f};
    Color muted{0.64f,0.72f,0.78f,1.0f};
    Color accent{0.39f,0.85f,0.78f,1.0f};
    Color accent_hover{0.17f,0.31f,0.35f,1.0f};
    Color border{0.20f,0.30f,0.35f,1.0f};
    // Base typography multiplier in logical layout units. Resolution/user zoom
    // is applied once by the shared UI transform, including to this text.
    float font_scale{1.50f};
};
void panel(SceneRenderer& r, Rect rect, const Theme& t);
bool button(SceneRenderer& r,const InputState& input,Rect rect,const std::string& label,const Theme& t,bool enabled=true);
void label(SceneRenderer& r,Vec2 pos,const std::string& text,float scale,const Theme& t,bool muted=false);
void title(SceneRenderer& r,Vec2 pos,const std::string& text,const Theme& t);
}
}
