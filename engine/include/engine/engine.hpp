#pragma once
#include "engine/math.hpp"
#include "engine/graphics_settings.hpp"
#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace eng {
struct Color { float r{},g{},b{},a{1.0f}; };
enum class LunarMaterial { Paint, Alloy, Regolith, Solar, Glass, Fabric, Pad, Emissive, Skin, Marking, Hair };

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

class Renderer {
public:
    virtual ~Renderer() = default;
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
    virtual void draw_rect(Rect r, Color color)=0;
    virtual void draw_line(Vec2 a, Vec2 b, Color color, float width=1.0f)=0;
    virtual void draw_text(Vec2 p, const std::string& text, float scale, Color color)=0;
    virtual int width() const=0;
    virtual int height() const=0;
    virtual bool hardware_accelerated() const=0;
    virtual std::string gpu_vendor() const=0;
    virtual std::string gpu_renderer() const=0;
    virtual std::string gpu_version() const=0;
};

class TaskPool {
public:
    TaskPool();
    ~TaskPool();
    TaskPool(const TaskPool&)=delete;
    TaskPool& operator=(const TaskPool&)=delete;

    static TaskPool& shared();
    std::size_t parallel_lanes() const;
    std::size_t background_workers() const;
    std::future<void> submit(std::function<void()> task);

    // Runs fn(0..count-1) across the calling thread and the helper workers and
    // returns when every index finished. Indices are claimed dynamically, so a
    // helper that never gets scheduled cannot stall the caller; only a helper
    // that claimed an index and was preempted mid-item can delay it.
    template<class Fn>
    void parallel_for(std::size_t count, Fn&& fn){
        if(count==0) return;
        const std::size_t helpers=std::min<std::size_t>(background_workers(),count-1);
        if(helpers==0){
            for(std::size_t i=0;i<count;++i) fn(i);
            return;
        }
        // The functor stays on the caller's stack: helpers only dereference it
        // for an index they claimed, and the caller does not return before every
        // index completed. A late helper finds no index left and never touches it.
        using Callable=std::remove_reference_t<Fn>;
        struct Job {
            std::atomic<std::size_t> next{0};
            std::atomic<std::size_t> done{0};
            std::size_t count{0};
            Callable* fn{nullptr};
        };
        auto job=std::make_shared<Job>();
        job->count=count;
        job->fn=&fn;
        auto run=std::make_shared<std::function<void()>>([job](){
            for(;;){
                const std::size_t i=job->next.fetch_add(1,std::memory_order_relaxed);
                if(i>=job->count) break;
                (*job->fn)(i);
                if(job->done.fetch_add(1,std::memory_order_acq_rel)+1==job->count) job->done.notify_all();
            }
        });
        post_shared(run,helpers);
        (*run)();
        for(std::size_t cur=job->done.load(std::memory_order_acquire);cur<count;cur=job->done.load(std::memory_order_acquire))
            job->done.wait(cur);
    }

private:
    // Queues the same task for `copies` workers with a single lock/allocation.
    void post_shared(const std::shared_ptr<std::function<void()>>& task,std::size_t copies);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

std::size_t cpu_parallel_lanes();

class GameDelegate {
public:
    virtual ~GameDelegate()=default;
    virtual void on_start() {}
    virtual void on_resize(int,int) {}
    virtual void update(float dt,const InputState& input)=0;
    virtual void render(Renderer& r)=0;
};

struct AppConfig {
    int width=1600;
    int height=900;
    std::string title="Game";
    bool fullscreen=false;
};

class Application {
public:
    Application();
    ~Application();
    int run(const AppConfig& config, std::unique_ptr<GameDelegate> game);
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
    Color panel{0.055f,0.065f,0.08f,0.96f};
    Color panel2{0.09f,0.105f,0.13f,0.96f};
    Color text{0.90f,0.94f,0.98f,1.0f};
    Color muted{0.52f,0.60f,0.67f,1.0f};
    Color accent{0.22f,0.72f,0.82f,1.0f};
    Color accent_hover{0.34f,0.83f,0.91f,1.0f};
    Color border{0.20f,0.25f,0.30f,1.0f};
    // Base typography multiplier in logical layout units. Resolution/user zoom
    // is applied once by the shared UI transform, including to this text.
    float font_scale{1.50f};
};
void panel(Renderer& r, Rect rect, const Theme& t);
bool button(Renderer& r,const InputState& input,Rect rect,const std::string& label,const Theme& t,bool enabled=true);
void label(Renderer& r,Vec2 pos,const std::string& text,float scale,const Theme& t,bool muted=false);
void title(Renderer& r,Vec2 pos,const std::string& text,const Theme& t);
}
}
