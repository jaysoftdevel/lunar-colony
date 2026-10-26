#pragma once
#include "engine/math.hpp"
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace eng {
struct Color { float r{},g{},b{},a{1.0f}; };
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
};

class Renderer {
public:
    virtual ~Renderer() = default;
    virtual bool initialize(void* native_display, unsigned long native_window, int width, int height) = 0;
    virtual void resize(int width,int height)=0;
    virtual void begin_frame(Color clear)=0;
    virtual void end_frame()=0;
    virtual void set_3d(const Mat4& view_proj)=0;
    virtual void set_2d(float scale=1.0f)=0;
    virtual void draw_lines_3d(const std::vector<Vec3>& vertices, Color color, float width=1.0f)=0;
    virtual void draw_triangles_3d(const std::vector<Vec3>& vertices, Color color)=0;
    virtual void draw_rect(Rect r, Color color)=0;
    virtual void draw_line(Vec2 a, Vec2 b, Color color, float width=1.0f)=0;
    virtual void draw_text(Vec2 p, const std::string& text, float scale, Color color)=0;
    virtual int width() const=0;
    virtual int height() const=0;
};

class GameDelegate {
public:
    virtual ~GameDelegate()=default;
    virtual void on_start() {}
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
    // Independent from Renderer::set_2d() / UI geometry scaling.
    // Base text size is 50% larger than the original prototype without
    // changing panel/control geometry. Renderer::set_2d() remains the
    // independent user-configurable UI geometry scale.
    float font_scale{1.50f};
};
void panel(Renderer& r, Rect rect, const Theme& t);
bool button(Renderer& r,const InputState& input,Rect rect,const std::string& label,const Theme& t,bool enabled=true);
void label(Renderer& r,Vec2 pos,const std::string& text,float scale,const Theme& t,bool muted=false);
void title(Renderer& r,Vec2 pos,const std::string& text,const Theme& t);
}
}
