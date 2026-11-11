#pragma once
#include "engine/engine.hpp"

namespace eng {

// The OpenGL experiment accepts a resolver from the active platform context.
// All GPU work is confined to the lunar world pass, before the regular HUD.
class LunarRenderer {
public:
    using Resolve = std::function<void*(const char*)>;

    LunarRenderer();
    ~LunarRenderer();
    bool initialize(const Resolve& resolve);
    bool begin(const LunarScene& scene,int width,int height);
    bool active() const;
    void set_graphics_settings(const GraphicsSettings& settings);
    GraphicsCapabilities graphics_capabilities() const;
    bool volume(const LunarVolume& volume);
    void invalidate_history();
    bool campus_atmosphere(const Mat4& view_projection,Vec3 eye,float seconds,int width,int height);
    void triangles(const std::vector<Vec3>& vertices,LunarMaterial material,Color tint);
    void triangles(const std::vector<Vec3>& vertices,LunarMaterial material,Color tint,const LunarSurfaceMapping& mapping);
    void lines(const std::vector<Vec3>& vertices,Color tint,float width);
    void finish();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace eng
