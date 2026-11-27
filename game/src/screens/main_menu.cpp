#include "game/game_app.hpp"
#include "game/version.hpp"
#include <jsd/game_shell.hpp>

namespace game {
using namespace game::runtime;
namespace shell = jsd::shell;

namespace {
class MenuCanvas final : public shell::Canvas {
    SceneRenderer& renderer;
    std::vector<Vec3> vertices;

    static Color color(rf::Color c) { return {c.r,c.g,c.b,c.a}; }

public:
    explicit MenuCanvas(SceneRenderer& r) : renderer(r) {}

    void gradient(shell::Rect box, rf::Color top, rf::Color bottom) override {
        renderer.draw_gradient_rect({box.x,box.y,box.w,box.h},color(top),color(bottom));
    }

    void line(shell::Point a, shell::Point b, rf::Color tint, float width) override {
        renderer.draw_line({a.x,a.y},{b.x,b.y},color(tint),width);
    }

    void triangles(std::span<const rf::V3> positions, rf::Color tint) override {
        vertices.clear();
        vertices.reserve(positions.size());
        for (auto p : positions) vertices.push_back({p.x,p.y,p.z});
        renderer.draw_triangles_3d(vertices,color(tint));
    }

    void text(shell::Point p, std::string_view value, float scale, rf::Color tint) override {
        renderer.draw_text({p.x,p.y},std::string(value),scale,color(tint));
    }
};
}

void GameApp::render_menu(SceneRenderer& r) {
    const auto recent=std::find_if(save_games_.begin(),save_games_.end(),[](const SaveGameEntry& entry){
        return build::debug_features || !entry.debug_only;
    });
    const bool can_continue=!game_session_active_ && recent!=save_games_.end();

    const shell::TitleItems title({game_session_active_,true,can_continue});

    std::array<shell::ContextLine,4> context{};
    std::size_t lines=3;
    std::string detail;
    if (game_session_active_) {
        context={{{"ACTIVE SESSION",0,1,true},{game_phase_short_name(active_game_phase_),32,1.55f},
                  {save_game_name_.empty()?std::string_view("UNSAVED COLONY"):std::string_view(save_game_name_),65,1,true}}};
    } else if (can_continue) {
        detail="SAVED WITH V"+recent->version;
        context={{{"MOST RECENT SAVE",0,1,true},{recent->name,32,1.55f},{detail,65,1,true}}};
    } else {
        context={{{"CAMPAIGN",0,1.55f},{"PLAN AND LAUNCH A LUNAR EXPEDITION",32,1,true},
                  {"FREE PLAY",76,1.55f},{"START AT A PLAYABLE STAGE",108,1,true}}};
        lines=4;
    }

    const std::string release=std::string(build::debug_features?"DEBUG  /  V":"RELEASE  /  V")+std::string(version::current);
    const shell::Page page{"LUNAR FRONTIER",title.items(),std::span(context).first(lines),
                           release,save_status_message_};
    const auto color=[](Color c) { return rf::Color{c.r,c.g,c.b,c.a}; };
    const shell::Theme theme{color(theme_.panel),color(theme_.panel2),color(theme_.text),color(theme_.muted),
                             color(theme_.accent),color(theme_.accent_hover),color(theme_.border),theme_.font_scale};
    const shell::Input input{{float(ui_input_.mouse_x),float(ui_input_.mouse_y)},ui_input_.left_pressed,
                             ui_input_.enter_pressed,ui_input_.edit_up,ui_input_.edit_down};
    MenuCanvas canvas(r);
    const auto selected=title_menu_.draw(canvas,ui_width(r),ui_height(r),page,input,theme);
    if (!selected)
        return;

    title_menu_.reset();
    switch (title.action(*selected)) {
    case shell::TitleAction::Resume:
        screen_=Screen::Playfield;
        break;
    case shell::TitleAction::Save:
        save_status_message_.clear();refresh_save_games();save_list_scroll_=0;save_list_selected_=-1;
        save_name_editor_.reset(save_game_name_,true);save_name_focus_=true;save_name_drag_=false;save_name_scroll_=0;save_caret_epoch_=ui_time_seconds_;
        screen_=Screen::SaveGame;
        break;
    case shell::TitleAction::Continue:
        resume_save(*recent);
        break;
    case shell::TitleAction::NewGame:
        new_game_mode_selection_=NewGameMode::Career;
        new_game_phase_selection_=GamePhase::Planning;
        new_game_status_message_.clear();
        screen_=Screen::NewGame;
        break;
    case shell::TitleAction::Load:
        save_status_message_.clear();
        refresh_save_games();
        load_save_scroll_=0;
        screen_=Screen::LoadGame;
        break;
    case shell::TitleAction::Settings:
        ensure_display_modes();
        screen_=Screen::Settings;
        break;
    case shell::TitleAction::Credits:
        screen_=Screen::Credits;
        break;
    case shell::TitleAction::Exit:
        app_.request_quit();
        break;
    }
}
} // namespace game
