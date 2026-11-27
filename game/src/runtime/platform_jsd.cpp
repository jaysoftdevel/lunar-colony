#include "jsd_bridge.hpp"
#include <jsd/game_shell.hpp>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <stdexcept>
#include <tuple>

// Additional SDL2 public ABI declarations; frontier/platform.hpp supplies the
// engine's SDL subset. Including SDL.h as well would redefine that public ABI.
struct SDL_DisplayMode { U32 format; int w,h,refresh_rate; void* driverdata; };
extern "C" {
int SDL_GetWindowDisplayIndex(SDL_Window*);
int SDL_GetNumDisplayModes(int);
int SDL_GetDisplayMode(int,int,SDL_DisplayMode*);
int SDL_GetCurrentDisplayMode(int,SDL_DisplayMode*);
int SDL_SetWindowDisplayMode(SDL_Window*,const SDL_DisplayMode*);
void SDL_SetWindowSize(SDL_Window*,int,int);
struct SDL_Cursor;
SDL_Cursor* SDL_CreateSystemCursor(int);void SDL_FreeCursor(SDL_Cursor*);
struct SDL_Surface;
SDL_Surface* SDL_CreateRGBSurfaceFrom(void*,int,int,int,int,U32,U32,U32,U32);
SDL_Cursor* SDL_CreateColorCursor(SDL_Surface*,int,int);void SDL_FreeSurface(SDL_Surface*);
void SDL_SetCursor(SDL_Cursor*);SDL_Cursor* SDL_GetDefaultCursor();
SDL_Window* SDL_GetMouseFocus();
}
namespace game::runtime {
struct GameHost::Impl {
    std::unique_ptr<rf::Platform> platform;
    std::unique_ptr<SceneRenderer> renderer;
    InputState input;
    bool quit{},cursor=true,relative{},focused=true,selected_mode{},world_cursor{};
    SDL_Cursor* crosshair{};
    ~Impl(){if(crosshair)SDL_FreeCursor(crosshair);}
    int width{},height{};
    void capture(){
        if(platform && platform->window){
            platform->captureMouse(relative && focused);
            SDL_SetCursor(world_cursor && focused && crosshair?crosshair:SDL_GetDefaultCursor());
            SDL_ShowCursor(cursor || !focused?1:0);
        }
    }
    int display() const{return platform && platform->window?std::max(0,SDL_GetWindowDisplayIndex(platform->window)):0;}
};
GameHost::GameHost()=default;
GameHost::~GameHost()=default;
int GameHost::run(const WindowConfig& config,std::unique_ptr<GameLoop> game){
    return run(config,[&game]{return std::move(game);});
}
int GameHost::run(const WindowConfig& config,const std::function<std::unique_ptr<GameLoop>()>& create_game){
    std::unique_ptr<GameLoop> game;
    try{
        impl_=std::make_unique<Impl>();auto& p=*impl_;
        p.platform=std::make_unique<rf::Platform>(false,config.width,config.height,config.fullscreen,config.title);
        SDL_SetWindowTitle(p.platform->window,config.title.c_str());
        if(config.fullscreen&&!fullscreen()){
            set_fullscreen(true);
            if(!fullscreen())throw std::runtime_error(std::string("Unable to enter fullscreen: ")+SDL_GetError());
        }
        p.platform->dimensions();p.width=p.platform->width;p.height=p.platform->height;
        {
            // A minimal splash precedes both renderer/shader work and game assets.
            p.platform->setVSync(false);
            jsd::shell::Window splash(*p.platform,"LUNAR COLONY");
            splash.showStartup("Starting Lunar Colony",{},true);
            p.renderer=jsd_bridge::create_renderer(*p.platform,splash.rendererStartup().progress);
            splash.showStartup("Preparing game assets",{},true);
            game=create_game();if(!game)throw std::runtime_error("Game factory returned no game");
            splash.showStartup("Starting colony systems",{},true);
            p.platform->dimensions();p.width=p.platform->width;p.height=p.platform->height;p.renderer->resize(p.width,p.height);
            game->on_resize(p.width,p.height);game->on_start();
            if(config.fullscreen&&!fullscreen()){
                set_fullscreen(true);
                if(!fullscreen())throw std::runtime_error(std::string("Fullscreen startup failed: ")+SDL_GetError());
            }
            // Preferences may select a display mode. Present the ready frame at
            // the resulting drawable size, including on high-DPI Windows displays.
            p.platform->dimensions();p.width=p.platform->width;p.height=p.platform->height;
            p.renderer->resize(p.width,p.height);game->on_resize(p.width,p.height);
            splash.showStartup("Preparing first frame",{},true);
            game->render(*p.renderer); // Replace the splash only with a completed frame.
        }
        SDL_StartTextInput();std::cout<<cpu_worker_summary()<<'\n';
        auto previous=std::chrono::steady_clock::now();
        int limit=0;
        if(const char* value=std::getenv("GAME_FRAME_LIMIT_FPS"))limit=std::clamp(std::atoi(value),0,1000);
        bool previous_delete=false;
        bool previous_digits[3]{};
        while(!p.quit){
            const auto frame_begin=std::chrono::steady_clock::now();
            auto& in=p.input;
            const int mx=in.mouse_x,my=in.mouse_y;
            in={};in.mouse_x=mx;in.mouse_y=my;
            SDL_Event event{};
            while(SDL_PollEvent(&event)){
                if(event.type==vl::sdl::quit){p.quit=true;continue;}
                if(event.type==vl::sdl::window){
                    if(event.window.event==13){p.focused=false;in={};p.capture();}
                    if(event.window.event==12){p.focused=true;p.capture();}
                }
                if(!p.focused)continue;
                if(event.type==vl::sdl::textInput)in.text_input+=event.text.text;
                if(event.type==vl::sdl::mouseMotion){in.mouse_dx+=event.motion.xrel;in.mouse_dy+=event.motion.yrel;}
                if(event.type==vl::sdl::mouseWheel)in.wheel+=float(event.wheel.y)*(event.wheel.direction==1?-1:1);
                if(event.type==vl::sdl::mouseDown){
                    if(event.button.button==1)in.left_pressed=true;
                    if(event.button.button==2)in.middle_pressed=true;
                    if(event.button.button==3)in.right_pressed=true;
                }
                if(event.type==vl::sdl::keyDown){
                    const auto key=event.key.keysym.scancode;const bool control=(event.key.keysym.mod&0x00c0)!=0;
                    in.key_ctrl_down|=control;in.key_shift_down|=(event.key.keysym.mod&3)!=0;
                    in.edit_left|=key==80;in.edit_right|=key==79;in.edit_home|=key==74;in.edit_end|=key==77;
                    in.edit_up|=key==82;in.edit_down|=key==81;in.edit_insert|=key==73&&!event.key.repeat;
                    in.backspace_pressed|=key==42;in.delete_pressed|=key==76;
                    if(control&&!event.key.repeat){in.edit_select_all|=key==4;in.edit_copy|=key==6;in.edit_cut|=key==27;in.edit_paste|=key==25;
                        in.edit_undo|=key==29&&!(event.key.keysym.mod&3);in.edit_redo|=key==28||(key==29&&(event.key.keysym.mod&3));}
                }
                if(event.type==vl::sdl::keyDown && !event.key.repeat){
                    const int key=event.key.keysym.scancode;
                    if(key==41)in.escape_pressed=true;
                    if(key==43)in.key_tab_pressed=true;
                    if(key==15 && !(event.key.keysym.mod & (0x00c0|0x0300)))in.key_l_pressed=true;
                    if(key==8)in.key_e_pressed=true;
                    if(key==40 || key==88)in.enter_pressed=true;
                    if(key==42)in.backspace_pressed=true;
                    if(key==76 || event.key.keysym.sym==127)in.delete_pressed=true;
                    if(key==17)in.key_n_pressed=true;
                    if(key==16)in.key_m_pressed=true;
                    if(key==25)in.key_v_pressed=true;
                    if(key==44)in.key_space_pressed=true;
                    if(key==13)in.key_j_pressed=true;
                    // Ignore Control/Alt combinations, including speed shortcuts.
                    if(!(event.key.keysym.mod & (0x00c0|0x0300))){
                        if(key>=30 && key<=32)in.key_number_pressed=static_cast<std::uint8_t>(key-29);
                        if(key>=89 && key<=91)in.key_number_pressed=static_cast<std::uint8_t>(key-88);
                        const int sym=event.key.keysym.sym;
                        if(sym>='1' && sym<='3')in.key_number_pressed=static_cast<std::uint8_t>(sym-'0');
                    }
                }
            }
            p.platform->dimensions();
            const int w=p.platform->width,h=p.platform->height;
            if(w!=p.width || h!=p.height){p.width=w;p.height=h;p.renderer->resize(w,h);}
            in.window_focused=p.focused;in.pointer_inside_window=SDL_GetMouseFocus()==p.platform->window;
            if(p.focused){
                const auto* keys=SDL_GetKeyboardState(nullptr);
                in.key_w_down=keys[26];in.key_a_down=keys[4];in.key_s_down=keys[22];in.key_d_down=keys[7];
                in.key_alt_down=keys[226] || keys[230];in.key_shift_down|=keys[225] || keys[229];in.key_ctrl_down|=keys[224] || keys[228];in.key_v_down=keys[25];in.key_space_down=keys[44];in.key_j_down=keys[13];in.key_e_down=keys[8];
                // A press edge can still be recovered from SDL's state when a
                // backend drops its key-down notification (e.g. after focus changes).
                for(int i=0;i<3;++i){const bool down=keys[30+i]||keys[89+i];
                    if(down&&!previous_digits[i]&&!in.key_ctrl_down&&!in.key_alt_down)in.key_number_pressed=static_cast<std::uint8_t>(i+1);
                    previous_digits[i]=down;
                }
                in.delete_pressed|=keys[76]&&!previous_delete;previous_delete=keys[76];
                int x=0,y=0,ww=1,wh=1;const auto buttons=SDL_GetMouseState(&x,&y);SDL_GetWindowSize(p.platform->window,&ww,&wh);
                in.mouse_x=x*w/std::max(1,ww);in.mouse_y=y*h/std::max(1,wh);
                in.left_down=(buttons&1)!=0;in.middle_down=(buttons&2)!=0;in.right_down=(buttons&4)!=0;
                if(p.relative){in.mouse_x=w/2;in.mouse_y=h/2;}
            }
            if(!p.focused){previous_delete=false;for(auto& down:previous_digits)down=false;}
            const auto now=std::chrono::steady_clock::now();in.frame_seconds=std::max(0.f,std::chrono::duration<float>(now-previous).count());
            const float dt=std::clamp(in.frame_seconds,0.f,.1f);previous=now;
            game->on_resize(w,h);game->update(dt,in);game->render(*p.renderer);
            if(limit>0){
                const auto budget=std::chrono::duration<double>(1.0/limit);
                const auto elapsed=std::chrono::steady_clock::now()-frame_begin;
                if(elapsed<budget)std::this_thread::sleep_for(budget-elapsed);
            }
        }
        SDL_StopTextInput();p.platform->captureMouse(false);
        // GL objects must be released while the engine context is still live.
        p.renderer.reset();game.reset();if(p.crosshair){SDL_SetCursor(SDL_GetDefaultCursor());SDL_FreeCursor(p.crosshair);p.crosshair=nullptr;}p.platform.reset();return 0;
    }catch(const jsd::shell::StartupCancelled&){game.reset();impl_.reset();return 0;
    }catch(const std::exception& e){
        std::cerr<<"Could not run Lunar Colony with JSD: "<<e.what()<<'\n';game.reset();impl_.reset();return 3;
    }
}
void GameHost::request_quit(){if(impl_)impl_->quit=true;}
void GameHost::set_fullscreen(bool enabled){
    if(!impl_ || !impl_->platform || !impl_->platform->window)return;
    auto& p=*impl_;
    if(SDL_SetWindowFullscreen(p.platform->window,enabled?(p.selected_mode?1:vl::sdl::fullscreen):0)<0&&enabled&&p.selected_mode){
        // A selected mode may disappear after a monitor/driver change.
        p.selected_mode=false;
        SDL_SetWindowFullscreen(p.platform->window,vl::sdl::fullscreen);
    }
}
bool GameHost::fullscreen() const{
    // SDL_WINDOW_FULLSCREEN_DESKTOP includes bit 1; exclusive fullscreen is
    // bit 1 alone after a resolution change. Both are fullscreen.
    return impl_&&impl_->platform&&impl_->platform->window&&(SDL_GetWindowFlags(impl_->platform->window)&1u)!=0;
}
std::vector<DisplayMode> GameHost::display_modes() const{
    std::vector<DisplayMode> result;
    if(!impl_ || !impl_->platform)return result;
    const int display=impl_->display(),count=SDL_GetNumDisplayModes(display);
    for(int i=0;i<count;++i){
        SDL_DisplayMode mode{};
        if(SDL_GetDisplayMode(display,i,&mode)!=0)continue;
        DisplayMode item{mode.w,mode.h,mode.refresh_rate*1000};
        auto found=std::find(result.begin(),result.end(),item);
        if(found==result.end())result.push_back(item);else found->refresh_millihz=std::max(found->refresh_millihz,item.refresh_millihz);
    }
    std::sort(result.begin(),result.end(),[](const auto& a,const auto& b){return std::tie(a.width,a.height)<std::tie(b.width,b.height);});return result;
}
DisplayMode GameHost::current_display_mode() const{
    if(!impl_ || !impl_->platform)return {};
    SDL_DisplayMode mode{};
    if(SDL_GetCurrentDisplayMode(impl_->display(),&mode)!=0)return {};
    return {mode.w,mode.h,mode.refresh_rate*1000};
}
bool GameHost::set_display_mode(int width,int height){
    if(!impl_ || !impl_->platform || !impl_->platform->window)return false;
    auto& p=*impl_;SDL_DisplayMode selected{};
    for(int i=0;i<SDL_GetNumDisplayModes(p.display());++i){
        SDL_DisplayMode mode{};
        if(SDL_GetDisplayMode(p.display(),i,&mode)==0 && mode.w==width && mode.h==height && mode.refresh_rate>=selected.refresh_rate)selected=mode;
    }
    if(!selected.w || SDL_SetWindowDisplayMode(p.platform->window,&selected)!=0)return false;
    p.selected_mode=true;
    if(fullscreen()){if(SDL_SetWindowFullscreen(p.platform->window,1)!=0)return false;}
    else SDL_SetWindowSize(p.platform->window,width,height);
    return true;
}
void GameHost::set_world_cursor(bool world){
    if(!impl_ || !impl_->platform || !impl_->platform->window)return;
    auto& p=*impl_;
    if(world && !p.crosshair){
        constexpr int size=40,mid=size/2;
        std::array<U32,size*size> pixels{};
        // Native RGBA cursor: black halo, luminous green arms, amber center.
        // Explicit channel masks describe packed 32-bit colors to SDL.
        constexpr U32 outline=0xff080b0d,green=0xff35ff7b,amber=0xffffe62a;
        for(int y=0;y<size;++y)for(int x=0;x<size;++x){const int dx=std::abs(x-mid),dy=std::abs(y-mid);
            if((dx<=3&&dy>=3&&dy<=17)||(dy<=3&&dx>=3&&dx<=17))pixels[y*size+x]=outline;
            if((dx<=1&&dy>=5&&dy<=15)||(dy<=1&&dx>=5&&dx<=15))pixels[y*size+x]=green;
            if(dx<=2&&dy<=2)pixels[y*size+x]=outline;
            if(dx<=1&&dy<=1)pixels[y*size+x]=amber;
        }
        if(auto* surface=SDL_CreateRGBSurfaceFrom(pixels.data(),size,size,32,size*4,0x00ff0000,0x0000ff00,0x000000ff,0xff000000)){
            p.crosshair=SDL_CreateColorCursor(surface,mid,mid);SDL_FreeSurface(surface);
        }
        if(!p.crosshair)p.crosshair=SDL_CreateSystemCursor(3);
    }
    if(p.world_cursor!=world){p.world_cursor=world;p.capture();}
}
void GameHost::set_cursor_visible(bool visible){if(impl_ && impl_->cursor!=visible){impl_->cursor=visible;impl_->capture();}}
bool GameHost::cursor_visible() const{return !impl_ || impl_->cursor;}
void GameHost::set_relative_mouse_mode(bool enabled){if(impl_ && impl_->relative!=enabled){impl_->relative=enabled;impl_->capture();SDL_GetRelativeMouseState(nullptr,nullptr);}}
bool GameHost::relative_mouse_mode() const{return impl_ && impl_->relative;}
std::string GameHost::clipboard_text() const {
    if(!impl_||!impl_->platform)return local_clipboard_;
    char* text=SDL_GetClipboardText();if(!text)return local_clipboard_;
    std::string result(text);SDL_free(text);return result;
}
void GameHost::set_clipboard_text(const std::string& text){local_clipboard_=text;if(impl_&&impl_->platform)SDL_SetClipboardText(text.c_str());}

}
