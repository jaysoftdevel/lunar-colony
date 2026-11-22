#include "jsd_bridge.hpp"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
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
}
namespace game::runtime {
struct GameHost::Impl {
    std::unique_ptr<rf::Platform> platform;
    std::unique_ptr<SceneRenderer> renderer;
    InputState input;
    bool quit{},cursor=true,relative{},focused=true,selected_mode{};
    int width{},height{};
    void capture(){
        if(platform && platform->window){
            platform->captureMouse(relative && focused);
            SDL_ShowCursor(cursor || !focused?1:0);
        }
    }
    int display() const{return platform && platform->window?std::max(0,SDL_GetWindowDisplayIndex(platform->window)):0;}
};
GameHost::GameHost()=default;
GameHost::~GameHost()=default;
int GameHost::run(const WindowConfig& config,std::unique_ptr<GameLoop> game){
    try{
        impl_=std::make_unique<Impl>();auto& p=*impl_;
        p.platform=std::make_unique<rf::Platform>(false,config.width,config.height);
        SDL_SetWindowTitle(p.platform->window,config.title.c_str());
        p.platform->dimensions();p.width=p.platform->width;p.height=p.platform->height;
        p.renderer=jsd_bridge::create_renderer(*p.platform);
        set_fullscreen(config.fullscreen);SDL_StartTextInput();
        std::cout<<cpu_worker_summary()<<'\n';
        game->on_start();auto previous=std::chrono::steady_clock::now();
        int limit=0;
        if(const char* value=std::getenv("GAME_FRAME_LIMIT_FPS"))limit=std::clamp(std::atoi(value),0,1000);
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
                if(event.type==vl::sdl::keyDown && !event.key.repeat){
                    const int key=event.key.keysym.scancode;
                    if(key==41)in.escape_pressed=true;
                    if(key==40 || key==88)in.enter_pressed=true;
                    if(key==42)in.backspace_pressed=true;
                    if(key==76)in.delete_pressed=true;
                    if(key==17)in.key_n_pressed=true;
                    if(key==16)in.key_m_pressed=true;
                    if(key==25)in.key_v_pressed=true;
                    if(key==44)in.key_space_pressed=true;
                    if(key==13)in.key_j_pressed=true;
                    // Ignore Control/Alt combinations, including speed shortcuts.
                    if(!(event.key.keysym.mod & (0x00c0|0x0300))){
                        if(key>=30 && key<=32)in.key_number_pressed=static_cast<std::uint8_t>(key-29);
                        if(key>=89 && key<=91)in.key_number_pressed=static_cast<std::uint8_t>(key-88);
                    }
                }
            }
            p.platform->dimensions();
            const int w=p.platform->width,h=p.platform->height;
            if(w!=p.width || h!=p.height){p.width=w;p.height=h;p.renderer->resize(w,h);}
            if(p.focused){
                const auto* keys=SDL_GetKeyboardState(nullptr);
                in.key_w_down=keys[26];in.key_a_down=keys[4];in.key_s_down=keys[22];in.key_d_down=keys[7];
                in.key_shift_down=keys[225] || keys[229];in.key_v_down=keys[25];in.key_space_down=keys[44];in.key_j_down=keys[13];
                int x=0,y=0,ww=1,wh=1;const auto buttons=SDL_GetMouseState(&x,&y);SDL_GetWindowSize(p.platform->window,&ww,&wh);
                in.mouse_x=x*w/std::max(1,ww);in.mouse_y=y*h/std::max(1,wh);
                in.left_down=(buttons&1)!=0;in.middle_down=(buttons&2)!=0;in.right_down=(buttons&4)!=0;
                if(p.relative){in.mouse_x=w/2;in.mouse_y=h/2;}
            }
            const auto now=std::chrono::steady_clock::now();const float dt=std::clamp(std::chrono::duration<float>(now-previous).count(),0.f,.1f);previous=now;
            game->on_resize(w,h);game->update(dt,in);game->render(*p.renderer);
            if(limit>0){
                const auto budget=std::chrono::duration<double>(1.0/limit);
                const auto elapsed=std::chrono::steady_clock::now()-frame_begin;
                if(elapsed<budget)std::this_thread::sleep_for(budget-elapsed);
            }
        }
        SDL_StopTextInput();p.platform->captureMouse(false);
        // GL objects must be released while the engine context is still live.
        p.renderer.reset();game.reset();p.platform.reset();return 0;
    }catch(const std::exception& e){
        std::cerr<<"Could not run Lunar Colony with JSD: "<<e.what()<<'\n';game.reset();impl_.reset();return 3;
    }
}
void GameHost::request_quit(){if(impl_)impl_->quit=true;}
void GameHost::set_fullscreen(bool enabled){
    if(!impl_ || !impl_->platform || !impl_->platform->window)return;
    auto& p=*impl_;
    SDL_SetWindowFullscreen(p.platform->window,enabled?(p.selected_mode?1:vl::sdl::fullscreen):0);
}
bool GameHost::fullscreen() const{return impl_ && impl_->platform && impl_->platform->isFullscreen();}
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
void GameHost::set_cursor_visible(bool visible){if(impl_ && impl_->cursor!=visible){impl_->cursor=visible;impl_->capture();}}
bool GameHost::cursor_visible() const{return !impl_ || impl_->cursor;}
void GameHost::set_relative_mouse_mode(bool enabled){if(impl_ && impl_->relative!=enabled){impl_->relative=enabled;impl_->capture();SDL_GetRelativeMouseState(nullptr,nullptr);}}
bool GameHost::relative_mouse_mode() const{return impl_ && impl_->relative;}
}
