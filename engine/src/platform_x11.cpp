#include "engine/engine.hpp"
#include <X11/Xatom.h>
#include <X11/Xlib.h>
#include <X11/cursorfont.h>
#include <X11/Xutil.h>
#include <X11/XKBlib.h>
#include <X11/keysym.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <dlfcn.h>
#include <iostream>
#include <thread>
#include <vector>

namespace eng { std::unique_ptr<Renderer> create_gl_renderer(); }

namespace eng {
namespace {
using RROutput=unsigned long;
using RRCrtc=unsigned long;
using RRMode=unsigned long;
using Rotation=unsigned short;
using Connection=unsigned short;
using SubpixelOrder=unsigned short;
using XRRModeFlags=unsigned long;
constexpr Connection RR_CONNECTED=0;

struct XRRModeInfoLite {
    RRMode id;
    unsigned int width, height;
    unsigned long dotClock;
    unsigned int hSyncStart, hSyncEnd, hTotal, hSkew;
    unsigned int vSyncStart, vSyncEnd, vTotal;
    char* name;
    unsigned int nameLength;
    XRRModeFlags modeFlags;
};
struct XRRScreenResourcesLite {
    Time timestamp;
    Time configTimestamp;
    int ncrtc;
    RRCrtc* crtcs;
    int noutput;
    RROutput* outputs;
    int nmode;
    XRRModeInfoLite* modes;
};
struct XRROutputInfoLite {
    Time timestamp;
    RRCrtc crtc;
    char* name;
    int nameLen;
    unsigned long mm_width, mm_height;
    Connection connection;
    SubpixelOrder subpixel_order;
    int ncrtc;
    RRCrtc* crtcs;
    int nclone;
    RROutput* clones;
    int nmode;
    int npreferred;
    RRMode* modes;
};
struct XRRCrtcInfoLite {
    Time timestamp;
    int x,y;
    unsigned int width,height;
    RRMode mode;
    Rotation rotation;
    int noutput;
    RROutput* outputs;
    Rotation rotations;
    int npossible;
    RROutput* possible;
};

struct RandRApi {
    void* lib=nullptr;
    XRRScreenResourcesLite* (*get_resources)(Display*,Window)=nullptr;
    void (*free_resources)(XRRScreenResourcesLite*)=nullptr;
    RROutput (*get_primary)(Display*,Window)=nullptr;
    XRROutputInfoLite* (*get_output_info)(Display*,XRRScreenResourcesLite*,RROutput)=nullptr;
    void (*free_output_info)(XRROutputInfoLite*)=nullptr;
    XRRCrtcInfoLite* (*get_crtc_info)(Display*,XRRScreenResourcesLite*,RRCrtc)=nullptr;
    void (*free_crtc_info)(XRRCrtcInfoLite*)=nullptr;
    Status (*set_crtc_config)(Display*,XRRScreenResourcesLite*,RRCrtc,Time,int,int,RRMode,Rotation,RROutput*,int)=nullptr;
    void (*set_screen_size)(Display*,Window,int,int,int,int)=nullptr;

    template<class T> bool load(T& dst,const char* name){ dst=reinterpret_cast<T>(dlsym(lib,name)); return dst!=nullptr; }
    bool init(){
        if(lib) return true;
        lib=dlopen("libXrandr.so.2",RTLD_NOW|RTLD_LOCAL);
        if(!lib) return false;
        bool ok=true;
        ok=load(get_resources,"XRRGetScreenResourcesCurrent")&&ok;
        ok=load(free_resources,"XRRFreeScreenResources")&&ok;
        load(get_primary,"XRRGetOutputPrimary");
        ok=load(get_output_info,"XRRGetOutputInfo")&&ok;
        ok=load(free_output_info,"XRRFreeOutputInfo")&&ok;
        ok=load(get_crtc_info,"XRRGetCrtcInfo")&&ok;
        ok=load(free_crtc_info,"XRRFreeCrtcInfo")&&ok;
        ok=load(set_crtc_config,"XRRSetCrtcConfig")&&ok;
        ok=load(set_screen_size,"XRRSetScreenSize")&&ok;
        if(!ok){ dlclose(lib); lib=nullptr; }
        return ok;
    }
    ~RandRApi(){ if(lib) dlclose(lib); }
};

int refresh_millihz(const XRRModeInfoLite& m){
    if(!m.dotClock || !m.hTotal || !m.vTotal) return 0;
    return (int)std::lround((double)m.dotClock*1000.0/((double)m.hTotal*(double)m.vTotal));
}
}

struct Application::Impl {
    Display* display=nullptr;
    Window window=0;
    Atom wm_delete{};
    int width=0,height=0;
    bool quit=false;
    bool fullscreen=false;
    InputState input{};
    std::unique_ptr<Renderer> renderer;
    mutable RandRApi randr{};

    RROutput original_output=0;
    RRCrtc original_crtc=0;
    RRMode original_mode=0;
    int original_x=0,original_y=0;
    Rotation original_rotation=1;
    std::vector<RROutput> original_outputs;
    bool display_mode_changed=false;
    int original_screen_width=0,original_screen_height=0;
    int original_screen_mm_width=0,original_screen_mm_height=0;

    Cursor hidden_cursor=0;
    Cursor default_cursor=0;
    bool cursor_is_visible=true;
    // relative_mouse_mode is the requested mode. The actual X11 pointer grab
    // exists only while this window owns keyboard focus.
    bool relative_mouse_mode=false;
    bool window_focused=false;

    void ensure_cursors(){
        if(!display || !window) return;
        if(!default_cursor) default_cursor=XCreateFontCursor(display,XC_left_ptr);
        if(!hidden_cursor){
            static const char data[1]={0};
            Pixmap blank=XCreateBitmapFromData(display,window,data,1,1);
            XColor dummy{};
            hidden_cursor=XCreatePixmapCursor(display,blank,blank,&dummy,&dummy,0,0);
            XFreePixmap(display,blank);
        }
    }
    void define_cursor(bool visible){
        if(!display || !window || cursor_is_visible==visible) return;
        ensure_cursors();
        XDefineCursor(display,window,visible?default_cursor:hidden_cursor);
        XFlush(display);
        cursor_is_visible=visible;
    }

    void set_cursor_visible(bool visible){
        // Never hide the system pointer while another application owns focus.
        // This also protects against render-side cursor requests that continue
        // to run while the game is in the background.
        if(!window_focused && !visible) visible=true;
        define_cursor(visible);
    }

    void apply_relative_mouse_capture(){
        if(!display || !window) return;
        if(relative_mouse_mode && window_focused){
            ensure_cursors();
            const int grab=XGrabPointer(display,window,True,ButtonPressMask|ButtonReleaseMask|PointerMotionMask,
                                        GrabModeAsync,GrabModeAsync,window,hidden_cursor,CurrentTime);
            if(grab==GrabSuccess){
                XWarpPointer(display,None,window,0,0,0,0,std::max(1,width)/2,std::max(1,height)/2);
                define_cursor(false);
            }else{
                define_cursor(true);
            }
        }else{
            XUngrabPointer(display,CurrentTime);
            define_cursor(true);
        }
        XFlush(display);
    }

    void set_relative_mouse_mode(bool enabled){
        if(!display || !window) return;
        if(relative_mouse_mode==enabled){
            apply_relative_mouse_capture();
            return;
        }
        relative_mouse_mode=enabled;
        apply_relative_mouse_capture();
    }

    void clear_held_input(){
        input.left_down=input.middle_down=input.right_down=false;
        input.key_v_down=false;
        input.key_w_down=input.key_a_down=input.key_s_down=input.key_d_down=false;
        input.key_shift_down=false;
        input.mouse_dx=input.mouse_dy=0;
    }

    void send_fullscreen(bool on){
        Atom state=XInternAtom(display,"_NET_WM_STATE",False); Atom fs=XInternAtom(display,"_NET_WM_STATE_FULLSCREEN",False);
        XEvent e{}; e.type=ClientMessage; e.xclient.window=window; e.xclient.message_type=state; e.xclient.format=32;
        e.xclient.data.l[0]=on?1:0; e.xclient.data.l[1]=fs; e.xclient.data.l[2]=0; e.xclient.data.l[3]=1;
        XSendEvent(display,DefaultRootWindow(display),False,SubstructureNotifyMask|SubstructureRedirectMask,&e); XFlush(display);
        fullscreen=on;
    }

    bool choose_output(XRRScreenResourcesLite* res,RROutput& output,XRROutputInfoLite*& info) const {
        if(!res) return false;
        RROutput preferred=randr.get_primary?randr.get_primary(display,DefaultRootWindow(display)):0;
        auto try_output=[&](RROutput candidate)->bool{
            if(!candidate) return false;
            XRROutputInfoLite* oi=randr.get_output_info(display,res,candidate);
            if(!oi) return false;
            if(oi->connection==RR_CONNECTED && oi->crtc!=0 && oi->nmode>0){ output=candidate; info=oi; return true; }
            randr.free_output_info(oi); return false;
        };
        if(try_output(preferred)) return true;
        for(int i=0;i<res->noutput;++i) if(try_output(res->outputs[i])) return true;
        return false;
    }

    std::vector<DisplayMode> modes() const {
        std::vector<DisplayMode> out;
        if(!display || !randr.init()) return {{width,height,0}};
        XRRScreenResourcesLite* res=randr.get_resources(display,DefaultRootWindow(display));
        RROutput output=0; XRROutputInfoLite* oi=nullptr;
        if(!choose_output(res,output,oi)){ if(res)randr.free_resources(res); return {{width,height,0}}; }
        for(int i=0;i<oi->nmode;++i){
            const RRMode id=oi->modes[i];
            for(int m=0;m<res->nmode;++m){
                if(res->modes[m].id!=id) continue;
                DisplayMode dm{(int)res->modes[m].width,(int)res->modes[m].height,refresh_millihz(res->modes[m])};
                auto existing=std::find_if(out.begin(),out.end(),[&](const DisplayMode& x){return x.width==dm.width&&x.height==dm.height;});
                if(existing==out.end()) out.push_back(dm); else if(dm.refresh_millihz>existing->refresh_millihz) *existing=dm;
                break;
            }
        }
        randr.free_output_info(oi); randr.free_resources(res);
        std::sort(out.begin(),out.end(),[](const DisplayMode& a,const DisplayMode& b){
            const long aa=(long)a.width*a.height, bb=(long)b.width*b.height;
            return aa<bb || (aa==bb && a.width<b.width);
        });
        if(out.empty()) out.push_back({width,height,0});
        return out;
    }

    DisplayMode current_mode() const {
        if(!display || !randr.init()) return {width,height,0};
        XRRScreenResourcesLite* res=randr.get_resources(display,DefaultRootWindow(display));
        RROutput output=0; XRROutputInfoLite* oi=nullptr;
        if(!choose_output(res,output,oi)){ if(res)randr.free_resources(res); return {width,height,0}; }
        XRRCrtcInfoLite* ci=randr.get_crtc_info(display,res,oi->crtc);
        DisplayMode dm{ci?(int)ci->width:width,ci?(int)ci->height:height,0};
        if(ci){
            for(int m=0;m<res->nmode;++m) if(res->modes[m].id==ci->mode){ dm.refresh_millihz=refresh_millihz(res->modes[m]); break; }
            randr.free_crtc_info(ci);
        }
        randr.free_output_info(oi); randr.free_resources(res);
        return dm;
    }

    bool set_mode(int target_w,int target_h){
        if(!display || !randr.init() || target_w<=0 || target_h<=0) return false;
        const Window root=DefaultRootWindow(display);
        const int screen=DefaultScreen(display);
        XRRScreenResourcesLite* res=randr.get_resources(display,root);
        RROutput output=0; XRROutputInfoLite* oi=nullptr;
        if(!choose_output(res,output,oi)){ if(res)randr.free_resources(res); return false; }
        XRRCrtcInfoLite* ci=randr.get_crtc_info(display,res,oi->crtc);
        if(!ci){ randr.free_output_info(oi); randr.free_resources(res); return false; }

        RRMode selected=0; int best_refresh=-1;
        for(int i=0;i<oi->nmode;++i){
            const RRMode id=oi->modes[i];
            for(int m=0;m<res->nmode;++m){
                const auto& mi=res->modes[m];
                if(mi.id==id && (int)mi.width==target_w && (int)mi.height==target_h){
                    const int hz=refresh_millihz(mi);
                    if(hz>best_refresh){ best_refresh=hz; selected=id; }
                }
            }
        }
        if(!selected){
            randr.free_crtc_info(ci); randr.free_output_info(oi); randr.free_resources(res);
            return false;
        }

        if(!display_mode_changed){
            original_output=output;
            original_crtc=oi->crtc;
            original_mode=ci->mode;
            original_x=ci->x;
            original_y=ci->y;
            original_rotation=ci->rotation;
            original_outputs.assign(ci->outputs,ci->outputs+ci->noutput);
            original_screen_width=DisplayWidth(display,screen);
            original_screen_height=DisplayHeight(display,screen);
            original_screen_mm_width=DisplayWidthMM(display,screen);
            original_screen_mm_height=DisplayHeightMM(display,screen);
        }

        // Determine the framebuffer required by all active CRTCs after changing
        // the selected one. This keeps multi-monitor desktops valid while still
        // allowing a single-monitor framebuffer to really shrink/grow.
        int framebuffer_w=std::max(target_w,ci->x+target_w);
        int framebuffer_h=std::max(target_h,ci->y+target_h);
        for(int i=0;i<res->ncrtc;++i){
            if(res->crtcs[i]==oi->crtc) continue;
            XRRCrtcInfoLite* other=randr.get_crtc_info(display,res,res->crtcs[i]);
            if(other){
                if(other->mode!=0){
                    framebuffer_w=std::max(framebuffer_w,other->x+(int)other->width);
                    framebuffer_h=std::max(framebuffer_h,other->y+(int)other->height);
                }
                randr.free_crtc_info(other);
            }
        }
        framebuffer_w=std::max(1,framebuffer_w);
        framebuffer_h=std::max(1,framebuffer_h);

        const int current_fb_w=DisplayWidth(display,screen);
        const int current_fb_h=DisplayHeight(display,screen);
        const int current_mm_w=std::max(1,DisplayWidthMM(display,screen));
        const int current_mm_h=std::max(1,DisplayHeightMM(display,screen));
        auto mm_for=[&](int pixels,int old_pixels,int old_mm){
            return std::max(1,(int)std::lround((double)pixels*(double)old_mm/(double)std::max(1,old_pixels)));
        };
        const int framebuffer_mm_w=mm_for(framebuffer_w,current_fb_w,current_mm_w);
        const int framebuffer_mm_h=mm_for(framebuffer_h,current_fb_h,current_mm_h);

        const bool was_fullscreen=fullscreen;
        if(was_fullscreen){
            send_fullscreen(false);
            XSync(display,False);
        }

        // RandR requires the framebuffer to be large enough before a CRTC is
        // enlarged. Never shrink either dimension until after the CRTC itself
        // has switched; mixed transitions such as 1920x1080 -> 2560x720 would
        // otherwise produce BadMatch on stricter X servers.
        const int pre_fb_w=std::max(current_fb_w,framebuffer_w);
        const int pre_fb_h=std::max(current_fb_h,framebuffer_h);
        if(pre_fb_w!=current_fb_w || pre_fb_h!=current_fb_h){
            const int pre_mm_w=mm_for(pre_fb_w,current_fb_w,current_mm_w);
            const int pre_mm_h=mm_for(pre_fb_h,current_fb_h,current_mm_h);
            randr.set_screen_size(display,root,pre_fb_w,pre_fb_h,pre_mm_w,pre_mm_h);
            XSync(display,False);
        }

        const Status result=randr.set_crtc_config(display,res,oi->crtc,CurrentTime,ci->x,ci->y,selected,ci->rotation,ci->outputs,ci->noutput);
        if(result==Success && (framebuffer_w!=pre_fb_w || framebuffer_h!=pre_fb_h)){
            randr.set_screen_size(display,root,framebuffer_w,framebuffer_h,framebuffer_mm_w,framebuffer_mm_h);
        } else if(result!=Success && (pre_fb_w!=current_fb_w || pre_fb_h!=current_fb_h)){
            // Put the root framebuffer back if the CRTC mode itself was rejected.
            randr.set_screen_size(display,root,current_fb_w,current_fb_h,current_mm_w,current_mm_h);
        }

        randr.free_crtc_info(ci);
        randr.free_output_info(oi);
        randr.free_resources(res);

        if(result!=Success){
            if(was_fullscreen) send_fullscreen(true);
            XSync(display,False);
            return false;
        }

        display_mode_changed=true;
        width=target_w;
        height=target_h;
        XMoveResizeWindow(display,window,0,0,(unsigned int)target_w,(unsigned int)target_h);
        if(renderer) renderer->resize(target_w,target_h);
        if(was_fullscreen) send_fullscreen(true);
        XSync(display,False);
        return true;
    }

    void restore_mode(){
        if(!display_mode_changed || !display || !randr.init() || !original_crtc || !original_mode) return;
        const Window root=DefaultRootWindow(display);
        const int screen=DefaultScreen(display);
        XRRScreenResourcesLite* res=randr.get_resources(display,root);
        if(res){
            const int current_w=DisplayWidth(display,screen);
            const int current_h=DisplayHeight(display,screen);
            if(original_screen_width>current_w || original_screen_height>current_h){
                const int pre_w=std::max(current_w,original_screen_width);
                const int pre_h=std::max(current_h,original_screen_height);
                const int pre_mm_w=std::max(1,(int)std::lround((double)pre_w*std::max(1,original_screen_mm_width)/(double)std::max(1,original_screen_width)));
                const int pre_mm_h=std::max(1,(int)std::lround((double)pre_h*std::max(1,original_screen_mm_height)/(double)std::max(1,original_screen_height)));
                randr.set_screen_size(display,root,pre_w,pre_h,pre_mm_w,pre_mm_h);
                XSync(display,False);
            }

            RROutput* outs=original_outputs.empty()?nullptr:original_outputs.data();
            const Status result=randr.set_crtc_config(display,res,original_crtc,CurrentTime,
                                                      original_x,original_y,original_mode,original_rotation,
                                                      outs,(int)original_outputs.size());
            if(result==Success){
                randr.set_screen_size(display,root,
                                      std::max(1,original_screen_width),std::max(1,original_screen_height),
                                      std::max(1,original_screen_mm_width),std::max(1,original_screen_mm_height));
            }
            randr.free_resources(res);
            XSync(display,False);
        }
        display_mode_changed=false;
    }

    ~Impl(){
        restore_mode();
        if(display && relative_mouse_mode) XUngrabPointer(display,CurrentTime);
        if(display && hidden_cursor) XFreeCursor(display,hidden_cursor);
        if(display && default_cursor) XFreeCursor(display,default_cursor);
        if(display && window) XDestroyWindow(display,window);
        if(display) XCloseDisplay(display);
    }
};

Application::Application()=default;
Application::~Application()=default;

int Application::run(const AppConfig& config,std::unique_ptr<GameDelegate> game){
    impl_=std::make_unique<Impl>(); auto& p=*impl_; p.width=config.width; p.height=config.height;
    p.display=XOpenDisplay(nullptr); if(!p.display){ std::cerr<<"Unable to open X11 display. Set DISPLAY and run inside a graphical Linux session.\n"; return 2; }
    int s=DefaultScreen(p.display);
    XSetWindowAttributes attrs{}; attrs.event_mask=ExposureMask|KeyPressMask|KeyReleaseMask|ButtonPressMask|ButtonReleaseMask|PointerMotionMask|StructureNotifyMask|FocusChangeMask;
    p.window=XCreateWindow(p.display,RootWindow(p.display,s),0,0,p.width,p.height,0,CopyFromParent,InputOutput,CopyFromParent,CWEventMask,&attrs);
    XStoreName(p.display,p.window,config.title.c_str());
    p.wm_delete=XInternAtom(p.display,"WM_DELETE_WINDOW",False); XSetWMProtocols(p.display,p.window,&p.wm_delete,1);
    XMapWindow(p.display,p.window); XFlush(p.display);
    Bool detectable=False;
    XkbSetDetectableAutoRepeat(p.display,True,&detectable);
    p.renderer=create_gl_renderer();
    if(!p.renderer->initialize(p.display,p.window,p.width,p.height)){
        std::cerr<<"OpenGL hardware renderer initialization failed. A direct GPU-backed GLX context is required.\n";
        return 3;
    }
    std::cout<<"CPU simulation lanes: "<<cpu_parallel_lanes()<<" (bounded to ~50% of logical CPUs)\n";
    if(config.fullscreen) p.send_fullscreen(true);
    game->on_start();
    auto prev=std::chrono::steady_clock::now();
    int lastx=0,lasty=0;
    while(!p.quit){
        p.input.left_pressed=p.input.middle_pressed=p.input.right_pressed=false;
        p.input.escape_pressed=p.input.key_n_pressed=p.input.key_m_pressed=p.input.key_v_pressed=p.input.delete_pressed=false;
        p.input.backspace_pressed=p.input.enter_pressed=false;
        p.input.text_input.clear();
        p.input.wheel=0; p.input.mouse_dx=0; p.input.mouse_dy=0;
        while(XPending(p.display)){
            XEvent e{}; XNextEvent(p.display,&e);
            if(e.type==ClientMessage && (Atom)e.xclient.data.l[0]==p.wm_delete) p.quit=true;
            else if(e.type==FocusIn){
                p.window_focused=true;
                if(p.relative_mouse_mode) p.apply_relative_mouse_capture();
            }
            else if(e.type==FocusOut){
                p.window_focused=false;
                p.clear_held_input();
                // Relative mode stays requested so FPV resumes naturally when
                // the user returns, but the OS pointer is immediately released.
                p.apply_relative_mouse_capture();
            }
            else if(e.type==ConfigureNotify){ p.width=e.xconfigure.width; p.height=e.xconfigure.height; p.renderer->resize(p.width,p.height); }
            else if(e.type==MotionNotify){
                if(p.relative_mouse_mode && p.window_focused){
                    const int cx=std::max(1,p.width)/2, cy=std::max(1,p.height)/2;
                    const int dx=e.xmotion.x-cx, dy=e.xmotion.y-cy;
                    if(dx!=0 || dy!=0){
                        p.input.mouse_dx += dx; p.input.mouse_dy += dy;
                        XWarpPointer(p.display,None,p.window,0,0,0,0,cx,cy);
                    }
                    p.input.mouse_x=cx; p.input.mouse_y=cy; lastx=cx; lasty=cy;
                }else{
                    p.input.mouse_x=e.xmotion.x; p.input.mouse_y=e.xmotion.y;
                    p.input.mouse_dx += e.xmotion.x-lastx; p.input.mouse_dy += e.xmotion.y-lasty;
                    lastx=e.xmotion.x; lasty=e.xmotion.y;
                }
            }
            else if(e.type==ButtonPress){
                if(e.xbutton.button==1){ p.input.left_down=true; p.input.left_pressed=true; }
                if(e.xbutton.button==2){ p.input.middle_down=true; p.input.middle_pressed=true; }
                if(e.xbutton.button==3){ p.input.right_down=true; p.input.right_pressed=true; }
                if(e.xbutton.button==4) p.input.wheel += 1.0f;
                if(e.xbutton.button==5) p.input.wheel -= 1.0f;
            } else if(e.type==ButtonRelease){ if(e.xbutton.button==1)p.input.left_down=false; if(e.xbutton.button==2)p.input.middle_down=false; if(e.xbutton.button==3)p.input.right_down=false; }
            else if(e.type==KeyPress){
                KeySym k=NoSymbol;
                char text[64]{};
                const int text_len=XLookupString(&e.xkey,text,(int)sizeof(text),&k,nullptr);
                if(k==XK_Escape) p.input.escape_pressed=true;
                else if(k==XK_BackSpace) p.input.backspace_pressed=true;
                else if(k==XK_Return || k==XK_KP_Enter) p.input.enter_pressed=true;
                else if(k==XK_n || k==XK_N) p.input.key_n_pressed=true;
                else if(k==XK_m || k==XK_M) p.input.key_m_pressed=true;
                else if(k==XK_v || k==XK_V){ if(!p.input.key_v_down) p.input.key_v_pressed=true; p.input.key_v_down=true; }
                else if(k==XK_w || k==XK_W) p.input.key_w_down=true;
                else if(k==XK_a || k==XK_A) p.input.key_a_down=true;
                else if(k==XK_s || k==XK_S) p.input.key_s_down=true;
                else if(k==XK_d || k==XK_D) p.input.key_d_down=true;
                else if(k==XK_Shift_L || k==XK_Shift_R) p.input.key_shift_down=true;
                else if(k==XK_Delete) p.input.delete_pressed=true;

                if(!(e.xkey.state&(ControlMask|Mod1Mask)) && text_len>0){
                    for(int i=0;i<text_len;++i){
                        const unsigned char ch=(unsigned char)text[i];
                        if(ch>=32 && ch!=127) p.input.text_input.push_back((char)ch);
                    }
                }
            } else if(e.type==KeyRelease){
                KeySym k=XLookupKeysym(&e.xkey,0);
                if(k==XK_v || k==XK_V) p.input.key_v_down=false;
                else if(k==XK_w || k==XK_W) p.input.key_w_down=false;
                else if(k==XK_a || k==XK_A) p.input.key_a_down=false;
                else if(k==XK_s || k==XK_S) p.input.key_s_down=false;
                else if(k==XK_d || k==XK_D) p.input.key_d_down=false;
                else if(k==XK_Shift_L || k==XK_Shift_R) p.input.key_shift_down=false;
            }
        }
        auto now=std::chrono::steady_clock::now(); float dt=std::chrono::duration<float>(now-prev).count(); prev=now; if(dt>0.1f)dt=0.1f;
        game->update(dt,p.input); game->render(*p.renderer);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    p.restore_mode();
    return 0;
}
void Application::request_quit(){ if(impl_) impl_->quit=true; }
void Application::set_fullscreen(bool enabled){ if(impl_ && impl_->fullscreen!=enabled) impl_->send_fullscreen(enabled); }
bool Application::fullscreen() const { return impl_?impl_->fullscreen:false; }
std::vector<DisplayMode> Application::display_modes() const { return impl_?impl_->modes():std::vector<DisplayMode>{}; }
DisplayMode Application::current_display_mode() const { return impl_?impl_->current_mode():DisplayMode{}; }
bool Application::set_display_mode(int width,int height){ return impl_?impl_->set_mode(width,height):false; }
void Application::set_cursor_visible(bool visible){ if(impl_) impl_->set_cursor_visible(visible); }
bool Application::cursor_visible() const { return impl_?impl_->cursor_is_visible:true; }
void Application::set_relative_mouse_mode(bool enabled){ if(impl_) impl_->set_relative_mouse_mode(enabled); }
bool Application::relative_mouse_mode() const { return impl_?impl_->relative_mouse_mode:false; }
}
