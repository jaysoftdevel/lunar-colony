#include "engine/engine.hpp"
#include <X11/Xatom.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <chrono>
#include <iostream>
#include <thread>

namespace eng { std::unique_ptr<Renderer> create_gl_renderer(); }

namespace eng {
struct Application::Impl {
    Display* display=nullptr;
    Window window=0;
    Atom wm_delete{};
    int width=0,height=0;
    bool quit=false;
    bool fullscreen=false;
    InputState input{};
    std::unique_ptr<Renderer> renderer;

    void hide_cursor(){
        static const char data[1]={0}; Pixmap blank=XCreateBitmapFromData(display,window,data,1,1);
        XColor dummy{}; Cursor cur=XCreatePixmapCursor(display,blank,blank,&dummy,&dummy,0,0); XDefineCursor(display,window,cur); XFreePixmap(display,blank);
    }
    void send_fullscreen(bool on){
        Atom state=XInternAtom(display,"_NET_WM_STATE",False); Atom fs=XInternAtom(display,"_NET_WM_STATE_FULLSCREEN",False);
        XEvent e{}; e.type=ClientMessage; e.xclient.window=window; e.xclient.message_type=state; e.xclient.format=32;
        e.xclient.data.l[0]=on?1:0; e.xclient.data.l[1]=fs; e.xclient.data.l[2]=0; e.xclient.data.l[3]=1;
        XSendEvent(display,DefaultRootWindow(display),False,SubstructureNotifyMask|SubstructureRedirectMask,&e); XFlush(display);
        fullscreen=on;
    }
};


Application::Application()=default;
Application::~Application()=default;

int Application::run(const AppConfig& config,std::unique_ptr<GameDelegate> game){
    impl_=std::make_unique<Impl>(); auto& p=*impl_; p.width=config.width; p.height=config.height;
    p.display=XOpenDisplay(nullptr); if(!p.display){ std::cerr<<"Unable to open X11 display. Set DISPLAY and run inside a graphical Linux session.\n"; return 2; }
    int s=DefaultScreen(p.display);
    XSetWindowAttributes attrs{}; attrs.event_mask=ExposureMask|KeyPressMask|KeyReleaseMask|ButtonPressMask|ButtonReleaseMask|PointerMotionMask|StructureNotifyMask;
    p.window=XCreateWindow(p.display,RootWindow(p.display,s),0,0,p.width,p.height,0,CopyFromParent,InputOutput,CopyFromParent,CWEventMask,&attrs);
    XStoreName(p.display,p.window,config.title.c_str());
    p.wm_delete=XInternAtom(p.display,"WM_DELETE_WINDOW",False); XSetWMProtocols(p.display,p.window,&p.wm_delete,1);
    XMapWindow(p.display,p.window); XFlush(p.display); p.hide_cursor();
    p.renderer=create_gl_renderer();
    if(!p.renderer->initialize(p.display,p.window,p.width,p.height)){ std::cerr<<"OpenGL renderer initialization failed.\n"; return 3; }
    if(config.fullscreen) p.send_fullscreen(true);
    game->on_start();
    auto prev=std::chrono::steady_clock::now();
    int lastx=0,lasty=0;
    while(!p.quit){
        p.input.left_pressed=p.input.middle_pressed=p.input.right_pressed=false; p.input.escape_pressed=false; p.input.wheel=0; p.input.mouse_dx=0; p.input.mouse_dy=0;
        while(XPending(p.display)){
            XEvent e{}; XNextEvent(p.display,&e);
            if(e.type==ClientMessage && (Atom)e.xclient.data.l[0]==p.wm_delete) p.quit=true;
            else if(e.type==ConfigureNotify){ p.width=e.xconfigure.width; p.height=e.xconfigure.height; p.renderer->resize(p.width,p.height); }
            else if(e.type==MotionNotify){ p.input.mouse_x=e.xmotion.x; p.input.mouse_y=e.xmotion.y; p.input.mouse_dx += e.xmotion.x-lastx; p.input.mouse_dy += e.xmotion.y-lasty; lastx=e.xmotion.x; lasty=e.xmotion.y; }
            else if(e.type==ButtonPress){
                if(e.xbutton.button==1){ p.input.left_down=true; p.input.left_pressed=true; }
                if(e.xbutton.button==2){ p.input.middle_down=true; p.input.middle_pressed=true; }
                if(e.xbutton.button==3){ p.input.right_down=true; p.input.right_pressed=true; }
                if(e.xbutton.button==4) p.input.wheel += 1.0f;
                if(e.xbutton.button==5) p.input.wheel -= 1.0f;
            } else if(e.type==ButtonRelease){ if(e.xbutton.button==1)p.input.left_down=false; if(e.xbutton.button==2)p.input.middle_down=false; if(e.xbutton.button==3)p.input.right_down=false; }
            else if(e.type==KeyPress){ KeySym k=XLookupKeysym(&e.xkey,0); if(k==XK_Escape) p.input.escape_pressed=true; }
        }
        auto now=std::chrono::steady_clock::now(); float dt=std::chrono::duration<float>(now-prev).count(); prev=now; if(dt>0.1f)dt=0.1f;
        game->update(dt,p.input); game->render(*p.renderer);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return 0;
}
void Application::request_quit(){ if(impl_) impl_->quit=true; }
void Application::set_fullscreen(bool enabled){ if(impl_ && impl_->fullscreen!=enabled) impl_->send_fullscreen(enabled); }
bool Application::fullscreen() const { return impl_?impl_->fullscreen:false; }
}
