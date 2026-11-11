#include "engine/engine.hpp"

#ifndef _WIN32
#error platform_win32.cpp is Windows-only
#endif

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <windowsx.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace eng { std::unique_ptr<Renderer> create_d3d12_renderer(); }

namespace eng {
namespace {

std::wstring widen_utf8(const std::string& text){
    if(text.empty()) return {};
    const int chars=MultiByteToWideChar(CP_UTF8,0,text.data(),(int)text.size(),nullptr,0);
    if(chars<=0) return {};
    std::wstring out((std::size_t)chars,L'\0');
    MultiByteToWideChar(CP_UTF8,0,text.data(),(int)text.size(),out.data(),chars);
    return out;
}

int refresh_millihz(const DEVMODEW& dm){
    if(dm.dmDisplayFrequency<=1) return 0;
    return (int)dm.dmDisplayFrequency*1000;
}

} // namespace

struct Application::Impl {
    HINSTANCE instance{GetModuleHandleW(nullptr)};
    HWND window{};
    std::unique_ptr<Renderer> renderer{};
    bool quit{false};
    bool fullscreen{false};
    bool cursor_is_visible{true};
    bool relative_mouse_mode{false};
    bool window_focused{true};
    bool minimized{false};
    int width{1600};
    int height{900};
    InputState input{};

    DWORD windowed_style{WS_OVERLAPPEDWINDOW};
    DWORD windowed_ex_style{0};
    RECT windowed_rect{100,100,1700,1000};

    bool display_mode_changed{false};
    std::wstring display_device{};
    DEVMODEW original_display_mode{};

    static constexpr wchar_t kWindowClassName[]=L"MoonBaseGameWindow";

    static LRESULT CALLBACK wnd_proc(HWND hwnd,UINT msg,WPARAM wp,LPARAM lp){
        Impl* p=reinterpret_cast<Impl*>(GetWindowLongPtrW(hwnd,GWLP_USERDATA));
        if(msg==WM_NCCREATE){
            const auto* cs=reinterpret_cast<const CREATESTRUCTW*>(lp);
            p=reinterpret_cast<Impl*>(cs->lpCreateParams);
            SetWindowLongPtrW(hwnd,GWLP_USERDATA,reinterpret_cast<LONG_PTR>(p));
        }
        if(!p) return DefWindowProcW(hwnd,msg,wp,lp);
        return p->handle_message(hwnd,msg,wp,lp);
    }

    LRESULT handle_message(HWND hwnd,UINT msg,WPARAM wp,LPARAM lp){
        switch(msg){
            case WM_CLOSE:
                quit=true;
                return 0;
            case WM_DESTROY:
                quit=true;
                PostQuitMessage(0);
                return 0;
            case WM_SETFOCUS:
                window_focused=true;
                apply_relative_mouse_capture();
                return 0;
            case WM_KILLFOCUS:
                window_focused=false;
                clear_held_input();
                apply_relative_mouse_capture();
                return 0;
            case WM_SIZE: {
                minimized=(wp==SIZE_MINIMIZED);
                const int w=LOWORD(lp);
                const int h=HIWORD(lp);
                if(!minimized && w>0 && h>0){
                    width=w;
                    height=h;
                    if(renderer) renderer->resize(width,height);
                    if(relative_mouse_mode) apply_relative_mouse_capture();
                }
                return 0;
            }
            case WM_DPICHANGED: {
                const auto* suggested=reinterpret_cast<const RECT*>(lp);
                if(suggested && !fullscreen){
                    SetWindowPos(hwnd,nullptr,suggested->left,suggested->top,
                                 suggested->right-suggested->left,suggested->bottom-suggested->top,
                                 SWP_NOZORDER|SWP_NOACTIVATE);
                }
                return 0;
            }
            case WM_SETCURSOR:
                if(LOWORD(lp)==HTCLIENT && (!cursor_is_visible || (relative_mouse_mode && window_focused))){
                    SetCursor(nullptr);
                    return TRUE;
                }
                break;
            case WM_MOUSEMOVE:
                if(!relative_mouse_mode){
                    static int last_x=GET_X_LPARAM(lp);
                    static int last_y=GET_Y_LPARAM(lp);
                    const int x=GET_X_LPARAM(lp), y=GET_Y_LPARAM(lp);
                    input.mouse_x=x; input.mouse_y=y;
                    input.mouse_dx += x-last_x;
                    input.mouse_dy += y-last_y;
                    last_x=x; last_y=y;
                }
                return 0;
            case WM_INPUT:
                if(relative_mouse_mode && window_focused){
                    UINT size=0;
                    if(GetRawInputData(reinterpret_cast<HRAWINPUT>(lp),RID_INPUT,nullptr,&size,sizeof(RAWINPUTHEADER))==0 && size>0){
                        std::vector<std::byte> storage(size);
                        if(GetRawInputData(reinterpret_cast<HRAWINPUT>(lp),RID_INPUT,storage.data(),&size,sizeof(RAWINPUTHEADER))==size){
                            const auto* raw=reinterpret_cast<const RAWINPUT*>(storage.data());
                            if(raw->header.dwType==RIM_TYPEMOUSE && !(raw->data.mouse.usFlags&MOUSE_MOVE_ABSOLUTE)){
                                input.mouse_dx += raw->data.mouse.lLastX;
                                input.mouse_dy += raw->data.mouse.lLastY;
                                input.mouse_x=std::max(1,width)/2;
                                input.mouse_y=std::max(1,height)/2;
                            }
                        }
                    }
                }
                return 0;
            case WM_LBUTTONDOWN:
                SetCapture(hwnd); input.left_down=true; input.left_pressed=true; return 0;
            case WM_MBUTTONDOWN:
                SetCapture(hwnd); input.middle_down=true; input.middle_pressed=true; return 0;
            case WM_RBUTTONDOWN:
                SetCapture(hwnd); input.right_down=true; input.right_pressed=true; return 0;
            case WM_LBUTTONUP:
                input.left_down=false; if(!input.middle_down&&!input.right_down) ReleaseCapture(); return 0;
            case WM_MBUTTONUP:
                input.middle_down=false; if(!input.left_down&&!input.right_down) ReleaseCapture(); return 0;
            case WM_RBUTTONUP:
                input.right_down=false; if(!input.left_down&&!input.middle_down) ReleaseCapture(); return 0;
            case WM_MOUSEWHEEL:
                input.wheel += (float)GET_WHEEL_DELTA_WPARAM(wp)/(float)WHEEL_DELTA;
                return 0;
            case WM_KEYDOWN:
            case WM_SYSKEYDOWN: {
                const bool first=(lp&(1ll<<30))==0;
                const UINT k=(UINT)wp;
                if(msg==WM_SYSKEYDOWN && k==VK_F4 && (GetKeyState(VK_MENU)&0x8000)){ quit=true; return 0; }
                if(k==VK_ESCAPE && first) input.escape_pressed=true;
                else if(k==VK_BACK && first) input.backspace_pressed=true;
                else if(k==VK_RETURN && first) input.enter_pressed=true;
                else if(k=='N' && first) input.key_n_pressed=true;
                else if(k=='M' && first) input.key_m_pressed=true;
                else if(k==VK_SPACE){if(first) input.key_space_pressed=true;input.key_space_down=true;}
                else if(k=='J'){if(first) input.key_j_pressed=true;input.key_j_down=true;}
                else if(first && ((k>='1' && k<='3') || (k>=VK_NUMPAD1 && k<=VK_NUMPAD3)) &&
                        !(GetKeyState(VK_CONTROL)&0x8000) && !(GetKeyState(VK_MENU)&0x8000))
                    input.key_number_pressed=static_cast<std::uint8_t>(k-(k>=VK_NUMPAD1?VK_NUMPAD1:'1')+1);
                else if(k=='V'){ if(first) input.key_v_pressed=true; input.key_v_down=true; }
                else if(k=='W') input.key_w_down=true;
                else if(k=='A') input.key_a_down=true;
                else if(k=='S') input.key_s_down=true;
                else if(k=='D') input.key_d_down=true;
                else if(k==VK_SHIFT) input.key_shift_down=true;
                else if(k==VK_DELETE && first) input.delete_pressed=true;
                return 0;
            }
            case WM_KEYUP:
            case WM_SYSKEYUP: {
                const UINT k=(UINT)wp;
                if(k==VK_SPACE) input.key_space_down=false;
                else if(k=='J') input.key_j_down=false;
                else if(k=='V') input.key_v_down=false;
                else if(k=='W') input.key_w_down=false;
                else if(k=='A') input.key_a_down=false;
                else if(k=='S') input.key_s_down=false;
                else if(k=='D') input.key_d_down=false;
                else if(k==VK_SHIFT) input.key_shift_down=false;
                return 0;
            }
            case WM_CHAR: {
                const wchar_t wc=(wchar_t)wp;
                if(wc>=32 && wc!=127 && !(GetKeyState(VK_CONTROL)&0x8000) && !(GetKeyState(VK_MENU)&0x8000)){
                    char utf8[5]{};
                    const int n=WideCharToMultiByte(CP_UTF8,0,&wc,1,utf8,4,nullptr,nullptr);
                    if(n>0) input.text_input.append(utf8,utf8+n);
                }
                return 0;
            }
        }
        return DefWindowProcW(hwnd,msg,wp,lp);
    }

    bool register_window_class(){
        WNDCLASSEXW wc{};
        wc.cbSize=sizeof(wc);
        wc.style=CS_HREDRAW|CS_VREDRAW|CS_OWNDC;
        wc.lpfnWndProc=&Impl::wnd_proc;
        wc.hInstance=instance;
        wc.hCursor=LoadCursorW(nullptr,IDC_ARROW);
        wc.hbrBackground=(HBRUSH)GetStockObject(BLACK_BRUSH);
        wc.lpszClassName=kWindowClassName;
        const ATOM atom=RegisterClassExW(&wc);
        return atom!=0 || GetLastError()==ERROR_CLASS_ALREADY_EXISTS;
    }

    bool create_window(const AppConfig& config){
        if(!register_window_class()) return false;
        width=config.width; height=config.height;
        RECT rect{0,0,width,height};
        AdjustWindowRectEx(&rect,windowed_style,FALSE,windowed_ex_style);
        const int window_w=rect.right-rect.left;
        const int window_h=rect.bottom-rect.top;
        const int x=std::max(0,(GetSystemMetrics(SM_CXSCREEN)-window_w)/2);
        const int y=std::max(0,(GetSystemMetrics(SM_CYSCREEN)-window_h)/2);
        const std::wstring title=widen_utf8(config.title);
        window=CreateWindowExW(windowed_ex_style,kWindowClassName,title.c_str(),windowed_style,
                               x,y,window_w,window_h,nullptr,nullptr,instance,this);
        if(!window) return false;
        GetWindowRect(window,&windowed_rect);
        ShowWindow(window,SW_SHOW);
        UpdateWindow(window);

        RAWINPUTDEVICE rid{};
        rid.usUsagePage=0x01;
        rid.usUsage=0x02;
        rid.dwFlags=0;
        rid.hwndTarget=nullptr;
        RegisterRawInputDevices(&rid,1,sizeof(rid));
        return true;
    }

    void clear_held_input(){
        input.left_down=input.middle_down=input.right_down=false;
        input.key_v_down=false;
        input.key_w_down=input.key_a_down=input.key_s_down=input.key_d_down=false;
        input.key_shift_down=false;
        input.key_space_down=false;input.key_space_pressed=false;
        input.key_j_down=false;input.key_j_pressed=false;
        input.key_number_pressed=0;
        input.mouse_dx=input.mouse_dy=0;
    }

    void define_cursor(bool visible){
        cursor_is_visible=visible;
        if(!window) return;
        if(visible || !window_focused) SetCursor(LoadCursorW(nullptr,IDC_ARROW));
        else SetCursor(nullptr);
    }

    void set_cursor_visible(bool visible){
        if(!window_focused && !visible) visible=true;
        define_cursor(visible);
    }

    RECT client_screen_rect() const {
        RECT r{};
        if(!window) return r;
        GetClientRect(window,&r);
        POINT tl{r.left,r.top}, br{r.right,r.bottom};
        ClientToScreen(window,&tl); ClientToScreen(window,&br);
        r.left=tl.x; r.top=tl.y; r.right=br.x; r.bottom=br.y;
        return r;
    }

    void apply_relative_mouse_capture(){
        if(!window) return;
        if(relative_mouse_mode && window_focused){
            const RECT clip=client_screen_rect();
            ClipCursor(&clip);
            define_cursor(false);
            POINT center{std::max(1,width)/2,std::max(1,height)/2};
            ClientToScreen(window,&center);
            SetCursorPos(center.x,center.y);
            input.mouse_x=std::max(1,width)/2;
            input.mouse_y=std::max(1,height)/2;
        }else{
            ClipCursor(nullptr);
            define_cursor(true);
        }
    }

    void set_relative_mouse_mode(bool enabled){
        relative_mouse_mode=enabled;
        apply_relative_mouse_capture();
    }

    std::wstring monitor_device_name() const {
        if(!window) return {};
        MONITORINFOEXW mi{}; mi.cbSize=sizeof(mi);
        const HMONITOR monitor=MonitorFromWindow(window,MONITOR_DEFAULTTONEAREST);
        if(!GetMonitorInfoW(monitor,&mi)) return {};
        return mi.szDevice;
    }

    std::vector<DisplayMode> modes() const {
        std::vector<DisplayMode> out;
        const std::wstring device=monitor_device_name();
        if(device.empty()) return {{width,height,0}};
        DEVMODEW dm{}; dm.dmSize=sizeof(dm);
        for(DWORD i=0;EnumDisplaySettingsExW(device.c_str(),i,&dm,0);++i){
            if(dm.dmBitsPerPel<32) continue;
            DisplayMode candidate{(int)dm.dmPelsWidth,(int)dm.dmPelsHeight,refresh_millihz(dm)};
            auto it=std::find_if(out.begin(),out.end(),[&](const DisplayMode& m){return m.width==candidate.width&&m.height==candidate.height;});
            if(it==out.end()) out.push_back(candidate);
            else if(candidate.refresh_millihz>it->refresh_millihz) *it=candidate;
            dm={}; dm.dmSize=sizeof(dm);
        }
        std::sort(out.begin(),out.end(),[](const DisplayMode& a,const DisplayMode& b){
            const long long aa=(long long)a.width*a.height, bb=(long long)b.width*b.height;
            return aa<bb || (aa==bb && a.width<b.width);
        });
        if(out.empty()) out.push_back({width,height,0});
        return out;
    }

    DisplayMode current_mode() const {
        const std::wstring device=monitor_device_name();
        DEVMODEW dm{}; dm.dmSize=sizeof(dm);
        if(!device.empty() && EnumDisplaySettingsExW(device.c_str(),ENUM_CURRENT_SETTINGS,&dm,0))
            return {(int)dm.dmPelsWidth,(int)dm.dmPelsHeight,refresh_millihz(dm)};
        return {width,height,0};
    }

    void fit_fullscreen_to_monitor(){
        if(!window || !fullscreen) return;
        MONITORINFO mi{}; mi.cbSize=sizeof(mi);
        const HMONITOR monitor=MonitorFromWindow(window,MONITOR_DEFAULTTONEAREST);
        if(GetMonitorInfoW(monitor,&mi)){
            SetWindowPos(window,HWND_TOP,mi.rcMonitor.left,mi.rcMonitor.top,
                         mi.rcMonitor.right-mi.rcMonitor.left,mi.rcMonitor.bottom-mi.rcMonitor.top,
                         SWP_FRAMECHANGED|SWP_NOOWNERZORDER|SWP_NOACTIVATE);
        }
    }

    void send_fullscreen(bool on){
        if(!window || fullscreen==on) return;
        if(on){
            windowed_style=(DWORD)GetWindowLongPtrW(window,GWL_STYLE);
            windowed_ex_style=(DWORD)GetWindowLongPtrW(window,GWL_EXSTYLE);
            GetWindowRect(window,&windowed_rect);
            SetWindowLongPtrW(window,GWL_STYLE,WS_POPUP|WS_VISIBLE);
            SetWindowLongPtrW(window,GWL_EXSTYLE,windowed_ex_style&~WS_EX_WINDOWEDGE);
            fullscreen=true;
            fit_fullscreen_to_monitor();
        }else{
            fullscreen=false;
            SetWindowLongPtrW(window,GWL_STYLE,windowed_style);
            SetWindowLongPtrW(window,GWL_EXSTYLE,windowed_ex_style);
            SetWindowPos(window,nullptr,windowed_rect.left,windowed_rect.top,
                         windowed_rect.right-windowed_rect.left,windowed_rect.bottom-windowed_rect.top,
                         SWP_FRAMECHANGED|SWP_NOZORDER|SWP_NOOWNERZORDER|SWP_NOACTIVATE);
        }
    }

    bool set_mode(int target_w,int target_h){
        if(target_w<=0 || target_h<=0) return false;
        const std::wstring device=monitor_device_name();
        if(device.empty()) return false;

        DEVMODEW best{}; best.dmSize=sizeof(best);
        bool found=false;
        DWORD best_hz=0;
        DEVMODEW dm{}; dm.dmSize=sizeof(dm);
        for(DWORD i=0;EnumDisplaySettingsExW(device.c_str(),i,&dm,0);++i){
            if((int)dm.dmPelsWidth==target_w && (int)dm.dmPelsHeight==target_h && dm.dmBitsPerPel>=32){
                const DWORD hz=dm.dmDisplayFrequency;
                if(!found || hz>best_hz){ best=dm; best_hz=hz; found=true; }
            }
            dm={}; dm.dmSize=sizeof(dm);
        }
        if(!found) return false;

        if(!display_mode_changed){
            original_display_mode={}; original_display_mode.dmSize=sizeof(original_display_mode);
            if(!EnumDisplaySettingsExW(device.c_str(),ENUM_CURRENT_SETTINGS,&original_display_mode,0)) return false;
            display_device=device;
        }

        best.dmFields=DM_PELSWIDTH|DM_PELSHEIGHT|DM_BITSPERPEL|DM_DISPLAYFREQUENCY;
        const LONG rc=ChangeDisplaySettingsExW(device.c_str(),&best,nullptr,CDS_FULLSCREEN,nullptr);
        if(rc!=DISP_CHANGE_SUCCESSFUL) return false;
        display_mode_changed=true;
        fit_fullscreen_to_monitor();
        return true;
    }

    void restore_mode(){
        if(!display_mode_changed || display_device.empty()) return;
        ChangeDisplaySettingsExW(display_device.c_str(),&original_display_mode,nullptr,0,nullptr);
        display_mode_changed=false;
    }

    ~Impl(){
        restore_mode();
        ClipCursor(nullptr);
        if(window) DestroyWindow(window);
        UnregisterClassW(kWindowClassName,instance);
    }
};

Application::Application()=default;
Application::~Application()=default;

int Application::run(const AppConfig& config,std::unique_ptr<GameDelegate> game){
    // Windows 11: use per-monitor DPI awareness so the DirectX backbuffer and
    // mouse coordinates remain in physical pixels on high-DPI displays.
    SetProcessDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);

    impl_=std::make_unique<Impl>();
    auto& p=*impl_;
    if(!p.create_window(config)){
        std::cerr<<"Unable to create the Win32 game window (error "<<GetLastError()<<").\n";
        return 2;
    }

    p.renderer=create_d3d12_renderer();
    if(!p.renderer || !p.renderer->initialize(nullptr,reinterpret_cast<std::uintptr_t>(p.window),p.width,p.height)){
        std::cerr<<"Direct3D 12 hardware renderer initialization failed. A D3D12-capable GPU/driver is required.\n";
        return 3;
    }

    std::cout<<cpu_worker_summary()<<'\n';
    int frame_limit_fps=0;
    if(const char* env=std::getenv("GAME_FRAME_LIMIT_FPS")){
        char* end=nullptr;
        const long parsed=std::strtol(env,&end,10);
        if(end!=env && parsed>0) frame_limit_fps=(int)std::clamp<long>(parsed,15,1000);
    }
    if(frame_limit_fps>0) std::cout<<"Frame limiter: "<<frame_limit_fps<<" FPS (GAME_FRAME_LIMIT_FPS)\n";
    else std::cout<<"Frame limiter: disabled (full host throughput)\n";

    if(config.fullscreen) p.send_fullscreen(true);
    game->on_start();

    auto prev=std::chrono::steady_clock::now();
    while(!p.quit){
        const auto frame_begin=std::chrono::steady_clock::now();
        p.input.left_pressed=p.input.middle_pressed=p.input.right_pressed=false;
        p.input.escape_pressed=p.input.key_n_pressed=p.input.key_m_pressed=p.input.key_v_pressed=p.input.delete_pressed=false;
        p.input.backspace_pressed=p.input.enter_pressed=false;
        p.input.key_space_pressed=p.input.key_j_pressed=false;
        p.input.key_number_pressed=0;
        p.input.text_input.clear();
        p.input.wheel=0; p.input.mouse_dx=0; p.input.mouse_dy=0;

        MSG msg{};
        while(PeekMessageW(&msg,nullptr,0,0,PM_REMOVE)){
            if(msg.message==WM_QUIT){ p.quit=true; break; }
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
        if(p.quit) break;

        if(p.minimized){
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            prev=std::chrono::steady_clock::now();
            continue;
        }

        const auto now=std::chrono::steady_clock::now();
        float dt=std::chrono::duration<float>(now-prev).count();
        prev=now;
        if(dt>0.1f) dt=0.1f;
        game->on_resize(p.width,p.height);
        game->update(dt,p.input);
        game->render(*p.renderer);

        if(frame_limit_fps>0){
            const auto frame_budget=std::chrono::duration<double>(1.0/(double)frame_limit_fps);
            const auto elapsed=std::chrono::steady_clock::now()-frame_begin;
            if(elapsed<frame_budget) std::this_thread::sleep_for(frame_budget-elapsed);
        }
    }

    p.restore_mode();
    return 0;
}

void Application::request_quit(){ if(impl_) impl_->quit=true; }
void Application::set_fullscreen(bool enabled){ if(impl_) impl_->send_fullscreen(enabled); }
bool Application::fullscreen() const { return impl_?impl_->fullscreen:false; }
std::vector<DisplayMode> Application::display_modes() const { return impl_?impl_->modes():std::vector<DisplayMode>{}; }
DisplayMode Application::current_display_mode() const { return impl_?impl_->current_mode():DisplayMode{}; }
bool Application::set_display_mode(int width,int height){ return impl_?impl_->set_mode(width,height):false; }
void Application::set_cursor_visible(bool visible){ if(impl_) impl_->set_cursor_visible(visible); }
bool Application::cursor_visible() const { return impl_?impl_->cursor_is_visible:true; }
void Application::set_relative_mouse_mode(bool enabled){ if(impl_) impl_->set_relative_mouse_mode(enabled); }
bool Application::relative_mouse_mode() const { return impl_?impl_->relative_mouse_mode:false; }

} // namespace eng
