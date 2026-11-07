#include "engine/engine.hpp"
#include "engine/ui_metrics.hpp"
#include "engine/ui_font.hpp"
#include "renderer_d3d12_shaders.hpp"

#ifndef _WIN32
#error renderer_d3d12.cpp is Windows-only
#endif

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <d3d12.h>
#include <d3dcompiler.h>
#include <dxgi1_6.h>
#include <wrl/client.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace eng {
namespace {
using Microsoft::WRL::ComPtr;

constexpr UINT kFrameCount=3;
constexpr std::size_t kUploadBytesPerFrame=64u*1024u*1024u;
constexpr DXGI_FORMAT kBackBufferFormat=DXGI_FORMAT_R8G8B8A8_UNORM;
constexpr DXGI_FORMAT kDepthFormat=DXGI_FORMAT_D32_FLOAT;
constexpr DXGI_FORMAT kLunarFormat=DXGI_FORMAT_R16G16B16A16_FLOAT;
constexpr std::array kLunarClear{0.002f,0.003f,0.005f,1.0f};
constexpr UINT kSceneRtv=kFrameCount;
constexpr UINT kBloomRtv=kFrameCount+1;
constexpr int kBloomScale=4;
static_assert(sizeof(Vec3)==3*sizeof(float));

std::string wide_to_utf8(const wchar_t* text){
    if(!text || !*text) return {};
    const int bytes=WideCharToMultiByte(CP_UTF8,0,text,-1,nullptr,0,nullptr,nullptr);
    if(bytes<=1) return {};
    std::string out((std::size_t)bytes-1,'\0');
    WideCharToMultiByte(CP_UTF8,0,text,-1,out.data(),bytes,nullptr,nullptr);
    return out;
}

const char* feature_level_name(D3D_FEATURE_LEVEL level){
    switch(level){
        case D3D_FEATURE_LEVEL_12_2: return "12_2 (DirectX 12 Ultimate class)";
        case D3D_FEATURE_LEVEL_12_1: return "12_1";
        case D3D_FEATURE_LEVEL_12_0: return "12_0";
        case D3D_FEATURE_LEVEL_11_1: return "11_1";
        default: return "11_0";
    }
}

std::string vendor_name(UINT vendor){
    switch(vendor){
        case 0x10DE: return "NVIDIA";
        case 0x1002:
        case 0x1022: return "AMD";
        case 0x8086: return "Intel";
        case 0x1414: return "Microsoft";
        default: return "Vendor 0x"+[] (UINT v){
            char b[16]{}; std::snprintf(b,sizeof(b),"%04X",v); return std::string(b);
        }(vendor);
    }
}

void log_hresult(const char* what,HRESULT hr){
    std::cerr<<what<<" failed (HRESULT 0x"<<std::hex<<(unsigned long)hr<<std::dec<<").\n";
}

struct DrawConstants {
    float mvp[16]{};
    float color[4]{};
    float viewport[2]{};
    float line_width{1.0f};
    float material{};
};
static_assert(sizeof(DrawConstants)==24*sizeof(float));

struct SceneConstants {
    struct Fixture {
        float position_radius[4]{};
        float color_power[4]{};
    };
    float sun_daylight[4]{};
    float eye[4]{};
    UINT light_count{};
    UINT pbr{};
    UINT material_detail{};
    UINT terrain_relief{};
    std::array<Fixture,64> lights{};
};
static_assert(offsetof(SceneConstants,lights)==48 && sizeof(SceneConstants::Fixture)==32);
static_assert(static_cast<int>(LunarMaterial::Paint)==0 && static_cast<int>(LunarMaterial::Alloy)==1 &&
              static_cast<int>(LunarMaterial::Regolith)==2 && static_cast<int>(LunarMaterial::Solar)==3 &&
              static_cast<int>(LunarMaterial::Glass)==4 && static_cast<int>(LunarMaterial::Fabric)==5 &&
              static_cast<int>(LunarMaterial::Pad)==6 && static_cast<int>(LunarMaterial::Emissive)==7 &&
              static_cast<int>(LunarMaterial::Skin)==8 && static_cast<int>(LunarMaterial::Marking)==9 &&
              static_cast<int>(LunarMaterial::Hair)==10);

struct PostConstants {
    float pixel_size[2]{};
    float exposure{};
    float bloom{};
    UINT fxaa{};
    UINT filmic{};
    UINT vignette{};
    UINT padding{};
};
static_assert(sizeof(PostConstants)==8*sizeof(UINT));

struct FrameResources {
    ComPtr<ID3D12CommandAllocator> allocator{};
    ComPtr<ID3D12Resource> upload{};
    std::byte* mapped{};
    std::size_t upload_offset{};
    UINT64 fence_value{};
};

ComPtr<ID3DBlob> compile_shader(const char* source,const char* entry,const char* target){
    UINT flags=D3DCOMPILE_ENABLE_STRICTNESS;
#ifdef _DEBUG
    flags|=D3DCOMPILE_DEBUG|D3DCOMPILE_SKIP_OPTIMIZATION;
#else
    flags|=D3DCOMPILE_OPTIMIZATION_LEVEL3;
#endif
    ComPtr<ID3DBlob> shader,error;
    const HRESULT hr=D3DCompile(source,std::strlen(source),"embedded_d3d12.hlsl",nullptr,nullptr,
                                entry,target,flags,0,&shader,&error);
    if(FAILED(hr)){
        if(error) std::cerr<<"HLSL compile error ("<<entry<<"): "
                           <<static_cast<const char*>(error->GetBufferPointer())<<"\n";
        else log_hresult("D3DCompile",hr);
        return {};
    }
    return shader;
}

constexpr const char* kBasicShader=R"HLSL(
struct VSOut { float4 position : SV_POSITION; };

VSOut VSMain(float3 position : POSITION)
{
    VSOut o;
    o.position = clip_position(position);
    return o;
}

[maxvertexcount(4)]
void GSLine(line VSOut inputLine[2], inout TriangleStream<VSOut> stream)
{
    float wa = abs(inputLine[0].position.w) < 1e-6 ? (inputLine[0].position.w < 0 ? -1e-6 : 1e-6) : inputLine[0].position.w;
    float wb = abs(inputLine[1].position.w) < 1e-6 ? (inputLine[1].position.w < 0 ? -1e-6 : 1e-6) : inputLine[1].position.w;
    float2 a = inputLine[0].position.xy / wa;
    float2 b = inputLine[1].position.xy / wb;
    float2 pixelDelta = (b-a) * 0.5 * max(gViewport, float2(1.0,1.0));
    float len = length(pixelDelta);
    float2 perp = len > 1e-5 ? float2(-pixelDelta.y, pixelDelta.x) / len : float2(0.0,1.0);
    float2 offsetNdc = perp * max(gLineWidth,1.0) / max(gViewport,float2(1.0,1.0));

    VSOut v;
    v.position = inputLine[0].position; v.position.xy += offsetNdc * wa; stream.Append(v);
    v.position = inputLine[0].position; v.position.xy -= offsetNdc * wa; stream.Append(v);
    v.position = inputLine[1].position; v.position.xy += offsetNdc * wb; stream.Append(v);
    v.position = inputLine[1].position; v.position.xy -= offsetNdc * wb; stream.Append(v);
    stream.RestartStrip();
}

float4 PSMain(VSOut input) : SV_TARGET
{
    return gColor;
}

float4 PSLinear(VSOut input) : SV_TARGET
{
    return float4(linear_color(gColor.rgb), gColor.a);
}
)HLSL";

constexpr D3D12_INPUT_ELEMENT_DESC kPositionInput{
    "POSITION",0,DXGI_FORMAT_R32G32B32_FLOAT,0,0,D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA,0
};

D3D12_BLEND_DESC blend_desc(){
    D3D12_BLEND_DESC d{};
    d.AlphaToCoverageEnable=FALSE;
    d.IndependentBlendEnable=FALSE;
    auto& rt=d.RenderTarget[0];
    rt.BlendEnable=TRUE;
    rt.LogicOpEnable=FALSE;
    rt.SrcBlend=D3D12_BLEND_SRC_ALPHA;
    rt.DestBlend=D3D12_BLEND_INV_SRC_ALPHA;
    rt.BlendOp=D3D12_BLEND_OP_ADD;
    rt.SrcBlendAlpha=D3D12_BLEND_ONE;
    rt.DestBlendAlpha=D3D12_BLEND_INV_SRC_ALPHA;
    rt.BlendOpAlpha=D3D12_BLEND_OP_ADD;
    rt.LogicOp=D3D12_LOGIC_OP_NOOP;
    rt.RenderTargetWriteMask=D3D12_COLOR_WRITE_ENABLE_ALL;
    return d;
}

D3D12_RASTERIZER_DESC raster_desc(){
    D3D12_RASTERIZER_DESC d{};
    d.FillMode=D3D12_FILL_MODE_SOLID;
    d.CullMode=D3D12_CULL_MODE_NONE;
    d.FrontCounterClockwise=FALSE;
    d.DepthBias=D3D12_DEFAULT_DEPTH_BIAS;
    d.DepthBiasClamp=D3D12_DEFAULT_DEPTH_BIAS_CLAMP;
    d.SlopeScaledDepthBias=D3D12_DEFAULT_SLOPE_SCALED_DEPTH_BIAS;
    d.DepthClipEnable=TRUE;
    d.MultisampleEnable=FALSE;
    d.AntialiasedLineEnable=FALSE;
    d.ForcedSampleCount=0;
    d.ConservativeRaster=D3D12_CONSERVATIVE_RASTERIZATION_MODE_OFF;
    return d;
}

D3D12_DEPTH_STENCIL_DESC depth_desc(bool enabled){
    D3D12_DEPTH_STENCIL_DESC d{};
    d.DepthEnable=enabled?TRUE:FALSE;
    d.DepthWriteMask=enabled?D3D12_DEPTH_WRITE_MASK_ALL:D3D12_DEPTH_WRITE_MASK_ZERO;
    d.DepthFunc=D3D12_COMPARISON_FUNC_LESS_EQUAL;
    d.StencilEnable=FALSE;
    d.StencilReadMask=D3D12_DEFAULT_STENCIL_READ_MASK;
    d.StencilWriteMask=D3D12_DEFAULT_STENCIL_WRITE_MASK;
    return d;
}

D3D12_HEAP_PROPERTIES heap_props(D3D12_HEAP_TYPE type){
    D3D12_HEAP_PROPERTIES h{};
    h.Type=type;
    h.CPUPageProperty=D3D12_CPU_PAGE_PROPERTY_UNKNOWN;
    h.MemoryPoolPreference=D3D12_MEMORY_POOL_UNKNOWN;
    h.CreationNodeMask=1;
    h.VisibleNodeMask=1;
    return h;
}

D3D12_RESOURCE_DESC buffer_desc(UINT64 bytes){
    D3D12_RESOURCE_DESC d{};
    d.Dimension=D3D12_RESOURCE_DIMENSION_BUFFER;
    d.Alignment=0;
    d.Width=bytes;
    d.Height=1;
    d.DepthOrArraySize=1;
    d.MipLevels=1;
    d.Format=DXGI_FORMAT_UNKNOWN;
    d.SampleDesc.Count=1;
    d.SampleDesc.Quality=0;
    d.Layout=D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    d.Flags=D3D12_RESOURCE_FLAG_NONE;
    return d;
}

D3D12_RESOURCE_DESC target_desc(int width,int height,DXGI_FORMAT format,D3D12_RESOURCE_FLAGS flags){
    D3D12_RESOURCE_DESC d{};
    d.Dimension=D3D12_RESOURCE_DIMENSION_TEXTURE2D;
    d.Width=static_cast<UINT64>(width);
    d.Height=static_cast<UINT>(height);
    d.DepthOrArraySize=1;
    d.MipLevels=1;
    d.Format=format;
    d.SampleDesc.Count=1;
    d.Layout=D3D12_TEXTURE_LAYOUT_UNKNOWN;
    d.Flags=flags;
    return d;
}

// The first ring joins the flat build plane. Relief grows only outside it.
std::vector<Vec3> lunar_horizon(float half,bool relief){
    constexpr std::array offsets{0.0f,32.0f,72.0f,128.0f,210.0f,340.0f,540.0f,810.0f,1150.0f,1700.0f};
    constexpr int segments=32;
    using Ring=std::array<Vec3,4*segments>;
    Ring previous{},current{};
    std::vector<Vec3> terrain;
    terrain.reserve((offsets.size()-1)*current.size()*6);
    for(float offset:offsets){
        const float radius=half+offset;
        const float relief_scale=relief?std::clamp(offset/280.0f,0.0f,1.0f):0.0f;
        for(int step=0;step<segments;++step){
            const float edge=radius*(2.0f*step/segments-1.0f);
            current[step]={edge,0,-radius};
            current[step+segments]={radius,0,edge};
            current[step+2*segments]={-edge,0,radius};
            current[step+3*segments]={-radius,0,-edge};
        }
        for(auto& p:current){
            const float hills=std::sin(p.x*0.008f)*std::cos(p.z*0.008f);
            const float ridges=std::sin(p.x*0.020f+p.z*0.012f)*std::cos(p.z*0.017f);
            p.y=relief_scale*(22.0f*hills+5.0f*ridges+10.0f);
        }
        if(offset>0.0f){
            for(std::size_t i=0;i<current.size();++i){
                const std::size_t j=(i+1)%current.size();
                terrain.insert(terrain.end(),{previous[i],current[j],current[i],previous[i],previous[j],current[j]});
            }
        }
        previous=current;
    }
    return terrain;
}

} // namespace

class D3D12Renderer final : public Renderer {
    HWND hwnd_{};
    int width_{};
    int height_{};
    int pending_width_{};
    int pending_height_{};
    bool depth_enabled_{true};
    Mat4 mvp_{Mat4::identity()};

    ComPtr<IDXGIFactory6> factory_{};
    ComPtr<IDXGIAdapter1> adapter_{};
    DXGI_ADAPTER_DESC1 adapter_desc_{};
    ComPtr<ID3D12Device> device_{};
    D3D_FEATURE_LEVEL max_feature_level_{D3D_FEATURE_LEVEL_11_0};
    ComPtr<ID3D12CommandQueue> queue_{};
    ComPtr<IDXGISwapChain3> swap_chain_{};
    ComPtr<ID3D12DescriptorHeap> rtv_heap_{};
    ComPtr<ID3D12DescriptorHeap> dsv_heap_{};
    UINT rtv_increment_{};
    std::array<ComPtr<ID3D12Resource>,kFrameCount> render_targets_{};
    ComPtr<ID3D12Resource> depth_buffer_{};
    std::array<FrameResources,kFrameCount> frames_{};
    ComPtr<ID3D12GraphicsCommandList> command_list_{};
    ComPtr<ID3D12Fence> fence_{};
    HANDLE fence_event_{};
    UINT64 next_fence_value_{1};
    UINT frame_index_{};
    bool frame_open_{false};
    bool tearing_supported_{false};
    bool vsync_{false};
    bool upload_overflow_reported_{false};

    GraphicsSettings graphics_{};
    bool enhanced_barriers_{};
    bool variable_rate_shading_{};
    bool coarse_shading_{};
    bool lunar_active_{};
    float terrain_half_{};
    bool terrain_relief_{};
    std::vector<Vec3> horizon_{};
    D3D12_GPU_VIRTUAL_ADDRESS scene_constants_{};
#ifdef __ID3D12GraphicsCommandList7_INTERFACE_DEFINED__
    ComPtr<ID3D12GraphicsCommandList7> barrier_list_{};
#endif
#ifdef __ID3D12GraphicsCommandList5_INTERFACE_DEFINED__
    ComPtr<ID3D12GraphicsCommandList5> shading_rate_list_{};
#endif

    ComPtr<ID3D12RootSignature> root_signature_{};
    ComPtr<ID3D12PipelineState> triangles_depth_{};
    ComPtr<ID3D12PipelineState> triangles_no_depth_{};
    ComPtr<ID3D12PipelineState> lines_depth_{};
    ComPtr<ID3D12PipelineState> lines_no_depth_{};
    ComPtr<ID3D12PipelineState> lunar_triangles_{};
    ComPtr<ID3D12PipelineState> lunar_lines_{};
    ComPtr<ID3D12RootSignature> post_signature_{};
    ComPtr<ID3D12PipelineState> bloom_pso_{};
    ComPtr<ID3D12PipelineState> composite_pso_{};
    ComPtr<ID3D12DescriptorHeap> lunar_srv_heap_{};
    UINT srv_increment_{};
    ComPtr<ID3D12Resource> lunar_target_{};
    ComPtr<ID3D12Resource> bloom_target_{};

    bool create_factory_and_device(){
        UINT flags=0;
#ifdef _DEBUG
        ComPtr<ID3D12Debug> debug;
        if(SUCCEEDED(D3D12GetDebugInterface(IID_PPV_ARGS(&debug)))){
            debug->EnableDebugLayer();
            flags|=DXGI_CREATE_FACTORY_DEBUG;
        }
#endif
        ComPtr<IDXGIFactory4> factory4;
        HRESULT hr=CreateDXGIFactory2(flags,IID_PPV_ARGS(&factory4));
        if(FAILED(hr) && flags) hr=CreateDXGIFactory2(0,IID_PPV_ARGS(&factory4));
        if(FAILED(hr)){ log_hresult("CreateDXGIFactory2",hr); return false; }
        factory4.As(&factory_);

        auto try_adapter=[&](IDXGIAdapter1* candidate)->bool{
            if(!candidate) return false;
            DXGI_ADAPTER_DESC1 desc{};
            candidate->GetDesc1(&desc);
            if(desc.Flags&DXGI_ADAPTER_FLAG_SOFTWARE) return false;
            ComPtr<ID3D12Device> test;
            if(FAILED(D3D12CreateDevice(candidate,D3D_FEATURE_LEVEL_11_0,IID_PPV_ARGS(&test)))) return false;
            adapter_=candidate;
            adapter_desc_=desc;
            device_=std::move(test);
            return true;
        };

        if(factory_){
            for(UINT i=0;;++i){
                ComPtr<IDXGIAdapter1> candidate;
                if(factory_->EnumAdapterByGpuPreference(i,DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE,IID_PPV_ARGS(&candidate))==DXGI_ERROR_NOT_FOUND) break;
                if(try_adapter(candidate.Get())) break;
            }
        }
        if(!device_){
            for(UINT i=0;;++i){
                ComPtr<IDXGIAdapter1> candidate;
                if(factory4->EnumAdapters1(i,&candidate)==DXGI_ERROR_NOT_FOUND) break;
                if(try_adapter(candidate.Get())) break;
            }
        }
        if(!device_){
            std::cerr<<"No hardware Direct3D 12 adapter was found. WARP is intentionally not used for this game.\n";
            return false;
        }

        std::array<D3D_FEATURE_LEVEL,5> requested{
            D3D_FEATURE_LEVEL_12_2,
            D3D_FEATURE_LEVEL_12_1,
            D3D_FEATURE_LEVEL_12_0,
            D3D_FEATURE_LEVEL_11_1,
            D3D_FEATURE_LEVEL_11_0
        };
        D3D12_FEATURE_DATA_FEATURE_LEVELS levels{};
        levels.NumFeatureLevels=(UINT)requested.size();
        levels.pFeatureLevelsRequested=requested.data();
        levels.MaxSupportedFeatureLevel=D3D_FEATURE_LEVEL_11_0;
        if(SUCCEEDED(device_->CheckFeatureSupport(D3D12_FEATURE_FEATURE_LEVELS,&levels,sizeof(levels))))
            max_feature_level_=levels.MaxSupportedFeatureLevel;

        BOOL allow_tearing=FALSE;
        if(factory_ && SUCCEEDED(factory_->CheckFeatureSupport(DXGI_FEATURE_PRESENT_ALLOW_TEARING,&allow_tearing,sizeof(allow_tearing))))
            tearing_supported_=allow_tearing==TRUE;

        if(const char* env=std::getenv("GAME_VSYNC")){
            std::string v(env);
            std::transform(v.begin(),v.end(),v.begin(),[](unsigned char c){return (char)std::tolower(c);});
            vsync_=(v=="1"||v=="true"||v=="on");
        }

        std::cout<<"GPU renderer: "<<vendor_name(adapter_desc_.VendorId)<<" | "<<wide_to_utf8(adapter_desc_.Description)
                 <<" | Direct3D 12 feature level "<<feature_level_name(max_feature_level_)<<"\n";
        std::cout<<"DXGI present: "<<(vsync_?"vsync":"unlimited")
                 <<(tearing_supported_?"; variable-refresh tearing supported":"; tearing unavailable")
                 <<" (override with GAME_VSYNC=0|1)\n";
        return true;
    }

    bool create_queue_and_swapchain(){
        D3D12_COMMAND_QUEUE_DESC q{};
        q.Type=D3D12_COMMAND_LIST_TYPE_DIRECT;
        q.Priority=D3D12_COMMAND_QUEUE_PRIORITY_NORMAL;
        q.Flags=D3D12_COMMAND_QUEUE_FLAG_NONE;
        HRESULT hr=device_->CreateCommandQueue(&q,IID_PPV_ARGS(&queue_));
        if(FAILED(hr)){ log_hresult("CreateCommandQueue",hr); return false; }

        DXGI_SWAP_CHAIN_DESC1 sd{};
        sd.Width=(UINT)std::max(1,width_);
        sd.Height=(UINT)std::max(1,height_);
        sd.Format=kBackBufferFormat;
        sd.Stereo=FALSE;
        sd.SampleDesc.Count=1;
        sd.BufferUsage=DXGI_USAGE_RENDER_TARGET_OUTPUT;
        sd.BufferCount=kFrameCount;
        sd.Scaling=DXGI_SCALING_STRETCH;
        sd.SwapEffect=DXGI_SWAP_EFFECT_FLIP_DISCARD;
        sd.AlphaMode=DXGI_ALPHA_MODE_IGNORE;
        sd.Flags=tearing_supported_?DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING:0;

        ComPtr<IDXGISwapChain1> swap1;
        hr=factory_->CreateSwapChainForHwnd(queue_.Get(),hwnd_,&sd,nullptr,nullptr,&swap1);
        if(FAILED(hr)){ log_hresult("CreateSwapChainForHwnd",hr); return false; }
        factory_->MakeWindowAssociation(hwnd_,DXGI_MWA_NO_ALT_ENTER);
        if(FAILED(swap1.As(&swap_chain_))) return false;
        frame_index_=swap_chain_->GetCurrentBackBufferIndex();
        return true;
    }

    bool create_descriptor_heaps(){
        D3D12_DESCRIPTOR_HEAP_DESC rtv{};
        rtv.Type=D3D12_DESCRIPTOR_HEAP_TYPE_RTV;
        rtv.NumDescriptors=kFrameCount+2;
        if(FAILED(device_->CreateDescriptorHeap(&rtv,IID_PPV_ARGS(&rtv_heap_)))) return false;
        rtv_increment_=device_->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);

        D3D12_DESCRIPTOR_HEAP_DESC dsv{};
        dsv.Type=D3D12_DESCRIPTOR_HEAP_TYPE_DSV;
        dsv.NumDescriptors=1;
        if(FAILED(device_->CreateDescriptorHeap(&dsv,IID_PPV_ARGS(&dsv_heap_)))) return false;
        return true;
    }

    bool create_back_buffers(){
        auto handle=rtv_heap_->GetCPUDescriptorHandleForHeapStart();
        for(UINT i=0;i<kFrameCount;++i){
            HRESULT hr=swap_chain_->GetBuffer(i,IID_PPV_ARGS(&render_targets_[i]));
            if(FAILED(hr)) return false;
            device_->CreateRenderTargetView(render_targets_[i].Get(),nullptr,handle);
            handle.ptr += rtv_increment_;
        }
        return create_depth_buffer();
    }

    bool create_depth_buffer(){
        depth_buffer_.Reset();
        const auto d=target_desc(width_,height_,kDepthFormat,D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL);
        D3D12_CLEAR_VALUE clear{};
        clear.Format=kDepthFormat;
        clear.DepthStencil.Depth=1.0f;
        clear.DepthStencil.Stencil=0;
        const auto heap=heap_props(D3D12_HEAP_TYPE_DEFAULT);
        HRESULT hr=device_->CreateCommittedResource(&heap,D3D12_HEAP_FLAG_NONE,&d,D3D12_RESOURCE_STATE_DEPTH_WRITE,
                                                    &clear,IID_PPV_ARGS(&depth_buffer_));
        if(FAILED(hr)){ log_hresult("Create depth buffer",hr); return false; }
        D3D12_DEPTH_STENCIL_VIEW_DESC view{};
        view.Format=kDepthFormat;
        view.ViewDimension=D3D12_DSV_DIMENSION_TEXTURE2D;
        device_->CreateDepthStencilView(depth_buffer_.Get(),&view,dsv_heap_->GetCPUDescriptorHandleForHeapStart());
        return true;
    }

    int bloom_width() const { return (width_+kBloomScale-1)/kBloomScale; }
    int bloom_height() const { return (height_+kBloomScale-1)/kBloomScale; }

    bool create_lunar_targets(){
        lunar_target_.Reset();
        bloom_target_.Reset();
        D3D12_CLEAR_VALUE clear{};
        clear.Format=kLunarFormat;
        std::copy(kLunarClear.begin(),kLunarClear.end(),clear.Color);
        const auto heap=heap_props(D3D12_HEAP_TYPE_DEFAULT);
        auto make_target=[&](int width,int height,const D3D12_CLEAR_VALUE* optimized_clear,
                             ComPtr<ID3D12Resource>& out){
            const auto desc=target_desc(width,height,kLunarFormat,D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET);
            return device_->CreateCommittedResource(&heap,D3D12_HEAP_FLAG_NONE,&desc,
                D3D12_RESOURCE_STATE_RENDER_TARGET,optimized_clear,IID_PPV_ARGS(&out));
        };
        ComPtr<ID3D12Resource> scene,bloom;
        HRESULT hr=make_target(width_,height_,&clear,scene);
        if(SUCCEEDED(hr)) hr=make_target(bloom_width(),bloom_height(),nullptr,bloom);
        if(FAILED(hr)){
            log_hresult("Create lunar render targets",hr);
            return false;
        }
        device_->CreateRenderTargetView(scene.Get(),nullptr,rtv_handle(kSceneRtv));
        device_->CreateRenderTargetView(bloom.Get(),nullptr,rtv_handle(kBloomRtv));

        D3D12_SHADER_RESOURCE_VIEW_DESC srv{};
        srv.Format=kLunarFormat;
        srv.ViewDimension=D3D12_SRV_DIMENSION_TEXTURE2D;
        srv.Shader4ComponentMapping=D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        srv.Texture2D.MipLevels=1;
        auto cpu=lunar_srv_heap_->GetCPUDescriptorHandleForHeapStart();
        device_->CreateShaderResourceView(scene.Get(),&srv,cpu);
        cpu.ptr+=srv_increment_;
        device_->CreateShaderResourceView(bloom.Get(),&srv,cpu);
        lunar_target_=std::move(scene);
        bloom_target_=std::move(bloom);
        return true;
    }

    bool create_frame_resources(){
        for(auto& frame:frames_){
            HRESULT hr=device_->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT,IID_PPV_ARGS(&frame.allocator));
            if(FAILED(hr)) return false;
            const auto heap=heap_props(D3D12_HEAP_TYPE_UPLOAD);
            const auto desc=buffer_desc(kUploadBytesPerFrame);
            hr=device_->CreateCommittedResource(&heap,D3D12_HEAP_FLAG_NONE,&desc,D3D12_RESOURCE_STATE_GENERIC_READ,nullptr,
                                                IID_PPV_ARGS(&frame.upload));
            if(FAILED(hr)) return false;
            D3D12_RANGE read_range{0,0};
            void* mapped=nullptr;
            hr=frame.upload->Map(0,&read_range,&mapped);
            if(FAILED(hr)) return false;
            frame.mapped=static_cast<std::byte*>(mapped);
        }
        HRESULT hr=device_->CreateCommandList(0,D3D12_COMMAND_LIST_TYPE_DIRECT,frames_[0].allocator.Get(),nullptr,
                                              IID_PPV_ARGS(&command_list_));
        if(FAILED(hr)) return false;
        if(FAILED(command_list_->Close())) return false;

#ifdef __ID3D12GraphicsCommandList7_INTERFACE_DEFINED__
        D3D12_FEATURE_DATA_D3D12_OPTIONS12 barriers{};
        enhanced_barriers_=SUCCEEDED(device_->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS12,&barriers,sizeof(barriers))) &&
                           barriers.EnhancedBarriersSupported && SUCCEEDED(command_list_.As(&barrier_list_));
#endif
#ifdef __ID3D12GraphicsCommandList5_INTERFACE_DEFINED__
        D3D12_FEATURE_DATA_D3D12_OPTIONS6 rates{};
        variable_rate_shading_=SUCCEEDED(device_->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS6,&rates,sizeof(rates))) &&
                               rates.VariableShadingRateTier>=D3D12_VARIABLE_SHADING_RATE_TIER_1 &&
                               SUCCEEDED(command_list_.As(&shading_rate_list_));
#endif

        hr=device_->CreateFence(0,D3D12_FENCE_FLAG_NONE,IID_PPV_ARGS(&fence_));
        if(FAILED(hr)) return false;
        fence_event_=CreateEventW(nullptr,FALSE,FALSE,nullptr);
        return fence_event_!=nullptr;
    }

    bool create_signature(const D3D12_ROOT_SIGNATURE_DESC& desc,ComPtr<ID3D12RootSignature>& out){
        ComPtr<ID3DBlob> signature,error;
        const HRESULT hr=D3D12SerializeRootSignature(&desc,D3D_ROOT_SIGNATURE_VERSION_1,&signature,&error);
        if(FAILED(hr)){
            if(error) std::cerr<<static_cast<const char*>(error->GetBufferPointer())<<"\n";
            else log_hresult("D3D12SerializeRootSignature",hr);
            return false;
        }
        const HRESULT result=device_->CreateRootSignature(0,signature->GetBufferPointer(),signature->GetBufferSize(),
                                                         IID_PPV_ARGS(&out));
        if(FAILED(result)) log_hresult("CreateRootSignature",result);
        return SUCCEEDED(result);
    }

    bool create_pipeline(ID3D12RootSignature* signature,ID3DBlob* vs,ID3DBlob* ps,ID3DBlob* gs,
                         DXGI_FORMAT format,bool depth,bool mesh,ComPtr<ID3D12PipelineState>& out){
        D3D12_GRAPHICS_PIPELINE_STATE_DESC p{};
        p.pRootSignature=signature;
        p.VS={vs->GetBufferPointer(),vs->GetBufferSize()};
        p.PS={ps->GetBufferPointer(),ps->GetBufferSize()};
        if(gs) p.GS={gs->GetBufferPointer(),gs->GetBufferSize()};
        p.BlendState=blend_desc();
        p.BlendState.RenderTarget[0].BlendEnable=mesh?TRUE:FALSE;
        p.SampleMask=UINT_MAX;
        p.RasterizerState=raster_desc();
        p.DepthStencilState=depth_desc(depth);
        if(mesh) p.InputLayout={&kPositionInput,1};
        p.PrimitiveTopologyType=gs?D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE:D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
        p.NumRenderTargets=1;
        p.RTVFormats[0]=format;
        p.DSVFormat=mesh?kDepthFormat:DXGI_FORMAT_UNKNOWN;
        p.SampleDesc.Count=1;
        const HRESULT hr=device_->CreateGraphicsPipelineState(&p,IID_PPV_ARGS(&out));
        if(FAILED(hr)) log_hresult("CreateGraphicsPipelineState",hr);
        return SUCCEEDED(hr);
    }

    bool create_root_signature_and_psos(){
        D3D12_ROOT_PARAMETER roots[2]{};
        roots[0].ParameterType=D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        roots[0].Constants.Num32BitValues=sizeof(DrawConstants)/sizeof(UINT);
        roots[0].ShaderVisibility=D3D12_SHADER_VISIBILITY_ALL;
        roots[1].ParameterType=D3D12_ROOT_PARAMETER_TYPE_CBV;
        roots[1].Descriptor.ShaderRegister=1;
        roots[1].ShaderVisibility=D3D12_SHADER_VISIBILITY_PIXEL;
        D3D12_ROOT_SIGNATURE_DESC rs{};
        rs.NumParameters=2;
        rs.pParameters=roots;
        rs.Flags=D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;
        if(!create_signature(rs,root_signature_)) return false;

        const std::string basic=std::string(d3d12_shaders::draw_constants)+kBasicShader;
        auto vs=compile_shader(basic.c_str(),"VSMain","vs_5_1");
        auto gs=compile_shader(basic.c_str(),"GSLine","gs_5_1");
        auto ps=compile_shader(basic.c_str(),"PSMain","ps_5_1");
        if(!vs||!gs||!ps) return false;

        const auto classic=[&](bool lines,bool depth,ComPtr<ID3D12PipelineState>& pso){
            return create_pipeline(root_signature_.Get(),vs.Get(),ps.Get(),lines?gs.Get():nullptr,
                                   kBackBufferFormat,depth,true,pso);
        };
        if(!classic(false,true,triangles_depth_) || !classic(false,false,triangles_no_depth_) ||
           !classic(true,true,lines_depth_) || !classic(true,false,lines_no_depth_)) return false;

        const bool lunar=create_lunar_pipeline(basic,vs.Get(),gs.Get()) && create_lunar_targets();
        std::cout<<"D3D12 lunar rendering: "<<(lunar?"FP16 lighting and post-processing":"classic meshes")
                 <<" | barriers: "<<(enhanced_barriers_?"enhanced":"legacy")
                 <<" | terrain VRS: "<<(variable_rate_shading_?"available":"unavailable")<<"\n";
        return true;
    }

    bool create_lunar_pipeline(const std::string& basic,ID3DBlob* vs,ID3DBlob* gs){
        constexpr UINT needed=static_cast<UINT>(D3D12_FORMAT_SUPPORT1_RENDER_TARGET) |
                              static_cast<UINT>(D3D12_FORMAT_SUPPORT1_SHADER_SAMPLE) |
                              static_cast<UINT>(D3D12_FORMAT_SUPPORT1_BLENDABLE);
        D3D12_FEATURE_DATA_FORMAT_SUPPORT format{};
        format.Format=kLunarFormat;
        if(FAILED(device_->CheckFeatureSupport(D3D12_FEATURE_FORMAT_SUPPORT,&format,sizeof(format))) ||
           (static_cast<UINT>(format.Support1)&needed)!=needed) return false;

        const std::string lunar=std::string(d3d12_shaders::draw_constants)+d3d12_shaders::lunar;
        auto lunar_vs=compile_shader(lunar.c_str(),"VSLunar","vs_5_1");
        auto lunar_ps=compile_shader(lunar.c_str(),"PSLunar","ps_5_1");
        auto linear_ps=compile_shader(basic.c_str(),"PSLinear","ps_5_1");
        if(!lunar_vs || !lunar_ps || !linear_ps) return false;
        if(!create_pipeline(root_signature_.Get(),lunar_vs.Get(),lunar_ps.Get(),nullptr,kLunarFormat,true,true,lunar_triangles_) ||
           !create_pipeline(root_signature_.Get(),vs,linear_ps.Get(),gs,kLunarFormat,true,true,lunar_lines_)) return false;

        D3D12_DESCRIPTOR_RANGE ranges[2]{};
        D3D12_ROOT_PARAMETER roots[3]{};
        roots[0].ParameterType=D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        roots[0].Constants.Num32BitValues=sizeof(PostConstants)/sizeof(UINT);
        roots[0].ShaderVisibility=D3D12_SHADER_VISIBILITY_PIXEL;
        for(UINT i=0;i<2;++i){
            ranges[i].RangeType=D3D12_DESCRIPTOR_RANGE_TYPE_SRV;
            ranges[i].NumDescriptors=1;
            ranges[i].BaseShaderRegister=i;
            roots[i+1].ParameterType=D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE;
            roots[i+1].DescriptorTable={1,&ranges[i]};
            roots[i+1].ShaderVisibility=D3D12_SHADER_VISIBILITY_PIXEL;
        }
        D3D12_STATIC_SAMPLER_DESC sampler{};
        sampler.Filter=D3D12_FILTER_MIN_MAG_LINEAR_MIP_POINT;
        sampler.AddressU=sampler.AddressV=sampler.AddressW=D3D12_TEXTURE_ADDRESS_MODE_CLAMP;
        sampler.MaxAnisotropy=1;
        sampler.ComparisonFunc=D3D12_COMPARISON_FUNC_NEVER;
        sampler.MaxLOD=D3D12_FLOAT32_MAX;
        sampler.ShaderVisibility=D3D12_SHADER_VISIBILITY_PIXEL;
        D3D12_ROOT_SIGNATURE_DESC post{3,roots,1,&sampler,D3D12_ROOT_SIGNATURE_FLAG_NONE};
        if(!create_signature(post,post_signature_)) return false;

        auto post_vs=compile_shader(d3d12_shaders::post,"VSFullscreen","vs_5_1");
        auto bloom_ps=compile_shader(d3d12_shaders::post,"PSBloom","ps_5_1");
        auto composite_ps=compile_shader(d3d12_shaders::post,"PSComposite","ps_5_1");
        if(!post_vs || !bloom_ps || !composite_ps) return false;
        if(!create_pipeline(post_signature_.Get(),post_vs.Get(),bloom_ps.Get(),nullptr,kLunarFormat,false,false,bloom_pso_) ||
           !create_pipeline(post_signature_.Get(),post_vs.Get(),composite_ps.Get(),nullptr,kBackBufferFormat,false,false,composite_pso_))
            return false;

        D3D12_DESCRIPTOR_HEAP_DESC heap{};
        heap.Type=D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
        heap.NumDescriptors=2;
        heap.Flags=D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
        srv_increment_=device_->GetDescriptorHandleIncrementSize(heap.Type);
        return SUCCEEDED(device_->CreateDescriptorHeap(&heap,IID_PPV_ARGS(&lunar_srv_heap_)));
    }

    void wait_for_fence(UINT64 value){
        if(!fence_ || value==0 || fence_->GetCompletedValue()>=value) return;
        if(SUCCEEDED(fence_->SetEventOnCompletion(value,fence_event_))) WaitForSingleObject(fence_event_,INFINITE);
    }

    void wait_for_gpu(){
        if(!queue_ || !fence_) return;
        const UINT64 value=next_fence_value_++;
        if(SUCCEEDED(queue_->Signal(fence_.Get(),value))) wait_for_fence(value);
    }

    D3D12_CPU_DESCRIPTOR_HANDLE rtv_handle(UINT index) const {
        auto h=rtv_heap_->GetCPUDescriptorHandleForHeapStart();
        h.ptr+=static_cast<SIZE_T>(index)*rtv_increment_;
        return h;
    }

    enum class TextureUse { Present, RenderTarget, PixelRead };

    void transition(ID3D12Resource* texture,TextureUse from,TextureUse to){
#ifdef __ID3D12GraphicsCommandList7_INTERFACE_DEFINED__
        if(enhanced_barriers_){
            struct Usage {
                D3D12_BARRIER_SYNC sync;
                D3D12_BARRIER_ACCESS access;
                D3D12_BARRIER_LAYOUT layout;
            };
            constexpr std::array<Usage,3> usage{{
                {D3D12_BARRIER_SYNC_NONE,D3D12_BARRIER_ACCESS_NO_ACCESS,D3D12_BARRIER_LAYOUT_PRESENT},
                {D3D12_BARRIER_SYNC_RENDER_TARGET,D3D12_BARRIER_ACCESS_RENDER_TARGET,D3D12_BARRIER_LAYOUT_RENDER_TARGET},
                {D3D12_BARRIER_SYNC_PIXEL_SHADING,D3D12_BARRIER_ACCESS_SHADER_RESOURCE,D3D12_BARRIER_LAYOUT_SHADER_RESOURCE}
            }};
            const auto before=usage[static_cast<unsigned>(from)];
            const auto after=usage[static_cast<unsigned>(to)];
            D3D12_TEXTURE_BARRIER barrier{};
            barrier.SyncBefore=before.sync;
            barrier.SyncAfter=after.sync;
            barrier.AccessBefore=before.access;
            barrier.AccessAfter=after.access;
            barrier.LayoutBefore=before.layout;
            barrier.LayoutAfter=after.layout;
            barrier.pResource=texture;
            // These targets have one mip, one array slice, and one plane.
            D3D12_BARRIER_GROUP group{};
            group.Type=D3D12_BARRIER_TYPE_TEXTURE;
            group.NumBarriers=1;
            group.pTextureBarriers=&barrier;
            barrier_list_->Barrier(1,&group);
            return;
        }
#endif
        constexpr std::array states{
            D3D12_RESOURCE_STATE_PRESENT,D3D12_RESOURCE_STATE_RENDER_TARGET,D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE
        };
        D3D12_RESOURCE_BARRIER barrier{};
        barrier.Type=D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barrier.Transition.pResource=texture;
        barrier.Transition.Subresource=D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        barrier.Transition.StateBefore=states[static_cast<unsigned>(from)];
        barrier.Transition.StateAfter=states[static_cast<unsigned>(to)];
        command_list_->ResourceBarrier(1,&barrier);
    }

    void set_coarse_shading(bool coarse){
#ifdef __ID3D12GraphicsCommandList5_INTERFACE_DEFINED__
        if(!variable_rate_shading_ || coarse==coarse_shading_) return;
        shading_rate_list_->RSSetShadingRate(coarse?D3D12_SHADING_RATE_2X2:D3D12_SHADING_RATE_1X1,nullptr);
        coarse_shading_=coarse;
#else
        (void)coarse;
#endif
    }

    void set_viewport(int width,int height){
        D3D12_VIEWPORT viewport{0.0f,0.0f,static_cast<float>(width),static_cast<float>(height),0.0f,1.0f};
        D3D12_RECT scissor{0,0,width,height};
        command_list_->RSSetViewports(1,&viewport);
        command_list_->RSSetScissorRects(1,&scissor);
    }

    D3D12_GPU_VIRTUAL_ADDRESS upload(const void* data,std::size_t bytes,std::size_t alignment){
        auto& frame=frames_[frame_index_];
        const std::size_t aligned=(frame.upload_offset+alignment-1)&~(alignment-1);
        if(aligned>kUploadBytesPerFrame || bytes>kUploadBytesPerFrame-aligned){
            if(!upload_overflow_reported_){
                upload_overflow_reported_=true;
                std::cerr<<"D3D12 per-frame upload budget exceeded (64 MiB). Further draws in this frame are skipped.\n";
            }
            return 0;
        }
        std::memcpy(frame.mapped+aligned,data,bytes);
        frame.upload_offset=aligned+bytes;
        return frame.upload->GetGPUVirtualAddress()+aligned;
    }

    void draw(const void* data,std::size_t vertices,bool lines,Color c,float width,
              LunarMaterial material=LunarMaterial::Paint,bool coarse=false){
        if(!frame_open_ || vertices==0) return;
        constexpr UINT stride=3u*sizeof(float);
        const auto bytes=vertices*stride;
        const auto address=upload(data,bytes,16u);
        if(!address) return;
        const D3D12_VERTEX_BUFFER_VIEW vb{address,static_cast<UINT>(bytes),stride};

        ID3D12PipelineState* pso=nullptr;
        if(lunar_active_) pso=lines?lunar_lines_.Get():lunar_triangles_.Get();
        else if(lines) pso=depth_enabled_?lines_depth_.Get():lines_no_depth_.Get();
        else pso=depth_enabled_?triangles_depth_.Get():triangles_no_depth_.Get();
        set_coarse_shading(coarse);
        command_list_->SetPipelineState(pso);
        command_list_->SetGraphicsRootSignature(root_signature_.Get());
        if(lunar_active_ && !lines) command_list_->SetGraphicsRootConstantBufferView(1,scene_constants_);

        DrawConstants constants{};
        std::memcpy(constants.mvp,mvp_.m.data(),sizeof(constants.mvp));
        constants.color[0]=c.r; constants.color[1]=c.g; constants.color[2]=c.b; constants.color[3]=c.a;
        constants.viewport[0]=(float)std::max(1,width_);
        constants.viewport[1]=(float)std::max(1,height_);
        constants.line_width=std::max(1.0f,width);
        constants.material=static_cast<float>(material);
        command_list_->SetGraphicsRoot32BitConstants(0,sizeof(constants)/sizeof(UINT),&constants,0);
        command_list_->IASetVertexBuffers(0,1,&vb);
        command_list_->IASetPrimitiveTopology(lines?D3D_PRIMITIVE_TOPOLOGY_LINELIST:D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        command_list_->DrawInstanced(static_cast<UINT>(vertices),1,0,0);
    }

    void finish_lunar_scene(){
        if(!lunar_active_) return;
        lunar_active_=false;
        set_coarse_shading(false);
        transition(lunar_target_.Get(),TextureUse::RenderTarget,TextureUse::PixelRead);

        const PostConstants constants{
            {1.0f/width_,1.0f/height_},std::exp2(graphics_.exposure_tenths*0.1f),graphics_.bloom*0.006f,
            graphics_.aa!=GraphicsAA::Off,graphics_.enabled(GraphicsFeature::Filmic),
            graphics_.enabled(GraphicsFeature::Vignette)
        };
        ID3D12DescriptorHeap* heaps[]{lunar_srv_heap_.Get()};
        command_list_->SetDescriptorHeaps(1,heaps);
        command_list_->SetGraphicsRootSignature(post_signature_.Get());
        command_list_->SetGraphicsRoot32BitConstants(0,sizeof(constants)/sizeof(UINT),&constants,0);
        const auto scene_srv=lunar_srv_heap_->GetGPUDescriptorHandleForHeapStart();
        command_list_->SetGraphicsRootDescriptorTable(1,scene_srv);
        command_list_->SetGraphicsRootDescriptorTable(2,scene_srv);
        command_list_->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        auto bloom_srv=scene_srv;
        if(constants.bloom>0.0f){
            const auto bloom_rtv=rtv_handle(kBloomRtv);
            command_list_->OMSetRenderTargets(1,&bloom_rtv,FALSE,nullptr);
            set_viewport(bloom_width(),bloom_height());
            command_list_->SetPipelineState(bloom_pso_.Get());
            command_list_->DrawInstanced(3,1,0,0);
            transition(bloom_target_.Get(),TextureUse::RenderTarget,TextureUse::PixelRead);
            bloom_srv.ptr+=srv_increment_;
            command_list_->SetGraphicsRootDescriptorTable(2,bloom_srv);
        }

        const auto back=rtv_handle(frame_index_);
        command_list_->OMSetRenderTargets(1,&back,FALSE,nullptr);
        set_viewport(width_,height_);
        command_list_->SetPipelineState(composite_pso_.Get());
        command_list_->DrawInstanced(3,1,0,0);

        // Both textures begin and end in RT layout. One set of targets can
        // serve all swap-chain frames on the same direct queue.
        transition(lunar_target_.Get(),TextureUse::PixelRead,TextureUse::RenderTarget);
        if(constants.bloom>0.0f)
            transition(bloom_target_.Get(),TextureUse::PixelRead,TextureUse::RenderTarget);
        const auto dsv=dsv_heap_->GetCPUDescriptorHandleForHeapStart();
        command_list_->OMSetRenderTargets(1,&back,FALSE,&dsv);
    }

public:
    ~D3D12Renderer() override {
        wait_for_gpu();
        for(auto& f:frames_) if(f.upload && f.mapped){ f.upload->Unmap(0,nullptr); f.mapped=nullptr; }
        if(fence_event_) CloseHandle(fence_event_);
    }

    bool initialize(void*,std::uintptr_t native_window,int width,int height) override {
        hwnd_=reinterpret_cast<HWND>(native_window);
        width_=std::max(1,width);
        height_=std::max(1,height);
        if(!hwnd_) return false;
        return create_factory_and_device() &&
               create_queue_and_swapchain() &&
               create_descriptor_heaps() &&
               create_back_buffers() &&
               create_frame_resources() &&
               create_root_signature_and_psos();
    }

    void resize(int width,int height) override {
        if(width<=0 || height<=0 || !swap_chain_) return;
        if(frame_open_){
            pending_width_=width;
            pending_height_=height;
            return;
        }
        if(width==width_ && height==height_ && depth_buffer_) return;
        wait_for_gpu();
        for(auto& target:render_targets_) target.Reset();
        depth_buffer_.Reset();
        const UINT flags=tearing_supported_?DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING:0;
        const HRESULT hr=swap_chain_->ResizeBuffers(kFrameCount,(UINT)width,(UINT)height,kBackBufferFormat,flags);
        if(SUCCEEDED(hr)){ width_=width; height_=height; }
        else log_hresult("ResizeBuffers",hr);
        frame_index_=swap_chain_->GetCurrentBackBufferIndex();
        if(create_back_buffers() && SUCCEEDED(hr) && lunar_srv_heap_) create_lunar_targets();
    }

    void begin_frame(Color clear) override {
        if(frame_open_ || !swap_chain_ || !depth_buffer_) return;
        frame_index_=swap_chain_->GetCurrentBackBufferIndex();
        if(!render_targets_[frame_index_]) return;
        auto& frame=frames_[frame_index_];
        wait_for_fence(frame.fence_value);
        frame.upload_offset=0;
        upload_overflow_reported_=false;
        HRESULT hr=frame.allocator->Reset();
        if(SUCCEEDED(hr)) hr=command_list_->Reset(frame.allocator.Get(),nullptr);
        if(FAILED(hr)){ log_hresult("Reset D3D12 frame",hr); return; }
        coarse_shading_=false;
        transition(render_targets_[frame_index_].Get(),TextureUse::Present,TextureUse::RenderTarget);

        const auto rtv=rtv_handle(frame_index_);
        const auto dsv=dsv_heap_->GetCPUDescriptorHandleForHeapStart();
        const float clear_color[4]{clear.r,clear.g,clear.b,clear.a};
        command_list_->OMSetRenderTargets(1,&rtv,FALSE,&dsv);
        command_list_->ClearRenderTargetView(rtv,clear_color,0,nullptr);
        command_list_->ClearDepthStencilView(dsv,D3D12_CLEAR_FLAG_DEPTH,1.0f,0,0,nullptr);

        set_viewport(width_,height_);
        depth_enabled_=true;
        frame_open_=true;
    }

    void end_frame() override {
        if(!frame_open_) return;
        finish_lunar_scene();
        transition(render_targets_[frame_index_].Get(),TextureUse::RenderTarget,TextureUse::Present);
        if(FAILED(command_list_->Close())){ frame_open_=false; return; }
        ID3D12CommandList* lists[]{command_list_.Get()};
        queue_->ExecuteCommandLists(1,lists);

        auto& frame=frames_[frame_index_];
        const UINT64 signal=next_fence_value_++;
        if(SUCCEEDED(queue_->Signal(fence_.Get(),signal))) frame.fence_value=signal;
        const UINT sync=vsync_?1u:0u;
        const UINT flags=(!vsync_&&tearing_supported_)?DXGI_PRESENT_ALLOW_TEARING:0u;
        const HRESULT present=swap_chain_->Present(sync,flags);
        if(FAILED(present) && present!=DXGI_STATUS_OCCLUDED){
            log_hresult("Present",present);
            const HRESULT removed=device_?device_->GetDeviceRemovedReason():S_OK;
            if(FAILED(removed)) log_hresult("D3D12 device removed",removed);
        }
        frame_open_=false;
        if(pending_width_){
            const int width=std::exchange(pending_width_,0);
            resize(width,pending_height_);
        }
    }

    void set_3d(const Mat4& view_proj) override {
        finish_lunar_scene();
        mvp_=view_proj;
        depth_enabled_=true;
    }
    void set_2d(float scale=1.0f) override {
        finish_lunar_scene();
        scale=std::max(0.01f,scale);
        mvp_=ortho(0,(float)width_/scale,(float)height_/scale,0,-1,1);
        depth_enabled_=false;
    }

    void set_graphics_settings(const GraphicsSettings& settings) override {
        graphics_=settings;
        graphics_.normalize();
    }
    GraphicsCapabilities graphics_capabilities() override {
        GraphicsCapabilities caps{};
        caps.enhanced=lunar_target_ && bloom_target_;
        caps.enhanced_barriers=enhanced_barriers_;
        caps.variable_rate_shading=variable_rate_shading_;
        return caps;
    }

    bool begin_lunar_scene(const LunarScene& scene) override {
        if(!frame_open_ || lunar_active_ || !lunar_target_ || !bloom_target_ ||
           graphics_.path==GraphicsPath::Classic) return false;

        SceneConstants constants{};
        const Vec3 sun=normalize(scene.sun_direction);
        constants.sun_daylight[0]=sun.x;
        constants.sun_daylight[1]=sun.y;
        constants.sun_daylight[2]=sun.z;
        constants.sun_daylight[3]=std::clamp(scene.daylight,0.0f,1.0f);
        constants.eye[0]=scene.eye.x;
        constants.eye[1]=scene.eye.y;
        constants.eye[2]=scene.eye.z;
        constants.pbr=graphics_.enabled(GraphicsFeature::Pbr);
        constants.material_detail=graphics_.enabled(GraphicsFeature::MaterialDetail);
        constants.terrain_relief=graphics_.enabled(GraphicsFeature::TerrainRelief);
        if(graphics_.enabled(GraphicsFeature::LocalLights)){
            const auto count=std::min(scene.light_count,scene.lights.size());
            for(std::size_t i=0;i<count;++i){
                const auto& light=scene.lights[i];
                if(light.radius<=0.0f || light.power<=0.0f) continue;
                constants.lights[constants.light_count++]={
                    {light.position.x,light.position.y,light.position.z,light.radius},
                    {light.color.r,light.color.g,light.color.b,light.power}
                };
            }
        }
        scene_constants_=upload(&constants,sizeof(constants),D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT);
        if(!scene_constants_) return false;

        const auto rtv=rtv_handle(kSceneRtv);
        const auto dsv=dsv_heap_->GetCPUDescriptorHandleForHeapStart();
        command_list_->OMSetRenderTargets(1,&rtv,FALSE,&dsv);
        command_list_->ClearRenderTargetView(rtv,kLunarClear.data(),0,nullptr);
        mvp_=scene.view_projection;
        depth_enabled_=true;
        lunar_active_=true;

        const float half=std::max(1.0f,scene.half_extent);
        const bool relief=constants.terrain_relief!=0;
        if(terrain_half_!=half || terrain_relief_!=relief){
            horizon_=lunar_horizon(half,relief);
            terrain_half_=half;
            terrain_relief_=relief;
        }
        constexpr Color regolith{0.53f,0.51f,0.49f,1.0f};
        const std::array<Vec3,6> floor{{{-half,0,-half},{half,0,-half},{half,0,half},
                                      {-half,0,-half},{half,0,half},{-half,0,half}}};
        draw(horizon_.data(),horizon_.size(),false,regolith,1.0f,LunarMaterial::Regolith,graphics_.terrain_vrs);
        draw(floor.data(),floor.size(),false,regolith,1.0f,LunarMaterial::Regolith);
        return true;
    }

    bool lunar_scene_active() const override { return lunar_active_; }
    void draw_lunar_surface(const std::vector<Vec3>& vertices,LunarMaterial material,Color tint) override {
        draw(vertices.data(),vertices.size(),false,tint,1.0f,material);
    }

    void draw_lines_3d(const std::vector<Vec3>& vertices,Color color,float width=1.0f) override {
        draw(vertices.data(),vertices.size(),true,color,width);
    }
    void draw_triangles_3d(const std::vector<Vec3>& vertices,Color color) override {
        draw(vertices.data(),vertices.size(),false,color,1.0f);
    }
    void draw_rect(Rect r,Color c) override {
        const float v[]={r.x,r.y,0,r.x+r.w,r.y,0,r.x+r.w,r.y+r.h,0,
                         r.x,r.y,0,r.x+r.w,r.y+r.h,0,r.x,r.y+r.h,0};
        draw(v,6,false,c,1.0f);
    }
    void draw_line(Vec2 a,Vec2 b,Color c,float width=1.0f) override {
        const auto v=ui::line_quad(a,b,width);
        draw(v.data(),v.size()/3u,false,c,1.0f);
    }
    void draw_text(Vec2 p,const std::string& text,float scale,Color c) override;

    int width() const override { return width_; }
    int height() const override { return height_; }
    bool hardware_accelerated() const override { return device_ && !(adapter_desc_.Flags&DXGI_ADAPTER_FLAG_SOFTWARE); }
    std::string gpu_vendor() const override { return vendor_name(adapter_desc_.VendorId); }
    std::string gpu_renderer() const override { return wide_to_utf8(adapter_desc_.Description); }
    std::string gpu_version() const override { return std::string("Direct3D 12 / Feature Level ")+feature_level_name(max_feature_level_); }
};



void D3D12Renderer::draw_text(Vec2 p,const std::string& text,float scale,Color c){
    std::vector<float> vertices;
    vertices.reserve(text.size()*7u*5u*18u/2u);
    const float origin_x=p.x;
    auto quad=[&](float x,float y){
        const float x1=x+scale,y1=y+scale;
        const float q[]={x,y,0,x1,y,0,x1,y1,0, x,y,0,x1,y1,0,x,y1,0};
        vertices.insert(vertices.end(),q,q+18);
    };
    for(char raw:text){
        char ch=(raw>='a'&&raw<='z')?(char)(raw-'a'+'A'):raw;
        if(ch=='\n'){ p.y+=8*scale; p.x=origin_x; continue; }
        auto it=ui::kBitmapFont.find(ch);
        if(it==ui::kBitmapFont.end()){ p.x+=6*scale; continue; }
        const auto& rows=it->second;
        for(int y=0;y<7;++y) for(int x=0;x<5;++x)
            if(rows[y]&(1<<(4-x))) quad(p.x+x*scale,p.y+y*scale);
        p.x+=6*scale;
    }
    draw(vertices.data(),vertices.size()/3u,false,c,1.0f);
}

std::unique_ptr<Renderer> create_d3d12_renderer(){ return std::make_unique<D3D12Renderer>(); }

} // namespace eng
