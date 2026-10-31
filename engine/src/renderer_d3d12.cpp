#include "engine/engine.hpp"

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
    float padding{0.0f};
};
static_assert(sizeof(DrawConstants)==24*sizeof(float));

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

constexpr const char* kShaderSource=R"HLSL(
cbuffer DrawConstants : register(b0)
{
    column_major float4x4 gMVP;
    float4 gColor;
    float2 gViewport;
    float gLineWidth;
    float gPadding;
};

struct VSOut { float4 position : SV_POSITION; };

VSOut VSMain(float3 position : POSITION)
{
    VSOut o;
    o.position = mul(gMVP, float4(position, 1.0));
    // The original game matrices use OpenGL's -1..+1 NDC Z range. Convert it
    // here to Direct3D's 0..+1 range without changing any gameplay/camera math.
    o.position.z = o.position.z * 0.5 + o.position.w * 0.5;
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
)HLSL";

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

} // namespace

class D3D12Renderer final : public Renderer {
    HWND hwnd_{};
    int width_{};
    int height_{};
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

    ComPtr<ID3D12RootSignature> root_signature_{};
    ComPtr<ID3D12PipelineState> triangles_depth_{};
    ComPtr<ID3D12PipelineState> triangles_no_depth_{};
    ComPtr<ID3D12PipelineState> lines_depth_{};
    ComPtr<ID3D12PipelineState> lines_no_depth_{};

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
        rtv.NumDescriptors=kFrameCount;
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
        D3D12_RESOURCE_DESC d{};
        d.Dimension=D3D12_RESOURCE_DIMENSION_TEXTURE2D;
        d.Width=(UINT64)std::max(1,width_);
        d.Height=(UINT)std::max(1,height_);
        d.DepthOrArraySize=1;
        d.MipLevels=1;
        d.Format=kDepthFormat;
        d.SampleDesc.Count=1;
        d.Layout=D3D12_TEXTURE_LAYOUT_UNKNOWN;
        d.Flags=D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL;
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
        command_list_->Close();

        hr=device_->CreateFence(0,D3D12_FENCE_FLAG_NONE,IID_PPV_ARGS(&fence_));
        if(FAILED(hr)) return false;
        fence_event_=CreateEventW(nullptr,FALSE,FALSE,nullptr);
        return fence_event_!=nullptr;
    }

    bool create_root_signature_and_psos(){
        D3D12_ROOT_PARAMETER root{};
        root.ParameterType=D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS;
        root.Constants.ShaderRegister=0;
        root.Constants.RegisterSpace=0;
        root.Constants.Num32BitValues=24;
        root.ShaderVisibility=D3D12_SHADER_VISIBILITY_ALL;
        D3D12_ROOT_SIGNATURE_DESC rs{};
        rs.NumParameters=1;
        rs.pParameters=&root;
        rs.Flags=D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;
        ComPtr<ID3DBlob> signature,error;
        HRESULT hr=D3D12SerializeRootSignature(&rs,D3D_ROOT_SIGNATURE_VERSION_1,&signature,&error);
        if(FAILED(hr)){
            if(error) std::cerr<<static_cast<const char*>(error->GetBufferPointer())<<"\n";
            return false;
        }
        hr=device_->CreateRootSignature(0,signature->GetBufferPointer(),signature->GetBufferSize(),IID_PPV_ARGS(&root_signature_));
        if(FAILED(hr)) return false;

        auto vs=compile_shader(kShaderSource,"VSMain","vs_5_1");
        auto gs=compile_shader(kShaderSource,"GSLine","gs_5_1");
        auto ps=compile_shader(kShaderSource,"PSMain","ps_5_1");
        if(!vs||!gs||!ps) return false;

        D3D12_INPUT_ELEMENT_DESC input{};
        input.SemanticName="POSITION";
        input.SemanticIndex=0;
        input.Format=DXGI_FORMAT_R32G32B32_FLOAT;
        input.InputSlot=0;
        input.AlignedByteOffset=0;
        input.InputSlotClass=D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA;
        input.InstanceDataStepRate=0;

        auto create_pso=[&](bool lines,bool depth,ComPtr<ID3D12PipelineState>& out)->bool{
            D3D12_GRAPHICS_PIPELINE_STATE_DESC p{};
            p.pRootSignature=root_signature_.Get();
            p.VS={vs->GetBufferPointer(),vs->GetBufferSize()};
            p.PS={ps->GetBufferPointer(),ps->GetBufferSize()};
            if(lines) p.GS={gs->GetBufferPointer(),gs->GetBufferSize()};
            p.BlendState=blend_desc();
            p.SampleMask=UINT_MAX;
            p.RasterizerState=raster_desc();
            p.DepthStencilState=depth_desc(depth);
            p.InputLayout={&input,1};
            p.PrimitiveTopologyType=lines?D3D12_PRIMITIVE_TOPOLOGY_TYPE_LINE:D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
            p.NumRenderTargets=1;
            p.RTVFormats[0]=kBackBufferFormat;
            p.DSVFormat=kDepthFormat;
            p.SampleDesc.Count=1;
            const HRESULT result=device_->CreateGraphicsPipelineState(&p,IID_PPV_ARGS(&out));
            if(FAILED(result)) log_hresult("CreateGraphicsPipelineState",result);
            return SUCCEEDED(result);
        };
        return create_pso(false,true,triangles_depth_) &&
               create_pso(false,false,triangles_no_depth_) &&
               create_pso(true,true,lines_depth_) &&
               create_pso(true,false,lines_no_depth_);
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

    D3D12_CPU_DESCRIPTOR_HANDLE current_rtv() const {
        auto h=rtv_heap_->GetCPUDescriptorHandleForHeapStart();
        h.ptr += (SIZE_T)frame_index_*rtv_increment_;
        return h;
    }

    bool upload_vertices(const float* data,std::size_t floats,D3D12_VERTEX_BUFFER_VIEW& vb){
        if(!frame_open_ || !data || floats==0) return false;
        auto& frame=frames_[frame_index_];
        const std::size_t bytes=floats*sizeof(float);
        const std::size_t aligned=(frame.upload_offset+15u)&~std::size_t(15u);
        if(aligned+bytes>kUploadBytesPerFrame){
            if(!upload_overflow_reported_){
                upload_overflow_reported_=true;
                std::cerr<<"D3D12 per-frame upload budget exceeded (64 MiB). Further draws in this frame are skipped.\n";
            }
            return false;
        }
        std::memcpy(frame.mapped+aligned,data,bytes);
        vb.BufferLocation=frame.upload->GetGPUVirtualAddress()+aligned;
        vb.SizeInBytes=(UINT)bytes;
        vb.StrideInBytes=3u*sizeof(float);
        frame.upload_offset=aligned+bytes;
        return true;
    }

    void draw(const float* data,std::size_t floats,bool lines,Color c,float width){
        if(!frame_open_ || !data || floats==0) return;
        D3D12_VERTEX_BUFFER_VIEW vb{};
        if(!upload_vertices(data,floats,vb)) return;

        ID3D12PipelineState* pso=nullptr;
        if(lines) pso=depth_enabled_?lines_depth_.Get():lines_no_depth_.Get();
        else pso=depth_enabled_?triangles_depth_.Get():triangles_no_depth_.Get();
        command_list_->SetPipelineState(pso);
        command_list_->SetGraphicsRootSignature(root_signature_.Get());

        DrawConstants constants{};
        std::memcpy(constants.mvp,mvp_.m.data(),sizeof(constants.mvp));
        constants.color[0]=c.r; constants.color[1]=c.g; constants.color[2]=c.b; constants.color[3]=c.a;
        constants.viewport[0]=(float)std::max(1,width_);
        constants.viewport[1]=(float)std::max(1,height_);
        constants.line_width=std::max(1.0f,width);
        command_list_->SetGraphicsRoot32BitConstants(0,24,&constants,0);
        command_list_->IASetVertexBuffers(0,1,&vb);
        command_list_->IASetPrimitiveTopology(lines?D3D_PRIMITIVE_TOPOLOGY_LINELIST:D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        command_list_->DrawInstanced((UINT)(floats/3u),1,0,0);
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
        if(width==width_ && height==height_) return;
        wait_for_gpu();
        for(auto& target:render_targets_) target.Reset();
        depth_buffer_.Reset();
        width_=width; height_=height;
        DXGI_SWAP_CHAIN_DESC desc{};
        swap_chain_->GetDesc(&desc);
        const HRESULT hr=swap_chain_->ResizeBuffers(kFrameCount,(UINT)width_,(UINT)height_,kBackBufferFormat,desc.Flags);
        if(FAILED(hr)){ log_hresult("ResizeBuffers",hr); return; }
        frame_index_=swap_chain_->GetCurrentBackBufferIndex();
        create_back_buffers();
    }

    void begin_frame(Color clear) override {
        if(frame_open_ || !swap_chain_) return;
        frame_index_=swap_chain_->GetCurrentBackBufferIndex();
        auto& frame=frames_[frame_index_];
        wait_for_fence(frame.fence_value);
        frame.upload_offset=0;
        upload_overflow_reported_=false;
        frame.allocator->Reset();
        command_list_->Reset(frame.allocator.Get(),nullptr);

        D3D12_RESOURCE_BARRIER barrier{};
        barrier.Type=D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barrier.Transition.pResource=render_targets_[frame_index_].Get();
        barrier.Transition.Subresource=D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        barrier.Transition.StateBefore=D3D12_RESOURCE_STATE_PRESENT;
        barrier.Transition.StateAfter=D3D12_RESOURCE_STATE_RENDER_TARGET;
        command_list_->ResourceBarrier(1,&barrier);

        const auto rtv=current_rtv();
        const auto dsv=dsv_heap_->GetCPUDescriptorHandleForHeapStart();
        const float clear_color[4]{clear.r,clear.g,clear.b,clear.a};
        command_list_->OMSetRenderTargets(1,&rtv,FALSE,&dsv);
        command_list_->ClearRenderTargetView(rtv,clear_color,0,nullptr);
        command_list_->ClearDepthStencilView(dsv,D3D12_CLEAR_FLAG_DEPTH,1.0f,0,0,nullptr);

        D3D12_VIEWPORT viewport{0.0f,0.0f,(float)width_,(float)height_,0.0f,1.0f};
        D3D12_RECT scissor{0,0,width_,height_};
        command_list_->RSSetViewports(1,&viewport);
        command_list_->RSSetScissorRects(1,&scissor);
        depth_enabled_=true;
        frame_open_=true;
    }

    void end_frame() override {
        if(!frame_open_) return;
        D3D12_RESOURCE_BARRIER barrier{};
        barrier.Type=D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
        barrier.Transition.pResource=render_targets_[frame_index_].Get();
        barrier.Transition.Subresource=D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
        barrier.Transition.StateBefore=D3D12_RESOURCE_STATE_RENDER_TARGET;
        barrier.Transition.StateAfter=D3D12_RESOURCE_STATE_PRESENT;
        command_list_->ResourceBarrier(1,&barrier);
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
    }

    void set_3d(const Mat4& view_proj) override { mvp_=view_proj; depth_enabled_=true; }
    void set_2d(float scale=1.0f) override {
        scale=std::max(0.25f,scale);
        mvp_=ortho(0,(float)width_/scale,(float)height_/scale,0,-1,1);
        depth_enabled_=false;
    }

    void draw_lines_3d(const std::vector<Vec3>& vertices,Color color,float width=1.0f) override {
        if(vertices.empty()) return;
        draw(reinterpret_cast<const float*>(vertices.data()),vertices.size()*3u,true,color,width);
    }
    void draw_triangles_3d(const std::vector<Vec3>& vertices,Color color) override {
        if(vertices.empty()) return;
        draw(reinterpret_cast<const float*>(vertices.data()),vertices.size()*3u,false,color,1.0f);
    }
    void draw_rect(Rect r,Color c) override {
        const float v[]={r.x,r.y,0,r.x+r.w,r.y,0,r.x+r.w,r.y+r.h,0,
                         r.x,r.y,0,r.x+r.w,r.y+r.h,0,r.x,r.y+r.h,0};
        draw(v,18,false,c,1.0f);
    }
    void draw_line(Vec2 a,Vec2 b,Color c,float width=1.0f) override {
        const float v[]={a.x,a.y,0,b.x,b.y,0};
        draw(v,6,true,c,width);
    }
    void draw_text(Vec2 p,const std::string& text,float scale,Color c) override;

    int width() const override { return width_; }
    int height() const override { return height_; }
    bool hardware_accelerated() const override { return device_ && !(adapter_desc_.Flags&DXGI_ADAPTER_FLAG_SOFTWARE); }
    std::string gpu_vendor() const override { return vendor_name(adapter_desc_.VendorId); }
    std::string gpu_renderer() const override { return wide_to_utf8(adapter_desc_.Description); }
    std::string gpu_version() const override { return std::string("Direct3D 12 / Feature Level ")+feature_level_name(max_feature_level_); }
};

static const std::unordered_map<char,std::array<unsigned char,7>> FONT={
{'A',{14,17,17,31,17,17,17}},{'B',{30,17,17,30,17,17,30}},{'C',{14,17,16,16,16,17,14}},{'D',{30,17,17,17,17,17,30}},{'E',{31,16,16,30,16,16,31}},{'F',{31,16,16,30,16,16,16}},{'G',{14,17,16,23,17,17,14}},{'H',{17,17,17,31,17,17,17}},{'I',{31,4,4,4,4,4,31}},{'J',{7,2,2,2,18,18,12}},{'K',{17,18,20,24,20,18,17}},{'L',{16,16,16,16,16,16,31}},{'M',{17,27,21,21,17,17,17}},{'N',{17,25,21,19,17,17,17}},{'O',{14,17,17,17,17,17,14}},{'P',{30,17,17,30,16,16,16}},{'Q',{14,17,17,17,21,18,13}},{'R',{30,17,17,30,20,18,17}},{'S',{15,16,16,14,1,1,30}},{'T',{31,4,4,4,4,4,4}},{'U',{17,17,17,17,17,17,14}},{'V',{17,17,17,17,17,10,4}},{'W',{17,17,17,21,21,21,10}},{'X',{17,17,10,4,10,17,17}},{'Y',{17,17,10,4,4,4,4}},{'Z',{31,1,2,4,8,16,31}},
{'0',{14,17,19,21,25,17,14}},{'1',{4,12,4,4,4,4,14}},{'2',{14,17,1,2,4,8,31}},{'3',{30,1,1,14,1,1,30}},{'4',{2,6,10,18,31,2,2}},{'5',{31,16,16,30,1,1,30}},{'6',{14,16,16,30,17,17,14}},{'7',{31,1,2,4,8,8,8}},{'8',{14,17,17,14,17,17,14}},{'9',{14,17,17,15,1,1,14}},
{'%',{17,2,4,4,8,16,17}},{'+',{0,4,4,31,4,4,0}},{'-',{0,0,0,31,0,0,0}},{'.',{0,0,0,0,0,12,12}},{':',{0,12,12,0,12,12,0}},{'/',{1,2,4,8,16,0,0}},{'x',{0,0,17,10,4,10,17}},{' ',{0,0,0,0,0,0,0}},{'<',{2,4,8,16,8,4,2}},{'>',{8,4,2,1,2,4,8}},{'=',{0,0,31,0,31,0,0}},{'(',{2,4,8,8,8,4,2}},{')',{8,4,2,2,2,4,8}}
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
        auto it=FONT.find(ch);
        if(it==FONT.end()){ p.x+=6*scale; continue; }
        const auto& rows=it->second;
        for(int y=0;y<7;++y) for(int x=0;x<5;++x)
            if(rows[y]&(1<<(4-x))) quad(p.x+x*scale,p.y+y*scale);
        p.x+=6*scale;
    }
    if(!vertices.empty()) draw(vertices.data(),vertices.size(),false,c,1.0f);
}

std::unique_ptr<Renderer> create_d3d12_renderer(){ return std::make_unique<D3D12Renderer>(); }

} // namespace eng
