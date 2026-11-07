#pragma once

namespace eng::d3d12_shaders {

inline constexpr char draw_constants[]=R"HLSL(
cbuffer DrawConstants : register(b0)
{
    column_major float4x4 gMVP;
    float4 gColor;
    float2 gViewport;
    float gLineWidth;
    float gMaterial;
};

float4 clip_position(float3 position)
{
    float4 clip = mul(gMVP, float4(position, 1.0));
    // The game uses OpenGL projection matrices.
    clip.z = 0.5 * (clip.z + clip.w);
    return clip;
}

float3 linear_color(float3 color)
{
    return pow(max(color, 0.0), 2.2);
}
)HLSL";

inline constexpr char lunar[]=R"HLSL(
struct Fixture
{
    float4 positionRadius;
    float4 colorPower;
};

cbuffer SceneConstants : register(b1)
{
    float4 gSunDaylight;
    float4 gEye;
    uint gLightCount;
    uint gPbr;
    uint gMaterialDetail;
    uint gTerrainRelief;
    Fixture gLights[64];
};

// Values are shared with the LunarMaterial enum.
static const int Paint = 0, Alloy = 1, Regolith = 2, Solar = 3, Glass = 4;
static const int Fabric = 5, Pad = 6, Emissive = 7, Skin = 8, Marking = 9, Hair = 10;
static const float Pi = 3.14159265;

struct SurfaceVertex
{
    float4 position : SV_POSITION;
    float3 world : TEXCOORD0;
};

SurfaceVertex VSLunar(float3 position : POSITION)
{
    SurfaceVertex o;
    o.position = clip_position(position);
    o.world = position;
    return o;
}

float3 safe_normalize(float3 v)
{
    return v * rsqrt(max(dot(v, v), 1e-10));
}

float hash_cell(float2 cell)
{
    float3 p = frac(cell.xyx * 0.1031);
    p += dot(p, p.yzx + 33.33);
    return frac((p.x + p.y) * p.z);
}

float value_noise(float2 p)
{
    float2 cell = floor(p);
    float2 f = frac(p);
    float2 w = f * f * (3.0 - 2.0 * f);
    return lerp(lerp(hash_cell(cell), hash_cell(cell + float2(1, 0)), w.x),
                lerp(hash_cell(cell + float2(0, 1)), hash_cell(cell + 1.0), w.x), w.y);
}

void crater(float2 p, float spacing, out float2 slope, out float ejecta)
{
    float2 cell = floor(p / spacing);
    float2 center = 0.5 + 0.32 * (float2(hash_cell(cell), hash_cell(cell + 47.0)) - 0.5);
    float radius = lerp(0.12, 0.23, hash_cell(cell + 8.0));
    float2 delta = frac(p / spacing) - center;
    float r = length(delta) / radius;
    float wall = smoothstep(0.22, 0.92, r) * (1.0 - smoothstep(0.95, 1.16, r));
    float rim = smoothstep(0.85, 1.02, r) * (1.0 - smoothstep(1.03, 1.34, r));
    slope = delta * ((0.55 * wall - 0.35 * rim) / max(length(delta), 1e-4));
    ejecta = 1.0 + 0.18 * rim - 0.17 * (1.0 - smoothstep(0.55, 0.91, r));
}

void shade_regolith(float3 world, inout float3 n, inout float3 albedo, inout float roughness)
{
    if (gTerrainRelief != 0u) {
        float2 largeSlope, smallSlope;
        float largeEjecta, smallEjecta;
        crater(world.xz, 60.0, largeSlope, largeEjecta);
        crater(world.xz, 17.0, smallSlope, smallEjecta);
        float2 slope = largeSlope + 0.55 * smallSlope;
        n = safe_normalize(n - float3(slope.x, 0.0, slope.y));
        albedo *= largeEjecta * smallEjecta;
    }
    if (gMaterialDetail != 0u) {
        float patch = value_noise(world.xz * 0.10);
        float grain = value_noise(world.xz * 5.6);
        float pixelSize = max(length(ddx(world.xz)), length(ddy(world.xz)));
        grain = lerp(grain, 0.5, smoothstep(0.07, 0.28, pixelSize));
        albedo *= 0.72 + 0.43 * patch + 0.16 * grain;
        roughness = 0.84 + 0.10 * patch;
    }
}

float3 direct_light(float3 albedo, float roughness, float metalness, float3 n, float3 v, float3 l)
{
    float nl = saturate(dot(n, l));
    if (gPbr == 0u) return albedo * nl;

    float3 h = safe_normalize(l + v);
    float nh = saturate(dot(n, h));
    float nv = max(saturate(dot(n, v)), 1e-3);
    float vh = saturate(dot(v, h));
    float alpha = roughness * roughness;
    float a2 = alpha * alpha;
    float d = nh * nh * (a2 - 1.0) + 1.0;
    float distribution = a2 / max(Pi * d * d, 1e-5);
    float k = (roughness + 1.0) * (roughness + 1.0) * 0.125;
    float visibility = nv / (nv * (1.0 - k) + k) * nl / (nl * (1.0 - k) + k);
    float3 f0 = lerp(float3(0.04, 0.04, 0.04), albedo, metalness);
    float3 fresnel = f0 + (1.0 - f0) * pow(1.0 - vh, 5.0);
    float3 diffuse = (1.0 - fresnel) * (1.0 - metalness) * albedo / Pi;
    return (diffuse + distribution * visibility * fresnel / max(4.0 * nl * nv, 1e-4)) * nl;
}

float4 PSLunar(SurfaceVertex input) : SV_TARGET
{
    float3 albedo = linear_color(gColor.rgb);
    int kind = int(gMaterial);
    if (kind == Emissive) return float4(albedo * 6.0, gColor.a);

    float3 world = input.world;
    float3 v = safe_normalize(gEye.xyz - world);
    float3 n = safe_normalize(cross(ddx(world), ddy(world)));
    if (kind == Regolith ? n.y < 0.0 : dot(n, v) < 0.0) n = -n;

    float roughness = 0.62, metalness = 0.02;
    if (kind == Alloy) { roughness = 0.33; metalness = 0.78; }
    if (kind == Solar) { roughness = 0.28; metalness = 0.48; }
    if (kind == Glass) { roughness = 0.17; metalness = 0.04; }
    if (kind == Fabric || kind == Pad) roughness = 0.86;
    if (kind == Skin || kind == Hair) roughness = 0.48;
    if (kind == Regolith) {
        roughness = 0.9;
        shade_regolith(world, n, albedo, roughness);
    } else if (gMaterialDetail != 0u && kind != Glass && kind != Marking && kind != Skin) {
        float2 uv = abs(n.y) > 0.6 ? world.xz : abs(n.x) > 0.6 ? world.zy : world.xy;
        float grain = value_noise(uv * (kind == Fabric ? 19.0 : 8.0));
        float footprint = max(length(ddx(world)), length(ddy(world)));
        float detail = 1.0 - smoothstep(0.012, 0.16, footprint);
        albedo *= lerp(1.0, 0.88 + 0.23 * grain, detail);
        if (kind == Solar && abs(n.y) > 0.28) {
            float2 cell = abs(frac(uv * 4.0) - 0.5);
            float seam = smoothstep(0.42, 0.5, max(cell.x, cell.y));
            albedo = lerp(albedo, albedo * 0.32, detail * seam);
        }
        if (kind == Paint || kind == Pad) {
            float dust = value_noise(world.xz * 0.88) * (1.0 - smoothstep(0.1, 1.8, world.y));
            albedo = lerp(albedo, float3(0.21, 0.18, 0.16), 0.32 * dust);
            roughness = lerp(roughness, 0.88, dust);
        }
    }

    float day = gSunDaylight.w;
    float3 ambient = lerp(float3(0.07, 0.10, 0.15), float3(0.25, 0.26, 0.28), day);
    float3 color = albedo * ambient * (0.70 + 0.30 * saturate(n.y));
    float sunStrength = day * (gPbr != 0u ? 3.8 : 1.2);
    color += direct_light(albedo, roughness, metalness, n, v, gSunDaylight.xyz) *
             sunStrength * float3(1.0, 0.96, 0.89);
    if (gPbr != 0u && kind == Glass) {
        float rim = pow(1.0 - saturate(dot(n, v)), 5.0);
        color += lerp(0.075, 0.21, day) * rim * float3(0.40, 0.62, 0.85);
    }

    [loop]
    for (uint i = 0u; i < gLightCount; ++i) {
        Fixture light = gLights[i];
        float3 delta = light.positionRadius.xyz - world;
        float distanceSq = dot(delta, delta);
        float radius = light.positionRadius.w;
        if (distanceSq < radius * radius) {
            float fade = saturate(1.0 - distanceSq / (radius * radius));
            float intensity = light.colorPower.w * fade * fade / max(1.0, distanceSq);
            color += direct_light(albedo, roughness, metalness, n, v, safe_normalize(delta)) *
                     light.colorPower.rgb * intensity * (gPbr != 0u ? 0.8 : 0.25);
        }
    }
    return float4(color, gColor.a);
}
)HLSL";

inline constexpr char post[]=R"HLSL(
Texture2D<float4> gScene : register(t0);
Texture2D<float4> gBloom : register(t1);
SamplerState gLinearClamp : register(s0);

cbuffer PostConstants : register(b0)
{
    float2 gPixelSize;
    float gExposure;
    float gBloomStrength;
    uint gFxaa;
    uint gFilmic;
    uint gVignette;
    uint gUnused;
};

struct FullscreenVertex
{
    float4 position : SV_POSITION;
    float2 uv : TEXCOORD0;
};

FullscreenVertex VSFullscreen(uint id : SV_VertexID)
{
    FullscreenVertex o;
    o.uv = float2((id << 1) & 2, id & 2);
    o.position = float4(o.uv * float2(2.0, -2.0) + float2(-1.0, 1.0), 0.0, 1.0);
    return o;
}

float3 bright_sample(float2 uv)
{
    float3 color = gScene.SampleLevel(gLinearClamp, uv, 0).rgb * gExposure;
    float peak = max(color.r, max(color.g, color.b));
    float knee = saturate((peak - 0.8) / 0.8);
    float contribution = max(peak - 1.2, 0.4 * knee * knee);
    return color * contribution / max(peak, 1e-4);
}

float4 PSBloom(FullscreenVertex input) : SV_TARGET
{
    float2 uv = input.uv;
    float2 nearOffset = gPixelSize * 1.5;
    float2 farOffset = gPixelSize * 6.0;
    float3 blur = bright_sample(uv) * 4.0;
    blur += 2.0 * (bright_sample(uv + nearOffset) + bright_sample(uv - nearOffset) +
                   bright_sample(uv + float2(nearOffset.x, -nearOffset.y)) +
                   bright_sample(uv + float2(-nearOffset.x, nearOffset.y)));
    blur += bright_sample(uv + float2(farOffset.x, 0)) + bright_sample(uv - float2(farOffset.x, 0)) +
            bright_sample(uv + float2(0, farOffset.y)) + bright_sample(uv - float2(0, farOffset.y));
    return float4(blur * (1.0 / 16.0), 1.0);
}

float3 tone_map(float3 color)
{
    color = max(color, 0.0);
    if (gFilmic != 0u)
        return saturate(color * (2.51 * color + 0.03) / (color * (2.43 * color + 0.59) + 0.14));
    return color / (1.0 + color);
}

float luma(float3 color)
{
    // Approximate perceptual luminance before testing for an edge.
    return sqrt(saturate(dot(tone_map(color * gExposure), float3(0.2126, 0.7152, 0.0722))));
}

float3 scene_at(float2 uv)
{
    return gScene.SampleLevel(gLinearClamp, uv, 0).rgb;
}

float3 fxaa(float2 uv, float3 center)
{
    float2 pixel = gPixelSize;
    float m = luma(center);
    float3 north = scene_at(uv - float2(0, pixel.y));
    float3 south = scene_at(uv + float2(0, pixel.y));
    float3 west = scene_at(uv - float2(pixel.x, 0));
    float3 east = scene_at(uv + float2(pixel.x, 0));
    float n = luma(north);
    float s = luma(south);
    float w = luma(west);
    float e = luma(east);
    float darkest = min(m, min(min(n, s), min(w, e)));
    float brightest = max(m, max(max(n, s), max(w, e)));
    float contrast = brightest - darkest;
    if (contrast < max(0.035, brightest * 0.13)) return center;

    float nw = luma(scene_at(uv - pixel));
    float se = luma(scene_at(uv + pixel));
    float ne = luma(scene_at(uv + float2(pixel.x, -pixel.y)));
    float sw = luma(scene_at(uv + float2(-pixel.x, pixel.y)));
    float2 tangent = float2((s - n) * 2.0 + sw + se - nw - ne,
                            (w - e) * 2.0 + nw + sw - ne - se);
    tangent /= max(max(abs(tangent.x), abs(tangent.y)), 1e-3);
    float3 aligned = 0.5 * (scene_at(uv - 0.55 * tangent * pixel) +
                            scene_at(uv + 0.55 * tangent * pixel));
    float subpixel = abs(0.25 * (n + s + w + e) - m) / contrast;
    float3 adjacent = (north + south + west + east) * 0.25;
    return lerp(aligned, adjacent, 0.45 * saturate(subpixel * subpixel));
}

float4 PSComposite(FullscreenVertex input) : SV_TARGET
{
    float2 uv = input.uv;
    float3 color = scene_at(uv);
    if (gFxaa != 0u) color = fxaa(uv, color);
    color *= gExposure;
    if (gBloomStrength > 0.0)
        color += gBloom.SampleLevel(gLinearClamp, uv, 0).rgb * gBloomStrength;
    color = tone_map(color);
    if (gVignette != 0u) {
        float2 edge = uv * 2.0 - 1.0;
        color *= 1.0 - 0.18 * smoothstep(0.2, 1.7, dot(edge, edge));
    }
    return float4(pow(color, 1.0 / 2.2), 1.0);
}
)HLSL";

} // namespace eng::d3d12_shaders
