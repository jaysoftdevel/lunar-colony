#pragma once

namespace eng::lunar_shaders {

inline constexpr const char* volume_inputs=R"GLSL(#version 330 core
in vec2 uv;
out vec4 pixel;
uniform sampler2D depthMap;
uniform mat4 inverseViewProjection;
uniform vec3 eye;
uniform vec3 sun;
uniform float daylight;
uniform float time;
uniform int volumeCount;
uniform vec4 volumeOriginLength[8];
uniform vec4 volumeAxisRadius[8];
uniform vec4 volumeColorDensity[8];
uniform vec4 volumeParameters[8];
)GLSL";

inline constexpr const char* volume_fragment=R"GLSL(
float turbulence(vec3 p){
    return 0.64+0.21*sin(dot(p,vec3(4.2,7.9,5.6))+time*7.2)*
                      cos(dot(p,vec3(-3.3,5.1,9.3))-time*11.4)
               +0.15*cos(dot(p,vec3(14.2,-12.7,17.9))+time*14.6);
}
float jet_radius(float nozzle,float radius,float t){
    return mix(nozzle,radius,pow(t,0.72));
}
bool clip_jet(vec3 eyeFromNozzle,vec3 ray,vec3 axis,float reach,float radius,
              inout float first,inout float last){
    float h=dot(eyeFromNozzle,axis),rate=dot(ray,axis);
    if(abs(rate)>0.00001){
        float a=-h/rate,b=(reach-h)/rate;
        first=max(first,min(a,b));
        last=min(last,max(a,b));
    }else if(h<0.0 || h>reach) return false;
    vec3 transverse=eyeFromNozzle-axis*h,direction=ray-axis*rate;
    float a=dot(direction,direction),b=dot(transverse,direction);
    float c=dot(transverse,transverse)-radius*radius;
    if(a>0.00001){
        float d=b*b-a*c;
        if(d<=0.0) return false;
        float root=sqrt(d);
        first=max(first,(-b-root)/a);
        last=min(last,(-b+root)/a);
    }else if(c>0.0) return false;
    return last>first;
}
void main(){
    float depth=texture(depthMap,uv).r;
    vec3 ray=normalize(reconstruct(uv,0.99,inverseViewProjection)-eye);
    float surfaceDepth=depth<1.0?distance(reconstruct(uv,depth,inverseViewProjection),eye):6000.0;
    vec3 energy=vec3(0.0);
    float opticalDepth=0.0;
    for(int v=0;v<volumeCount;++v){
        vec3 origin=volumeOriginLength[v].xyz,axis=volumeAxisRadius[v].xyz;
        float lengthAlong=volumeOriginLength[v].w,radius=volumeAxisRadius[v].w;
        bool cone=volumeParameters[v].y>0.5;
        float nozzle=volumeParameters[v].z,profileLength=max(lengthAlong,volumeParameters[v].w);
        bool jet=cone && nozzle>0.0;
        if(jet) radius=max(radius,nozzle);
        vec3 center=origin+axis*lengthAlong*0.5;
        float boundRadius=jet?jet_radius(nozzle,radius,lengthAlong/profileLength):radius;
        float bound=length(vec2(lengthAlong*0.5,boundRadius));
        vec3 relative=eye-center;
        float b=dot(relative,ray);
        float discriminant=b*b-dot(relative,relative)+bound*bound;
        if(discriminant<=0.0) continue;
        float root=sqrt(discriminant);
        float first=max(0.0,-b-root),last=min(surfaceDepth-0.008,-b+root);
        if(last<=first) continue;
        if(jet && !clip_jet(eye-origin,ray,axis,lengthAlong,boundRadius,first,last)) continue;
        int steps=taps(cone?OPT_volumetric_plumes:OPT_volumetric_dust,5,10,15);
        float stepLength=(last-first)/float(steps);
        float transmittance=1.0;
        vec3 scattered=vec3(0.0);
        for(int s=0;s<steps;++s){
            vec3 p=eye+ray*(first+(float(s)+0.5)*stepLength)-origin;
            float axial=dot(p,axis);
            if(axial<0.0 || axial>lengthAlong) continue;
            float t=axial/max(profileLength,0.001);
            float roundness=2.0*t-1.0;
            float r=jet?jet_radius(nozzle,radius,t):cone?radius*mix(0.22,1.0,pow(t,0.72)):
                radius*sqrt(max(0.02,1.0-roundness*roundness));
            float radial=length(p-axial*axis)/r;
            if(radial>=1.0) continue;
            float shape=(1.0-smoothstep(0.26,1.0,radial))*
                        (1.0-smoothstep(0.66,1.0,t));
            if(jet && lengthAlong<profileLength)
                shape*=smoothstep(0.0,min(0.38,lengthAlong),lengthAlong-axial);
            vec3 densityPoint=jet?p-axis*time*2.2+origin*0.09:p;
            float expansion=jet?r/nozzle:1.0;
            float dilution=1.0+0.36*(expansion*expansion-1.0);
            float variation=turbulence(densityPoint);
            if(jet) variation=mix(0.85,variation,0.44);
            float density=shape*variation*volumeColorDensity[v].w/dilution;
            float alpha=1.0-exp(-max(density,0.0)*stepLength);
            float phase=0.25+0.50*daylight*(0.5+0.5*dot(ray,sun));
            float emission=volumeParameters[v].x*(1.0-0.77*t);
            vec3 tint=volumeColorDensity[v].rgb;
            if(jet){
                // Advection down the axis and dilution as the gas expands in vacuum.
                tint=mix(vec3(0.87,0.96,1.0),tint,smoothstep(0.0,0.48,t));
                emission*=exp(-1.9*t);
            }
            scattered+=transmittance*alpha*tint*(phase+emission);
            transmittance*=1.0-alpha;
            if(transmittance<0.025) break;
        }
        float absorbed=1.0-transmittance;
        if(absorbed>0.0001){
            // World clouds use a commutative optical-depth accumulation.
            // Per-plume integration remains front to back and depth clipped.
            float tau=-log(max(transmittance,0.0001));
            energy+=scattered*(tau/max(absorbed,0.0001));
            opticalDepth+=tau;
        }
    }
    float coverage=1.0-exp(-opticalDepth);
    pixel=vec4(energy*(coverage/max(opticalDepth,0.0001)),coverage);
}
)GLSL";

} // namespace eng::lunar_shaders
