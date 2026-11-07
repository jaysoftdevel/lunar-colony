#pragma once

namespace eng::lunar_shaders {

inline constexpr const char* neural_cache=R"GLSL(
layout(std430,binding=7) buffer NeuralRadianceCache { vec4 neuralWeights[38]; };
uniform vec3 neuralFocus;
uniform bool useNeural;
const int hiddenSize=12;

vec4 positionFeatures(vec3 position){
    return vec4(clamp((position-neuralFocus)*vec3(0.019,0.041,0.019),-1.3,1.3),1.0);
}

vec3 predictRadiance(vec4 p,vec3 n){
    vec3 prediction=neuralWeights[36].rgb;
    for(int i=0;i<hiddenSize;++i){
        float activation=tanh(dot(neuralWeights[i],p)+dot(neuralWeights[12+i].xyz,n));
        prediction+=activation*neuralWeights[24+i].rgb;
    }
    return prediction;
}

vec3 neuralPrior(vec3 position,vec3 normal,vec3 estimate){
    if(!useNeural || neuralWeights[37].x<360.0 || distance(neuralFocus,position)>65.0) return estimate;
    vec3 prediction=clamp(predictRadiance(positionFeatures(position),normal),0.0,0.77);
    float confidence=0.20*smoothstep(360.0,1600.0,neuralWeights[37].x);
    return mix(estimate,clamp(prediction,estimate-0.12,estimate+0.12),confidence);
}
)GLSL";

inline constexpr const char* neural_train_inputs=R"GLSL(#version 430 core
layout(local_size_x=1) in;
uniform sampler2D rayIndirect;
uniform sampler2D depthMap;
uniform sampler2D normalMap;
uniform mat4 inverseViewProjection;
uniform int frameIndex;
)GLSL";

inline constexpr const char* neural_train=R"GLSL(
uint sampleHash(inout uint seed){
    seed=seed*747796405u+2891336453u;
    uint word=((seed>>((seed>>28u)+4u))^seed)*277803737u;
    return (word>>22u)^word;
}

void main(){
    if(!useNeural) return;
    ivec2 size=textureSize(rayIndirect,0);
    uint seed=uint(frameIndex)*143041u+527u;
    for(int step=0;step<76;++step){
        ivec2 pixel=ivec2(sampleHash(seed)%uint(size.x),sampleHash(seed)%uint(size.y));
        vec2 uv=(vec2(pixel)+0.5)/vec2(size);
        float depth=texture(depthMap,uv).r;
        if(depth>=0.99997) continue;
        vec3 p=reconstruct(uv,depth,inverseViewProjection);
        if(distance(p,neuralFocus)>65.0) continue;
        vec3 n=normalize(texture(normalMap,uv).rgb*2.0-1.0);
        vec3 target=clamp(texelFetch(rayIndirect,pixel,0).rgb,0.0,0.77);
        if(max(target.r,max(target.g,target.b))<0.001) continue;
        vec4 position=positionFeatures(p);
        float hidden[12];
        vec3 prediction=neuralWeights[36].rgb;
        for(int i=0;i<hiddenSize;++i){
            hidden[i]=tanh(dot(neuralWeights[i],position)+dot(neuralWeights[12+i].xyz,n));
            prediction+=hidden[i]*neuralWeights[24+i].rgb;
        }
        vec3 error=clamp(prediction-target,-0.32,0.32);
        const float learningRate=0.018;
        neuralWeights[36].rgb-=error*(learningRate*0.38);
        for(int i=0;i<hiddenSize;++i){
            vec3 outputWeight=neuralWeights[24+i].rgb;
            float h=hidden[i];
            float delta=dot(outputWeight,error)*(1.0-h*h);
            neuralWeights[24+i].rgb=clamp(outputWeight-learningRate*error*h,-1.4,1.4);
            neuralWeights[i]-=position*(learningRate*0.52*delta);
            neuralWeights[12+i].xyz-=n*(learningRate*0.52*delta);
        }
        neuralWeights[37].x=min(neuralWeights[37].x+1.0,65536.0);
    }
}
)GLSL";

} // namespace eng::lunar_shaders
