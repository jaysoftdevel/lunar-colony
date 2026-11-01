#pragma once
#include "engine/engine.hpp"

namespace eng {
// Ordered geometry for one projection/depth state. Each vertex is uploaded
// once; commands reference disjoint ranges rather than overwriting offset 0
// while an earlier draw may still be reading it on the GPU.
struct GeometryStream {
    struct Command {
        unsigned primitive;Color color;float width;std::size_t first,count;
    };
    std::vector<float> vertices;
    std::vector<Command> commands;
    void append(const float* data,std::size_t floats,unsigned primitive,Color color,float width){
        if(!data || floats==0)return;
        const std::size_t first=vertices.size()/3,count=floats/3;
        vertices.insert(vertices.end(),data,data+floats);
        if(!commands.empty()){
            auto& last=commands.back();
            if(last.primitive==primitive && last.width==width && last.color.r==color.r && last.color.g==color.g &&
               last.color.b==color.b && last.color.a==color.a && last.first+last.count==first){last.count+=count;return;}
        }
        commands.push_back({primitive,color,width,first,count});
    }
    void clear(){vertices.clear();commands.clear();}
};
}
