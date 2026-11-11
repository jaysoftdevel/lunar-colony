#pragma once
#include "engine/math.hpp"

namespace eng::lunar {

struct Bounds {
    Vec3 low{},high{};

    void include(Vec3 p){
        low={std::min(low.x,p.x),std::min(low.y,p.y),std::min(low.z,p.z)};
        high={std::max(high.x,p.x),std::max(high.y,p.y),std::max(high.z,p.z)};
    }

    // Extremal corner along a plane normal. The projected interval of an
    // axis-aligned box is the dot product of its two extremal corners.
    Vec3 support(Vec3 normal) const {
        return {normal.x<0?low.x:high.x,normal.y<0?low.y:high.y,normal.z<0?low.z:high.z};
    }
};

class ClipVolume {
    struct Plane { Vec3 normal;float offset; };
    std::array<Plane,6> planes_{};
public:
    explicit ClipVolume(const Mat4& projection){
        const auto& m=projection.m;
        // OpenGL clip coordinates satisfy -w <= x,y,z <= w.
        for(int axis=0;axis<3;++axis){
            for(int side=0;side<2;++side){
                const float sign=side==0?1.0f:-1.0f;
                planes_[axis*2+side]={{m[3]+sign*m[axis],m[7]+sign*m[4+axis],
                                       m[11]+sign*m[8+axis]},m[15]+sign*m[12+axis]};
            }
        }
    }

    bool intersects(const Bounds& bounds) const {
        for(const auto& plane:planes_){
            const float distance=dot(plane.normal,bounds.support(plane.normal))+plane.offset;
            if(distance<-0.0001f) return false;
        }
        return true;
    }
};

} // namespace eng::lunar
