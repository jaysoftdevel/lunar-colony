#include "lunar_bvh.hpp"

#include <algorithm>
#include <limits>
#include <utility>

namespace eng::lunar {
namespace {

constexpr float kInfinity=std::numeric_limits<float>::infinity();

struct Bounds {
    Float4 minimum{kInfinity,kInfinity,kInfinity,0.0f};
    Float4 maximum{-kInfinity,-kInfinity,-kInfinity,0.0f};

    void add(const Float4& point){
        for(int axis=0;axis<3;++axis){
            minimum[axis]=std::min(minimum[axis],point[axis]);
            maximum[axis]=std::max(maximum[axis],point[axis]);
        }
    }
    int widest_axis() const {
        const float x=maximum[0]-minimum[0],y=maximum[1]-minimum[1],z=maximum[2]-minimum[2];
        return x>=y && x>=z?0:y>=z?1:2;
    }
};

Float4 center(const RayTriangle& triangle){
    return {(triangle.a[0]+triangle.b[0]+triangle.c[0])/3.0f,
            (triangle.a[1]+triangle.b[1]+triangle.c[1])/3.0f,
            (triangle.a[2]+triangle.b[2]+triangle.c[2])/3.0f,0.0f};
}

std::uint32_t node_count(std::uint32_t triangles){
    std::uint32_t leaves=1;
    while(triangles/leaves>6) leaves*=2;
    // At this depth, leaves differ by one triangle. Each oversized leaf
    // splits once more, so pre-order subtree offsets are known before work.
    if(triangles/leaves==6) leaves+=triangles%leaves;
    return 2*leaves-1;
}

class Builder {
public:
    explicit Builder(RayTree& tree):tree_(tree){}

    void node(std::uint32_t index,std::uint32_t first,std::uint32_t last){
        Bounds bounds,centroids;
        for(auto i=first;i<last;++i){
            const auto& triangle=tree_.triangles[i];
            bounds.add(triangle.a);
            bounds.add(triangle.b);
            bounds.add(triangle.c);
            centroids.add(center(triangle));
        }
        const std::uint32_t count=last-first;
        UInt4 link{first,0,count,0};
        if(count>6){
            const auto axis=centroids.widest_axis();
            const auto middle=first+count/2;
            const auto centroid=[axis](const RayTriangle& t){ return (t.a[axis]+t.b[axis])+t.c[axis]; };
            auto begin=tree_.triangles.begin();
            std::nth_element(begin+first,begin+middle,begin+last,[&](const auto& a,const auto& b){
                return centroid(a)<centroid(b);
            });
            const auto left=index+1;
            const auto right=left+node_count(middle-first);
            if(count>=4096){
                TaskPool::shared().parallel_for(2,[&](std::size_t branch){
                    if(branch==0) node(left,first,middle);
                    else node(right,middle,last);
                });
            }else{
                node(left,first,middle);
                node(right,middle,last);
            }
            link={left,right,0,static_cast<std::uint32_t>(axis)};
        }
        tree_.nodes[index]={bounds.minimum,bounds.maximum,link};
    }

private:
    RayTree& tree_;
};

} // namespace

RayTriangle ray_triangle(Vec3 a,Vec3 b,Vec3 c,Color tint,float material){
    return {{a.x,a.y,a.z,material},{b.x,b.y,b.z,0.0f},{c.x,c.y,c.z,0.0f},{tint.r,tint.g,tint.b,tint.a}};
}

RayTree build_ray_tree(std::vector<RayTriangle> triangles,Vec3 focus,std::uint64_t generation){
    RayTree tree{std::move(triangles),{},focus,generation};
    if(!tree.triangles.empty()){
        const auto count=static_cast<std::uint32_t>(tree.triangles.size());
        tree.nodes.resize(node_count(count));
        Builder builder(tree);
        builder.node(0,0,count);
    }
    return tree;
}

} // namespace eng::lunar
