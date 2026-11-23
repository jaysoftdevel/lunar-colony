#pragma once
#include "game/world/supply_state.hpp"
#include <span>
#include <vector>
#include <utility>
#include <numeric>
namespace game::gas {
// Isothermal reduced cabin mixture: 30 mol% O2, 70% implicit inert buffer.
// Saves retain oxygen-equivalent mass. No separate species or thermal CFD.
inline constexpr double temperature=293., gamma=1.4, pressure_target=70000.;
inline constexpr double molar_mass=.30*.0319988+.70*.0280134;
inline constexpr double gas_constant=8.314462618/molar_mass;
inline constexpr double oxygen_mass_fraction=.30*.0319988/molar_mass;
inline double pressure(double oxygen_kg,double volume){
    return volume>0?std::max(0.,oxygen_kg)*pressure_target/(volume*kOxygenKgPerM3):0.;
}
// Compressible isentropic orifice flow, capped at sonic velocity. An 80 mm
// service duct uses Cd=.65, Darcy f=.025 and a 1.5 local loss coefficient.
inline double mass_rate(double upstream,double downstream,double length){
    if(upstream<=downstream||upstream<=0)return 0;
    const double ratio=std::clamp(downstream/upstream,0.,1.);
    const double critical=std::pow(2./(gamma+1.),gamma/(gamma-1.));
    const double factor=ratio<=critical?
        std::sqrt(gamma)*std::pow(2./(gamma+1.),(gamma+1.)/(2.*(gamma-1.))):
        std::sqrt(std::max(0.,2.*gamma/(gamma-1.)*(std::pow(ratio,2./gamma)-std::pow(ratio,(gamma+1.)/gamma))));
    constexpr double diameter=.08, area=3.141592653589793*diameter*diameter/4.;
    const double loss=std::sqrt(1.+.025*std::max(0.,length)/diameter+1.5);
    return .65*area/loss*upstream/std::sqrt(gas_constant*temperature)*factor;
}
struct Diagnostics {double mass_error{};int iterations{};};
// Backward Euler pressure-conductance Laplacian, solved by diagonal-PCG.
// Symmetry conserves mass; the implicit maximum principle avoids small-manifold
// CFL restrictions, negative inventory and pressure overshoot.
inline Diagnostics advance(std::span<SupplyState* const> nodes,
    std::span<const std::pair<std::size_t,std::size_t>> links,
    std::span<const double> lengths,double physical_seconds){
    Diagnostics result;if(physical_seconds<=0||nodes.empty())return result;
    const auto n=nodes.size();std::vector<double> rho(n),rhs(n),diag(n),r(n),z(n),p(n),ap(n),weights(links.size());
    double before=0;
    for(std::size_t i=0;i<n;++i){auto& s=*nodes[i];const double v=std::max(0.,s.volume_m3);
        rho[i]=v>0?s.oxygen_kg/v:0;rhs[i]=s.oxygen_kg;diag[i]=v>0?v:1.;before+=s.oxygen_kg;
    }
    for(std::size_t e=0;e<links.size();++e){const auto [a,b]=links[e];
        if(a>=n||b>=n||nodes[a]->volume_m3<=0||nodes[b]->volume_m3<=0)continue;
        const double pa=pressure(nodes[a]->oxygen_kg,nodes[a]->volume_m3),pb=pressure(nodes[b]->oxygen_kg,nodes[b]->volume_m3);
        const double dp=std::max(10.,std::abs(pa-pb)),up=std::max({pa,pb,10.});
        const double rate=mass_rate(up,std::max(0.,up-dp),e<lengths.size()?lengths[e]:0)*oxygen_mass_fraction;
        weights[e]=physical_seconds*rate/(dp*kOxygenKgPerM3/pressure_target);
        diag[a]+=weights[e];diag[b]+=weights[e];
    }
    const auto multiply=[&](const auto& x,auto& y){
        for(std::size_t i=0;i<n;++i)y[i]=(nodes[i]->volume_m3>0?nodes[i]->volume_m3:1.)*x[i];
        for(std::size_t e=0;e<links.size();++e)if(weights[e]>0){const auto [a,b]=links[e];const double flow=weights[e]*(x[a]-x[b]);y[a]+=flow;y[b]-=flow;}
    };
    const auto dot=[](const auto& a,const auto& b){return std::inner_product(a.begin(),a.end(),b.begin(),0.);};
    multiply(rho,ap);for(std::size_t i=0;i<n;++i){r[i]=rhs[i]-ap[i];z[i]=r[i]/diag[i];p[i]=z[i];}
    double rz=dot(r,z);const double tolerance=1e-22*std::max(1.,dot(rhs,rhs));
    for(int iteration=0;iteration<256&&dot(r,r)>tolerance;++iteration){
        multiply(p,ap);const double pap=dot(p,ap);if(pap<=1e-30)break;
        const double alpha=rz/pap;
        for(std::size_t i=0;i<n;++i){rho[i]+=alpha*p[i];r[i]-=alpha*ap[i];z[i]=r[i]/diag[i];}
        const double next=dot(r,z);const double beta=rz>0?next/rz:0;rz=next;
        for(std::size_t i=0;i<n;++i)p[i]=z[i]+beta*p[i];
        result.iterations=iteration+1;
    }
    double after=0;
    for(std::size_t i=0;i<n;++i){if(nodes[i]->volume_m3>0)nodes[i]->oxygen_kg=std::clamp(rho[i],0.,kOxygenKgPerM3)*nodes[i]->volume_m3;after+=nodes[i]->oxygen_kg;}
    result.mass_error=after-before;return result;
}
}

