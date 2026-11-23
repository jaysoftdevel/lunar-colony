#include "game/world/gas_flow.hpp"
#include <iostream>
#include <stdexcept>
using namespace game;
void check(bool value,const char* why){if(!value)throw std::runtime_error(why);}
int main(){try{
    using namespace gas;
    check(std::abs(pressure(kOxygenKgPerM3,1)-70000)<1e-8,"ideal gas pressure");
    check(mass_rate(70000,70000,10)==0,"equilibrium flow");
    check(mass_rate(70000,0,10)>0,"vacuum flow");
    check(std::abs(mass_rate(70000,0,10)-mass_rate(70000,20000,10))<1e-12,"sonic limit");
    check(mass_rate(70000,65000,10)>mass_rate(70000,65000,100),"duct friction");
    check(mass_rate(70000,50000,10)>mass_rate(70000,65000,10),"pressure response");
    SupplyState a,b,c;a.volume_m3=300;b.volume_m3=.02;c.volume_m3=900;a.oxygen_kg=a.volume_m3*kOxygenKgPerM3;
    std::vector<SupplyState*> nodes{&a,&b,&c};std::vector<std::pair<std::size_t,std::size_t>> links{{0,1},{1,2}};std::vector<double> lengths{10,20};
    const double inventory=a.oxygen_kg;
    for(int i=0;i<100;++i){auto d=advance(nodes,links,lengths,18);check(std::abs(d.mass_error)<1e-8,"gas solver lost mass");
        for(auto* s:nodes)check(s->oxygen_kg>=0&&s->oxygen_kg<=s->volume_m3*kOxygenKgPerM3,"pressure overshoot");
    }
    check(std::abs(a.oxygen_kg+b.oxygen_kg+c.oxygen_kg-inventory)<1e-7,"network inventory");
    check(c.oxygen_kg>inventory*.1,"small-manifold numerical bottleneck");
    for(int i=0;i<100;++i)advance(nodes,links,lengths,1800);
    check(std::abs(a.breathability()-c.breathability())<.01,"volume equilibrium");
    SupplyState isolated;isolated.volume_m3=10;isolated.oxygen_kg=1;nodes.push_back(&isolated);advance(nodes,links,lengths,1e6);check(isolated.oxygen_kg==1,"isolation");
    a.volume_m3=c.volume_m3=900;a.oxygen_kg=0;c.oxygen_kg=300*kOxygenKgPerM3;
    advance(nodes,links,lengths,18);check(a.oxygen_kg>0&&c.oxygen_kg<300*kOxygenKgPerM3,"flow reversal");
    std::cout<<"Physical gas pressure, choking, friction, conservation, equilibrium, isolation and reversal passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}

