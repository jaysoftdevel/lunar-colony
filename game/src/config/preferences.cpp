#include "game/config/preferences.hpp"
#include <algorithm>
#include <charconv>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string_view>
#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

namespace game {
namespace {
using namespace eng;

std::filesystem::path config_path(){
    if(const char* path=std::getenv("GAME_GRAPHICS_CONFIG"); path && *path) return path;
#ifdef _WIN32
    if(const char* appdata=std::getenv("APPDATA"); appdata && *appdata)
        return std::filesystem::path(appdata)/"Lunar Frontier"/"graphics.cfg";
#else
    if(const char* xdg=std::getenv("XDG_CONFIG_HOME"); xdg && *xdg)
        return std::filesystem::path(xdg)/"lunar-frontier"/"graphics.cfg";
    if(const char* home=std::getenv("HOME"); home && *home)
        return std::filesystem::path(home)/".config"/"lunar-frontier"/"graphics.cfg";
#endif
    return {};
}

constexpr std::pair<const char*,int GraphicsSettings::*> numbers[]{
    {"render_scale",&GraphicsSettings::render_scale},{"ray_scale",&GraphicsSettings::ray_scale},
    {"bloom",&GraphicsSettings::bloom},{"exposure_tenths",&GraphicsSettings::exposure_tenths}
};
}

Preferences load_preferences(){
    Preferences preferences{eng::graphics_settings_from_environment()};
    auto& settings=preferences.graphics;
    std::ifstream file(config_path());
    for(std::string line;std::getline(file,line);){
        if(!line.empty() && line.back()=='\r') line.pop_back();
        const auto separator=line.find('=');
        if(separator==std::string::npos || separator+1==line.size()) continue;
        const std::string_view key(line.data(),separator);
        const char* first=line.data()+separator+1;
        const char* last=line.data()+line.size();
        int value{};
        const auto parsed=std::from_chars(first,last,value);
        if(parsed.ec!=std::errc{} || parsed.ptr!=last) continue;
        if(key=="path" && value>=0 && value<=2) settings.path=static_cast<GraphicsPath>(value);
        if(key=="aa" && value>=0 && value<=2) settings.aa=static_cast<GraphicsAA>(value);
        if(key=="shadows" && value>=0 && value<=2) settings.shadows=static_cast<GraphicsShadows>(value);
        if(key=="transparency" && value>=0 && value<=1) settings.transparency=static_cast<GraphicsTransparency>(value);
        for(const auto& [name,field]:numbers) if(key==name) settings.*field=value;
        if(value!=0 && value!=1) continue;
        if(key=="mesh_layout") preferences.mesh_layout=value==1;
        if(key=="music") preferences.music_enabled=value==1;
#ifdef _WIN32
        if(key=="terrain_vrs") settings.terrain_vrs=value==1;
#endif
        const auto it=std::find(graphics_feature_keys.begin(),graphics_feature_keys.end(),key);
        if(it!=graphics_feature_keys.end())
            settings.set(static_cast<GraphicsFeature>(it-graphics_feature_keys.begin()),value==1);
    }
    settings.normalize();
    return preferences;
}

bool save_preferences(const Preferences& preferences){
    const auto& settings=preferences.graphics;
    const auto path=config_path();
    if(path.empty()) return false;
    std::error_code error;
    if(!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path(),error);
    if(error) return false;
    auto temporary=path;
#ifdef _WIN32
    const auto process_id=_getpid();
#else
    const auto process_id=getpid();
#endif
    temporary+="."+std::to_string(process_id)+".tmp";
    std::ofstream file(temporary,std::ios::trunc);
    if(!file) return false;
    file<<"path="<<int(settings.path)<<'\n'<<"aa="<<int(settings.aa)<<'\n'
        <<"shadows="<<int(settings.shadows)<<'\n'<<"transparency="<<int(settings.transparency)<<'\n';
    for(const auto& [name,field]:numbers) file<<name<<'='<<settings.*field<<'\n';
    for(std::size_t i=0;i<graphics_feature_keys.size();++i)
        file<<graphics_feature_keys[i]<<'='<<settings.enabled(static_cast<GraphicsFeature>(i))<<'\n';
    file<<"mesh_layout="<<preferences.mesh_layout<<'\n'
        <<"music="<<preferences.music_enabled<<'\n';
#ifdef _WIN32
    file<<"terrain_vrs="<<settings.terrain_vrs<<'\n';
#endif
    file.close();
    if(file){
        std::filesystem::rename(temporary,path,error);
        if(!error) return true;
    }
    std::filesystem::remove(temporary,error);
    return false;
}
} // namespace game
