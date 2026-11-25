#include "game/config/preferences.hpp"
#include "game/ui/layout.hpp"
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
namespace fs=std::filesystem;
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
struct Environment {
    using String=fs::path::string_type;
    std::vector<std::pair<String,String>> saved;
    void set(const char* name,const fs::path& value){
        const auto key=fs::path(name).native();
#ifdef _WIN32
        const auto old=_wgetenv(key.c_str());saved.emplace_back(key,old?old:L"");_wputenv_s(key.c_str(),value.c_str());
#else
        const auto old=std::getenv(key.c_str());saved.emplace_back(key,old?old:"");if(value.empty())unsetenv(key.c_str());else setenv(key.c_str(),value.c_str(),1);
#endif
    }
    ~Environment(){for(auto it=saved.rbegin();it!=saved.rend();++it){
#ifdef _WIN32
        _wputenv_s(it->first.c_str(),it->second.c_str());
#else
        if(it->second.empty())unsetenv(it->first.c_str());else setenv(it->first.c_str(),it->second.c_str(),1);
#endif
    }}
};
std::string read(const fs::path& p){std::ifstream f(p);return {std::istreambuf_iterator<char>(f),{}};}
int main(){try{
    Environment env;const auto root=fs::temp_directory_path()/fs::path(u8"lunar settings \u00fc")/std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    struct Cleanup{fs::path p;~Cleanup(){std::error_code e;fs::remove_all(p,e);}} cleanup{root};
    env.set("GAME_SETTINGS_CONFIG",{});env.set("GAME_GRAPHICS_CONFIG",{});
#ifdef _WIN32
    env.set("APPDATA",root);const auto folder=root/"Lunar Frontier";
#else
    env.set("XDG_CONFIG_HOME",root);const auto folder=root/"lunar-frontier";
#endif
    fs::create_directories(folder);const auto old=folder/"graphics.cfg",current=folder/"settings.cfg";
    {std::ofstream f(old);f<<"first_person_fov=86\nmusic=0\n";}
    const auto before=read(old);auto p=game::load_preferences();require(p.autosave_minutes==10,"Autosave default");require(p.first_person_fov==86&&!p.music_enabled,"Legacy preferences were not migrated");
    p.autosave_minutes=25;p.ui_scale_index=3;p.faded_hulls=true;p.sound_volume_percent=0;p.display_width=1280;p.display_height=720;p.debug_frame_enabled=false;
    require(game::save_preferences(p)&&fs::exists(current),"Game-wide settings file was not created");require(read(old)==before,"Migration modified legacy graphics.cfg");
    p=game::load_preferences();require(p.autosave_minutes==25&&p.ui_scale_index==3&&p.faded_hulls&&p.sound_volume_percent==0&&p.display_width==1280&&p.display_height==720&&!p.debug_frame_enabled,"Game-wide setting or mute was lost");
    const auto global=read(current);const auto alias=root/"old-override.cfg",override_file=root/fs::path(u8"explicit settings \u00fc.cfg");
    {std::ofstream f(alias);f<<"first_person_fov=64\n";}env.set("GAME_GRAPHICS_CONFIG",alias);require(game::load_preferences().first_person_fov==64,"Legacy override was broken");
    {std::ofstream f(override_file);f<<"first_person_fov=108\n";}env.set("GAME_SETTINGS_CONFIG",override_file);p=game::load_preferences();require(p.first_person_fov==108,"New override did not take precedence");
    p.sound_volume_percent=45;require(game::save_preferences(p),"Explicit settings write failed");require(read(current)==global&&read(alias)=="first_person_fov=64\n","Explicit override wrote another game's settings");
    {std::ofstream f(override_file);f<<"ui_scale_index=999\nsound_volume_percent=999\nfirst_person_fov=-20\nautosave_minutes=999\ndisplay_width=99999\ndisplay_height=4\nfullscreen=0\n";}
    p=game::load_preferences();require(p.autosave_minutes==60&&p.ui_scale_index==game::layout::UI_SCALE_COUNT-1&&p.sound_volume_percent==100&&p.first_person_fov==50&&p.display_width==0&&p.display_height==0,"Invalid preferences were not clamped safely");
    std::cout<<"PASS game-wide paths, legacy migration, Unicode paths, override precedence, mute/resolution/UI persistence and invalid-value bounds\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
