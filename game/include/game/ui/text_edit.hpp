#pragma once
#include "game/runtime/presentation.hpp"
#include <algorithm>
#include <cctype>
#include <optional>
#include <string>
#include <vector>
namespace game::text_edit {
inline std::size_t next(const std::string& s,std::size_t i){if(i>=s.size())return s.size();++i;while(i<s.size()&&(static_cast<unsigned char>(s[i])&0xc0)==0x80)++i;return i;}
inline std::size_t previous(const std::string& s,std::size_t i){if(!i)return 0;--i;while(i&&(static_cast<unsigned char>(s[i])&0xc0)==0x80)--i;return i;}
inline std::size_t count(const std::string& s,std::size_t end=std::string::npos){std::size_t n=0;for(std::size_t i=0;i<std::min(end,s.size());i=next(s,i))++n;return n;}
inline std::size_t offset(const std::string& s,std::size_t column){std::size_t i=0;while(column--&&i<s.size())i=next(s,i);return i;}
inline std::string display(const std::string& s){std::string out;for(std::size_t i=0;i<s.size();i=next(s,i))out+=next(s,i)==i+1?s[i]:'?';return out;}
inline std::string clean(const std::string& s,std::size_t budget){
    std::string out;
    for(std::size_t i=0;i<s.size();){
        const auto c=static_cast<unsigned char>(s[i]);std::size_t n=c<128?1:c>=0xc2&&c<=0xdf?2:c>=0xe0&&c<=0xef?3:c>=0xf0&&c<=0xf4?4:0;
        if(!n||i+n>s.size()){++i;continue;}bool valid=true;unsigned cp=c&((1u<<(7-n))-1);
        if(n==1)cp=c;
        for(std::size_t j=1;j<n;++j){const auto b=static_cast<unsigned char>(s[i+j]);valid&=(b&0xc0)==0x80;cp=(cp<<6)|(b&63);}
        valid&=!(n==3&&cp<0x800)&&!(n==4&&cp<0x10000)&&!(cp>=0xd800&&cp<=0xdfff)&&cp<=0x10ffff;
        if(valid&&cp>=32&&!(cp>=127&&cp<=159)){if(out.size()+n>budget)break;out.append(s,i,n);}i+=n;
    }
    return out;
}
struct State {std::string text;std::size_t caret{},anchor{};};
struct Editor {
    std::size_t caret{},anchor{};bool overwrite{};std::string observed;
    std::vector<State> undo,redo;
    void reset(const std::string& s,bool select=false){observed=s;caret=s.size();anchor=select?0:caret;undo.clear();redo.clear();}
    void sync(const std::string& s){if(s!=observed)reset(s);caret=std::min(caret,s.size());anchor=std::min(anchor,s.size());}
    std::size_t low()const{return std::min(caret,anchor);}std::size_t high()const{return std::max(caret,anchor);}
    void move(std::size_t to,bool select){caret=to;if(!select)anchor=caret;}
    void remember(const std::string& s){undo.push_back({s,caret,anchor});if(undo.size()>64)undo.erase(undo.begin());redo.clear();}
    void replace(std::string& s,const std::string& raw){
        auto first=low(),last=high();if(first==last&&overwrite&&!raw.empty())last=next(s,caret);
        const auto retained=s.size()-(last-first);const auto inserted=clean(raw,retained>=64?0:64-retained);
        if((first==last&&inserted.empty())||(!raw.empty()&&inserted.empty()))return;
        remember(s);s.replace(first,last-first,inserted);caret=anchor=first+inserted.size();observed=s;
    }
    void history(std::string& s,bool forward){auto& from=forward?redo:undo;auto& to=forward?undo:redo;if(from.empty())return;
        to.push_back({s,caret,anchor});auto state=from.back();from.pop_back();s=std::move(state.text);caret=state.caret;anchor=state.anchor;observed=s;}
    std::optional<std::string> update(std::string& s,const runtime::InputState& in,const std::string& clipboard){
        sync(s);std::optional<std::string> copied;
        const bool control=in.key_ctrl_down,select=in.key_shift_down;
        if(in.edit_select_all){anchor=0;caret=s.size();}
        if(in.edit_copy||in.edit_cut){if(low()!=high())copied=s.substr(low(),high()-low());if(in.edit_cut)replace(s,"");}
        if(in.edit_undo)history(s,false);else if(in.edit_redo)history(s,true);
        if(in.edit_insert)overwrite=!overwrite;
        if(in.edit_home)move(0,select);
        if(in.edit_end)move(s.size(),select);
        if(in.edit_left){auto to=!select&&low()!=high()?low():previous(s,caret);
            if(control){while(to&&std::isspace(static_cast<unsigned char>(s[to])))to=previous(s,to);while(to&&!std::isspace(static_cast<unsigned char>(s[previous(s,to)])))to=previous(s,to);}move(to,select);}
        if(in.edit_right){auto to=!select&&low()!=high()?high():next(s,caret);
            if(control){while(to<s.size()&&!std::isspace(static_cast<unsigned char>(s[to])))to=next(s,to);while(to<s.size()&&std::isspace(static_cast<unsigned char>(s[to])))to=next(s,to);}move(to,select);}
        if(in.backspace_pressed){const bool selected=low()!=high();if(!selected)anchor=previous(s,caret);if(control&&!selected)while(anchor&&!std::isspace(static_cast<unsigned char>(s[previous(s,anchor)])))anchor=previous(s,anchor);replace(s,"");}
        if(in.delete_pressed){const bool selected=low()!=high();if(!selected)anchor=next(s,caret);if(control&&!selected)while(anchor<s.size()&&!std::isspace(static_cast<unsigned char>(s[anchor])))anchor=next(s,anchor);replace(s,"");}
        if(in.edit_paste)replace(s,clipboard);
        if(!in.text_input.empty()&&!control)replace(s,in.text_input);
        return copied;
    }
};
} // namespace game::text_edit
