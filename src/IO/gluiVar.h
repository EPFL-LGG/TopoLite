//
// Created by ziqwang on 21.02.19.
//

#ifndef TOPOLOCKCREATOR_GLUIVAR_H
#define TOPOLOCKCREATOR_GLUIVAR_H
#include "string.h"
#include "../Utility/HelpStruct.h"
#include <map>
#include <pugixml.hpp>
using std::string;

#if USE_OPENGL_DRAW
    #include "glui.h"
    extern GLUI *glui;
#endif

enum gluiVarValueType{
    GLUI_VAR_VALUE_NONE,
    GLUI_VAR_VALUE_BOOL,
    GLUI_VAR_VALUE_INT,
    GLUI_VAR_VALUE_FLOAT,
};

enum gluiVarControlType{
    GLUI_VAR_CONTROL_NONE,
    GLUI_VAR_CONTROL_SPINNER,
    GLUI_VAR_CONTROL_CHECKBOX,
    GLUI_VAR_CONTROL_EDITTEXT,
};

struct gluiVarType
{
    gluiVarValueType valueType;
    gluiVarControlType controlType;
};

class gluiVar
{
public:
    gluiVar()
    {
        var_control_type = GLUI_VAR_CONTROL_NONE;
        var_value_type = GLUI_VAR_VALUE_NONE;
        visible = true;
#if USE_OPENGL_DRAW
        aligned_settings = GLUI_ALIGN_LEFT;
        gluiID = -1;
#endif
    }

    gluiVar& operator<<(string name){
        var_names.push_back(name);
        return *this;
    }

    void operator =(string name){
        series_name = name;
    }

    virtual void clear_value(){
        return;
    }

public:
    vector<string> var_names;
    string series_name;
    string label;
    bool visible;
    gluiVarControlType var_control_type;
    gluiVarValueType var_value_type;
    int aligned_settings;
    int gluiID;
};

class gluiVarInt : public gluiVar
{
public:
    gluiVarInt(int val, Vector2f _bound = Vector2f(0, 0))
    {
        default_value = value = val;
        var_control_type = GLUI_VAR_CONTROL_SPINNER;
        var_value_type = GLUI_VAR_VALUE_INT;
        bound = _bound;

#if USE_OPENGL_DRAW
        aligned_settings = GLUI_ALIGN_LEFT;
#endif
    }

    void clear_value()
    {
        value = default_value;
    }

    int value;
    int default_value;
    Vector2f bound;
};

class gluiVarFloat: public gluiVar
{
public:
    gluiVarFloat(float val, Vector2f _bound = Vector2f(0, 0))
    {
        default_value = value = val;
        var_control_type = GLUI_VAR_CONTROL_SPINNER;
        var_value_type = GLUI_VAR_VALUE_FLOAT;
        bound = _bound;
#if USE_OPENGL_DRAW
        aligned_settings = GLUI_ALIGN_LEFT;
#endif
    }

    void clear_value()
    {
        value = default_value;
    }

    float value;
    float default_value;
    Vector2f bound;
};

class gluiVarBool: public gluiVar{
public:
    gluiVarBool(bool val){
        default_value = value = val;
        var_control_type = GLUI_VAR_CONTROL_CHECKBOX;
        var_value_type = GLUI_VAR_VALUE_BOOL;
#if USE_OPENGL_DRAW
        aligned_settings = GLUI_ALIGN_LEFT;
#endif
    }

    void clear_value()
    {
        value = default_value;
    }

    int value;
    int default_value;

};

class gluiVarList
{
public:
    int index;
    std::vector<shared_ptr<gluiVar>> varLists;
    map<std::string, gluiVar *> varMap;
    string filename;

public:

    gluiVarList()
    {
        index = 200;
    }

public:
    template <typename Scalar>
    Scalar get(string name)
    {
        gluiVar* var = find(name);
        if(var != nullptr)
        {
            if (typeid(Scalar) == typeid(int)) return ((gluiVarInt *) var)->value;
            if (typeid(Scalar) == typeid(float)) return ((gluiVarFloat *) var)->value;
            if (typeid(Scalar) == typeid(bool)) return ((gluiVarBool *) var)->value;
        }
        else{
            std::cout << "Error: get a unresgistered variable" << std::endl;
        }
        return Scalar();
    }

    template <typename Scalar>
    void set(string name, Scalar _v){
        gluiVar* var = find(name);
        if(var != nullptr)
        {
            if (typeid(Scalar) == typeid(int)) ((gluiVarInt *) var)->value = _v;
            if (typeid(Scalar) == typeid(float)) ((gluiVarFloat *) var)->value = _v;
            if (typeid(Scalar) == typeid(bool)) ((gluiVarBool *) var)->value = _v;
        }
        return;
    }

    vector<shared_ptr<gluiVar>> findSeries(string name)
    {
        vector<shared_ptr<gluiVar>> series;
        for(shared_ptr<gluiVar> var: varLists){
            if(name == var->series_name) series.push_back(var);
        }
        return series;
    }

    gluiVar* find(string name)
    {
        auto find_it = varMap.find(name);
        if(find_it != varMap.end()) return find_it->second;
        return nullptr;
    }

    gluiVar& add(int value, string name, string label)
    {
        shared_ptr<gluiVar> var = make_shared<gluiVarInt>(value);
        *var << name;
        var->label = label;
        varLists.push_back(var);
        varMap[name] = var.get();
        var->gluiID = index++;
        return *var;
    }

    gluiVar& add(int value, Vector2f bound, string name, string label){
        shared_ptr<gluiVar> var = make_shared<gluiVarInt>(value, bound);
        *var << name;
        var->label = label;
        varLists.push_back(var);
        varMap[name] = var.get();
        var->gluiID = index++;
        return *var;
    }

    gluiVar& add(float value, string name, string label)
    {
        shared_ptr<gluiVar> var = make_shared<gluiVarFloat>(value);
        *var << name;
        var->label = label;
        varLists.push_back(var);
        varMap[name] = var.get();
        var->gluiID = index++;
        return *var;
    }

    gluiVar& add(float value, Vector2f bound, string name, string label){
        shared_ptr<gluiVar> var = make_shared<gluiVarFloat>(value, bound);
        *var << name;
        var->label = label;
        varLists.push_back(var);
        varMap[name] = var.get();
        var->gluiID = index++;
        return *var;
    }

    gluiVar& add(bool value, string name, string label)
    {
        shared_ptr<gluiVar> var = make_shared<gluiVarBool>(value);
        *var << name;
        var->label = label;
        varLists.push_back(var);
        varMap[name] = var.get();
        var->gluiID = index++;
        return *var;
    }

    void clear(){
        varLists.clear();
        varMap.clear();
    }
};

class gluiVarOrganizer
{
public:

#if USE_OPENGL_DRAW
    void draw(gluiVar *var, GLUI_Panel *panel, void (*callbackGLUI)(int))
    {
        if(var != nullptr && var->visible){
            if(var->var_control_type == GLUI_VAR_CONTROL_CHECKBOX){
                drawCheckBox((gluiVarBool *)var, panel, callbackGLUI);
            }
            if(var->var_control_type == GLUI_VAR_CONTROL_SPINNER){
                if(var->var_value_type == GLUI_VAR_VALUE_INT)
                    drawSpinnerInt((gluiVarInt *)var, panel, callbackGLUI);
                if(var->var_value_type == GLUI_VAR_VALUE_FLOAT)
                    drawSpinnerFloat((gluiVarFloat *)var, panel, callbackGLUI);
            }
        }
    }
#endif

    void write(gluiVar *var, pugi::xml_node &node){
        pugi::xml_node node_var = node.child(var->var_names.front().c_str());
        if(!node_var) node_var  =  node.append_child(var->var_names.front().c_str());
        pugi::xml_attribute attr_var = node_var.attribute("value");
        if(!attr_var) attr_var = node_var.append_attribute("value");
        switch(var->var_value_type)
        {
            case GLUI_VAR_VALUE_INT:
                attr_var.set_value(((gluiVarInt *)var)->value);
                break;
            case GLUI_VAR_VALUE_BOOL:
                attr_var.set_value(((gluiVarBool *)var)->value);
                break;
            case GLUI_VAR_VALUE_FLOAT:
                attr_var.set_value(((gluiVarFloat *)var)->value);
                break;
            default:
                break;
        }
    }

    void read(gluiVar *var, pugi::xml_node &node)
    {
        var->clear_value();
        pugi::xml_node node_var;
        for(string name : var->var_names)
        {
            node_var = node.child(name.c_str());
            if(node_var) {
                break;
            }
        }

        if(!node_var) return;
        pugi::xml_attribute attr_var = node_var.attribute("value");
        if(!attr_var) return;

        switch(var->var_value_type)
        {
            case GLUI_VAR_VALUE_INT:
                ((gluiVarInt *)var)->value = attr_var.as_int();
                break;
            case GLUI_VAR_VALUE_BOOL:
                ((gluiVarBool *)var)->value = attr_var.as_bool();
                break;
            case GLUI_VAR_VALUE_FLOAT:
                ((gluiVarFloat *)var)->value = attr_var.as_float();
                break;
            default:
                break;
        }
        return;
    }

#if USE_OPENGL_DRAW
public:

    void drawCheckBox(gluiVarBool *var, GLUI_Panel *panel, void (*callbackGLUI)(int))
    {
        GLUI_Checkbox *checkbox = glui->add_checkbox_to_panel(panel, var->label, &var->value, var->gluiID, callbackGLUI);
        checkbox->set_alignment(var->aligned_settings);
    }

    void drawSpinnerInt(gluiVarInt *var, GLUI_Panel *panel, void (*callbackGLUI)(int))
    {
        GLUI_Spinner *spinner = glui->add_spinner_to_panel(panel, var->label, GLUI_SPINNER_INT, &(var->value), var->gluiID, callbackGLUI);
        spinner->set_alignment(var->aligned_settings);
        spinner->set_w(120);
        spinner->set_int_limits((int)var->bound.x, (int)var->bound.y, GLUI_LIMIT_WRAP);
        spinner->set_speed(10 / var->bound[1]);
        return;
    }

    void drawSpinnerFloat(gluiVarFloat *var, GLUI_Panel *panel, void (*callbackGLUI)(int))
    {
        GLUI_Spinner *spinner = glui->add_spinner_to_panel(panel, var->label, GLUI_SPINNER_FLOAT, &(var->value), var->gluiID, callbackGLUI);
        spinner->set_alignment(var->aligned_settings);
        spinner->set_w(120);
        spinner->set_float_limits(var->bound.x, var->bound.y, GLUI_LIMIT_WRAP);
        spinner->set_speed(0.3);
        return;
    }
#endif
};

void InitVar(gluiVarList &varList);

void InitDemoVar(gluiVarList &varList);

#endif //TOPOLOCKCREATOR_GLUIVAR_H
