//
// Created by ziqwang on 21.02.19.
//

#ifndef TOPOLOCKCREATOR_GLUIVAR_H
#define TOPOLOCKCREATOR_GLUIVAR_H
#include <string>
#include <map>
#include <pugixml.hpp>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <iostream>
using std::string;
using std::vector;
using Eigen::Vector2f;
using std::shared_ptr;
using std::make_shared;
using std::map;

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

struct inputVarType
{
    gluiVarValueType valueType;
    gluiVarControlType controlType;
};

class InputVar
{
public:
    InputVar()
    {
        var_control_type = GLUI_VAR_CONTROL_NONE;
        var_value_type = GLUI_VAR_VALUE_NONE;
        visible = true;
    }

    InputVar& operator<<(string name){
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

class InputVarInt : public InputVar
{
public:
    InputVarInt(int val, Vector2f _bound = Vector2f(0, 0))
    {
        default_value = value = val;
        var_control_type = GLUI_VAR_CONTROL_SPINNER;
        var_value_type = GLUI_VAR_VALUE_INT;
        bound = _bound;
    }

    void clear_value()
    {
        value = default_value;
    }

    int value;
    int default_value;
    Vector2f bound;
};

class InputVarFloat: public InputVar
{
public:
    InputVarFloat(float val, Vector2f _bound = Vector2f(0, 0))
    {
        default_value = value = val;
        var_control_type = GLUI_VAR_CONTROL_SPINNER;
        var_value_type = GLUI_VAR_VALUE_FLOAT;
        bound = _bound;
    }

    void clear_value()
    {
        value = default_value;
    }

    float value;
    float default_value;
    Vector2f bound;
};

class InputVarBool: public InputVar{
public:
    InputVarBool(bool val){
        default_value = value = val;
        var_control_type = GLUI_VAR_CONTROL_CHECKBOX;
        var_value_type = GLUI_VAR_VALUE_BOOL;
    }

    void clear_value()
    {
        value = default_value;
    }

    int value;
    int default_value;

};

class InputVarList
{
public:
    int index;
    std::vector<shared_ptr<InputVar>> varLists;
    map<std::string, InputVar *> varMap;
    string filename;

public:

    InputVarList()
    {
        index = 200;
    }

public:
    template <typename Scalar>
    Scalar get(string name)
    {
        InputVar* var = find(name);
        if(var != nullptr)
        {
            if (typeid(Scalar) == typeid(int)) return ((InputVarInt *) var)->value;
            if (typeid(Scalar) == typeid(float)) return ((InputVarFloat *) var)->value;
            if (typeid(Scalar) == typeid(bool)) return ((InputVarBool *) var)->value;
        }
        else{
            std::cout << "Error: get a unresgistered variable" << std::endl;
        }
        return Scalar();
    }

    template <typename Scalar>
    void set(string name, Scalar _v){
        InputVar* var = find(name);
        if(var != nullptr)
        {
            if (typeid(Scalar) == typeid(int)) ((InputVarInt *) var)->value = _v;
            if (typeid(Scalar) == typeid(float)) ((InputVarFloat *) var)->value = _v;
            if (typeid(Scalar) == typeid(bool)) ((InputVarBool *) var)->value = _v;
        }
        return;
    }

    vector<shared_ptr<InputVar>> findSeries(string name)
    {
        vector<shared_ptr<InputVar>> series;
        for(shared_ptr<InputVar> var: varLists){
            if(name == var->series_name) series.push_back(var);
        }
        return series;
    }

    InputVar* find(string name)
    {
        auto find_it = varMap.find(name);
        if(find_it != varMap.end()) return find_it->second;
        return nullptr;
    }

    InputVar& add(int value, string name, string label)
    {
        shared_ptr<InputVar> var = make_shared<InputVarInt>(value);
        *var << name;
        var->label = label;
        varLists.push_back(var);
        varMap[name] = var.get();
        var->gluiID = index++;
        return *var;
    }

    InputVar& add(int value, Vector2f bound, string name, string label){
        shared_ptr<InputVar> var = make_shared<InputVarInt>(value, bound);
        *var << name;
        var->label = label;
        varLists.push_back(var);
        varMap[name] = var.get();
        var->gluiID = index++;
        return *var;
    }

    InputVar& add(float value, string name, string label)
    {
        shared_ptr<InputVar> var = make_shared<InputVarFloat>(value);
        *var << name;
        var->label = label;
        varLists.push_back(var);
        varMap[name] = var.get();
        var->gluiID = index++;
        return *var;
    }

    InputVar& add(float value, Vector2f bound, string name, string label){
        shared_ptr<InputVar> var = make_shared<InputVarFloat>(value, bound);
        *var << name;
        var->label = label;
        varLists.push_back(var);
        varMap[name] = var.get();
        var->gluiID = index++;
        return *var;
    }

    InputVar& add(bool value, string name, string label)
    {
        shared_ptr<InputVar> var = make_shared<InputVarBool>(value);
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

class InputVarManager
{
public:

    void write(InputVar *var, pugi::xml_node &node){
        pugi::xml_node node_var = node.child(var->var_names.front().c_str());
        if(!node_var) node_var  =  node.append_child(var->var_names.front().c_str());
        pugi::xml_attribute attr_var = node_var.attribute("value");
        if(!attr_var) attr_var = node_var.append_attribute("value");
        switch(var->var_value_type)
        {
            case GLUI_VAR_VALUE_INT:
                attr_var.set_value(((InputVarInt *)var)->value);
                break;
            case GLUI_VAR_VALUE_BOOL:
                attr_var.set_value(((InputVarBool *)var)->value);
                break;
            case GLUI_VAR_VALUE_FLOAT:
                attr_var.set_value(((InputVarFloat *)var)->value);
                break;
            default:
                break;
        }
    }

    void read(InputVar *var, pugi::xml_node &node)
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
                ((InputVarInt *)var)->value = attr_var.as_int();
                break;
            case GLUI_VAR_VALUE_BOOL:
                ((InputVarBool *)var)->value = attr_var.as_bool();
                break;
            case GLUI_VAR_VALUE_FLOAT:
                ((InputVarFloat *)var)->value = attr_var.as_float();
                break;
            default:
                break;
        }
        return;
    }
};

void InitVar(InputVarList *varList);

void InitVarLite(InputVarList *varList);

#endif //TOPOLOCKCREATOR_GLUIVAR_H
