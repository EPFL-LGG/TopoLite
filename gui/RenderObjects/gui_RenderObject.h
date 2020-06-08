//
// Created by ziqwang on 09.04.20.
//

#ifndef TOPOLITE_GUI_RENDEROBJECT_H
#define TOPOLITE_GUI_RENDEROBJECT_H
#include <vector>
#include "TopoLite/IO/InputVar.h"
#include "nanogui/vector.h"
using std::vector;

enum AnimationState{
    Stop,
    Run,
    Pause
};

template<typename Scalar>
class gui_RenderObject{

public:
    vector<nanogui::Color> object_colors;
    Eigen::Vector3d object_center;

public: //shader and render pass
    nanogui::ref<nanogui::RenderPass> render_pass;
    nanogui::ref<nanogui::Shader> shader;

public: //animation
    AnimationState state;
    float prev_time;

public:  //buffers
    vector<float> buffer_positions;
    vector<float> buffer_colors;

    
public: // uniform
    
    Eigen::Matrix4f proj_mat;
    Eigen::Matrix4f model_mat;
    Eigen::Matrix4f model_init_mat;
    Eigen::Matrix4f view_mat;
    Eigen::Vector3f eye;

    float simtime;
    bool visible;
    bool model_mat_fixed;

    shared_ptr<InputVarList> varList;

public:

    gui_RenderObject(nanogui::ref<nanogui::RenderPass> _render_pass){
        simtime = prev_time = 0;
        state = Stop;
        model_init_mat = Eigen::Matrix4f::Identity();
        model_mat = Eigen::Matrix4f::Identity();
        view_mat = Eigen::Matrix4f::Identity();
        proj_mat = Eigen::Matrix4f::Identity();
        eye = Eigen::Vector3f::Zero();
        render_pass = _render_pass;
        visible = true;
        model_mat_fixed = false;
        varList = make_shared<InputVarList>();
    }

public:

    virtual void init_shader()
    {
        shader = new nanogui::Shader(render_pass, "Empty Shader", "", "", nanogui::Shader::BlendMode::None);
    }

    virtual void update_buffer(){

    }

    virtual void update_uniform(){

    }

    void update_time(float curr_time, float speed = 0.1){
        switch(state){
            case Stop:
                simtime = 0;
                break;
            case Run:
                simtime += (curr_time - prev_time) * speed;
                break;
            case Pause:
                break;
        }
        prev_time = curr_time;
    }

    nanogui::Matrix4f toNanoguiMatrix(Eigen::Matrix4f mat){
        nanogui::Matrix4f nanomat;
        for(int id = 0; id < 4; id++){
            for(int jd = 0; jd < 4; jd++){
                nanomat.m[id][jd] = mat(jd, id);
            }
        }
        return nanomat;
    }

    nanogui::Vector3f toNanoguiVector(Eigen::Vector3f vec){
        return nanogui::Vector3f(vec.x(), vec.y(), vec.z());
    }

    virtual void draw_object()
    {
        if(visible){
            update_uniform();
            if(!buffer_positions.empty())
            {
                shader->begin();
                shader->draw_array(nanogui::Shader::PrimitiveType::Triangle, 0, buffer_positions.size() / 3, false);
                shader->end();
            }
        }
    }

//    template<typename AttrType>
//    void update_attr(string name, AttrType value){
//        varList->set(name, value);
//    }

};

#endif //TOPOLITE_GUI_RENDEROBJECT_H
