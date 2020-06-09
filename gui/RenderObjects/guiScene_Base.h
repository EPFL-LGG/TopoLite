//
//
// Created by ziqwang on 09.04.20.
//

#ifndef TOPOLITE_GUISCENE_BASE_H
#define TOPOLITE_GUISCENE_BASE_H

#include "guiShader_Base.h"
#include <memory>
#include <Eigen/Dense>

using std::shared_ptr;

template<typename Scalar>
class guiScene_Base{
public:
    vector<shared_ptr<guiShader_Base<Scalar>>> objects;
    nanogui::ref<nanogui::RenderPass> render_pass;
    AnimationState state;
    float simtime;
    int focus_item = -1;
public:

    Eigen::Vector3f focus(){
        if(focus_item >= 0 && focus_item < objects.size()){
            return Eigen::Vector3f(objects[focus_item]->object_center.x(), objects[focus_item]->object_center.y(), objects[focus_item]->object_center.z());
        }
        else if(objects.size() >= 1){
            return Eigen::Vector3f(objects[0]->object_center.x(), objects[0]->object_center.y(), objects[0]->object_center.z());
        }
        else{
            return Eigen::Vector3f(0, 0, 0);
        }
    }

    void update_time(float curr_time, float speed = 0.1){
        for(shared_ptr<guiShader_Base<Scalar>> object : objects){
            object->update_time(curr_time, speed);
            simtime = object->simtime;
        }
    }

    void update_simtime(float _simtime){
        for(shared_ptr<guiShader_Base<Scalar>> object : objects){
            object->simtime = _simtime;
        }
        simtime = _simtime;
    }

    void update_state(AnimationState _state){
        for(shared_ptr<guiShader_Base<Scalar>> object : objects){
            object->state = _state;
        }
        state = _state;
    }

    void update_proj(Eigen::Matrix4f proj){
        for(shared_ptr<guiShader_Base<Scalar>> object : objects){
            object->proj_mat = proj;
        }
    }

    void update_model(Eigen::Matrix4f model){
        for(shared_ptr<guiShader_Base<Scalar>> object : objects){
            if(object->model_mat_fixed){
                object->model_mat = object->model_init_mat;
            }
            else{
                object->model_mat = model * object->model_init_mat;
            }
        }
    }

    void update_view(Eigen::Matrix4f view){
        for(shared_ptr<guiShader_Base<Scalar>> object : objects){
            object->view_mat = view;
        }
    }

    void update_eye(Eigen::Vector3f eye){
        for(shared_ptr<guiShader_Base<Scalar>> object : objects){
            object->eye = eye;
        }
    }

    void draw(){
        /* MVP uniforms */
//#if defined(NANOGUI_USE_OPENGL)
//        glClear(GL_DEPTH_BUFFER_BIT);
//        glEnable(GL_DEPTH_TEST);
//        glDepthMask( true );
//#endif
        for(shared_ptr<guiShader_Base<Scalar>> object : objects){
            object->draw_object();
        }
        
    }

    template<typename AttrType>
    void update_attr(string name, AttrType value)
    {
        for(shared_ptr<guiShader_Base<Scalar>> object : objects){
            object->update_attr(name, value);
        }
    }
};

#endif //TOPOLITE_GUISCENE_BASE_H
