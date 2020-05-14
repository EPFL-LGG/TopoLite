//
//
// Created by ziqwang on 09.04.20.
//

#ifndef TOPOLITE_GUI_SCENEOBJECT_H
#define TOPOLITE_GUI_SCENEOBJECT_H

#include "gui_RenderObject.h"
#include <memory>
#include <Eigen/Dense>

using std::shared_ptr;

template<typename Scalar>
class gui_SceneObject{
public:
    vector<shared_ptr<gui_RenderObject<Scalar>>> objects;
    nanogui::ref<nanogui::RenderPass> render_pass;
    AnimationState state;
    float simtime;

public:

    Eigen::Vector3f focus(){
        if(objects.size() >= 1){
            return Eigen::Vector3f(objects[0]->object_center.x(), objects[0]->object_center.y(), objects[0]->object_center.z());
        }
        else{
            return Eigen::Vector3f(0, 0, 0);
        }
    }

    void update_time(float curr_time, float speed = 0.1){
        for(shared_ptr<gui_RenderObject<Scalar>> object : objects){
            object->update_time(curr_time, speed);
            simtime = object->simtime;
        }
    }

    void update_simtime(float _simtime){
        for(shared_ptr<gui_RenderObject<Scalar>> object : objects){
            object->simtime = _simtime;
        }
        simtime = _simtime;
    }

    void update_state(AnimationState _state){
        for(shared_ptr<gui_RenderObject<Scalar>> object : objects){
            object->state = _state;
        }
        state = _state;
    }

    void update_proj(Eigen::Matrix4f proj){
        for(shared_ptr<gui_RenderObject<Scalar>> object : objects){
            object->proj = proj;
        }
    }

    void update_model(Eigen::Matrix4f model){
        for(shared_ptr<gui_RenderObject<Scalar>> object : objects){
            object->model = model;
        }
    }

    void update_view(Eigen::Matrix4f view){
        for(shared_ptr<gui_RenderObject<Scalar>> object : objects){
            object->view = view;
        }
    }

    void update_eye(Eigen::Vector3f eye){
        for(shared_ptr<gui_RenderObject<Scalar>> object : objects){
            object->eye = eye;
        }
    }

    void draw(){
        /* MVP uniforms */
        render_pass->begin();
#if defined(NANOGUI_USE_OPENGL)
        glClear(GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glDepthMask( true );
#endif
        for(shared_ptr<gui_RenderObject<Scalar>> object : objects){
            if(object->visible){
                object->update_uniform();
                object->shader->begin();
                object->shader->draw_array(nanogui::Shader::PrimitiveType::Triangle, 0, object->buffer_positions.size() / 3, false);
                object->shader->end();
            }
        }
        render_pass->end();
    }

    template<typename AttrType>
    void update_attr(string name, AttrType value)
    {
        for(shared_ptr<gui_RenderObject<Scalar>> object : objects){
            object->update_attr(name, value);
        }
    }
};

#endif //TOPOLITE_GUI_SCENEOBJECT_H
