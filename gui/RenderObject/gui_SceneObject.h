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
            return objects[0]->object_center;
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

    void update_mvp(Eigen::Matrix4f mvp){
        for(shared_ptr<gui_RenderObject<Scalar>> object : objects){
            object->mvp = mvp;
        }
    }

    void draw(){
        /* MVP uniforms */
        render_pass->begin();
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
