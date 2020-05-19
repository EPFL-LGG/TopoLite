//
// Created by ziqwang on 09.04.20.
//

#ifndef TOPOLITE_GUI_2D_CANVAS_H
#define TOPOLITE_GUI_2D_CANVAS_H

#include <nanogui/opengl.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/button.h>
#include <nanogui/toolbutton.h>
#include <nanogui/popupbutton.h>
#include <nanogui/combobox.h>
#include <nanogui/progressbar.h>
#include <nanogui/icons.h>
#include <nanogui/messagedialog.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/imagepanel.h>
#include <nanogui/imageview.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/colorwheel.h>
#include <nanogui/colorpicker.h>
#include <nanogui/graph.h>
#include <nanogui/tabwidget.h>
#include <nanogui/texture.h>
#include <nanogui/shader.h>
#include <nanogui/renderpass.h>
#include <iostream>
#include <memory>
#include <stb_image.h>
#include <cmath>
#include <filesystem>

#include "gui_2Dball.h"
#include "gui_SceneObject.h"

class gui_2D_Canvas : public nanogui::Canvas {
public:
    gui_2D_Canvas(Widget *parent) : Canvas(parent, 1)
    {
        scene = make_shared<gui_SceneObject<double>>();
        init_render_pass();
    }

    /********************************************************************************************
     *
     *                              Key Board and Mouse
     *
     ********************************************************************************************/
    virtual bool keyboard_event(int key, int scancode, int action, int modifiers) {
        if (Widget::keyboard_event(key, scancode, action, modifiers))
            return true;
        return false;
    }
    
    virtual bool scroll_event(const nanogui::Vector2i &p, const nanogui::Vector2f &rel) {
        if (!Widget::scroll_event(p, rel)) {
            camera_.zoom = std::max(0.1, camera_.zoom * (rel.y() > 0 ? 1.1 : 0.9));
        }
        return true;
    }

    void computeCameraMatrices(Eigen::Matrix4f &model,
                               Eigen::Matrix4f &view,
                               Eigen::Matrix4f &proj){

        view = lookAt(camera_.eye, camera_.center, camera_.up);

        float fH = std::tan(camera_.viewAngle / 360.0f * M_PI) * camera_.dnear;
        float fW = fH * (float) m_size.x() / (float) m_size.y();

        proj = frustum(-fW, fW, -fH, fH, camera_.dnear, camera_.dfar);
        model = camera_.arcball.matrix();

        model = scale(model, Eigen::Vector3f::Constant(camera_.zoom * camera_.modelZoom));
        model = translate(model, camera_.modelTranslation);
    }

    virtual bool mouse_motion_event(const nanogui::Vector2i &p, const nanogui::Vector2i &rel,
                                    int button, int modifiers){
        nanogui::Vector2i mouse_p = p - m_pos;
        if (!Widget::mouse_motion_event(p, rel, button, modifiers))
        {
            if (camera_.arcball.motion(Eigen::Vector2i(mouse_p.x(), mouse_p.y()))) {
                //
            } else if (translate_) {
                Eigen::Matrix4f model, view, proj;
                computeCameraMatrices(model, view, proj);

                Eigen::Vector3f mesh_center = scene->focus();

                float zval = project(mesh_center, view * model, proj, m_size).z();
                
                Eigen::Vector3f pos1 = unproject(
                        Eigen::Vector3f(mouse_p.x(), m_size.y() - mouse_p.y(), zval),
                        view * model, proj, m_size);
                Eigen::Vector3f pos0 = unproject(Eigen::Vector3f(translateStart_.x(), m_size.y() -
                                                                                      translateStart_.y(), zval), view * model, proj, m_size);
                camera_.modelTranslation = camera_.modelTranslation_start + (pos1 - pos0);
            }
        }
        return true;
    }

    virtual bool mouse_button_event(const nanogui::Vector2i &p, int button, bool down, int modifiers)
    {
        nanogui::Vector2i mouse_p = p - m_pos;
        if (!Widget::mouse_button_event(mouse_p, button, down, modifiers))
        {
            if (button == GLFW_MOUSE_BUTTON_1 && modifiers == 0) {
                camera_.arcball.button(Eigen::Vector2i(mouse_p.x(), mouse_p.y()), down);
            } else if (button == GLFW_MOUSE_BUTTON_2 ||
                       (button == GLFW_MOUSE_BUTTON_1 && modifiers == GLFW_MOD_SHIFT)) {
                camera_.modelTranslation_start = camera_.modelTranslation;
                translate_ = true;
                translateStart_ = Eigen::Vector2i(mouse_p.x(), mouse_p.y());
            }
        }
        if (button == GLFW_MOUSE_BUTTON_1 && !down) {
            camera_.arcball.button(Eigen::Vector2i(mouse_p.x(), mouse_p.y()), false);
        }
        if (!down) {
            translate_ = false;
        }
        return true;
    }
    
    void resize_arcball(nanogui::Vector2i size){
         camera_.arcball.setSize(Eigen::Vector2i(size.x(), size.y()));
    }

    void refresh_trackball_center() {
        // Re-center the mesh
        camera_.arcball = gui_2Dball(0.5);
        camera_.arcball.setSize(Eigen::Vector2i(m_size.x(), m_size.y()));
        camera_.modelZoom = 1;
        
        camera_.eye = Eigen::Vector3f(scene->focus().x(), scene->focus().y(), 1.5);
        camera_.center = Eigen::Vector3f(scene->focus().x(), scene->focus().y(), 0);
    }

    void init_render_pass(){
        #if defined(NANOGUI_USE_METAL)
        m_render_pass = render_pass();
        #elif defined(NANOGUI_USE_OPENGL)
          m_render_pass = render_pass();
        #endif

        m_render_pass->set_clear_color(0, nanogui::Color(0.9f, 0.9f, 0.9f, 1.f));
        m_render_pass->set_cull_mode(nanogui::RenderPass::CullMode::Disabled);
        m_render_pass->set_depth_test(nanogui::RenderPass::DepthTest::Less, true);

        scene->render_pass = m_render_pass;
    }

    std::string float_to_string(float a_value, int n){
        std::ostringstream out;
        out.precision(n);
        return out.str();
    }

    virtual void draw_contents() {
        scene->render_pass->resize(screen()->framebuffer_size());

        Eigen::Matrix4f model, view, proj;
        computeCameraMatrices(model, view, proj);

        scene->update_proj(proj);
        scene->update_view(view);
        scene->update_model(model);
        scene->update_eye(camera_.eye);

        /* MVP uniforms */
        scene->draw();
    }

public:
    //shader and render pass
    shared_ptr<gui_SceneObject<double>> scene;
    nanogui::ref<nanogui::RenderPass> m_render_pass;

private:
    //camera
    struct CameraParameters {
        gui_2Dball arcball;
        float zoom = 1.0f, viewAngle = 60.0f;
        float dnear = 0.1f, dfar = 100.0f;
        Eigen::Vector3f eye = Eigen::Vector3f(0.0f, 0.0f, 1.5f);
        // Eigen::Vector3f eye = Eigen::Vector3f(0.0f, 5.0f, 0.0f);
        Eigen::Vector3f center = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
        Eigen::Vector3f up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
        // Eigen::Vector3f up = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
        Eigen::Vector3f modelTranslation = Eigen::Vector3f::Zero();
        Eigen::Vector3f modelTranslation_start = Eigen::Vector3f::Zero();
        float modelZoom = 1.0f;
    } camera_;

    bool translate_ = false;
    Eigen::Vector2i translateStart_ = Eigen::Vector2i(0, 0);


};


#endif //TOPOLITE_GUI_ARCBALLVIEWER_H
