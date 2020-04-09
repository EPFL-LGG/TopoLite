//
// Created by ziqwang on 09.04.20.
//

#ifndef TOPOLITE_GUI_ARCBALLVIEWER_H
#define TOPOLITE_GUI_ARCBALLVIEWER_H

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


class gui_ArcballScene : public nanogui::Screen {
public:
    gui_ArcballScene() : Screen(nanogui::Vector2i(1024, 768), "TopoLite GUI", false)
    {
        inc_ref();
        init_render_pass();
    }

    /********************************************************************************************
     *
     *                              Key Board and Mouse
     *
     ********************************************************************************************/
    virtual bool keyboard_event(int key, int scancode, int action, int modifiers) {
        if (Screen::keyboard_event(key, scancode, action, modifiers))
            return true;
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
            set_visible(false);
            return true;
        }
        if(key == GLFW_KEY_1){
            camera_.arcball = Arcball();
            camera_.arcball.setSize(Eigen::Vector2i(m_size.x(), m_size.y()));
            camera_.modelZoom = 2;
            camera_.eye = Eigen::Vector3f(0, 0, 5);
            camera_.center = scene.focus();
            camera_.up = Eigen::Vector3f(0, 1, 0);
            return true;
        }
        if(key == GLFW_KEY_2){
            camera_.arcball = Arcball();
            camera_.arcball.setSize(Eigen::Vector2i(m_size.x(), m_size.y()));
            camera_.modelZoom = 2;
            camera_.eye = Eigen::Vector3f(0, -5, 0);
            camera_.center = scene.focus();
            camera_.up = Eigen::Vector3f(0, 0, 1);
            return true;
        }
        if(key == GLFW_KEY_3){
            camera_.arcball = Arcball();
            camera_.arcball.setSize(Eigen::Vector2i(m_size.x(), m_size.y()));
            camera_.modelZoom = 2;
            camera_.eye = Eigen::Vector3f(5, 0, 0);
            camera_.center = scene.focus();
            camera_.up = Eigen::Vector3f(0, 0, 1);
            return true;
        }
        return false;
    }

    virtual bool scroll_event(const nanogui::Vector2i &p, const nanogui::Vector2f &rel) {
        if (!Screen::scroll_event(p, rel)) {
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
                                    int button, int modifiers) {
        if (!Screen::mouse_motion_event(p, rel, button, modifiers)) {
            if (camera_.arcball.motion(Eigen::Vector2i(p.x(), p.y()))) {
                //
            } else if (translate_) {
                Eigen::Matrix4f model, view, proj;
                computeCameraMatrices(model, view, proj);

                Eigen::Vector3f mesh_center = scene.focus();

                float zval = project(mesh_center, view * model, proj, m_size).z();
                Eigen::Vector3f pos1 = unproject(
                        Eigen::Vector3f(p.x(), m_size.y() - p.y(), zval),
                        view * model, proj, m_size);
                Eigen::Vector3f pos0 = unproject(Eigen::Vector3f(translateStart_.x(), m_size.y() -
                                                                                      translateStart_.y(), zval), view * model, proj, m_size);
                camera_.modelTranslation = camera_.modelTranslation_start + (pos1 - pos0);
            }
        }
        return true;
    }

    virtual bool mouse_button_event(const nanogui::Vector2i &p, int button, bool down, int modifiers) {
        if (!Screen::mouse_button_event(p, button, down, modifiers)) {
            if (button == GLFW_MOUSE_BUTTON_1 && modifiers == 0) {
                camera_.arcball.button(Eigen::Vector2i(p.x(), p.y()), down);
            } else if (button == GLFW_MOUSE_BUTTON_2 ||
                       (button == GLFW_MOUSE_BUTTON_1 && modifiers == GLFW_MOD_SHIFT)) {
                camera_.modelTranslation_start = camera_.modelTranslation;
                translate_ = true;
                translateStart_ = Eigen::Vector2i(p.x(), p.y());
            }
        }
        if (button == GLFW_MOUSE_BUTTON_1 && !down) {
            camera_.arcball.button(Eigen::Vector2i(p.x(), p.y()), false);
        }
        if (!down) {
            translate_ = false;
        }
        return true;
    }

    void refresh_trackball_center() {

        // Re-center the mesh
        camera_.arcball = Arcball();
        camera_.arcball.setSize(Eigen::Vector2i(m_size.x(), m_size.y()));
        camera_.modelZoom = 2;
        camera_.modelTranslation = -scene.focus();
    }

    void init_render_pass(){
        nanogui::ref<nanogui::Texture> depth_stencil = new nanogui::Texture(
                nanogui::Texture::PixelFormat::DepthStencil,
                nanogui::Texture::ComponentFormat::Float32,
                framebuffer_size(),
                nanogui::Texture::InterpolationMode::Bilinear,
                nanogui::Texture::InterpolationMode::Bilinear,
                nanogui::Texture::WrapMode::ClampToEdge,
                1,
                nanogui::Texture::TextureFlags::RenderTarget
        );

        m_render_pass = new nanogui::RenderPass({ this }, depth_stencil);
        m_render_pass->set_clear_color(0, nanogui::Color(0.9f, 0.9f, 0.9f, 1.f));
        m_render_pass->set_cull_mode(nanogui::RenderPass::CullMode::Disabled);
        m_render_pass->set_depth_test(nanogui::RenderPass::DepthTest::Less, true);

        scene.render_pass = m_render_pass;
    }

    std::string float_to_string(float a_value, int n){
        std::ostringstream out;
        out.precision(n);
        out << std::fixed << a_value;
        return out.str();
    }

public:
    //shader and render pass
    gui_SceneObject<double> scene;
    nanogui::ref<nanogui::RenderPass> m_render_pass;

private:
    //camera
    struct CameraParameters {
        Arcball arcball;
        float zoom = 1.0f, viewAngle = 60.0f;
        float dnear = 0.1f, dfar = 100.0f;
        Eigen::Vector3f eye = Eigen::Vector3f(0.0f, 0.0f, 5.0f);
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
