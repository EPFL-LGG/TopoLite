/*
    src/example1.cpp -- C++ version of an example application that shows
    how to use the various widget classes. For a Python implementation, see
    '../python/example1.py'.

    NanoGUI was developed by Wenzel Jakob <wenzel.jakob@epfl.ch>.
    The widget drawing code is based on the NanoVG demo application
    by Mikko Mononen.

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include "Arcball.h"

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

#include "../Mesh/gui_PolyMesh.h"
#include "CrossMesh/PatternCreator.h"

class ExampleApplication : public nanogui::Screen {
public:
    ExampleApplication() : Screen(nanogui::Vector2i(1024, 768), "TopoLite GUI")
    {
        inc_ref();
        nanogui::Window *window = new nanogui::Window(this, "Menu");
        window->set_position(nanogui::Vector2i(15, 15));
        window->set_layout(new nanogui::GroupLayout());

        new nanogui::Label(window, "Invisible Faces Num:", "sans-bold");

        Widget *panel = new Widget(window);
        panel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                                 nanogui::Alignment::Middle, 0, 20));

        nanogui::Slider *slider = new nanogui::Slider(panel);
        slider->set_value(0);
        slider->set_fixed_width(150);
        slider->set_callback([&](float value) {
            this->num_faces_invisible = (int)value;
        });

        perform_layout();
        refresh_trackball_center();

        init_render_pass();
        init_mesh();

        slider->set_range({0, gui_polyMesh->size()});
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
                Eigen::Vector3f mesh_center(0, 0, 0);
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
        camera_.modelTranslation = -Eigen::Vector3f(0, 0, 0);
    }

    /********************************************************************************************
    *
    *                              Shader and Render Pass
    *
    ********************************************************************************************/

    nanogui::Matrix4f toNanoguiMatrix(Eigen::Matrix4f mat){
        nanogui::Matrix4f nanomat;
        for(int id = 0; id < 4; id++){
            for(int jd = 0; jd < 4; jd++){
                nanomat.m[id][jd] = mat(jd, id);
            }
        }
        return nanomat;
    }

    void init_mesh()
    {
        shared_ptr<InputVarList> varList = make_shared<InputVarList>();
        InitVarLite(varList.get());

        shared_ptr<PolyMesh<double>> polyMesh = std::make_shared<PolyMesh<double>>(varList);
        bool textureModel;
        polyMesh->readOBJModel("data/TopoInterlock/XML/origin_data/origin_CrossMesh.obj", textureModel, true);
        gui_polyMesh = make_shared<gui_PolyMesh<double>>(*polyMesh, true, m_render_pass);

//        PatternCreator<double> patternCreator(varList);
//        PatternCreator<double>::pCrossMesh crossMesh;
//        patternCreator.create2DPattern(CROSS_SQUARE, 10, crossMesh);
//        gui_polyMesh = make_shared<gui_PolyMesh<double>>(*(PolyMesh<double> *)crossMesh.get(), true, m_render_pass);
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
    }

    virtual void draw(NVGcontext *ctx) {
        /* Draw the user interface */
        Screen::draw(ctx);
    }

    virtual void draw_contents() {
        Eigen::Matrix4f model, view, proj;
        computeCameraMatrices(model, view, proj);

        Eigen::Matrix4f mvp = proj * view * model;

        /* MVP uniforms */
        int face_index = std::max((int)gui_polyMesh->size() - num_faces_invisible - 1, (int)0);

        gui_polyMesh->shader->set_uniform("mvp", toNanoguiMatrix(mvp));
        m_render_pass->begin();
        gui_polyMesh->shader->begin();
        gui_polyMesh->shader->draw_array(nanogui::Shader::PrimitiveType::Triangle, 0, gui_polyMesh->face_index_in_position[face_index] / 3, false);
        gui_polyMesh->shader->end();
        m_render_pass->end();
    }

private:
    //shader and render pass
    shared_ptr<gui_PolyMesh<double>> gui_polyMesh;
    nanogui::ref<nanogui::RenderPass> m_render_pass;
    int num_faces_invisible = 0;

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

int main(int /* argc */, char ** /* argv */) {
    try {
        nanogui::init();

        /* scoped variables */ {
            nanogui::ref<ExampleApplication> app = new ExampleApplication();
            app->dec_ref();
            app->draw_all();
            app->set_visible(true);
            nanogui::mainloop(1 / 60.f * 1000);
        }

        nanogui::shutdown();
    } catch (const std::exception &e) {
        std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
#if defined(_WIN32)
        MessageBoxA(nullptr, error_msg.c_str(), NULL, MB_ICONERROR | MB_OK);
#else
        std::cerr << error_msg << std::endl;
#endif
        return -1;
    } catch (...) {
        std::cerr << "Caught an unknown error!" << std::endl;
    }

    return 0;
}
