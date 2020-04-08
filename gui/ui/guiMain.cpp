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
#include "../Mesh/gui_PolyMeshLists.h"
#include "Interlocking/ContactGraph.h"
#include "Interlocking/InterlockingSolver_Clp.h"
#include "CrossMesh/PatternCreator.h"

class ExampleApplication : public nanogui::Screen {
public:
    ExampleApplication() : Screen(nanogui::Vector2i(1024, 768), "TopoLite GUI", false)
    {
        inc_ref();
        nanogui::Window *window = new nanogui::Window(this, "Menu");
        window->set_position(nanogui::Vector2i(15, 15));
        window->set_layout(new nanogui::GroupLayout());

        new nanogui::Label(window, "Rendering Settings", "sans-bold");
        nanogui::CheckBox *checkbox = new nanogui::CheckBox(window, "wireframe");
        checkbox->set_checked(true);
        checkbox->set_callback([&](bool check){
            if(polyMeshLists != nullptr){
                polyMeshLists->wireframe = check;
            }
            return check;
        });

        new nanogui::Label(window, "Animation Control", "sans-bold");
        Widget *tools = new Widget(window);
        tools->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                                 nanogui::Alignment::Middle, 0, 6));


        play = new nanogui::ToolButton(tools, FA_PLAY);
        pause = new nanogui::ToolButton(tools, FA_PAUSE);
        stop = new nanogui::ToolButton(tools, FA_STOP);

        play->set_callback([&](){
            prev_animate_state = gui_PolyMeshLists<double>::Run;
            if(polyMeshLists) polyMeshLists->state = prev_animate_state;
            return true;
        });

        pause->set_callback([&](){
            prev_animate_state = gui_PolyMeshLists<double>::Pause;
            if(polyMeshLists) polyMeshLists->state = prev_animate_state;
            return true;
        });

        stop->set_callback([&](){
            prev_animate_state = gui_PolyMeshLists<double>::Stop;
            if(polyMeshLists) polyMeshLists->state = prev_animate_state;
            return true;
        });

        new nanogui::Label(window, "speed");
        Widget *panel = new Widget(window);
        panel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                                 nanogui::Alignment::Middle, 0, 20));

        nanogui::Slider *slider = new nanogui::Slider(panel);
        slider->set_value(0.01f);
        slider->set_fixed_width(100);
        slider->set_range({0.001f, 0.1f});
        animation_speed = 0.01f;

        text_box_speed = new nanogui::TextBox(panel);
        text_box_speed->set_fixed_size(nanogui::Vector2i(60, 25));
        text_box_speed->set_value("0.01f");

        slider->set_callback([&](float value) {
            text_box_speed->set_value(std::to_string(value));
            animation_speed = value;
        });

        new nanogui::Label(window, "timeline");
        panel = new Widget(window);
        panel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                                 nanogui::Alignment::Middle, 0, 20));

        slider = new nanogui::Slider(panel);
        slider->set_value(0);
        slider->set_fixed_width(100);
        slider->set_range({0.0f, 0.01f});

        text_box_timeline = new nanogui::TextBox(panel);
        text_box_timeline->set_fixed_size(nanogui::Vector2i(60, 25));
        text_box_timeline->set_value("0");

        slider->set_callback([&](float value) {
            text_box_timeline->set_value(std::to_string(value));
            if(polyMeshLists) polyMeshLists->simtime = value;
        });


        perform_layout();
        refresh_trackball_center();

        init_render_pass();
        init_mesh();
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
            camera_.center = polyMeshLists ? polyMeshLists->mesh_center : Eigen::Vector3f(0, 0, 0);
            camera_.up = Eigen::Vector3f(0, 1, 0);
        }
        if(key == GLFW_KEY_2){
            camera_.arcball = Arcball();
            camera_.arcball.setSize(Eigen::Vector2i(m_size.x(), m_size.y()));
            camera_.modelZoom = 2;
            camera_.eye = Eigen::Vector3f(0, -5, 0);
            camera_.center = polyMeshLists ? polyMeshLists->mesh_center : Eigen::Vector3f(0, 0, 0);
            camera_.up = Eigen::Vector3f(0, 0, 1);
        }
        if(key == GLFW_KEY_3){
            camera_.arcball = Arcball();
            camera_.arcball.setSize(Eigen::Vector2i(m_size.x(), m_size.y()));
            camera_.modelZoom = 2;
            camera_.eye = Eigen::Vector3f(5, 0, 0);
            camera_.center = polyMeshLists ? polyMeshLists->mesh_center : Eigen::Vector3f(0, 0, 0);
            camera_.up = Eigen::Vector3f(0, 0, 1);
        }
        if(key == GLFW_KEY_SPACE && action == GLFW_RELEASE){
            if(prev_animate_state == gui_PolyMeshLists<double>::Run){
                prev_animate_state = gui_PolyMeshLists<double>::Pause;
                pause->set_pushed(true);
                play->set_pushed(false);
                if(polyMeshLists) polyMeshLists->state = prev_animate_state;
            }
            else if(prev_animate_state != gui_PolyMeshLists<double>::Run){
                prev_animate_state = gui_PolyMeshLists<double>::Run;
                play->set_pushed(true);
                pause->set_pushed(false);
                if(polyMeshLists) polyMeshLists->state = prev_animate_state;
            }
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
                if(polyMeshLists) mesh_center = polyMeshLists->mesh_center;

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
        if(polyMeshLists != nullptr) {
            camera_.modelTranslation = -polyMeshLists->mesh_center;
        }
        else{
            camera_.modelTranslation = Eigen::Vector3f(0, 0, 0);
        }
    }

    /********************************************************************************************
    *
    *                              Shader and Render Pass
    *
    ********************************************************************************************/

    void init_mesh()
    {
        shared_ptr<InputVarList> varList = make_shared<InputVarList>();
        InitVarLite(varList.get());

        //Read all Parts
        vector<shared_ptr<PolyMesh<double>>> meshLists;
        vector<bool> atboundary;
        vector<nanogui::Color> colors;

        {
            std::string part_filename = "data/TopoInterlock/XML/SphereA80_Quad_T0.08_data/PartGeometry/Boundary.obj";
            shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
            bool textureModel;
            if(polyMesh->readOBJModel(part_filename.c_str(), textureModel, false)){
                meshLists.push_back(polyMesh);
                atboundary.push_back(false);
                colors.push_back(nanogui::Color(255, 255, 255, 255));
            }
        }

        for(int id = 0; id <= 77; id++){
            char number[50];
            sprintf(number, "%02d.obj", id);
            std::string part_filename = "data/TopoInterlock/XML/SphereA80_Quad_T0.08_data/PartGeometry/Part_";
            part_filename += number;
            shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
            bool textureModel;
            if(polyMesh->readOBJModel(part_filename.c_str(), textureModel, false)){
                meshLists.push_back(polyMesh);
                atboundary.push_back(false);
                colors.push_back(nanogui::Color(255, 255, 255, 255));
            }
        }


        atboundary[0] = true;
        colors[0] = nanogui::Color(0, 0, 0, 0);
        //atboundary[1] = true;

        // construct the contact graph
        shared_ptr<ContactGraph<double>>graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(meshLists, atboundary, 1e-3);
        // solve the interlocking problem by using CLP library
        InterlockingSolver_Clp<double> solver(graph, varList);
        shared_ptr<typename InterlockingSolver<double>::InterlockingData> interlockData;
        solver.isTranslationalInterlocking(interlockData);

        polyMeshLists = make_shared<gui_PolyMeshLists<double>>(meshLists, colors, true, m_render_pass);

        for(int id = 0; id < polyMeshLists->object_translation.size(); id++){
            polyMeshLists->object_translation[id] = interlockData->traslation[id];
        }
        polyMeshLists->update_buffer();

//        shared_ptr<PolyMesh<double>> polyMesh = std::make_shared<PolyMesh<double>>(varList);
//        bool textureModel;
//        polyMesh->readOBJModel("data/TopoInterlock/XML/origin_data/origin_CrossMesh.obj", textureModel, true);
//        gui_polyMesh = make_shared<gui_PolyMesh<double>>(*polyMesh, true, m_render_pass);

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

        if(polyMeshLists == nullptr) return;

        if(!play->pushed() && !pause->pushed() && !stop->pushed()){
            if(prev_animate_state == gui_PolyMeshLists<double>::Run){
                polyMeshLists->state = gui_PolyMeshLists<double>::Stop;
            }
            if(prev_animate_state == gui_PolyMeshLists<double>::Pause){
                polyMeshLists->state = gui_PolyMeshLists<double>::Run;
                play->set_pushed(true);
            }
            prev_animate_state = polyMeshLists->state;
        }

        Eigen::Matrix4f mvp = proj * view * model;
        polyMeshLists->mvp = mvp;

        //polyMeshLists->updateTime((float)glfwGetTime(), animation_speed);


        /* MVP uniforms */
        polyMeshLists->updateUniforms();
        m_render_pass->begin();
        polyMeshLists->shader->begin();
        polyMeshLists->shader->draw_array(nanogui::Shader::PrimitiveType::Triangle, 0, polyMeshLists->positions.size() / 3, false);
        polyMeshLists->shader->end();
        m_render_pass->end();
    }

private:
    //shader and render pass
    shared_ptr<gui_PolyMeshLists<double>> polyMeshLists;
    nanogui::ref<nanogui::RenderPass> m_render_pass;
    int num_faces_invisible = 0;

private:

    nanogui::ToolButton *play, *pause, *stop;
    nanogui::TextBox *text_box_speed, *text_box_timeline;
    gui_PolyMeshLists<double>::AnimationState prev_animate_state;
    float animation_speed = 0.1;

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
