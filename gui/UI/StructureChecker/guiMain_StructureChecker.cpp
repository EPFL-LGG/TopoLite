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

#include "gui_TrackBall3D.h"

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

#include "guiScene_Base.h"
#include "guiShader_PolyMeshes.h"

#include "Canvas/guiCanvas_3DArcball.h"
#include "guiManager_StructureChecker.h"
#include "Dialog/guiDialog_YesNoCancel.h"

class TopoLiteApplication : public nanogui::Screen{
public:
    TopoLiteApplication() : nanogui::Screen(nanogui::Vector2i(1024, 768), "StructureChecker")
    {
        //main canvas
        main_canvas = new guiCanvas_3DArcball(this);
        main_canvas->set_size(this->m_size);

        checker_manager = std::make_shared<guiManager_StructureChecker>(main_canvas);

        nanogui::Window *window = new nanogui::Window(this, "Menu");
        window->set_position(nanogui::Vector2i(15, 15));
        window->set_layout(new nanogui::GroupLayout());

        new nanogui::Label(window, "Action", "sans-bold");
        nanogui::Button *open_button = new nanogui::Button(window, "Open");
        open_button->set_fixed_width(150);
        open_button->set_callback([&](){
            objFilesList = nanogui::file_dialog({ {"obj", "Object Files"} }, false, true);
            if(!objFilesList.empty()){
                new guiDialog_YesNoCancel(main_canvas->parent(), "Settings", "Set the meshes to be boundary parts.", [&](int value){
                    if(value == 0)
                    {
                        checker_manager->load_meshList(objFilesList, true);
                    }
                    else if(value == 1){
                        checker_manager->load_meshList(objFilesList, false);
                    }
                });
            }
        });

        nanogui::Button *interlock_button = new nanogui::Button(window, "Check");
        interlock_button->set_fixed_width(150);
        interlock_button->set_callback([&](){
            if(checker_manager){
                if(checker_manager->check_interlocking()){
                    new nanogui::MessageDialog(this, nanogui::MessageDialog::Type::Information, "Result", "Interlocking (Rotation & Translation)");
                }
                else{
                    new nanogui::MessageDialog(this, nanogui::MessageDialog::Type::Information, "Result", "Not Interlocking (Rotation & Translation)");
                }
            }
        });

        nanogui::Button *clear_button = new nanogui::Button(window, "Clear");
        clear_button->set_fixed_width(150);
        clear_button->set_callback([&](){
            if(checker_manager){
                checker_manager->clear_scene();
            }
        });

        init_render_settings(window);

        init_animation_controls(window);

        perform_layout();
        main_canvas->refresh_trackball_center();
    }

    /********************************************************************************************
     *
     *                              Key Board and Mouse
     *
     ********************************************************************************************/
    virtual bool keyboard_event(int key, int scancode, int action, int modifiers)
    {
        if(main_canvas->keyboard_event(key, scancode, action, modifiers)){
           return true;
        }

        if(key == GLFW_KEY_SPACE && action == GLFW_RELEASE){
            if(prev_animate_state == Run){
                prev_animate_state = Pause;
                pause->set_pushed(true);
                play->set_pushed(false);
                main_canvas->scene->update_state(prev_animate_state);
            }
            else if(prev_animate_state != Run){
                prev_animate_state = Run;
                play->set_pushed(true);
                pause->set_pushed(false);
                main_canvas->scene->update_state(prev_animate_state);
            }
        }

        return false;
    }

    /********************************************************************************************
    *
    *                              Shader and Render Pass
    *
    ********************************************************************************************/


    void init_render_settings(nanogui::Window *window)
    {
        new nanogui::Label(window, "Settings", "sans-bold");
        nanogui::PopupButton *popup_btn = new nanogui::PopupButton(window, "Scene");
        popup_btn->set_fixed_width(150);
        nanogui::Popup *popup = popup_btn->popup();
        popup->set_layout(new nanogui::GroupLayout());

        new nanogui::Label(popup, "Rendering Setting", "sans-bold");
        nanogui::CheckBox *checkbox = new nanogui::CheckBox(popup, "Faces");
        checkbox->set_checked(true);
        checkbox->set_callback([&](bool check){
            if(main_canvas && main_canvas->scene && main_canvas->scene->objects.size() > 0){
                main_canvas->scene->objects[0]->visible = check;
            }
            return check;
        });

        checkbox = new nanogui::CheckBox(popup, "Wireframe");
        checkbox->set_checked(true);
        checkbox->set_callback([&](bool check)
        {
            if(main_canvas && main_canvas->scene && main_canvas->scene->objects.size() > 1){
                main_canvas->scene->objects[1]->visible = check;
            }
            return check;
        });

        checkbox = new nanogui::CheckBox(popup, "Contacts");
        checkbox->set_checked(false);
        checkbox->set_callback([&](bool check)
        {
            if(main_canvas && main_canvas->scene && main_canvas->scene->objects.size() > 2){
                main_canvas->scene->objects[2]->visible = check;
            }
            return check;
        });
    }

    void init_animation_controls(nanogui::Window *window)
    {
        nanogui::Window *animation_window = new nanogui::Window(window->parent(), "Animation Control");
        animation_window->set_position(nanogui::Vector2i(15, 300));

        animation_window->set_layout(new nanogui::GroupLayout());

        new nanogui::Label(animation_window, "Control Panel:");
        nanogui::Widget *tools = new nanogui::Widget(animation_window);
        tools->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                                 nanogui::Alignment::Middle, 0, 20));

        play = new nanogui::ToolButton(tools, FA_PLAY);
        pause = new nanogui::ToolButton(tools, FA_PAUSE);
        stop = new nanogui::ToolButton(tools, FA_STOP);

        play->set_callback([&](){
            prev_animate_state = Run;
            main_canvas->scene->update_state(prev_animate_state);
            return true;
        });

        pause->set_callback([&](){
            prev_animate_state = Pause;
            main_canvas->scene->update_state(prev_animate_state);
            return true;
        });

        stop->set_callback([&](){
            prev_animate_state = Stop;
            main_canvas->scene->update_state(prev_animate_state);
            return true;
        });

        new nanogui::Label(animation_window, "speed:");
        Widget *panel = new Widget(animation_window);
        panel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                                 nanogui::Alignment::Middle, 0, 20));

        speed_slider = new nanogui::Slider(panel);
        speed_slider->set_value(1 / minimum_time_one_unit);
        speed_slider->set_fixed_width(100);
        speed_slider->set_range({1 / maximum_time_one_unit, 1 / minimum_time_one_unit});
        animation_speed = 1 / minimum_time_one_unit;

        text_box_speed = new nanogui::TextBox(panel);
        text_box_speed->set_fixed_size(nanogui::Vector2i(60, 25));
        text_box_speed->set_value(main_canvas->float_to_string(1 / minimum_time_one_unit, 3));

        speed_slider->set_callback([&](float value) {
            text_box_speed->set_value(main_canvas->float_to_string(value, 3));
            animation_speed = value;
        });

        new nanogui::Label(animation_window, "timeline:");
        panel = new Widget(animation_window);
        panel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                                 nanogui::Alignment::Middle, 0, 20));

        timeline_slider = new nanogui::Slider(panel);
        timeline_slider->set_value(0);
        timeline_slider->set_fixed_width(100);
        timeline_slider->set_range({0.0f, maximum_time_one_unit});

        text_box_timeline = new nanogui::TextBox(panel);
        text_box_timeline->set_fixed_size(nanogui::Vector2i(60, 25));
        text_box_timeline->set_value("0");

        timeline_slider->set_callback([&](float value) {
            text_box_timeline->set_value(main_canvas->float_to_string(value, 3));
            main_canvas->scene->update_simtime(value / maximum_time_one_unit);
        });
    }

    void update_animation_controls(){
        if(!play->pushed() && !pause->pushed() && !stop->pushed()){
            if(prev_animate_state == Run){
                main_canvas->scene->update_state(Stop);
            }
            if(prev_animate_state == Pause){
                main_canvas->scene->update_state(Run);
                play->set_pushed(true);
            }
            main_canvas->scene->update_state(prev_animate_state);
        }

        main_canvas->scene->update_time((float)glfwGetTime(), animation_speed);

        if(timeline_slider->value() > maximum_time_one_unit && Run){
            main_canvas->scene->update_state(Pause);
            pause->set_pushed(true);
            play->set_pushed(false);
            prev_animate_state = Pause;
        }
        timeline_slider->set_value(main_canvas->scene->simtime * maximum_time_one_unit);
        text_box_timeline->set_value(main_canvas->float_to_string(main_canvas->scene->simtime * maximum_time_one_unit, 3) );
    }

    virtual void draw(NVGcontext *ctx) {
        /* Draw the user interface */
        Screen::draw(ctx);
    }



    virtual void draw_contents()
    {
        main_canvas->scene->render_pass->resize(framebuffer_size());
        update_animation_controls();
        main_canvas->draw_contents();
    }
private:
    nanogui::ToolButton *play, *pause, *stop;
    nanogui::TextBox *text_box_speed, *text_box_timeline;
    nanogui::Slider *timeline_slider, *speed_slider;
    vector<std::string> objFilesList;

public:
    //animation
    AnimationState prev_animate_state;
    nanogui::ref<guiCanvas_3DArcball> main_canvas;
    shared_ptr<guiManager_StructureChecker> checker_manager;
    float animation_speed = 0.1;
    float minimum_time_one_unit = 3;
    float maximum_time_one_unit = 20;
};

int main(int /* argc */, char ** /* argv */) {
    try {
        nanogui::init();

        /* scoped variables */ {
            nanogui::ref<TopoLiteApplication> app = new TopoLiteApplication();
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
