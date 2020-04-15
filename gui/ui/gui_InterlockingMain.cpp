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

#include "gui_Arcball.h"

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

#include "gui_SceneObject.h"
#include "gui_PolyMeshLists.h"

#include "gui_ArcballScene.h"
#include "gui_LoadScene.h"

class TopoLiteApplication : public gui_ArcballScene{
public:
    TopoLiteApplication() : gui_ArcballScene()
    {
        nanogui::Window *window = new nanogui::Window(this, "Menu");
        window->set_position(nanogui::Vector2i(15, 15));
        window->set_layout(new nanogui::GroupLayout());

        new nanogui::Label(window, "Rendering Settings", "sans-bold");
        nanogui::CheckBox *checkbox = new nanogui::CheckBox(window, "wireframe");
        checkbox->set_checked(true);
        checkbox->set_callback([&](bool check){
            scene->objects[0]->update_attr("show_wireframe", check);
            return check;
        });

        checkbox = new nanogui::CheckBox(window, "faces");
        checkbox->set_checked(true);
        checkbox->set_callback([&](bool check){
            scene->objects[0]->update_attr("show_face", check);
            return check;
        });

        checkbox = new nanogui::CheckBox(window, "contact");
        checkbox->set_checked(false);
        checkbox->set_callback([&](bool check){
            scene->objects[1]->visible = check;
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
            prev_animate_state = Run;
            scene->update_state(prev_animate_state);
            return true;
        });

        pause->set_callback([&](){
            prev_animate_state = Pause;
            scene->update_state(prev_animate_state);
            return true;
        });

        stop->set_callback([&](){
            prev_animate_state = Stop;
            scene->update_state(prev_animate_state);
            return true;
        });

        new nanogui::Label(window, "speed");
        Widget *panel = new Widget(window);
        panel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                                 nanogui::Alignment::Middle, 0, 20));

        speed_slider = new nanogui::Slider(panel);
        speed_slider->set_value(1 / minimum_time_one_unit);
        speed_slider->set_fixed_width(100);
        speed_slider->set_range({1 / maximum_time_one_unit, 1 / minimum_time_one_unit});
        animation_speed = 1 / minimum_time_one_unit;

        text_box_speed = new nanogui::TextBox(panel);
        text_box_speed->set_fixed_size(nanogui::Vector2i(60, 25));
        text_box_speed->set_value(float_to_string(1 / minimum_time_one_unit, 3));

        speed_slider->set_callback([&](float value) {
            text_box_speed->set_value(float_to_string(value, 3));
            animation_speed = value;
        });

        new nanogui::Label(window, "timeline");
        panel = new Widget(window);
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
            text_box_timeline->set_value(float_to_string(value, 3));
            scene->update_simtime(value / maximum_time_one_unit);
        });

        perform_layout();
        refresh_trackball_center();

        init_mesh();
    }

    /********************************************************************************************
     *
     *                              Key Board and Mouse
     *
     ********************************************************************************************/
    virtual bool keyboard_event(int key, int scancode, int action, int modifiers)
    {
        if(gui_ArcballScene::keyboard_event(key, scancode, action, modifiers)){
           return true;
        }

        if(key == GLFW_KEY_SPACE && action == GLFW_RELEASE){
            if(prev_animate_state == Run){
                prev_animate_state = Pause;
                pause->set_pushed(true);
                play->set_pushed(false);
                scene->update_state(prev_animate_state);
            }
            else if(prev_animate_state != Run){
                prev_animate_state = Run;
                play->set_pushed(true);
                pause->set_pushed(false);
                scene->update_state(prev_animate_state);
            }
        }

        return false;
    }

    /********************************************************************************************
    *
    *                              Shader and Render Pass
    *
    ********************************************************************************************/


    void init_mesh()
    {
        gui_LoadScene loader(scene);
        loader.loadAnia();
    }

    virtual void draw(NVGcontext *ctx) {
        /* Draw the user interface */
        Screen::draw(ctx);
    }

    virtual void draw_contents() {
        if(!play->pushed() && !pause->pushed() && !stop->pushed()){
            if(prev_animate_state == Run){
                scene->update_state(Stop);
            }
            if(prev_animate_state == Pause){
                scene->update_state(Run);
                play->set_pushed(true);
            }
            scene->update_state(prev_animate_state);
        }

        scene->update_time((float)glfwGetTime(), animation_speed);


        if(timeline_slider->value() > maximum_time_one_unit && Run){
            scene->update_state(Pause);
            pause->set_pushed(true);
            play->set_pushed(false);
            prev_animate_state = Pause;
        }

        timeline_slider->set_value(scene->simtime * maximum_time_one_unit);
        text_box_timeline->set_value(float_to_string(scene->simtime * maximum_time_one_unit, 3) );

        gui_ArcballScene::draw_contents();
    }
private:
    nanogui::ToolButton *play, *pause, *stop;
    nanogui::TextBox *text_box_speed, *text_box_timeline;
    nanogui::Slider *timeline_slider, *speed_slider;

public:
    //animation
    AnimationState prev_animate_state;
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
