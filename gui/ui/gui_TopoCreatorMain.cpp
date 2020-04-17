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

#include <nanogui/opengl.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/texture.h>
#include <iostream>
#include <memory>

#include "gui_ArcballScene.h"
#include "gui_LoadScene.h"
#include "gui_Lines.h"

#include "Interlocking/ContactGraph.h"
#include "Interlocking/InterlockingSolver_Clp.h"

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
//            scene->objects[1]->visible = check;
//            return check;
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
        loader.loadminimalsurface();
        refresh_trackball_center();
    }

    virtual void draw(NVGcontext *ctx) {
        /* Draw the user interface */
        Screen::draw(ctx);
    }

    virtual void draw_contents() {
        gui_ArcballScene::draw_contents();
    }
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
