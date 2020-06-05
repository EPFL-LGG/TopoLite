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

#include "gui_Arcball_Canvas.h"
#include "gui_2D_Canvas.h"
#include "gui_TopoManager.h"
#include "gui_Lines.h"

#include "Interlocking/ContactGraph.h"
#include "Interlocking/InterlockingSolver_Clp.h"

class TopoLiteApplication : public nanogui::Screen{
public:
    TopoLiteApplication() : nanogui::Screen(nanogui::Vector2i(1024, 768), "TopoCreator")
    {
        nanogui::Window *window;

        //main canvas
        main_canvas = new gui_Arcball_Canvas(this);
        main_canvas->set_size(this->m_size);
        
        //pattern canvas
        pattern_canvas = new gui_2D_Canvas(this);
        int min_wh = std::min(m_size.x() / 2.5, m_size.y() / 2.5);
        pattern_canvas->set_position(nanogui::Vector2i(m_size.x() - min_wh, m_size.y() - min_wh));
        pattern_canvas->set_size(nanogui::Vector2i(min_wh, min_wh));

        //manager
        topo_manager = make_shared<gui_TopoManager>(main_canvas, pattern_canvas);
    
        window = new nanogui::Window(this, "Menu");
        window->set_position(nanogui::Vector2i(15, 15));
        window->set_layout(new nanogui::GroupLayout());

        new nanogui::Label(window, "File", "sans-bold");
        nanogui::Button *open_button = new nanogui::Button(window, "open");
        open_button->set_callback([&](){
            load_from_file(nanogui::file_dialog({ {"xml", "XML Portable"}, {"json", "JSON File"} }, false));
            return true;
        });
        nanogui::Button *save_button = new nanogui::Button(window, "save");
        save_button->set_callback([&](){
            topo_manager->write_to_json(nanogui::file_dialog({ {"json", "JSON File"} }, true));
            return true;
        });


        new nanogui::Label(window, "Rendering Settings", "sans-bold");
        nanogui::CheckBox *checkbox = new nanogui::CheckBox(window, "wireframe");
        checkbox->set_checked(true);
        checkbox->set_callback([&](bool check){
            main_canvas->scene->objects[0]->varList->add(check, "show_wireframe", "");
            return check;
        });

        checkbox = new nanogui::CheckBox(window, "faces");
        checkbox->set_checked(true);
        checkbox->set_callback([&](bool check){
            main_canvas->scene->objects[0]->varList->add(check, "show_face", "");
            return check;
        });

        checkbox = new nanogui::CheckBox(window, "augmented vectors");
        checkbox->set_checked(false);
        checkbox->set_callback([&](bool check){
            main_canvas->scene->objects[1]->visible = check;
            topo_manager->set_update_list_true({"update_augmented_vectors"});
            return check;
        });

        checkbox = new nanogui::CheckBox(window, "struc");
        checkbox->set_checked(true);
        checkbox->set_callback([&](bool check){
            main_canvas->scene->objects[2]->visible = check;
            topo_manager->set_update_list_true({"update_struc"});
            return check;
        });

        
        perform_layout();
    }

    /********************************************************************************************
     *
     *                              Key Board and Mouse
     *
     ********************************************************************************************/
    bool keyboard_event(int key, int scancode, int action, int modifiers) override
    {
        if(main_canvas->keyboard_event(key, scancode, action, modifiers)){
            return true;
        }
        return false;
    }
    
    bool resize_event(const nanogui::Vector2i& size) override{

        int min_wh = std::min(size.x() / 2.5, size.y() / 2.5);
        pattern_canvas->set_position(nanogui::Vector2i(size.x() - min_wh, size.y() - min_wh));
        pattern_canvas->set_size(nanogui::Vector2i(min_wh, min_wh));
        pattern_canvas->resize_arcball(size);
        
        main_canvas->set_size(size);
        main_canvas->resize_arcball(size);
        
        m_redraw = true;
        draw_all();
        return true;
    }

    /********************************************************************************************
    *
    *                              Shader and Render Pass
    *
    ********************************************************************************************/


    void load_from_file(std::filesystem::path path)
    {

        if(topo_manager)
        {
            bool has_load_successful = false;
            if(path.extension() == ".json")
            {
                has_load_successful = topo_manager->load_from_jsonfile(path.string());
            }
            else if(path.extension() == ".xml")
            {
                has_load_successful = topo_manager->load_from_xmlfile(path.string());
            }

            if(has_load_successful){
                topo_manager->init_main_canvas();
                topo_manager->init_pattern_canvas();
            }
        }
    }

    void draw(NVGcontext *ctx) override {
        /* Draw the user interface */
        Screen::draw(ctx);
        if(topo_manager)topo_manager->update();
    }
    
private:
    nanogui::ref<gui_Arcball_Canvas> main_canvas;
    nanogui::ref<gui_2D_Canvas> pattern_canvas;
    shared_ptr<gui_TopoManager> topo_manager;
};

int main(int /* argc */, char ** /* argv */) {
    try {
        nanogui::init();

        /* scoped variables */ {
            nanogui::ref<TopoLiteApplication> app = new TopoLiteApplication();
            app->dec_ref();
            app->draw_all();
            app->set_visible(true);
            nanogui::mainloop(1 / 120.f * 1000);
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
