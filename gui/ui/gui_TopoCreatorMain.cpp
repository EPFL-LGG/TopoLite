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

#include "gui_SliderFloat.h"
#include "gui_SliderInt.h"
#include "gui_CheckBoxBool.h"

#include "Interlocking/ContactGraph.h"
#include "Interlocking/InterlockingSolver_Clp.h"

class TopoLiteApplication : public nanogui::Screen{
public:
    TopoLiteApplication() : nanogui::Screen(nanogui::Vector2i(1024, 768), "TopoCreator")
    {
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

        menu_window = new nanogui::Window(this, "Menu");
        menu_window->set_position(nanogui::Vector2i(15, 15));
        menu_window->set_fixed_width(200);
        menu_window->set_layout(new nanogui::GroupLayout());

        new nanogui::Label(menu_window, "File", "sans-bold");
        nanogui::Button *open_button = new nanogui::Button(menu_window, "open");
        open_button->set_callback([&](){
            load_from_file(nanogui::file_dialog({ {"xml", "XML Portable"}, {"json", "JSON File"} }, false));
            return true;
        });
        nanogui::Button *save_button = new nanogui::Button(menu_window, "save");
        save_button->set_callback([&](){
            topo_manager->write_to_json(nanogui::file_dialog({ {"json", "JSON File"} }, true));
            return true;
        });

        new nanogui::Label(menu_window, "Settings", "sans-bold");
        init_scene_visible_options();
        init_parameter_options();

        perform_layout();

    }

    void init_scene_visible_options()
    {
        nanogui::PopupButton *popup_btn = new nanogui::PopupButton(menu_window, "Scene");
        nanogui::Popup *popup = popup_btn->popup();
        popup->set_layout(new nanogui::GroupLayout());

        new nanogui::Label(popup, "Rendering Setting", "sans-bold");
        nanogui::CheckBox *checkbox = new nanogui::CheckBox(popup, "cross mesh");
        checkbox->set_checked(false);
        checkbox->set_callback([&](bool check){
            if(main_canvas && main_canvas->scene && main_canvas->scene->objects.size() > 0){
                main_canvas->scene->objects[0]->visible = check;
                if(check) topo_manager->set_update_list_true({"update_cross_mesh"});
            }
            return check;
        });

        checkbox = new nanogui::CheckBox(popup, "augmented vectors");
        checkbox->set_checked(false);
        checkbox->set_callback([&](bool check)
        {
            if(main_canvas && main_canvas->scene && main_canvas->scene->objects.size() > 1) {
                main_canvas->scene->objects[1]->visible = check;
                if (check) topo_manager->set_update_list_true({"update_augmented_vectors"});
            }
            return check;
        });

        checkbox = new nanogui::CheckBox(popup, "struc");
        checkbox->set_checked(true);
        checkbox->set_callback([&](bool check)
        {
            if(main_canvas && main_canvas->scene && main_canvas->scene->objects.size() > 2) {
                main_canvas->scene->objects[2]->visible = check;
                if(check) topo_manager->set_update_list_true({"update_struc"});
            }
            return check;
        });
    }

    void init_parameter_options()
    {
        nanogui::PopupButton *parameter_pop = new nanogui::PopupButton(menu_window, "Parameters");
        nanogui::Popup *popup = parameter_pop->popup();
        popup->set_layout(new nanogui::GroupLayout());
        popup->set_fixed_width(250);

        new nanogui::Label(popup, "Parameters Setting", "sans-bold");

        if(topo_manager == nullptr) return;

        for(size_t id = 0; id < topo_manager->visible_var_nameList.size(); id++)
        {
            std::string name = topo_manager->visible_var_nameList[id];
            InputVar *var = topo_manager->iodata->varList->find(name);
            if(var)
            {
                if(var->var_gui_type == GUI_SLIDERFLOAT)
                {
                    parameter_objects.push_back(new gui_SliderFloat((InputVarFloat *)var, popup));
                }
                else if(var->var_gui_type == GUI_SLIDERINT){
                    parameter_objects.push_back(new gui_SliderInt((InputVarInt *)var, popup));
                }
                else if(var->var_gui_type == GUI_CHECKBOX){
                    parameter_objects.push_back(new gui_CheckBoxBool((InputVarBool *)var, popup));
                }
            }
        }
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

            if(has_load_successful)
            {
                for(int id = 0; id < topo_manager->visible_var_nameList.size(); id++){
                    std::string name = topo_manager->visible_var_nameList[id];
                    parameter_objects[id]->var = topo_manager->iodata->varList->find(name);
                    parameter_objects[id]->update_gui();
                }

                topo_manager->init_main_canvas();
                topo_manager->init_pattern_canvas();

            }
        }
    }

    void draw(NVGcontext *ctx) override {
        /* Draw the user interface */
        Screen::draw(ctx);
        if(topo_manager)
        {
            topo_manager->update();
        }
    }



private:
    nanogui::ref<gui_Arcball_Canvas> main_canvas;
    nanogui::ref<gui_2D_Canvas> pattern_canvas;
    shared_ptr<gui_TopoManager> topo_manager;
    nanogui::Window *menu_window;
    vector<nanogui::ref<gui_ParameterObject>> parameter_objects;
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
