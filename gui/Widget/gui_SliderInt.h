//
// Created by ziqwang on 06.06.20.
//

#ifndef TOPOLITE_GUI_SLIDERINT_H
#define TOPOLITE_GUI_SLIDERINT_H

#include <nanogui/widget.h>
#include <nanogui/window.h>
#include "IO/InputVar.h"
#include "gui_ParameterObject.h"
class gui_SliderInt: public nanogui::Widget, public gui_ParameterObject{
public:
    nanogui::IntBox<int> *int_box;
public:
    gui_SliderInt(InputVarInt *_var, nanogui::Window *window)
    :Widget(window), gui_ParameterObject(_var)
    {

        Widget *panel = new Widget(window);
        panel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                                 nanogui::Alignment::Middle, 0, 5));
        new nanogui::Label(panel, var->label, "sans-bold");
        int_box = new nanogui::IntBox<int>(panel);

        int_box->set_editable(true);
        int_box->set_value(((InputVarInt *)var)->value);
        int_box->set_fixed_size(nanogui::Vector2i(60, 20));
        int_box->set_default_value(std::to_string(((InputVarInt *)var)->value));
        int_box->set_format("[1-9][0-9]*");
        int_box->set_spinnable(true);
        int_box->set_min_value(((InputVarInt *)var)->bound.x());
        int_box->set_max_value(((InputVarInt *)var)->bound.y());
        int_box->set_value_increment(1);

        int_box->set_callback([&](int value)
        {
            ((InputVarInt *)var)->update = true;
            ((InputVarInt *)var)->value = value;
            int_box->set_value(value);
        });
    }

    void update_gui() override {
        int_box->set_value(((InputVarInt *)var)->value);
        int_box->set_min_value(((InputVarInt *)var)->bound.x());
        int_box->set_max_value(((InputVarInt *)var)->bound.y());
    }
};

#endif //TOPOLITE_GUI_SLIDERINT_H
