//
// Created by ziqwang on 06.06.20.
//

#ifndef TOPOLITE_GUI_SLIDERFLOAT_H
#define TOPOLITE_GUI_SLIDERFLOAT_H

#include <nanogui/widget.h>
#include <nanogui/window.h>
#include "IO/InputVar.h"
#include "gui_ParameterObject.h"
class gui_SliderFloat: public nanogui::Widget, public gui_ParameterObject{
public:
    nanogui::Slider *slider;
    nanogui::TextBox *text_box;
public:
    gui_SliderFloat(InputVarFloat *_var, nanogui::Window *window)
    :Widget(window), gui_ParameterObject(_var)
    {

        Widget *panel = new Widget(window);
        panel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                                 nanogui::Alignment::Middle, 0, 5));
        new nanogui::Label(panel, var->label, "sans-bold");
        slider = new nanogui::Slider(panel);
        text_box = new nanogui::TextBox(panel);

        slider->set_value(((InputVarFloat *) var)->value);
        slider->set_range({((InputVarFloat *) var)->bound.x(), ((InputVarFloat *) var)->bound.y()});
        text_box->set_value(float_to_string(((InputVarFloat *) var)->value, 3));
        text_box->set_fixed_size(nanogui::Vector2i(80, 20));

        slider->set_callback([&](float value)
        {
            ((InputVarFloat *) var)->update = true;
            ((InputVarFloat *) var)->value = value;
            text_box->set_value(float_to_string(value, 3));
        });
    }

    void update_gui() override {
        slider->set_value(((InputVarFloat *) var)->value);
        slider->set_range({((InputVarFloat *) var)->bound.x(), ((InputVarFloat *) var)->bound.y()});
        text_box->set_value(float_to_string(((InputVarFloat *) var)->value, 3));
    }

    std::string float_to_string(float a_value, int n){
        std::ostringstream out;
        out.precision(n);
        out << a_value;
        return out.str();
    }
};

#endif //TOPOLITE_GUI_SLIDERFLOAT_H
