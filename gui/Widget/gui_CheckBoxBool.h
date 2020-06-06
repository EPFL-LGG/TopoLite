//
// Created by ziqwang on 06.06.20.
//

#ifndef TOPOLITE_GUI_CHECKBOXBOOL_H
#define TOPOLITE_GUI_CHECKBOXBOOL_H

#include <nanogui/widget.h>
#include <nanogui/window.h>
#include "IO/InputVar.h"
#include "gui_ParameterObject.h"
class gui_CheckBoxBool: public nanogui::Widget, public gui_ParameterObject{
public:
    nanogui::CheckBox *checkBox;
public:
    gui_CheckBoxBool(InputVarBool *_var, nanogui::Window *window)
    :Widget(window), gui_ParameterObject(_var)
    {
        Widget *panel = new Widget(window);
        panel->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                                 nanogui::Alignment::Middle, 0, 5));
        new nanogui::Label(panel, var->label, "sans-bold");
        checkBox = new nanogui::CheckBox(panel, "");
        checkBox->set_checked(((InputVarBool *)var)->value);
        checkBox->set_callback([&](bool value) -> bool
        {
            var->update = true;
            ((InputVarBool *)var)->value = value;
            return value;
        });
    }

    void update_gui() override {
        checkBox->set_checked(((InputVarBool *)var)->value);
    }
};

#endif //TOPOLITE_GUI_CHECKBOXBOOL_H
