//
// Created by ziqwang on 06.06.20.
//

#ifndef TOPOLITE_GUICONTROLS_CHECKBOXBOOL_H
#define TOPOLITE_GUICONTROLS_CHECKBOXBOOL_H

#include <nanogui/widget.h>
#include <nanogui/window.h>
#include "IO/InputVar.h"
#include "guiControls_Base.h"
class guiControls_CheckBoxBool: public nanogui::Widget, public guiControls_Base{
public:
    nanogui::CheckBox *checkBox;
public:
    guiControls_CheckBoxBool(InputVarBool *_var, nanogui::Window *window)
    : Widget(window), guiControls_Base(_var)
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

#endif //TOPOLITE_GUICONTROLS_CHECKBOXBOOL_H
