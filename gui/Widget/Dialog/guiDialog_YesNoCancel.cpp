//
// Created by ziqwang on 09.06.20.
//

#include "guiDialog_YesNoCancel.h"

/*
    src/messagedialog.cpp -- Simple "OK" or "Yes/No"-style modal dialogs

    NanoGUI was developed by Wenzel Jakob <wenzel.jakob@epfl.ch>.
    The widget drawing code is based on the NanoVG demo application
    by Mikko Mononen.

    All rights reserved. Use of this source code is governed by a
    BSD-style license that can be found in the LICENSE.txt file.
*/

#include <nanogui/messagedialog.h>
#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/label.h>
#include <nanogui/icons.h>

guiDialog_YesNoCancel::guiDialog_YesNoCancel(Widget *parent, const std::string &title, const std::string &message, const std::function<void(int)> &callback)
: Window(parent, title) {

    if(callback) set_callback(callback);

    set_layout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,
            nanogui::Alignment::Middle, 10, 10));
    set_modal(true);

    Widget *panel1 = new Widget(this);
    panel1->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                              nanogui::Alignment::Middle, 10, 15));
    int icon = m_theme->m_message_question_icon;

    nanogui::Label *icon_label = new nanogui::Label(panel1, std::string(nanogui::utf8(icon).data()), "icons");
    icon_label->set_font_size(50);
    m_message_label = new nanogui::Label(panel1, message);
    m_message_label->set_fixed_width(200);

    nanogui::Widget *panel2 = new nanogui::Widget(this);
    panel2->set_layout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
                                     nanogui::Alignment::Middle, 0, 15));

    nanogui::Button *button = new nanogui::Button(panel2, "Yes", m_theme->m_message_primary_button_icon);
    button->set_callback([&] { if (m_callback) m_callback(0); dispose(); });

    button = new nanogui::Button(panel2, "No", m_theme->m_message_alt_button_icon);
    button->set_callback([&] { if (m_callback) m_callback(1); dispose(); });

    button = new nanogui::Button(panel2, "Cancel");
    button->set_callback([&] { if (m_callback) m_callback(2); dispose(); });

    center();
    request_focus();
}

