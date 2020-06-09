//
// Created by ziqwang on 09.06.20.
//

#ifndef TOPOLITE_GUIDIALOG_YESNOCANCEL_H
#define TOPOLITE_GUIDIALOG_YESNOCANCEL_H
#pragma once

#include <nanogui/window.h>

/**
 * \class gioDialog_YesNoCancel
 *
 * \brief "Yes/No/Cancel" Diaglog
 */
class guiDialog_YesNoCancel : public nanogui::Window {
public:

public:
    guiDialog_YesNoCancel(nanogui::Widget *parent,
            const std::string &title = "Untitled", const std::string &message = "Message",
            const std::function<void(int)> &callback = nullptr);

    nanogui::Label *message_label() { return m_message_label; }
    const nanogui::Label *message_label() const { return m_message_label; }

    std::function<void(int)> callback() const { return m_callback; }
    void set_callback(const std::function<void(int)> &callback) { m_callback = callback; }
protected:
    std::function<void(int)> m_callback;
    nanogui::Label *m_message_label;
};

#endif //TOPOLITE_GUIDIALOG_YESNOCANCEL_H
