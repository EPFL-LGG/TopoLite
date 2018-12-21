//!
// Created by ziqwang on 18.12.18.
//

#include "viewer.h"
int main(int /* argc */, char ** /* argv */) {
    try {
        nanogui::init();
        {
            nanogui::ref<Viewer> app = new Viewer();
            app->drawAll();
            app->setVisible(true);
            nanogui::mainloop();
        }

        nanogui::shutdown();
    } catch (const std::runtime_error &e) {
        std::string error_msg = std::string("Caught a fatal error: ") + std::string(e.what());
#if defined(_WIN32)
        MessageBoxA(nullptr, error_msg.c_str(), NULL, MB_ICONERROR | MB_OK);
#else
        std::cerr << error_msg << std::endl;
#endif
        return -1;
    }

    return 0;
}
