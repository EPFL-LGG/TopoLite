//
// Created by ziqwang on 08.06.20.
//

#include <nanogui/opengl.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/texture.h>
#include <iostream>
#include <memory>

#include "Canvas/guiCanvas_3DArcball.h"
#include "guiShader_Lines.h"

#include "Controls/guiControls_SliderFloat.h"
#include "Controls/guiControls_SliderInt.h"
#include "Controls/guiControls_CheckBoxBool.h"

#include "Interlocking/ContactGraph.h"
#include "Interlocking/InterlockingSolver_Clp.h"

#include "Utility/Bezier.h"
class BezierApplication : public nanogui::Screen {
public:
    BezierApplication() : nanogui::Screen(nanogui::Vector2i(1024, 768), "TopoCreator") {
        //main canvas

        inc_ref();

        main_canvas = new guiCanvas_3DArcball(this);
        main_canvas->set_size(this->m_size);

        perform_layout();

        init_lines();
    }

    void init_lines(){
        main_canvas->init_render_pass();

        // Create a cubic bezier with 4 points. Visualized at https://www.desmos.com/calculator/fivneeogmh
        Bezier::Bezier<3> cubicBezier({ {0, 0}, {0, 0.5}, {1, 0.5}, {1, 0} });

        // Get coordinates on the curve from a value between 0 and 1 (values outside this range are also valid because of the way bezier curves are defined).

        vector<gui_LinesGroup<double>> linegroups;
        gui_LinesGroup<double> lg;
        lg.color = nanogui::Color(0, 0, 0, 255);
        int N = 20;
        for(int id = 0; id < N; id++)
        {
            Bezier::Point p1 = cubicBezier.valueAt(1.0 / N * id);
            Bezier::Point p2 = cubicBezier.valueAt(1.0 / N * (id + 1));
            lg.lines.push_back(Line<double>(Eigen::Vector3d(p1.x, p1.y, 0), Eigen::Vector3d(p2.x, p2.y, 0)));
        }
        linegroups.push_back(lg);
        shared_ptr<guiShader_Lines<double>> LinesObject = make_shared<guiShader_Lines<double>>(linegroups, 0.002, main_canvas->scene->render_pass);
        main_canvas->scene->objects.push_back(LinesObject);
        main_canvas->resize_arcball(m_size);
    }

    virtual void draw(NVGcontext *ctx) {
        /* Draw the user interface */
        Screen::draw(ctx);
    }

    virtual void draw_contents() {
        main_canvas->scene->update_time(1.0, 1.0);
        main_canvas->draw_contents();
    }

public:
    nanogui::ref<guiCanvas_3DArcball> main_canvas;
};

int main(int /* argc */, char ** /* argv */) {
    try {
        nanogui::init();

        /* scoped variables */ {
            nanogui::ref<BezierApplication> app = new BezierApplication();
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
