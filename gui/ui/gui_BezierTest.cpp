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

#include "gui_Arcball_Canvas.h"
#include "gui_2D_Canvas.h"
#include "gui_TopoManager.h"
#include "gui_Lines.h"

#include "gui_SliderFloat.h"
#include "gui_SliderInt.h"
#include "gui_CheckBoxBool.h"

#include "Interlocking/ContactGraph.h"
#include "Interlocking/InterlockingSolver_Clp.h"
class BezierApplication : public nanogui::Screen {
public:
    BezierApplication() : nanogui::Screen(nanogui::Vector2i(1024, 768), "TopoCreator") {
        //main canvas
        inc_ref();
        perform_layout();

        m_render_pass = new nanogui::RenderPass({ this });
        m_render_pass->set_clear_color(0, nanogui::Color(0.3f, 0.3f, 0.32f, 1.f));

        m_shader = new nanogui::Shader(
                m_render_pass,

                /* An identifying name */
                "a_simple_shader",
                R"(using namespace metal;
            struct VertexOut {
                float4 position [[position]];
            };

            vertex VertexOut vertex_main(const device packed_float3 *position,
                                         uint id [[vertex_id]]) {
                VertexOut vert;
                vert.position = float4(position[id], 1.f);
                return vert;
            })",

                /* Fragment shader */
                R"(using namespace metal;
            fragment float4 fragment_main(const constant float &intensity) {
                return float4(intensity);
            })"
            );

        uint32_t indices[3*2] = {
                0, 1, 2,
                3, 4, 5,
        };

        float positions[3*6] = {
                -.5f, -.5f, 0.f,
                .5f, -.5f, 0.f,
                .5f, .5f, 0.f,
                -.5f, -.5f, .5f,
                .5f, .5f, .5f,
                -.5f, .5f, .5f,
        };

        m_shader->set_buffer("indices", nanogui::VariableType::UInt32, {3*2}, indices);
        m_shader->set_buffer("position", nanogui::VariableType::Float32, {6, 3}, positions);
        m_shader->set_uniform("intensity", 0.5f);
    }

    virtual void draw(NVGcontext *ctx) {
        /* Draw the user interface */
        Screen::draw(ctx);
    }

    virtual void draw_contents()
    {
        m_render_pass->resize(framebuffer_size());
        m_render_pass->begin();

        m_shader->begin();
        m_shader->draw_array(nanogui::Shader::PrimitiveType::Triangle, 0, 6, true);
        m_shader->end();

        m_render_pass->end();
    }

public:
    nanogui::ref<nanogui::RenderPass> m_render_pass;
    nanogui::ref<nanogui::Shader> m_shader;
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
