//
// Created by ziqwang on 2019-11-26.
//

#ifndef TOPOLITE_NANOVIEWER_H
#define TOPOLITE_NANOVIEWER_H

#include <nanogui/opengl.h>
#include <nanogui/screen.h>
#include <nanogui/window.h>
#include <nanogui/layout.h>
#include <nanogui/label.h>
#include <nanogui/checkbox.h>
#include <nanogui/button.h>
#include <nanogui/toolbutton.h>
#include <nanogui/popupbutton.h>
#include <nanogui/combobox.h>
#include <nanogui/progressbar.h>
#include <nanogui/icons.h>
#include <nanogui/messagedialog.h>
#include <nanogui/textbox.h>
#include <nanogui/slider.h>
#include <nanogui/imagepanel.h>
#include <nanogui/imageview.h>
#include <nanogui/vscrollpanel.h>
#include <nanogui/colorwheel.h>
#include <nanogui/colorpicker.h>
#include <nanogui/graph.h>
#include <nanogui/tabwidget.h>
#include <nanogui/texture.h>
#include <nanogui/shader.h>
#include <nanogui/renderpass.h>
#include <enoki/transform.h>
#include <stb_image.h>

using namespace nanogui;


class NanoViewer : public Screen {
public:
    NanoViewer() : Screen(Vector2i(1024, 768), "TopoGUI")
    {
        inc_ref();
        Window *window = new Window(this, "TopoGUI");
        window->set_position(Vector2i(15, 15));
        window->set_layout(new GroupLayout());

        perform_layout();

        m_render_pass = new RenderPass({ this });
        Color background(0.3f, 0.3f, 0.32f, 1.f);
        m_render_pass->set_clear_color(0, background);

        m_shader = new Shader(
                m_render_pass,

                /* An identifying name */
                "a_simple_shader",

#if defined(NANOGUI_USE_OPENGL)
        R"(/* Vertex shader */
            #version 330
            uniform mat4 mvp;
            in vec3 position;
            void main() {
                gl_Position = mvp * vec4(position, 1.0);
            })",
            /* Fragment shader */
            R"(#version 330
            out vec4 color;
            uniform float intensity;
            void main() {
                color = vec4(vec3(intensity), 1.0);
            })"
#elif defined(NANOGUI_USE_GLES)
        R"(/* Vertex shader */
            precision highp float;
            uniform mat4 mvp;
            attribute vec3 position;
            void main() {
                gl_Position = mvp * vec4(position, 1.0);
            })",

            /* Fragment shader */
            R"(precision highp float;
            uniform float intensity;
            void main() {
                gl_FragColor = vec4(vec3(intensity), 1.0);
            })"
#elif defined(NANOGUI_USE_METAL)
        R"(using namespace metal;
            struct VertexOut {
                float4 position [[position]];
            };

            vertex VertexOut vertex_main(const device packed_float3 *position,
                                         constant float4x4 &mvp,
                                         uint id [[vertex_id]]) {
                VertexOut vert;
                vert.position = mvp * float4(position[id], 1.f);
                return vert;
            })",

            /* Fragment shader */
            R"(using namespace metal;
            fragment float4 fragment_main(const constant float &intensity) {
                return float4(intensity);
            })"
#endif
        );

        uint32_t indices[3*2] = {
                0, 1, 2,
                2, 3, 0
        };

        float positions[3*4] = {
                -1.f, -1.f, 0.f,
                1.f, -1.f, 0.f,
                1.f, 1.f, 0.f,
                -1.f, 1.f, 0.f
        };

        m_shader->set_buffer("indices", enoki::EnokiType::UInt32, 1, {3*2, 1, 1}, indices);
        m_shader->set_buffer("position", enoki::EnokiType::Float32, 2, {4, 3, 1}, positions);
        m_shader->set_uniform("intensity", 0.5f);
    }

    virtual void draw(NVGcontext *ctx) {
        /* Draw the user interface */
        Screen::draw(ctx);
    }

    virtual void draw_contents() {
        Matrix4f mvp = enoki::scale<Matrix4f>(Vector3f(
                (float) m_size.y() / (float) m_size.x() * 0.25f, 0.25f, 0.25f)) *
                       enoki::rotate<Matrix4f>(Vector3f(0, 0, 1), (float) glfwGetTime());

        m_shader->set_uniform("mvp", mvp);

        m_render_pass->resize(framebuffer_size());
        m_render_pass->begin();

        m_shader->begin();
        m_shader->draw_array(Shader::PrimitiveType::Triangle, 0, 6, true);
        m_shader->end();

        m_render_pass->end();
    }

private:
    ProgressBar *m_progress;
    ref<Shader> m_shader;
    ref<RenderPass> m_render_pass;
};


#endif //TOPOLITE_NANOVIEWER_H
