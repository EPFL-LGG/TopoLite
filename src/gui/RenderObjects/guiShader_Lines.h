//
// Created by ziqwang on 14.04.20.
//

#ifndef TOPOLITE_GUISHADER_LINES_H
#define TOPOLITE_GUISHADER_LINES_H

#include "guiShader_Base.h"
#include "Utility/GeometricPrimitives.h"
#include "nanogui/vector.h"
#include <fstream>
#include "shader/metal_shader.h"
#include "shader/opengl_shader.h"
template <typename Scalar>
class gui_LinesGroup{
public:
    typedef Matrix<Scalar, 3, 1> Vector3;
    vector<Line<Scalar>> lines;
    nanogui::Color color;
};

template <typename Scalar>
class guiShader_Lines : public guiShader_Base<Scalar>{
public:

    typedef Matrix<Scalar, 3, 1> Vector3;

public:
    using guiShader_Base<Scalar>::object_colors;
    using guiShader_Base<Scalar>::object_center;
    vector<Vector3> ani_translation;
    vector<Vector3> ani_center;
    vector<Vector3> ani_rotation;
    vector<gui_LinesGroup<Scalar>> linegroups;

public:
    using guiShader_Base<Scalar>::state;

public:
    using guiShader_Base<Scalar>::render_pass;
    vector<nanogui::ref<nanogui::Shader>> shaders;

public:  //buffers
    using guiShader_Base<Scalar>::buffer_colors;
    using guiShader_Base<Scalar>::buffer_positions;

    vector<float> buffer_lineprev;
    vector<float> buffer_linep1;
    vector<float> buffer_linep2;
    vector<float> buffer_linenext;

    vector<float> buffer_translation;
    vector<float> buffer_rotation;
    vector<float> buffer_center;


    vector<int> buffer_objectindex;

public: // uniform
    using guiShader_Base<Scalar>::simtime;
    using guiShader_Base<Scalar>::varList;
    using guiShader_Base<Scalar>::proj_mat;
    using guiShader_Base<Scalar>::model_mat;
    using guiShader_Base<Scalar>::view_mat;
    using guiShader_Base<Scalar>::eye;
    float line_width;

public:

    guiShader_Lines(const vector<gui_LinesGroup<Scalar>> &_linegroups,
                    float _line_width,
                    nanogui::ref<nanogui::RenderPass> _render_pass)
            : guiShader_Base<Scalar>::guiShader_Base(_render_pass), line_width(_line_width), linegroups(_linegroups)
    {
        for(gui_LinesGroup<Scalar> lg: linegroups)
        {
            object_colors.push_back(lg.color);
        }

        ani_translation.clear();
        ani_rotation.clear();
        ani_center.clear();
        ani_translation.resize(linegroups.size(), Vector3(0, 0, 0));
        ani_rotation.resize(linegroups.size(), Vector3(0, 0, 0));
        ani_center.resize(linegroups.size(), Vector3(0, 0, 0));

        state = Stop;
        initShader();
    }

    void update_line(const vector<gui_LinesGroup<Scalar>> &_linegroups){
        linegroups.clear();
        linegroups = _linegroups;

        object_colors.clear();
        for(gui_LinesGroup<Scalar> lg: linegroups)
        {
            object_colors.push_back(lg.color);
        }

        ani_translation.clear(); ani_translation.resize(linegroups.size(), Vector3(0, 0, 0));
        ani_rotation.clear(); ani_rotation.resize(linegroups.size(), Vector3(0, 0, 0));
        ani_center.clear(); ani_center.resize(linegroups.size(), Vector3(0, 0, 0));

        update_buffer();
    }

public:
    void initShader(){


#if defined(NANOGUI_USE_OPENGL)
        //read text from file
        std::string shader_vert = opengl_lines_vert;
        std::string shader_frag = opengl_lines_frag;

#elif defined(NANOGUI_USE_METAL)
        std::string shader_vert = metal_lines_vert;
        std::string shader_frag = metal_lines_frag;
#endif

        shaders.push_back(new nanogui::Shader(render_pass, "LinesShader", shader_vert, shader_frag, nanogui::Shader::BlendMode::None));
        shaders.push_back(new nanogui::Shader(render_pass, "LinesShader", shader_vert, shader_frag, nanogui::Shader::BlendMode::None));

        update_buffer();
    }

    void update_buffer() override
    {
        //init buffer_positions
        int num_vertices = 0;
        object_center = Eigen::Vector3d(0, 0, 0);

        buffer_translation.clear();
        buffer_rotation.clear();
        buffer_center.clear();

        buffer_positions.clear();
        buffer_colors.clear();

        buffer_linep1.clear();
        buffer_linep2.clear();

        buffer_objectindex.clear();


#if defined(NANOGUI_USE_METAL)
        for(size_t mID = 0; mID < linegroups.size(); mID++){
            for(int kd = 0; kd < 3; kd++){
                buffer_colors.push_back(object_colors[mID][kd]);
                buffer_translation.push_back(ani_translation[mID][kd]);
                buffer_rotation.push_back(ani_rotation[mID][kd]);
                buffer_center.push_back(ani_center[mID][kd]);
            }
        }
#endif

        for(size_t mID = 0; mID < linegroups.size(); mID++)
        {
            const gui_LinesGroup<Scalar> &lg = linegroups[mID];
            size_t lgsize = lg.lines.size();
            for(size_t lID = 0; lID < lgsize; lID++)
            {
                Line<Scalar> line = lg.lines[lID];
                object_center = object_center + line.point2 + line.point1;
                num_vertices += 2;
                {
                    buffer_positions.push_back(1);
                    buffer_positions.push_back(0);
                    buffer_positions.push_back(0);

                    buffer_positions.push_back(0);
                    buffer_positions.push_back(1);
                    buffer_positions.push_back(0);

                    buffer_positions.push_back(0);
                    buffer_positions.push_back(0);
                    buffer_positions.push_back(1);
                }

                for(int id = 0; id < 3; id++)
                {
                    for(int jd = 0; jd < 3; jd++)
                    {
                        buffer_linep1.push_back(line.point1[jd]);
                        buffer_linep2.push_back(line.point2[jd]);
                    }
                }

#if defined(NANOGUI_USE_OPENGL)
                for(int vID = 0; vID < 3; vID ++)
                {
                    for(int kd = 0; kd < 3; kd ++)
                    {
                        buffer_colors.push_back(object_colors[mID][kd]);
                        buffer_translation.push_back(ani_translation[mID][kd]);
                        buffer_rotation.push_back(ani_rotation[mID][kd]);
                        buffer_center.push_back(ani_center[mID][kd]);
                    }
                }
#elif defined(NANOGUI_USE_METAL)
                for(int id = 0; id < 3; id++)
                    buffer_objectindex.push_back(mID);
#endif
            }
        }
        object_center /= (float)num_vertices;

        for(int id = 0; id < 2; id++)
        {
            nanogui::ref<nanogui::Shader> shader = shaders[id];
            shader->set_buffer("position", nanogui::VariableType::Float32, {buffer_positions.size() / 3, 3},  buffer_positions.data());
            shader->set_buffer("color", nanogui::VariableType::Float32, {buffer_colors.size() / 3, 3}, buffer_colors.data());
            shader->set_buffer("linep1", nanogui::VariableType::Float32, {buffer_linep1.size() / 3, 3}, buffer_linep1.data());
            shader->set_buffer("linep2", nanogui::VariableType::Float32, {buffer_linep2.size() / 3, 3}, buffer_linep2.data());

            shader->set_buffer("translation", nanogui::VariableType::Float32, {buffer_translation.size() / 3, 3}, buffer_translation.data());
            shader->set_buffer("rotation", nanogui::VariableType::Float32, {buffer_rotation.size() / 3, 3}, buffer_rotation.data());
            shader->set_buffer("center", nanogui::VariableType::Float32, {buffer_center.size() / 3, 3}, buffer_center.data());

#if defined(NANOGUI_USE_METAL)
            shader->set_buffer("objectindex", nanogui::VariableType::Int32, {buffer_objectindex.size(), 1}, buffer_objectindex.data());
#endif
            shader->set_uniform("type", id);
            shader->set_uniform("linewidth", line_width);
        }
    }

    void update_uniform() override
    {
        for(auto shader: shaders)
        {
            shader->set_uniform("mvp",this->toNanoguiMatrix(proj_mat * view_mat * model_mat));
            shader->set_uniform("simtime", simtime);
        }
    }

    void draw_object() override {
        if(guiShader_Base<Scalar>::visible) {
            if (!buffer_positions.empty()) {
                update_uniform();
                for (auto shader: shaders) {
                    shader->begin();
                    shader->draw_array(nanogui::Shader::PrimitiveType::Triangle, 0, buffer_positions.size() / 3, false);
                    shader->end();
                }
            }
        }
    }
};


#endif //TOPOLITE_GUISHADER_LINES_H
