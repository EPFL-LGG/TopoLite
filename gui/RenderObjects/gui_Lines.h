//
// Created by ziqwang on 14.04.20.
//

#ifndef TOPOLITE_GUI_LINES_H
#define TOPOLITE_GUI_LINES_H

#include "gui_RenderObject.h"
#include "Utility/GeometricPrimitives.h"
#include "nanogui/vector.h"
template <typename Scalar>
class gui_LinesGroup{
public:
    typedef Matrix<Scalar, 3, 1> Vector3;
    vector<Line<Scalar>> lines;
    nanogui::Color color;
    Vector3 ani_translation;
    Vector3 ani_center;
    Vector3 ani_rotation;
};

template <typename Scalar>
class gui_Lines : public gui_RenderObject<Scalar>{
public:

    typedef Matrix<Scalar, 3, 1> Vector3;

public:
    using gui_RenderObject<Scalar>::object_colors;
    using gui_RenderObject<Scalar>::object_center;
    vector<Vector3> ani_translation;
    vector<Vector3> ani_center;
    vector<Vector3> ani_rotation;
    vector<gui_LinesGroup<Scalar>> linegroups;

public:
    using gui_RenderObject<Scalar>::state;

public:
    using gui_RenderObject<Scalar>::render_pass;
    using gui_RenderObject<Scalar>::shader;

public:  //buffers
    using gui_RenderObject<Scalar>::buffer_colors;
    using gui_RenderObject<Scalar>::buffer_positions;

    vector<float> buffer_linesta;
    vector<float> buffer_linedrt;

    vector<float> buffer_translation;
    vector<float> buffer_rotation;
    vector<float> buffer_center;


    vector<int> buffer_objectindex;

public: // uniform
    using gui_RenderObject<Scalar>::simtime;
    using gui_RenderObject<Scalar>::varList;
    using gui_RenderObject<Scalar>::proj;
    using gui_RenderObject<Scalar>::model;
    using gui_RenderObject<Scalar>::view;
    using gui_RenderObject<Scalar>::eye;
    float line_width;

public:

    gui_Lines(  const vector<gui_LinesGroup<Scalar>> &_linegroups,
                float _line_width,
                nanogui::ref<nanogui::RenderPass> _render_pass)
            : gui_RenderObject<Scalar>::gui_RenderObject(_render_pass), line_width(_line_width), linegroups(_linegroups)
    {
        for(gui_LinesGroup<Scalar> lg: linegroups)
        {
            object_colors.push_back(lg.color);
            ani_translation.push_back(lg.ani_translation);
            ani_rotation.push_back(lg.ani_rotation);
            ani_center.push_back(lg.ani_center);
        }

        state = Stop;
        initShader();
    }

public:
    void initShader(){


#if defined(NANOGUI_USE_OPENGL)
        //read text from file
        std::ifstream file("shader/Lines.vert");
        std::string shader_vert((std::istreambuf_iterator<char>(file)),
                                std::istreambuf_iterator<char>());
        file = std::ifstream("shader/Lines.frag");
        string shader_frag((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
#elif defined(NANOGUI_USE_METAL)
        //read text from file
        std::ifstream file("shader/Lines_vert.metal");
        std::string shader_vert((std::istreambuf_iterator<char>(file)),
                                std::istreambuf_iterator<char>());

        file = std::ifstream("shader/Lines_frag.metal");
        string shader_frag((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
#endif

        shader = new nanogui::Shader(render_pass, "LinesShader", shader_vert, shader_frag, nanogui::Shader::BlendMode::None);

        update_buffer();
    }

    void update_buffer(){
        //init buffer_positions
        int num_vertices = 0;
        object_center = Eigen::Vector3d(0, 0, 0);
        buffer_positions.clear();

        buffer_colors.clear();
        buffer_translation.clear();
        buffer_rotation.clear();
        buffer_center.clear();

        for(size_t mID = 0; mID < linegroups.size(); mID++){
            for(int kd = 0; kd < 3; kd++){
                buffer_colors.push_back(object_colors[mID][kd]);
                buffer_translation.push_back(ani_translation[mID][kd]);
                buffer_rotation.push_back(ani_rotation[mID][kd]);
                buffer_center.push_back(ani_center[mID][kd]);
            }
        }

        for(size_t mID = 0; mID < linegroups.size(); mID++)
        {
            const gui_LinesGroup<Scalar> &lg = linegroups[mID];
            for(Line<Scalar> line: lg.lines)
            {
                object_center = object_center + line.point2 + line.point1;
                num_vertices += 2;
                Eigen::Vector3d drt = (line.point2 - line.point1).normalized();
                line.point1 = line.point1 - drt * line_width / 50;
                line.point2 = line.point2 + drt * line_width / 50;
                double length = (line.point2 - line.point1).norm();
                {
                    buffer_positions.push_back(0);
                    buffer_positions.push_back(line_width / 2);
                    buffer_positions.push_back(0);

                    buffer_positions.push_back(0);
                    buffer_positions.push_back(-line_width / 2);
                    buffer_positions.push_back(0);

                    buffer_positions.push_back(length);
                    buffer_positions.push_back(line_width / 2);
                    buffer_positions.push_back(0);
                }

                {
                    buffer_positions.push_back(0);
                    buffer_positions.push_back(-line_width / 2);
                    buffer_positions.push_back(0);

                    buffer_positions.push_back(length);
                    buffer_positions.push_back(-line_width / 2);
                    buffer_positions.push_back(0);

                    buffer_positions.push_back(length);
                    buffer_positions.push_back(line_width / 2);
                    buffer_positions.push_back(0);
                }

                for(int id = 0; id < 6; id++)
                {
                    for(int jd = 0; jd < 3; jd++)
                    {
                        buffer_linesta.push_back((line.point1)[jd]);
                        buffer_linedrt.push_back(drt[jd]);
                    }
                }

                for(int id = 0; id < 6; id++)
                    buffer_objectindex.push_back(mID);
            }
        }
        object_center /= (float)num_vertices;

        shader->set_buffer("position", nanogui::VariableType::Float32, {buffer_positions.size() / 3, 3},  buffer_positions.data());
        shader->set_buffer("color", nanogui::VariableType::Float32, {buffer_colors.size() / 3, 3}, buffer_colors.data());
        shader->set_buffer("linesta", nanogui::VariableType::Float32, {buffer_linesta.size() / 3, 3}, buffer_linesta.data());
        shader->set_buffer("linedrt", nanogui::VariableType::Float32, {buffer_linedrt.size() / 3, 3}, buffer_linedrt.data());

        shader->set_buffer("translation", nanogui::VariableType::Float32, {buffer_translation.size() / 3, 3}, buffer_translation.data());
        shader->set_buffer("rotation", nanogui::VariableType::Float32, {buffer_rotation.size() / 3, 3}, buffer_rotation.data());
        shader->set_buffer("center", nanogui::VariableType::Float32, {buffer_center.size() / 3, 3}, buffer_center.data());

        shader->set_buffer("objectindex", nanogui::VariableType::Int32, {buffer_objectindex.size(), 1}, buffer_objectindex.data());
    }

    void update_uniform(){
        shader->set_uniform("proj", this->toNanoguiMatrix(proj));
        shader->set_uniform("mv",this->toNanoguiMatrix(view * model));
        shader->set_uniform("simtime", simtime);
    }
};


#endif //TOPOLITE_GUI_LINES_H
