//
// Created by ziqwang on 04.04.20.
//

#ifndef TOPOLITE_GUI_POLYMESH_H
#define TOPOLITE_GUI_POLYMESH_H

#include "TopoLite/Mesh/PolyMesh.h"
#include <nanogui/vector.h>
#include <fstream>

template<typename Scalar>
class gui_PolyMesh : public PolyMesh<Scalar>{
public:
    typedef shared_ptr<_Polygon<Scalar>> pPolygon;
    typedef shared_ptr<VPoint<Scalar>> pVertex;
    using PolyMesh<Scalar>::polyList;
    typedef Matrix<Scalar, 3, 1> Vector3;

public:

    vector<nanogui::Color> face_colors;

    bool wireframe;

    nanogui::ref<nanogui::Shader> shader;

    nanogui::ref<nanogui::RenderPass> render_pass;

    vector<float> positions;

    vector<float> barycentric;

    vector<float> color;

    vector<int> face_index_in_position;

public:
    gui_PolyMesh(const PolyMesh<Scalar> &polyMesh,
                 bool _wire,
                 nanogui::ref<nanogui::RenderPass> _render_pass)
    : PolyMesh<Scalar>(polyMesh), wireframe(_wire), render_pass(_render_pass)
    {
        //all faces have white color
        face_colors.resize(PolyMesh<Scalar>::size(), nanogui::Color(1, 1, 1, 1));
        initShader();
    }

    gui_PolyMesh(const PolyMesh<Scalar> &polyMesh,
                 vector<nanogui::Color> _face_colors,
                 bool _wire,
                 nanogui::ref<nanogui::RenderPass> _render_pass)
            : PolyMesh<Scalar>(polyMesh), wireframe(_wire), render_pass(_render_pass)
    {
        //if the size of the input colors matches the size of the mesh faces
        if(_face_colors.size() == PolyMesh<Scalar>::size()){
            face_colors = _face_colors;
        }
        else{
            //all faces have white color
            face_colors.resize(PolyMesh<Scalar>::size(), nanogui::Color(1, 1, 1, 1));
        }
        initShader();
    }

public:

    void initShader()
    {
        //read text from file
        std::ifstream file("shader/PolyMesh_vert.metal");
        std::string shader_vert((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());

        file = std::ifstream("shader/PolyMesh_frag.metal");
        string shader_frag((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());

        shader = new nanogui::Shader(render_pass, "PolyMeshShader", shader_vert, shader_frag, nanogui::Shader::BlendMode::None);

        //init positions
        for(pPolygon polygon: polyList)
        {
            Vector3 center = polygon->center();
            for(size_t id = 0; id < polygon->vers.size(); id++)
            {
                Vector3 sta = polygon->vers[id]->pos;
                Vector3 end = polygon->vers[(id + 1) % polygon->vers.size()]->pos;

                positions.push_back(sta.x());
                positions.push_back(sta.y());
                positions.push_back(sta.z());

                positions.push_back(end.x());
                positions.push_back(end.y());
                positions.push_back(end.z());

                positions.push_back(center.x());
                positions.push_back(center.y());
                positions.push_back(center.z());

                barycentric.push_back(1);
                barycentric.push_back(0);
                barycentric.push_back(0);

                barycentric.push_back(0);
                barycentric.push_back(1);
                barycentric.push_back(0);

                barycentric.push_back(0.5);
                barycentric.push_back(0.5);
                barycentric.push_back(10);
            }

            face_index_in_position.push_back(positions.size());
        }
        shader->set_buffer("position", nanogui::VariableType::Float32, {positions.size() / 3, 3},  &positions[0]);
        shader->set_buffer("barycentric", nanogui::VariableType::Float32, {barycentric.size() / 3, 3}, &barycentric[0]);
        shader->set_uniform("wireframe", wireframe);
    }

};


#endif //TOPOLITE_GUI_POLYMESH_H
