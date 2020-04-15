//
// Created by ziqwang on 08.04.20.
//

#ifndef TOPOLITE_GUI_POLYMESHLIST_H
#define TOPOLITE_GUI_POLYMESHLIST_H

#include "Mesh/PolyMesh.h"
#include "gui_RenderObject.h"
#include <nanogui/vector.h>

template<typename Scalar>
class gui_PolyMeshLists : public gui_RenderObject<Scalar>{
public:

    typedef weak_ptr<PolyMesh<Scalar>> wpPolyMesh;
    typedef shared_ptr<PolyMesh<Scalar>> pPolyMesh;
    typedef shared_ptr<_Polygon<Scalar>> pPolygon;
    typedef Matrix<Scalar, 3, 1> Vector3;


public:
    using gui_RenderObject<Scalar>::object_colors;
    using gui_RenderObject<Scalar>::object_center;
    vector<wpPolyMesh> meshLists;
    vector<Vector3> ani_translation;
    vector<Vector3> ani_center;
    vector<Vector3> ani_rotation;

public:
    using gui_RenderObject<Scalar>::state;

public:
    using gui_RenderObject<Scalar>::render_pass;
    using gui_RenderObject<Scalar>::shader;

public:  //buffers
    using gui_RenderObject<Scalar>::buffer_colors;
    using gui_RenderObject<Scalar>::buffer_positions;
    vector<float> buffer_barycentric;
    vector<float> buffer_translation;
    vector<float> buffer_rotation;
    vector<float> buffer_center;
    vector<int> buffer_objectindex;

public: // uniform
    using gui_RenderObject<Scalar>::varList;
    using gui_RenderObject<Scalar>::proj;
    using gui_RenderObject<Scalar>::model;
    using gui_RenderObject<Scalar>::view;
    using gui_RenderObject<Scalar>::eye;
    using gui_RenderObject<Scalar>::simtime;

public:
    gui_PolyMeshLists(const vector<pPolyMesh> &_meshLists,
                      nanogui::ref<nanogui::RenderPass> _render_pass)
                      : gui_RenderObject<Scalar>::gui_RenderObject(_render_pass){
        for(pPolyMesh mesh: _meshLists)
        {
            meshLists.push_back(mesh);
        }

        //all faces have random color
        for(int id = 0; id < meshLists.size(); id++)
        {
            object_colors.push_back(nanogui::Color(rand() % 255, rand() % 255, rand() % 255, 255));
        }
        ani_translation.resize(meshLists.size(), Vector3(0, 0, 0));
        ani_rotation.resize(meshLists.size(), Vector3(0, 0, 0));
        ani_center.resize(meshLists.size(), Vector3(0, 0, 0));


        varList->add((bool)true, "show_wireframe",  "");
        varList->add((bool)true, "show_face", "");
        state = Stop;
        initShader();
    }

    gui_PolyMeshLists(const vector<pPolyMesh> &_meshLists,
                 vector<nanogui::Color> _object_colors,
                 nanogui::ref<nanogui::RenderPass> _render_pass)
            : gui_RenderObject<Scalar>::gui_RenderObject(_render_pass){

        for(pPolyMesh mesh: _meshLists)
        {
            meshLists.push_back(mesh);
        }

        //if the size of the input buffer_colors matches the size of the meshlist
        if(_object_colors.size() == meshLists.size()){
            object_colors = _object_colors;
        }
        else{
            //all faces have random color
            for(int id = 0; id < meshLists.size(); id++)
            {
                object_colors.push_back(nanogui::Color(rand() % 255, rand() % 255, rand() % 255, 255));
            }
        }
        ani_translation.resize(meshLists.size(), Vector3(0, 0, 0));
        ani_rotation.resize(meshLists.size(), Vector3(0, 0, 0));
        ani_center.resize(meshLists.size(), Vector3(0, 0, 0));

        varList->add((bool)true, "show_wireframe",  "");
        varList->add((bool)true, "show_face", "");
        state = Stop;
        initShader();
    }

public:


    void initShader()
    {
        //read text from file
        std::ifstream file("shader/PolyMeshAnimation_vert.metal");
        std::string shader_vert((std::istreambuf_iterator<char>(file)),
                                std::istreambuf_iterator<char>());

        file = std::ifstream("shader/PolyMeshAnimation_frag.metal");
        string shader_frag((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());

        shader = new nanogui::Shader(render_pass, "PolyMeshShader", shader_vert, shader_frag, nanogui::Shader::BlendMode::None);

        update_buffer();
    }

    void update_buffer(){
        //init buffer_positions
        int num_vertices = 0;
        object_center = Eigen::Vector3d(0, 0, 0);
        buffer_positions.clear();
        buffer_barycentric.clear();

        buffer_colors.clear();
        buffer_translation.clear();
        buffer_rotation.clear();
        buffer_center.clear();

        for(size_t mID = 0; mID < meshLists.size(); mID++){
            for(int kd = 0; kd < 3; kd++){
                buffer_colors.push_back(object_colors[mID][kd]);
                buffer_translation.push_back(ani_translation[mID][kd]);
                buffer_rotation.push_back(ani_rotation[mID][kd]);
                buffer_center.push_back(ani_center[mID][kd]);
            }
        }

        for(size_t mID = 0; mID < meshLists.size(); mID++)
        {
            wpPolyMesh mesh = meshLists[mID];
            for(pPolygon polygon: mesh.lock()->polyList)
            {
                Vector3 face_center = polygon->center();
                for(size_t id = 0; id < polygon->vers.size(); id++)
                {
                    object_center = object_center + polygon->vers[id]->pos;
                    num_vertices ++;

                    Vector3 sta = polygon->vers[id]->pos;
                    Vector3 end = polygon->vers[(id + 1) % polygon->vers.size()]->pos;

                    buffer_positions.push_back(sta.x());
                    buffer_positions.push_back(sta.y());
                    buffer_positions.push_back(sta.z());

                    buffer_positions.push_back(end.x());
                    buffer_positions.push_back(end.y());
                    buffer_positions.push_back(end.z());

                    buffer_positions.push_back(face_center.x());
                    buffer_positions.push_back(face_center.y());
                    buffer_positions.push_back(face_center.z());

                    buffer_barycentric.push_back(1);
                    buffer_barycentric.push_back(0);
                    buffer_barycentric.push_back(polygon->at_boundary(id) ? 0 : 10);

                    buffer_barycentric.push_back(0);
                    buffer_barycentric.push_back(1);
                    buffer_barycentric.push_back(polygon->at_boundary(id) ? 0 : 10);

                    buffer_barycentric.push_back(0.5);
                    buffer_barycentric.push_back(0.5);
                    buffer_barycentric.push_back(10);

                    for(int kd = 0; kd < 3; kd++){
                        buffer_objectindex.push_back(mID);
                    }
                }
            }
        }
        object_center /= (float)num_vertices;

        shader->set_buffer("position", nanogui::VariableType::Float32, {buffer_positions.size() / 3, 3},  buffer_positions.data());
        shader->set_buffer("barycentric", nanogui::VariableType::Float32, {buffer_barycentric.size() / 3, 3}, buffer_barycentric.data());
        shader->set_buffer("color", nanogui::VariableType::Float32, {buffer_colors.size() / 3, 3}, buffer_colors.data());

        shader->set_buffer("translation", nanogui::VariableType::Float32, {buffer_translation.size() / 3, 3}, buffer_translation.data());
        shader->set_buffer("rotation", nanogui::VariableType::Float32, {buffer_rotation.size() / 3, 3}, buffer_rotation.data());
        shader->set_buffer("center", nanogui::VariableType::Float32, {buffer_center.size() / 3, 3}, buffer_center.data());

        shader->set_buffer("objectindex", nanogui::VariableType::Int32, {buffer_objectindex.size(), 1}, buffer_objectindex.data());

    }

    void update_uniform(){
        shader->set_uniform("show_wireframe", varList->template get<bool>("show_wireframe"));
        shader->set_uniform("show_face", varList->template get<bool>("show_face"));
        shader->set_uniform("mvp", this->toNanoguiMatrix(proj * view * model));
        shader->set_uniform("simtime", simtime);
    }
};

#endif //TOPOLITE_GUI_POLYMESHLIST_H
