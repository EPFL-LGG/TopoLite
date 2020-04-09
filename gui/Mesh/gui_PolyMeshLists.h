//
// Created by ziqwang on 08.04.20.
//

#ifndef TOPOLITE_GUI_POLYMESHLIST_H
#define TOPOLITE_GUI_POLYMESHLIST_H

#include "Mesh/PolyMesh.h"
#include <nanogui/vector.h>

template<typename Scalar>
class gui_PolyMeshLists{
public:

    typedef weak_ptr<PolyMesh<Scalar>> wpPolyMesh;
    typedef shared_ptr<PolyMesh<Scalar>> pPolyMesh;
    typedef shared_ptr<_Polygon<Scalar>> pPolygon;
    typedef Matrix<Scalar, 3, 1> Vector3;

public:
    vector<wpPolyMesh> meshLists;
    vector<nanogui::Color> object_colors;
    vector<Vector3> object_translation;
    Eigen::Vector3f mesh_center;
public:
    enum AnimationState{
        Stop,
        Run,
        Pause
    }state;

    float prev_time;

public: //shader and render pass
    nanogui::ref<nanogui::RenderPass> render_pass;
    nanogui::ref<nanogui::Shader> shader;

public:  //buffers
    vector<float> positions;
    vector<float> barycentric;
    vector<float> colors;
    vector<float> translation;

public: // uniform
    Eigen::Matrix4f mvp;
    bool wireframe;
    float simtime;

public:
    gui_PolyMeshLists(const vector<pPolyMesh> &_meshLists,
                      bool _wire,
                      nanogui::ref<nanogui::RenderPass> _render_pass)
                      :wireframe(_wire), render_pass(_render_pass), simtime(0), prev_time(0){
        for(pPolyMesh mesh: _meshLists)
        {
            meshLists.push_back(mesh);
        }

        //all faces have random color
        for(int id = 0; id < meshLists.size(); id++)
        {
            object_colors.push_back(nanogui::Color(rand() % 255, rand() % 255, rand() % 255, 255));
        }
        object_translation.resize(meshLists.size(), Vector3(1, 0, 0));

        state = Stop;

        initShader();
    }

    gui_PolyMeshLists(const vector<pPolyMesh> &_meshLists,
                 vector<nanogui::Color> _object_colors,
                 bool _wire,
                 nanogui::ref<nanogui::RenderPass> _render_pass)
            : wireframe(_wire), render_pass(_render_pass), simtime(0), prev_time(0){

        for(pPolyMesh mesh: _meshLists)
        {
            meshLists.push_back(mesh);
        }

        //if the size of the input colors matches the size of the meshlist
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
        object_translation.resize(meshLists.size(), Vector3(1, 0, 0));

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
        //init positions
        int num_vertices = 0;
        mesh_center = Eigen::Vector3f(0, 0, 0);
        positions.clear();
        barycentric.clear();
        colors.clear();
        translation.clear();

        for(size_t mID = 0; mID < meshLists.size(); mID++)
        {
            wpPolyMesh mesh = meshLists[mID];
            nanogui::Color mesh_color = object_colors[mID];
            Vector3 mesh_translation = object_translation[mID];
            for(pPolygon polygon: mesh.lock()->polyList)
            {
                Vector3 face_center = polygon->center();
                for(size_t id = 0; id < polygon->vers.size(); id++)
                {
                    mesh_center = mesh_center + Eigen::Vector3f(polygon->vers[id]->pos.x(), polygon->vers[id]->pos.y(), polygon->vers[id]->pos.z());
                    num_vertices ++;

                    Vector3 sta = polygon->vers[id]->pos;
                    Vector3 end = polygon->vers[(id + 1) % polygon->vers.size()]->pos;

                    positions.push_back(sta.x());
                    positions.push_back(sta.y());
                    positions.push_back(sta.z());

                    positions.push_back(end.x());
                    positions.push_back(end.y());
                    positions.push_back(end.z());

                    positions.push_back(face_center.x());
                    positions.push_back(face_center.y());
                    positions.push_back(face_center.z());


                    barycentric.push_back(1);
                    barycentric.push_back(0);
                    barycentric.push_back(polygon->at_boundary(id) ? 0 : 10);

                    barycentric.push_back(0);
                    barycentric.push_back(1);
                    barycentric.push_back(polygon->at_boundary(id) ? 0 : 10);

                    barycentric.push_back(0.5);
                    barycentric.push_back(0.5);
                    barycentric.push_back(10);

                    // colors
                    for(int kd = 0; kd < 9; kd++){
                        colors.push_back(mesh_color[kd % 3]);
                    }

                    // animation
                    for(int kd = 0; kd < 9; kd++){
                        translation.push_back(mesh_translation[kd %3]);
                    }
                }
            }
        }
        mesh_center /= (float)num_vertices;

        shader->set_buffer("position", nanogui::VariableType::Float32, {positions.size() / 3, 3},  &positions[0]);
        shader->set_buffer("barycentric", nanogui::VariableType::Float32, {barycentric.size() / 3, 3}, &barycentric[0]);
        shader->set_buffer("color", nanogui::VariableType::Float32, {colors.size() / 3, 3}, &colors[0]);
        shader->set_buffer("translation", nanogui::VariableType::Float32, {translation.size() / 3, 3}, &translation[0]);
    }

    nanogui::Matrix4f toNanoguiMatrix(Eigen::Matrix4f mat){
        nanogui::Matrix4f nanomat;
        for(int id = 0; id < 4; id++){
            for(int jd = 0; jd < 4; jd++){
                nanomat.m[id][jd] = mat(jd, id);
            }
        }
        return nanomat;
    }

    void updateUniforms(){
        shader->set_uniform("wireframe", wireframe);
        shader->set_uniform("mvp", toNanoguiMatrix(mvp));
        shader->set_uniform("simtime", simtime);
    }

    void updateTime(float curr_time, float speed = 0.1){
        switch(state){
            case Stop:
                simtime = 0;
                break;
            case Run:
                simtime += (curr_time - prev_time) * speed;
                break;
            case Pause:
                break;
        }
        prev_time = curr_time;
    }

};

#endif //TOPOLITE_GUI_POLYMESHLIST_H
