//
// Created by ziqwang on 08.04.20.
//

#ifndef TOPOLITE_GUI_POLYMESHLIST_H
#define TOPOLITE_GUI_POLYMESHLIST_H
#include "Mesh/PolyMesh.h"
#include "guiShader_Base.h"
#include <nanogui/vector.h>
#include <fstream>
#include "igl/triangle/triangulate.h"
#include "shader/metal_shader.h"
#include "shader/opengl_shader.h"
template<typename Scalar>
class guiShader_PolyMeshes : public guiShader_Base<Scalar>{
public:

    typedef weak_ptr<PolyMesh<Scalar>> wpPolyMesh;
    typedef shared_ptr<PolyMesh<Scalar>> pPolyMesh;
    typedef shared_ptr<_Polygon<Scalar>> pPolygon;
    typedef shared_ptr<Triangle<Scalar>> pTriangle;
    typedef Matrix<Scalar, 3, 1> Vector3;


public:
    using guiShader_Base<Scalar>::object_colors;
    using guiShader_Base<Scalar>::object_center;
    nanogui::Color line_color;
    vector<wpPolyMesh> meshLists;
    vector<Vector3> ani_translation;
    vector<Vector3> ani_center;
    vector<Vector3> ani_rotation;

public:
    using guiShader_Base<Scalar>::state;

public:
    using guiShader_Base<Scalar>::render_pass;
    using guiShader_Base<Scalar>::shader;

public:  //buffers
    using guiShader_Base<Scalar>::buffer_colors;
    using guiShader_Base<Scalar>::buffer_positions;
    vector<float> buffer_translation;
    vector<float> buffer_rotation;
    vector<float> buffer_center;
    vector<int> buffer_objectindex;

public: // uniform
    using guiShader_Base<Scalar>::varList;
    using guiShader_Base<Scalar>::proj_mat;
    using guiShader_Base<Scalar>::model_mat;
    using guiShader_Base<Scalar>::model_init_mat;
    using guiShader_Base<Scalar>::view_mat;
    using guiShader_Base<Scalar>::eye;
    using guiShader_Base<Scalar>::simtime;

public:
    guiShader_PolyMeshes(const vector<pPolyMesh> &_meshLists,
                         nanogui::ref<nanogui::RenderPass> _render_pass) : guiShader_Base<Scalar>::guiShader_Base(_render_pass){
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
        
        line_color = nanogui::Color(0, 0, 0, 0);
        state = Stop;
        initShader();
    }

    guiShader_PolyMeshes(const vector<pPolyMesh> &_meshLists,
                         vector<nanogui::Color> _object_colors,
                         nanogui::ref<nanogui::RenderPass> _render_pass) : guiShader_Base<Scalar>::guiShader_Base(_render_pass){

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
        
        line_color = nanogui::Color(0, 0, 0, 0);
        
        ani_translation.resize(meshLists.size(), Vector3(0, 0, 0));
        ani_rotation.resize(meshLists.size(), Vector3(0, 0, 0));
        ani_center.resize(meshLists.size(), Vector3(0, 0, 0));

        state = Stop;
        initShader();
    }

    void update_mesh(const vector<pPolyMesh> &_meshLists, vector<nanogui::Color> _object_colors){
        meshLists.clear();
        object_colors.clear();
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
        ani_translation.clear(); ani_translation.resize(meshLists.size(), Vector3(0, 0, 0));
        ani_rotation.clear(); ani_rotation.resize(meshLists.size(), Vector3(0, 0, 0));
        ani_center.clear(); ani_center.resize(meshLists.size(), Vector3(0, 0, 0));
        update_buffer();
    }

public:



    void initShader()
    {
        //read text from file
#if defined(NANOGUI_USE_OPENGL)
        std::string shader_vert = opengl_polymeshes_vert;
        std::string shader_frag = opengl_polymeshes_frag;
#elif defined(NANOGUI_USE_METAL)
        std::string shader_vert = metal_polymeshes_vert;
        std::string shader_frag = metal_polymeshes_frag;
#endif
        
        shader = new nanogui::Shader(render_pass, "PolyMeshShader", shader_vert, shader_frag, nanogui::Shader::BlendMode::None);
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
        buffer_objectindex.clear();

#if defined(NANOGUI_USE_METAL)
        for(size_t mID = 0; mID < meshLists.size(); mID++){
            for(int kd = 0; kd < 3; kd++){
                buffer_colors.push_back(object_colors[mID][kd]);
                buffer_translation.push_back(ani_translation[mID][kd]);
                buffer_rotation.push_back(ani_rotation[mID][kd]);
                buffer_center.push_back(ani_center[mID][kd]);
            }
        }
#endif

        for(size_t mID = 0; mID < meshLists.size(); mID++)
        {
            wpPolyMesh mesh = meshLists[mID];
            for(pPolygon polygon: mesh.lock()->polyList)
            {
                vector<pTriangle> tris;
                polygon->triangulateNaive(tris);
                for(pTriangle tri: tris)
                {
                    for(int vID = 0; vID < 3; vID++)
                    {
                        buffer_positions.push_back(tri->v[vID].x());
                        buffer_positions.push_back(tri->v[vID].y());
                        buffer_positions.push_back(tri->v[vID].z());
                        object_center = object_center + tri->v[vID];
                        num_vertices++;
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
                    for(int kd = 0; kd < 3; kd++){
                        buffer_objectindex.push_back(mID);
                    }
#endif
                }
            }
        }
        object_center /= (float)num_vertices;

        shader->set_buffer("position", nanogui::VariableType::Float32, {buffer_positions.size() / 3, 3},  buffer_positions.data());

        shader->set_buffer("color", nanogui::VariableType::Float32, {buffer_colors.size() / 3, 3}, buffer_colors.data());
        shader->set_buffer("translation", nanogui::VariableType::Float32, {buffer_translation.size() / 3, 3}, buffer_translation.data());
        shader->set_buffer("rotation", nanogui::VariableType::Float32, {buffer_rotation.size() / 3, 3}, buffer_rotation.data());
        shader->set_buffer("center", nanogui::VariableType::Float32, {buffer_center.size() / 3, 3}, buffer_center.data());
#if defined(NANOGUI_USE_METAL)
        shader->set_buffer("objectindex", nanogui::VariableType::Int32, {buffer_objectindex.size(), 1}, buffer_objectindex.data());
#endif
    }

    void update_uniform(){
        shader->set_uniform("mvp", this->toNanoguiMatrix(proj_mat * view_mat * model_mat));
        shader->set_uniform("simtime", simtime);
    }
};

#endif //TOPOLITE_GUI_POLYMESHLIST_H
