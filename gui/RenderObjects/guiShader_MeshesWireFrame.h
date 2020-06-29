//
// Created by ziqwang on 08.04.20.
//

#ifndef TOPOLITE_GUI_MESHESWIREFRAME_H
#define TOPOLITE_GUI_MESHESWIREFRAME_H
#include "Mesh/PolyMesh.h"
#include "guiShader_Base.h"
#include <nanogui/vector.h>
#include <fstream>
#include "guiShader_PolyMeshes.h"
#include "guiShader_Lines.h"
template<typename Scalar>
class guiShader_MeshesWireFrame : public guiShader_Base<Scalar>{

public:
    typedef weak_ptr<PolyMesh<Scalar>> wpPolyMesh;
    typedef shared_ptr<PolyMesh<Scalar>> pPolyMesh;

public:
    shared_ptr<guiShader_Lines<Scalar>> linesShader;
    shared_ptr<guiShader_PolyMeshes<Scalar>> meshesShader;

public: // uniform
    using guiShader_Base<Scalar>::varList;
    using guiShader_Base<Scalar>::proj_mat;
    using guiShader_Base<Scalar>::model_mat;
    using guiShader_Base<Scalar>::model_init_mat;
    using guiShader_Base<Scalar>::view_mat;
    using guiShader_Base<Scalar>::eye;
    using guiShader_Base<Scalar>::simtime;
public:
    guiShader_MeshesWireFrame(const vector<pPolyMesh> &_meshLists,
                         Scalar linewidth,
                         nanogui::ref<nanogui::RenderPass> _render_pass) : guiShader_Base<Scalar>::guiShader_Base(_render_pass)
                         {
        meshesShader = make_shared<guiShader_PolyMeshes<Scalar>>(_meshLists, _render_pass);
        vector<gui_LinesGroup<double>> linegroups = get_wireframe(_meshLists);
        linesShader = make_shared<guiShader_Lines<Scalar>>(linegroups, linewidth, _render_pass);
    }

    guiShader_MeshesWireFrame(const vector<pPolyMesh> &_meshLists,
                         vector<nanogui::Color> _object_colors,
                         Scalar linewidth,
                         nanogui::ref<nanogui::RenderPass> _render_pass) : guiShader_Base<Scalar>::guiShader_Base(_render_pass){
        meshesShader = make_shared<guiShader_PolyMeshes<Scalar>>(_meshLists, _object_colors, _render_pass);
        vector<gui_LinesGroup<double>> linegroups = get_wireframe(_meshLists);
        linesShader = make_shared<guiShader_Lines<Scalar>>(linegroups, linewidth, _render_pass);
    }

    void update_mesh(const vector<pPolyMesh> &_meshLists, vector<nanogui::Color> _object_colors)
    {
        meshesShader->update_mesh(_meshLists, _object_colors);
        vector<gui_LinesGroup<double>> linegroups = get_wireframe(_meshLists);
        linesShader->update_line(linegroups);
    }

public:

    vector<gui_LinesGroup<double>> get_wireframe(const vector<pPolyMesh> &_meshLists){
        vector<gui_LinesGroup<double>> linegroups;
        for(auto mesh: _meshLists){
            gui_LinesGroup<double> lg;
            lg.color = nanogui::Color(0, 0, 0, 0);
            lg.lines = mesh->getWireFrame();
            linegroups.push_back(lg);
        }
        return linegroups;
    }

    void update_buffer() override {
       meshesShader->update_buffer();
       linesShader->update_buffer();
    }

    void draw_object() override {
        if(guiShader_Base<Scalar>::visible) {
            update_uniform();
            meshesShader->draw_object();
            linesShader->draw_object();
        }
    }

    void update_uniform() override{
        meshesShader->proj_mat = proj_mat;
        meshesShader->model_mat = model_mat;
        meshesShader->view_mat = view_mat;
        meshesShader->simtime = simtime;
        meshesShader->update_uniform();

        linesShader->proj_mat = proj_mat;
        linesShader->model_mat = model_mat;
        linesShader->view_mat = view_mat;
        linesShader->simtime = simtime;
        linesShader->update_uniform();
    }
};

#endif //TOPOLITE_GUI_MESHESWIREFRAME_H
