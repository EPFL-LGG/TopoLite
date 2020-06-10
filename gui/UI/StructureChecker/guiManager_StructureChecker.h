//
// Created by ziqwang on 10.04.20.
//

#ifndef TOPOLITE_GUIMANAGER_STRUCTURECHECKER_H
#define TOPOLITE_GUIMANAGER_STRUCTURECHECKER_H


#include "Interlocking/ContactGraph.h"
#include "Interlocking/InterlockingSolver.h"
#include "Interlocking/InterlockingSolver_Clp.h"
#if defined(IPOPT_INSTALLED)
    #include "Interlocking/InterlockingSolver_Ipopt.h"
#endif

#include "guiScene_Base.h"
#include "guiShader_PolyMeshes.h"
#include "guiShader_Lines.h"
using std::weak_ptr;
class guiManager_StructureChecker
{

public:

    using pPolygon = shared_ptr<_Polygon<double>> ;

private:
    nanogui::ref<guiCanvas_3DArcball> main_canvas;
    shared_ptr<InputVarList> varList;

    vector<shared_ptr<PolyMesh<double>>> meshLists;
    vector<bool> atboundary;

    shared_ptr<typename InterlockingSolver<double>::InterlockingData> interlockData;

    enum InterlockingSolver_Type{
        CLP_SIMPLEX = 0,
        CLP_BARRIER = 1,
#if defined(IPOPT_INSTALLED)
        IPOPT = 2
#endif
    };


public:

    guiManager_StructureChecker(nanogui::ref<guiCanvas_3DArcball> _main_canvas)
    :main_canvas(_main_canvas){
        varList = make_shared<InputVarList>();
#if defined(IPOPT_INSTALLED)
        varList->add((int)IPOPT, "interlock_solver_type", "Int. Solver");
#elif
        varList->add((int)CLP_SIMPLEX, "interlock_solver_type", "Int. Solver");
#endif
        varList->add((float)1e-3, "contact_eps", "Contact Eps");
    }

public:

    shared_ptr<InterlockingSolver<double>> interlock_solver_factor(shared_ptr<ContactGraph<double>> graph)
    {
        int type = varList->getInt("interlock_solver_type");
        if(type < 2){
            switch (type) {
                case CLP_SIMPLEX:
                    return make_shared<InterlockingSolver_Clp<double>>(graph, varList, SIMPLEX);
                    break;
                case CLP_BARRIER:
                    return make_shared<InterlockingSolver_Clp<double>>(graph, varList, BARRIER);
                    break;
                default:
                    return nullptr;
            }
        }
        else{
        #if defined(IPOPT_INSTALLED)
            return make_shared<InterlockingSolver_Ipopt<double>>(graph, varList);
        #elif
            return make_shared<InterlockingSolver_Clp<double>>(graph, varList, SIMPLEX);
        #endif
        }
        return nullptr;
    }


    bool check_interlocking()
    {
        // construct the contact graph
        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(rescale_meshes(), atboundary, varList->getFloat("contact_eps"), false);

        // solve the interlocking problem by using CLP library
        shared_ptr<InterlockingSolver<double>> solver = interlock_solver_factor(graph);
        bool is_interlocking = solver->isRotationalInterlocking(interlockData);

        init_scene();

        return is_interlocking;
    }

    void clear_scene(){
        main_canvas->init_render_pass();
        meshLists.clear();
        atboundary.clear();
    }

    void init_scene()
    {
        main_canvas->init_render_pass();
        weak_ptr<guiScene_Base<double>> scene = main_canvas->scene;

        auto scaled_meshes = rescale_meshes();

        for(shared_ptr<PolyMesh<double>> mesh: scaled_meshes){
            mesh->mergeFaces();
        }

        // construct the contact graph
        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(scaled_meshes, atboundary, varList->getFloat("contact_eps"), false);

        /*
         * Draw Meshes
         */
        {
            vector<nanogui::Color> colors;
            if(colors.empty()){
                for(int id = 0; id < atboundary.size(); id++){
                    if(atboundary[id]){
                        colors.push_back(nanogui::Color(100, 100 ,100 ,255));
                    }
                    else{
                        colors.push_back(nanogui::Color(255, 255 ,255 ,255));
                    }
                }
            }

            shared_ptr<guiShader_PolyMeshes<double>> polyMeshesShaders
            = make_shared<guiShader_PolyMeshes<double>>(scaled_meshes, colors, scene.lock()->render_pass);


            if(interlockData != nullptr)
            {
                double max_length = 0;
                for(int id = 0; id < polyMeshesShaders->ani_translation.size(); id++)
                {
                    max_length = std::max(max_length, interlockData->traslation[id].norm());
                    max_length = std::max(max_length, interlockData->rotation[id].norm());
                }

                max_length *= 2;

                for(int id = 0; id < polyMeshesShaders->ani_translation.size(); id++)
                {
                    polyMeshesShaders->ani_translation[id] = interlockData->traslation[id] / max_length;
                    polyMeshesShaders->ani_rotation[id] = interlockData->rotation[id] / max_length;
                    polyMeshesShaders->ani_center[id] = interlockData->center[id];
                }
            }

            polyMeshesShaders->update_buffer();
            scene.lock()->objects.push_back(polyMeshesShaders);
        }

        /*
         * Wireframe
         */
        {
            vector<gui_LinesGroup<double>> linegroups;
            for(shared_ptr<PolyMesh<double>> mesh: scaled_meshes){
                gui_LinesGroup<double> lg;
                lg.lines = mesh->getWireFrame();
                lg.color = nanogui::Color(0, 0, 0, 255);
                linegroups.push_back(lg);
            }

            shared_ptr<guiShader_Lines<double>> LinesShader
            = make_shared<guiShader_Lines<double>>(linegroups, 0.002, scene.lock()->render_pass);
            scene.lock()->objects.push_back(LinesShader);

            if(interlockData != nullptr)
            {
                double max_length = 0;
                for(int id = 0; id < LinesShader->ani_translation.size(); id++)
                {
                    max_length = std::max(max_length, interlockData->traslation[id].norm());
                    max_length = std::max(max_length, interlockData->rotation[id].norm());
                }

                max_length *= 2;

                for(int id = 0; id < LinesShader->ani_translation.size(); id++)
                {
                    LinesShader->ani_translation[id] = interlockData->traslation[id] / max_length;
                    LinesShader->ani_rotation[id] = interlockData->rotation[id] / max_length;
                    LinesShader->ani_center[id] = interlockData->center[id];
                }

                LinesShader->update_buffer();
            }
        }

        /*
         * Contact
         */
        {
            vector<nanogui::Color> colors;
            colors.push_back(nanogui::Color(255, 0, 0, 255));
            shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
            graph->getContactMesh(polyMesh);
            polyMesh->mergeFaces(1e-3);
            vector<shared_ptr<PolyMesh<double>>> mlist = {polyMesh};
            shared_ptr<guiShader_PolyMeshes<double>> polyMeshesShaders
            = make_shared<guiShader_PolyMeshes<double>>(mlist, colors, scene.lock()->render_pass);
            scene.lock()->objects.push_back(polyMeshesShaders);
            polyMeshesShaders->visible = false;
        }

        main_canvas->refresh_trackball_center();
    }

public:

    void load_meshList(std::vector<std::string> OBJFileList, bool atBoundary)
    {
        for(std::string name: OBJFileList)
        {
            shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
            polyMesh->readOBJModel(name.c_str(), false);
            meshLists.push_back(polyMesh);
            atboundary.push_back(atBoundary);
        }

        interlockData.reset();
        init_scene();
    }

    vector<shared_ptr<PolyMesh<double>>> rescale_meshes(){
        Box<double> bbx;
        for(auto mesh: meshLists){
            Box<double> meshbbx = mesh->bbox();
            if(bbx.size.norm() < FLOAT_ERROR_SMALL){
                bbx = meshbbx;
            }
            else{
                bbx = Box<double>(bbx, meshbbx);
            }
        }

        if(bbx.size.norm() > FLOAT_ERROR_LARGE){
            vector<shared_ptr<PolyMesh<double>>> scaled_meshes;
            for(auto mesh: meshLists){
                shared_ptr<PolyMesh<double>> scaled_mesh = make_shared<PolyMesh<double>>(*mesh);
                double scaleFactor = std::min(1.0 / bbx.size[0], std::min(1.0 / bbx.size[1], 1.0 / bbx.size[2]));
                scaled_mesh->scaleMesh(Eigen::Vector3d(scaleFactor, scaleFactor, scaleFactor));
                scaled_meshes.push_back(scaled_mesh);
            }

            return scaled_meshes;
        }
        else{
            return meshLists;
        }

    }
};




#endif //TOPOLITE_GUIMANAGER_STRUCTURECHECKER_H
