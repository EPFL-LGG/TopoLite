//
// Created by ziqwang on 10.04.20.
//

#ifndef TOPOLITE_GUI_LOADSCENE_H
#define TOPOLITE_GUI_LOADSCENE_H
#include "gui_SceneObject.h"
#include "Interlocking/ContactGraph.h"
#include "Interlocking/InterlockingSolver_Clp.h"
#include "Mesh/PolyMesh.h"
#include "gui_PolyMeshLists.h"
#include "gui_Lines.h"
#include "Mesh/CrossMesh.h"
#include "CrossMesh/AugmentedVectorCreator.h"
#include "CrossMesh/BaseMeshCreator.h"
#include "IO/JsonIOReader.h"
#include "CrossMesh/PatternCreator.h"
#include "CrossMesh/CrossMeshCreator.h"
using std::weak_ptr;
class gui_LoadScene
{
public:
    using pPolygon = shared_ptr<_Polygon<double>> ;

private:
    weak_ptr<gui_SceneObject<double>> scene;
    shared_ptr<InputVarList> varList;
    vector<shared_ptr<PolyMesh<double>>> meshLists;
    vector<bool> atboundary;
    shared_ptr<ContactGraph<double>> graph;

public:

    gui_LoadScene(weak_ptr<gui_SceneObject<double>> _scene):
            scene(_scene){
        varList = make_shared<InputVarList>();
        InitVar(varList.get());
    }

public:

    void insertMeshesIntoScene(){
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

        for(shared_ptr<PolyMesh<double>> mesh: meshLists){
            mesh->mergeFaces();
        }

        // construct the contact graph
        graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(meshLists, atboundary, 1e-3, false);
        // solve the interlocking problem by using CLP library
        InterlockingSolver_Clp<double> solver(graph, varList, BARRIER);
        shared_ptr<typename InterlockingSolver<double>::InterlockingData> interlockData;
        solver.isRotationalInterlocking(interlockData);

        shared_ptr<gui_PolyMeshLists<double>> polyMeshLists = make_shared<gui_PolyMeshLists<double>>(meshLists, colors, scene.lock()->render_pass);

        double max_length = 1;
        for(int id = 0; id < polyMeshLists->ani_translation.size(); id++){
            max_length = std::max(max_length, interlockData->traslation[id].norm());
            max_length = std::max(max_length, interlockData->rotation[id].norm());
        }

        for(int id = 0; id < polyMeshLists->ani_translation.size(); id++){
            polyMeshLists->ani_translation[id] = interlockData->traslation[id] / max_length;
            polyMeshLists->ani_rotation[id] = interlockData->rotation[id] / max_length;
            polyMeshLists->ani_center[id] = interlockData->center[id];
        }


        polyMeshLists->update_buffer();
        scene.lock()->objects.push_back(polyMeshLists);
    }

    void insertContactsIntoScene()
    {
        vector<nanogui::Color> colors;
        colors.push_back(nanogui::Color(255, 0, 0, 255));
        shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
        graph->getContactMesh(polyMesh);
        polyMesh->mergeFaces(1e-3);
        meshLists.clear();
        meshLists.push_back(polyMesh);
        shared_ptr<gui_PolyMeshLists<double>> polyMeshLists = make_shared<gui_PolyMeshLists<double>>(meshLists, colors, scene.lock()->render_pass);
        scene.lock()->objects.push_back(polyMeshLists);
        polyMeshLists->visible = false;
    }

    void insertMeshandContactsIntoScene(){
        insertMeshesIntoScene();
        insertContactsIntoScene();
    }

public:

    void loadSphereA80()
    {
        //Read Boundary
        {
            std::string part_filename = "data/TopoInterlock/XML/SphereA80_Hex_T40.0_data/PartGeometry/Boundary.obj";
            shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
            bool textureModel;
            if(std::filesystem::is_regular_file(part_filename.c_str())){
                if(polyMesh->readOBJModel(part_filename.c_str(), textureModel, false)){
                    polyMesh->mergeFaces(1e-3);
                    meshLists.push_back(polyMesh);
                    atboundary.push_back(false);
                }
            }
        }

        //Read Parts
        for(int id = 0; id <= 42; id++){
            char number[50];
            sprintf(number, "%02d.obj", id);
            std::string part_filename = "data/TopoInterlock/XML/SphereA80_Hex_T40.0_data/PartGeometry/Part_";
            part_filename += number;
            shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
            bool textureModel;
            if(std::filesystem::is_regular_file(part_filename.c_str())){
                if(polyMesh->readOBJModel(part_filename.c_str(), textureModel, false)){
                    polyMesh->mergeFaces(1e-3);
                    meshLists.push_back(polyMesh);
                    atboundary.push_back(false);
                }
            }
        }
        atboundary[0] = true;

        insertMeshandContactsIntoScene();
    }

    void loadAnia()
    {
        std::string file_name[5] = { "piece4_tri.obj", "piece0.obj", "piece1.obj", "piece3.obj", "piece2.obj"};
        bool textureModel;
        for(int id = 0; id < 5; id++){
            char number[50];
            std::string part_filename = "data/Mesh/Ania_200127_betweenbars/";
            part_filename += file_name[id];
            shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
            polyMesh->readOBJModel(part_filename.c_str(), textureModel, false);
            polyMesh->mergeFaces(1e-3);
            meshLists.push_back(polyMesh);
            atboundary.push_back(false);
        }
        atboundary[0] = false;
        atboundary[1] = true;
        atboundary[2] = true;
        atboundary[3] = true;
        atboundary[4] = true;

        insertMeshandContactsIntoScene();
    }

    void loadBunny()
    {
        //Read all Parts
        bool textureModel;
        for(int id = 1; id <= 80; id++){
            char number[50];
            sprintf(number, "%d.obj", id);
            std::string part_filename = "data/Voxel/bunny/part_";
            part_filename += number;
            shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
            polyMesh->readOBJModel(part_filename.c_str(), textureModel, false);

            meshLists.push_back(polyMesh);
            atboundary.push_back(false);
        }

        atboundary[0] = true;
        insertMeshandContactsIntoScene();
    }

    void load_wireframe(){
        std::string file_name[5] = { "piece4_tri.obj", "piece0.obj", "piece1.obj", "piece3.obj", "piece2.obj"};
        bool textureModel;

        vector<gui_LinesGroup<double>> lgroups;
        for(int id = 0; id < 5; id++){
            char number[50];
            std::string part_filename = "data/Mesh/Ania_200127_betweenbars/";
            part_filename += file_name[id];
            shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
            polyMesh->readOBJModel(part_filename.c_str(), textureModel, false);
            polyMesh->mergeFaces(1e-3);
            nanogui::Color randomcolor = nanogui::Color(rand() %256 , rand() %256 , rand() %256 , 1);
            for(pPolygon poly: polyMesh->polyList){
                gui_LinesGroup<double> lg;
                lg.color = randomcolor;
                lg.ani_center = Eigen::Vector3d(0, 0, 0);
                lg.ani_rotation = Eigen::Vector3d(0, 0, 0);
                lg.ani_translation = Eigen::Vector3d(0, 0, 0);
                for(int id = 0; id < poly->vers.size(); id++){
                    if(poly->at_boundary(id)){
                        int next_id = (id + 1) % poly->vers.size();
                        lg.lines.push_back(Line<double>(poly->vers[id]->pos, poly->vers[next_id]->pos));
                    }
                }
                lgroups.push_back(lg);
            }
        }

        shared_ptr<gui_Lines<double>> LinesObject = make_shared<gui_Lines<double>>(lgroups, 0.05, scene.lock()->render_pass);
        scene.lock()->objects.push_back(LinesObject);
        LinesObject->visible = true;
    }

    void loadAugmentationVector(){
        shared_ptr<CrossMesh<double>> crossMesh = make_shared<CrossMesh<double>>(varList);

        for(int id = 0; id < 5; id++)
        {
            for(int jd = 0; jd < 5; jd++){
                shared_ptr<Cross<double>> cross = make_shared<Cross<double>>(Cross<double>(varList));
                // vertices position vectors
                cross->push_back(Vector3d(id, jd, 0));
                cross->push_back(Vector3d(id + 1, jd, 0));
                cross->push_back(Vector3d(id + 1, jd + 1, 0));
                cross->push_back(Vector3d(id, jd + 1, 0));
                crossMesh->push_back(cross);
            }
        }

        crossMesh->update();
        BaseMeshCreator<double> baseMeshCreator(nullptr, nullptr, varList);
        baseMeshCreator.recomputeBoundary(crossMesh);

        AugmentedVectorCreator<double> augmentedVectorCreator(varList);
        augmentedVectorCreator.createAugmentedVectors(30, crossMesh);

        vector<nanogui::Color> colors;
        colors.push_back(nanogui::Color(200, 200 ,200, 255));

        meshLists.push_back(crossMesh->getPolyMesh());

        shared_ptr<gui_PolyMeshLists<double>> polyMeshLists = make_shared<gui_PolyMeshLists<double>>(meshLists, colors, scene.lock()->render_pass);
        scene.lock()->objects.push_back(polyMeshLists);

        vector<gui_LinesGroup<double>> linegroups;
        gui_LinesGroup<double> lg;
        lg.color = nanogui::Color(200, 100, 0, 255);
        for(int id = 0; id < crossMesh->size(); id++){
            shared_ptr<Cross<double>> cross = crossMesh->cross(id);
            for(int jd = 0; jd < cross->size(); jd++){
                shared_ptr<OrientPoint<double>> ori = cross->ori(jd);
                lg.lines.push_back(Line<double>(ori->point, ori->point + ori->normal * 0.5));
            }
        }
        linegroups.push_back(lg);

        shared_ptr<gui_Lines<double>> LinesObject = make_shared<gui_Lines<double>>(linegroups, 0.03, scene.lock()->render_pass);
        scene.lock()->objects.push_back(LinesObject);
        LinesObject->visible = true;
    }

    void load_ania_multihand(){
        std::string file_name[3] = { "piece0.obj", "piece1.obj", "piece2.obj"};
        bool textureModel;

        vector<gui_LinesGroup<double>> lgroups;
        for(int id = 0; id < 3; id++){
            std::string part_filename = "data/Mesh/Ania_200417_multihand/";
            part_filename += file_name[id];
            shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
            polyMesh->readOBJModel(part_filename.c_str(), textureModel, false);
            polyMesh->mergeFaces(1e-3);
            meshLists.push_back(polyMesh);
        }

        atboundary.resize(3);
        atboundary[0] = false;
        atboundary[1] = false;
        atboundary[2] = true;

        insertMeshandContactsIntoScene();
    }

    void load_ania_frametappered(){
        std::string file_name[14] = { "piece0.obj",
                                     "piece1.obj",
                                     "piece2.obj",
                                     "piece3.obj",
                                     "piece4.obj",
                                     "piece5.obj",
                                     "piece6.obj",
                                     "piece7.obj",
                                     "piece8.obj",
                                     "piece9.obj",
                                     "piece10.obj",
                                     "piece11.obj",
                                     "piece12.obj",
                                     "piece13.obj"};
        bool textureModel;

        vector<gui_LinesGroup<double>> lgroups;
        for(int id = 0; id < 14; id++){
            std::string part_filename = "data/Mesh/Ania_200417_frame/";
            part_filename += file_name[id];
            shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
            polyMesh->readOBJModel(part_filename.c_str(), textureModel, false);
            polyMesh->mergeFaces(1e-3);
            meshLists.push_back(polyMesh);
        }

        atboundary.resize(14, false);
        atboundary[0] = true;
        atboundary[1] = true;
        atboundary[4] = true;
        atboundary[13] = true;

        insertMeshandContactsIntoScene();
    }
};




#endif //TOPOLITE_GUI_LOADSCENE_H
