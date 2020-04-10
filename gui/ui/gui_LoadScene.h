//
// Created by ziqwang on 10.04.20.
//

#ifndef TOPOLITE_GUI_LOADSCENE_H
#define TOPOLITE_GUI_LOADSCENE_H

void load_SphereA80(shared_ptr<InputVarList> varList,
                    vector<shared_ptr<PolyMesh<double>>> &meshLists,
                    vector<nanogui::Color> &colors,
                    vector<bool> &atboundary)
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
}

void load_ania(shared_ptr<InputVarList> varList,
               vector<shared_ptr<PolyMesh<double>>> &meshLists,
               vector<nanogui::Color> &colors,
               vector<bool> &atboundary)
{
    {
        std::string file_name[5] = {"piece0.obj", "piece1.obj", "piece2.obj", "piece3.obj", "piece4_tri.obj"};
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
        atboundary[0] = true;
        atboundary[1] = true;
        atboundary[2] = true;
        atboundary[3] = true;
    }
}




#endif //TOPOLITE_GUI_LOADSCENE_H
