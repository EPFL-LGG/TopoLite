//
// Created by ziqwang on 2020-02-03.
//

#include <catch2/catch.hpp>
#include "Interlocking/InterlockingSolver_Clp.h"
#include "IO/XMLIO.h"


TEST_CASE("Solve Interlocking")
{
    vector<shared_ptr<PolyMesh<double>>> meshList;
    vector<bool> atboundary;
    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVarLite(varList.get());

    bool textureModel;
    //Parts
    for(int id = 1; id <= 79; id++){
        char number[50];
        sprintf(number, "%02d.obj", id);
        std::string part_filename = "data/Voxel/puz/Part";
        part_filename += number;
        shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
        polyMesh->readOBJModel(part_filename.c_str(), textureModel, false);
        meshList.push_back(polyMesh);
        atboundary.push_back(false);
    }

    //Boundary
    for(int id = 0; id <= 0; id++)
    {
        char number[50];
        sprintf(number, "%02d.obj", id);
        std::string part_filename = "data/Voxel/puz/Part";
        part_filename += number;
        shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
        polyMesh->readOBJModel(part_filename.c_str(), textureModel, false);
        meshList.push_back(polyMesh);
        atboundary.push_back(true);
    }

    shared_ptr<ContactGraph<double>>graph = make_shared<ContactGraph<double>>(varList);
    graph->buildFromMeshes(meshList, atboundary);

    InterlockingSolver_Clp<double> solver(graph, varList);
    shared_ptr<typename InterlockingSolver<double>::InterlockingData> interlockData;
    solver.isRotationalInterlocking(interlockData);
}
