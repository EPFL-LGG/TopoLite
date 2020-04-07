//
// Created by ziqwang on 2020-02-03.
//

#include <catch2/catch.hpp>
#include "Interlocking/InterlockingSolver_Clp.h"
#include "IO/XMLIO.h"


TEST_CASE("Bunny Example")
{
    vector<shared_ptr<PolyMesh<double>>> meshList;
    vector<bool> atboundary;
    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVarLite(varList.get());

    bool textureModel;

    //Read all Parts
    for(int id = 1; id <= 80; id++){
        char number[50];
        sprintf(number, "%d.obj", id);
        std::string part_filename = "data/Voxel/bunny/part_";
        part_filename += number;
        shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
        polyMesh->readOBJModel(part_filename.c_str(), textureModel, false);

        meshList.push_back(polyMesh);
        atboundary.push_back(false);
    }

    SECTION("fix key and second part"){
        // set the first and second part to be on the boundires
        // then the structure is interlocking
        atboundary[0] = true;
        atboundary[1] = true;

        // construct the contact graph
        shared_ptr<ContactGraph<double>>graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(meshList, atboundary, 1e-3);

        // solve the interlocking problem by using CLP library
        InterlockingSolver_Clp<double> solver(graph, varList);
        shared_ptr<typename InterlockingSolver<double>::InterlockingData> interlockData;
        REQUIRE(solver.isRotationalInterlocking(interlockData) == true);
    }

    SECTION("fix key"){
        // if only set the key to be fixed
        // the reset parts could move together, therefore the structure is not interlocking
        atboundary[0] = true;

        // construct the contact graph
        shared_ptr<ContactGraph<double>>graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(meshList, atboundary);

        // solve the interlocking problem by using CLP library
        InterlockingSolver_Clp<double> solver(graph, varList);
        shared_ptr<typename InterlockingSolver<double>::InterlockingData> interlockData;
        REQUIRE(solver.isRotationalInterlocking(interlockData) == false);
    }
}