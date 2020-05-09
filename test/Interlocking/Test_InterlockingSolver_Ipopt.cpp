//
// Created by ziqwang on 2020-02-03.
//

#include <catch2/catch.hpp>
//#include "Interlocking/InterlockingSolver_Clp.h"
#include "Interlocking/InterlockingSolver_Ipopt.h"
#include "IO/XMLIO.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SparseQR>

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
         InterlockingSolver_Ipopt<double> solver(graph, varList);
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
         InterlockingSolver_Ipopt<double> solver(graph, varList);
         shared_ptr<typename InterlockingSolver<double>::InterlockingData> interlockData;
         REQUIRE(solver.isRotationalInterlocking(interlockData) == false);
     }

     SECTION("fix key and merge key and second part"){
         // if only set the key to be fixed
         // the reset parts could move together, therefore the structure is not interlocking
         atboundary[0] = true;

         // construct the contact graph
         shared_ptr<ContactGraph<double>>graph = make_shared<ContactGraph<double>>(varList);
         graph->buildFromMeshes(meshList, atboundary);

         //merge
         graph->mergeNode(graph->nodes[0], graph->nodes[1]);

         // solve the interlocking problem by using CLP library
         InterlockingSolver_Ipopt<double> solver(graph, varList);
         shared_ptr<typename InterlockingSolver<double>::InterlockingData> interlockData;
         REQUIRE(solver.isRotationalInterlocking(interlockData) == true);
     }
 }



TEST_CASE("Ania Example"){
    std::string file_name[5] = { "piece4_tri.obj", "piece0.obj", "piece1.obj", "piece3.obj", "piece2.obj"};

    vector<shared_ptr<PolyMesh<double>>> meshList;
    vector<bool> atboundary;
    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVarLite(varList.get());
    bool textureModel;


    for(int id = 0; id < 5; id++){
        char number[50];
        std::string part_filename = "data/Mesh/Ania_200127_betweenbars/";
        part_filename += file_name[id];
        shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
        polyMesh->readOBJModel(part_filename.c_str(), textureModel, false);
        meshList.push_back(polyMesh);
        atboundary.push_back(true);
    }

    atboundary[0] = false;

    shared_ptr<ContactGraph<double>>graph = make_shared<ContactGraph<double>>(varList);
    graph->buildFromMeshes(meshList, atboundary, 1e-3);

    SECTION("Ipopt"){
        // solve the interlocking problem by using ipopt
        InterlockingSolver_Ipopt<double> solver(graph, varList);
        shared_ptr<typename InterlockingSolver<double>::InterlockingData> interlockData;
        REQUIRE(solver.isRotationalInterlocking(interlockData) == false);
    }

}

// TEST_CASE("Ania Example: Special Case"){
//     //Read all Parts

//     std::string file_name[3] = {"piece0.obj", "piece1.obj", "piece4.obj"};
//     vector<shared_ptr<PolyMesh<double>>> meshList;
//     vector<bool> atboundary;
//     shared_ptr<InputVarList> varList = make_shared<InputVarList>();
//     InitVarLite(varList.get());
//     bool textureModel;


//     for(int id = 0; id < 3; id++){
//         char number[50];
//         std::string part_filename = "data/Mesh/Ania_200127_betweenbars/";
//         part_filename += file_name[id];
//         shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);
//         polyMesh->readOBJModel(part_filename.c_str(), textureModel, false);
//         meshList.push_back(polyMesh);
//         atboundary.push_back(false);
//     }

//     atboundary[0] = true;
//     atboundary[1] = true;

//     shared_ptr<ContactGraph<double>>graph = make_shared<ContactGraph<double>>(varList);
//     graph->buildFromMeshes(meshList, atboundary, 1e-3);

//     // solve the interlocking problem by using CLP library
//     InterlockingSolver_Clp<double> solver(graph, varList);
//     shared_ptr<typename InterlockingSolver<double>::InterlockingData> interlockData;
//     REQUIRE(solver.isRotationalInterlocking(interlockData) == false);}
