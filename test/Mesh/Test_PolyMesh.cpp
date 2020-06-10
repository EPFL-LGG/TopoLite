//
// Created by ziqwang on 2020-02-20.
//

#include <catch2/catch.hpp>
#include "Mesh/PolyMesh.h"
#include <filesystem>

TEST_CASE("PolyMesh - Four Quad with index as input") {
    // Setup
    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVar(varList.get());
    PolyMesh<double> polyMesh(varList);

    vector<shared_ptr<_Polygon<double>>> polyLists;

    int dXY[4][2] = {{0, 0},
                     {1, 0},
                     {1, 1},
                     {0, 1}};                           // 0---3---8
    int verIDs[4][4] = {{0, 1, 2, 3},                   // | 0 | 3 |
                        {1, 4, 5, 2},                   // 1---2---7
                        {2, 5, 6, 7},                   // | 1 | 2 |
                        {3, 2, 7, 8}};                  // 4---5---6


    for (int id = 0; id < 4; id++) {
        shared_ptr<_Polygon<double>> poly = make_shared<_Polygon<double>>(_Polygon<double>());
        poly->push_back(Vector3d(0 + dXY[id][0], 0 + dXY[id][1], 0));
        poly->push_back(Vector3d(1 + dXY[id][0], 0 + dXY[id][1], 0));
        poly->push_back(Vector3d(1 + dXY[id][0], 1 + dXY[id][1], 0));
        poly->push_back(Vector3d(0 + dXY[id][0], 1 + dXY[id][1], 0));

        poly->vers[0]->verID = verIDs[id][0];
        poly->vers[1]->verID = verIDs[id][1];
        poly->vers[2]->verID = verIDs[id][2];
        poly->vers[3]->verID = verIDs[id][3];

        polyLists.push_back(poly);
    }

    polyMesh.setPolyLists(polyLists);

    SECTION("Test computeVertexList") {

        REQUIRE((polyMesh.vertexList[4]->pos - Vector3d(2, 0, 0)).norm() == Approx(0.0));
        REQUIRE((polyMesh.vertexList[5]->pos - Vector3d(2, 1, 0)).norm() == Approx(0.0));
    }

    SECTION("Test Box PolyMesh bbox") {
        Box<double> bbox = polyMesh.bbox();
        REQUIRE((bbox.minPt - Vector3d(0, 0, 0)).norm() == Approx(0.0));
        REQUIRE((bbox.maxPt - Vector3d(2, 2, 0)).norm() == Approx(0.0));
        REQUIRE((bbox.cenPt - Vector3d(1, 1, 0)).norm() == Approx(0.0));
        REQUIRE((bbox.size - Vector3d(2, 2, 0)).norm() == Approx(0.0));
    }

    SECTION("Test centroid") {
        REQUIRE((polyMesh.centroid() - Vector3d(0, 0, 0)).norm() == Approx(0.0));
    }

    SECTION("Test volume") {
        REQUIRE(polyMesh.volume() == Approx(0.0));
    }

    SECTION("Test lowest point") {
        REQUIRE((polyMesh.lowestPt() - Vector3d(0.5, 0, 0)).norm() == Approx(0.0));
    }
}

TEST_CASE("PolyMesh - Cube") {
    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVar(varList.get());
    PolyMesh<double> polyMesh(varList);

    SECTION("Cube") {
        vector<shared_ptr<_Polygon<double>>> polyLists;

        int XYZ[8][3] = {{0, 0, 0},
                         {1, 0, 0},
                         {1, 1, 0},
                         {0, 1, 0},
                         {0, 0, 1},
                         {1, 0, 1},
                         {1, 1, 1},
                         {0, 1, 1}};

        int face[6][4] = {{0, 3, 2, 1},
                          {6, 5, 1, 2},
                          {2, 3, 7, 6},
                          {0, 4, 7, 3},
                          {0, 1, 5, 4},
                          {4, 5, 6, 7}};

        for (int id = 0; id < 6; id++) {
            shared_ptr<_Polygon<double>> poly = make_shared<_Polygon<double>>(_Polygon<double>());
            for (int jd = 0; jd < 4; jd++) {
                Vector3d pt(XYZ[face[id][jd]][0], XYZ[face[id][jd]][1], XYZ[face[id][jd]][2]);
                poly->push_back(pt);
            }
            polyLists.push_back(poly);
        }

        polyMesh.setPolyLists(polyLists);
        REQUIRE((polyMesh.centroid() - Vector3d(0.5, 0.5, 0.5)).norm() == Approx(0.0).margin(1e-6));
        REQUIRE(polyMesh.volume() == Approx(1).margin(1e-6));

        //the reason for 8 is that the setPolyLists function will merge vertices with same position into one vertex
        //a cube only has 8 vertices
        REQUIRE(polyMesh.vertexList.size() == 8);

        REQUIRE((polyMesh.centroid() - Vector3d(0.5, 0.5, 0.5)).norm() == Approx(0.0).margin(1e-6));

        REQUIRE(polyMesh.volume() == Approx(1).margin(1e-6));

        // Eigen Matrix Mesh
        PolyMesh<double>::MatrixX V;
        PolyMesh<double>::MatrixXi F;
        Eigen::VectorXi C;
        polyMesh.convertPosToEigenMesh(V, F, C);
        REQUIRE(V.rows() == 8);
        REQUIRE(F.rows() == 12);
    }

    SECTION("read polyhedron") {

        std::filesystem::path dataFolder(UNITTEST_DATAPATH);
        std::filesystem::path filepath = dataFolder / "Mesh/primitives/Icosphere.obj";

        polyMesh.readOBJModel(filepath.c_str(), true);
        REQUIRE(polyMesh.texturedModel == true);

        polyMesh.rotateMesh(Vector3d(0, 0, 0), Vector3d(0, 0, 1), 10);
        polyMesh.translateMesh(Vector3d(1, 1, 1));
        polyMesh.scaleMesh(Vector3d(2, 2, 2));

        std::filesystem::path outputfile = dataFolder / "Mesh/primitives/Icosphere2.obj";
        polyMesh.getTextureMesh()->writeOBJModel(outputfile.c_str(), false);

        SECTION("Test copy and construct function") {
            PolyMesh<double> newmesh = polyMesh;
            REQUIRE(newmesh.vertexList[0]->pos == polyMesh.vertexList[0]->pos);
            REQUIRE(newmesh.vertexList[0] != polyMesh.vertexList[0]);

            REQUIRE(newmesh.textureList[0]->texCoord == polyMesh.textureList[0]->texCoord);
            REQUIRE(newmesh.textureList[0] != polyMesh.textureList[0]);

            REQUIRE(newmesh.polyList[0] != polyMesh.polyList[0]);
            REQUIRE((newmesh.polyList[0]->normal() - polyMesh.polyList[0]->normal()).norm() == Approx(0.0));
        }
    }
}