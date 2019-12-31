//
// Created by ziqwang on 2019-12-30.
//
#include <catch2/catch.hpp>
#include "CrossMesh/CrossMeshCreator.h"
#include "Mesh/PolyMesh.h"
#include "IO/XMLIO.h"
#include "Mesh/Polygon.h"
#include "Mesh/PolyMesh.h"
#include "Utility/HelpFunc.h"
TEST_CASE("Read CrossMesh")
{
    XMLIO Reader;
    XMLData data;

    boost::filesystem::path current_path(boost::filesystem::current_path());
    boost::filesystem::path debugobj_filepath;
    if (current_path.filename() == "TopoLite")
    {
        debugobj_filepath = current_path / "data/TopoInterlock/Surface/Vouga.obj";
    }
    else
    {
        debugobj_filepath = current_path / "../data/TopoInterlock/Surface/Vouga.obj";
    }

    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVarLite(varList.get());
    shared_ptr<CrossMeshCreator> crossMeshCreator = make_shared<CrossMeshCreator>(varList);
    REQUIRE(crossMeshCreator->loadSurface(debugobj_filepath.c_str()));
    REQUIRE(crossMeshCreator->crossMesh == nullptr);

    SECTION("create cross mesh by using default pattern")
    {
        double interactMat[16];
        LoadIdentityMatrix(interactMat);
        crossMeshCreator->CreateCrossMesh(true, interactMat);
        REQUIRE(crossMeshCreator->crossMesh != nullptr);
    }

    SECTION("create cross mesh by using a single quad")
    {
        pPolyMesh pattern2D = make_shared<PolyMesh>(varList);
        pPolygon poly = make_shared<_Polygon>();
        poly->push_back(Vector3f(0, 0, 0));
        poly->push_back(Vector3f(1, 0, 0));
        poly->push_back(Vector3f(1, 1, 0));
        poly->push_back(Vector3f(0, 1, 0));
        pattern2D->polyList.push_back(poly);
        crossMeshCreator->setPatternMesh(pattern2D);
        double interactMat[16];
        LoadIdentityMatrix(interactMat);
        crossMeshCreator->CreateCrossMesh(true, interactMat);
        REQUIRE(crossMeshCreator->crossMesh->crossList.size() != 0);
    }
}