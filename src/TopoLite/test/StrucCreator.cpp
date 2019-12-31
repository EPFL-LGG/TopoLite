//
// Created by ziqwang on 2019-12-30.
//

#include <catch2/catch.hpp>
#include "CrossMesh/CrossMeshCreator.h"
#include "Structure/StrucCreator.h"
#include "Mesh/PolyMesh.h"
#include "IO/XMLIO.h"
#include "Mesh/Polygon.h"
#include "Mesh/PolyMesh.h"
#include "Utility/HelpFunc.h"

TEST_CASE()
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
    shared_ptr<StrucCreator> strucCreator = make_shared<StrucCreator>(varList);
    REQUIRE(strucCreator->LoadSurface(debugobj_filepath.c_str()));
    REQUIRE(strucCreator->crossMeshCreator != nullptr);

    SECTION("create 2 parts from 2 quads pattern"){
        pPolyMesh pattern2D = make_shared<PolyMesh>(varList);
        pPolygon poly = make_shared<_Polygon>();
        poly->push_back(Vector3f(0, 0, 0));
        poly->push_back(Vector3f(1, 0, 0));
        poly->push_back(Vector3f(1, 1, 0));
        poly->push_back(Vector3f(0, 1, 0));
        pattern2D->polyList.push_back(poly);
        poly = make_shared<_Polygon>();
        poly->push_back(Vector3f(0, 0, 0));
        poly->push_back(Vector3f(0, 1, 0));
        poly->push_back(Vector3f(-1, 1, 0));
        poly->push_back(Vector3f(-1, 0, 0));
        pattern2D->polyList.push_back(poly);

        strucCreator->crossMeshCreator->setPatternMesh(pattern2D);
        double interactMat[16];
        LoadIdentityMatrix(interactMat);
        strucCreator->CreateStructure(true, interactMat, true);
        REQUIRE(strucCreator->struc->partList.size() == 2);
    }
}