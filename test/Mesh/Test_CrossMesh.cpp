//
// Created by ziqwang on 2020-02-21.
//

#include <catch2/catch.hpp>
#include "Mesh/CrossMesh.h"

TEST_CASE("CrossMesh")
{

    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVarLite(varList.get());
    PolyMesh<double> polyMesh(varList);

    SECTION("read polyhedron") {
        bool texturedModel;
        polyMesh.readOBJModel("../data/Mesh/primitives/Icosphere.obj", texturedModel, true);
        CrossMesh<double> crossMesh(polyMesh);
    }
}