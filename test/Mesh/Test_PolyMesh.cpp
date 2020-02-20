//
// Created by ziqwang on 2020-02-20.
//

#include <catch2/catch.hpp>
#include "Mesh/PolyMesh.h"

TEST_CASE("PolyMesh")
{
    shared_ptr<InputVarList> varList;

    PolyMesh<double> polyMesh(varList);
}