//
// Created by ziqwang on 2020-02-16.
//

#include <catch2/catch.hpp>
#include "Mesh/Cross.h"
#include "IO/InputVar.h"

TEST_CASE("Cross")
{
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVarLite(varList.get());
    Cross<double> cross(varList);

    cross.print();
}