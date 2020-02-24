//
// Created by ziqwang on 2020-02-22.
//

#include <catch2/catch.hpp>
#include <CrossMesh/BaseMeshCreator.h>
#include "Mesh/PolyMesh.h"
#include "Mesh/CrossMesh.h"
TEST_CASE("BaseMeshCreator")
{
    std::shared_ptr<PolyMesh<double>> _polyMesh;
    std::shared_ptr<CrossMesh<double>> _pattern2D;
    BaseMeshCreator baseMeshCreator(_polyMesh, _pattern2D);

}