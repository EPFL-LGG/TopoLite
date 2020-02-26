//
// Created by ziqwang on 2020-02-22.
//

#include <catch2/catch.hpp>
#include <CrossMesh/BaseMeshCreator.h>
#include "Mesh/PolyMesh.h"
#include "Mesh/CrossMesh.h"
#include "Mesh/PolyMesh_AABBTree.h"
TEST_CASE("BaseMeshCreator")
{
    std::shared_ptr<PolyMesh_AABBTree<double>> _polyMesh;
    std::shared_ptr<CrossMesh<double>> _pattern2D;
    BaseMeshCreator<double> baseMeshCreator(_polyMesh, _pattern2D);

}