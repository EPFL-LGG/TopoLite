//
// Created by ziqwang on 21.03.20.
//

#include "CrossMesh/AugmentedVectorCreator.h"
#include "CrossMesh/BaseMeshCreator.h"
#include <catch2/catch.hpp>

TEST_CASE("AugmentedVectorCreator"){
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVarLite(varList.get());

    AugmentedVectorCreator<double> augmentedVectorCreator(varList);

    SECTION("Four quad"){
        shared_ptr<CrossMesh<double>> crossMesh = make_shared<CrossMesh<double>>(varList);

        for(int id = 0; id < 5; id++)
        {
            for(int jd = 0; jd < 5; jd++){
                shared_ptr<Cross<double>> cross = make_shared<Cross<double>>(Cross<double>(varList));
                // vertices position vectors
                cross->push_back(Vector3d(id, jd, 0));
                cross->push_back(Vector3d(id + 1, jd, 0));
                cross->push_back(Vector3d(id + 1, jd + 1, 0));
                cross->push_back(Vector3d(id, jd + 1, 0));
                crossMesh->push_back(cross);
            }
        }

        crossMesh->update();
        BaseMeshCreator<double> baseMeshCreator(nullptr, nullptr, varList);
        baseMeshCreator.recomputeBoundary(crossMesh);

        augmentedVectorCreator.createAugmentedVectors(30, crossMesh);
    }

}