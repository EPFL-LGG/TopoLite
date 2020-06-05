//
// Created by ziqwang on 20.05.20.
//

#include <catch2/catch.hpp>
#include "Mesh/Cross.h"
#include <Structure/ConvexBlock.h>
#include "IO/InputVar.h"

TEST_CASE("Square") {
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVar(varList.get());

    // Define an Hexagon with 6 pt - Create a cross object + tilt it by 30 degrees
    shared_ptr<Cross<double>> cross = make_shared<Cross<double>>(varList);
    int N = 6;
    for (int id = 0; id < N; id++) {
        Vector3d pt(std::cos(2 * M_PI / N * id), std::sin(2 * M_PI / N * id), 0);
        cross->push_back(pt);
    }

    double angle = 30;
    for(int id = 0; id < N; id++){
        Vector3d edge = cross->pos(id + 1) - cross->pos(id);
        Vector3d mid = (cross->pos(id + 1) + cross->pos(id)) / 2;
        Vector3d normal = edge.cross(Eigen::Vector3d(0, 0, 1));
        shared_ptr<OrientPoint<double>> oript = make_shared<OrientPoint<double>>(mid, normal, edge);
        oript->tiltSign = id % 2 == 0? 1 : -1;        oript->updateAngle(angle);
        cross->oriPoints.push_back(oript);

    }

    ConvexBlock<double> block(cross, Eigen::Vector2d(1, 1));
    block.compute();

    block.polyMesh->writeOBJModel("block.obj");
}
