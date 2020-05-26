//
// Created by ziqwang on 25.05.20.
//

#ifndef TOPOLITE_IODATA_H
#define TOPOLITE_IODATA_H

#include "TopoLite/IO/InputVar.h"
#include "TopoLite/Mesh/CrossMesh.h"

struct InteractData{
    double angle;
    double x, y;
    double scale;
};

struct InputMeshNormalizeData{
    Vector3f trans;
    float scale;
};

struct XMLData{
    shared_ptr<InputVarList> varList;
    shared_ptr<PolyMesh<double>> reference_surface;
    shared_ptr<CrossMesh<double>> cross_mesh;
    vector<int> boundary_crossIDs;

    //old data
    vector<int> pickPartIDs;
    double interactMatrix[16];

    //for rhino dll
    InteractData interact_delta;
    InputMeshNormalizeData normalizedData;
};

#endif //TOPOLITE_IODATA_H
