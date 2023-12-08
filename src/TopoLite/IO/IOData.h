//
// Created by ziqwang on 25.05.20.
//

#ifndef TOPOLITE_IODATA_H
#define TOPOLITE_IODATA_H

#include "IO/InputVar.h"
#include "Mesh/CrossMesh.h"

struct InteractData{
    double angle;
    double x, y;
    double scale;
};

struct InputMeshNormalizeData{
    Vector3f trans;
    float scale;
};

struct IOData{
public:
    IOData(){
        varList = make_shared<InputVarList>();
        InitVar(varList.get());
    }

public:

    shared_ptr<InputVarList> varList;
    shared_ptr<PolyMesh<double>> reference_surface;
    shared_ptr<PolyMesh<double>> pattern_mesh;
    shared_ptr<CrossMesh<double>> cross_mesh;

    //backward data
    vector<int> boundary_crossIDs;
    vector<int> pickPartIDs;
    double interactMatrix[16];

    //for rhino dll
    InteractData interact_delta;
    InputMeshNormalizeData normalizedData;
};

#endif //TOPOLITE_IODATA_H
