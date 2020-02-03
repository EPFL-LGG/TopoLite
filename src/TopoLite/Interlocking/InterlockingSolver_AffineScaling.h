//
// Created by ziqwang on 2020-02-03.
//

#ifndef TOPOLITE_INTERLOCKINGSOLVER_AFFINESCALING_H
#define TOPOLITE_INTERLOCKINGSOLVER_AFFINESCALING_H

#include "InterlockingSolver.h"

class InterlockingSolver_AffineScaling : public  InterlockingSolver{

public:
    InterlockingSolver_AffineScaling(shared_ptr<ContactGraph> _graph, shared_ptr<InputVarList> varList)
    : InterlockingSolver(_graph, varList)
    {

    }

public:
    bool isTranslationalInterlocking(shared_ptr<InterlockingData> data);

    bool isRotationalInterlocking(shared_ptr<InterlockingData> data);
};


#endif //TOPOLITE_INTERLOCKINGSOLVER_AFFINESCALING_H
