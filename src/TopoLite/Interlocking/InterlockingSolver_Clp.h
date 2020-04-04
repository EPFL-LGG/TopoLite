//
// Created by ziqwang on 04.04.20.
//

#ifndef TOPOLITE_INTERLOCKINGSOLVER_CLP_H
#define TOPOLITE_INTERLOCKINGSOLVER_CLP_H


class InterlockingSolver_Clp : public  InterlockingSolver{{
public:
    InterlockingSolver_Clp(shared_ptr<ContactGraph> _graph, shared_ptr<InputVarList> varList)
    : InterlockingSolver(_graph, varList)
    {

    }

public:
    bool isTranslationalInterlocking(shared_ptr<InterlockingData> data);

    bool isRotationalInterlocking(shared_ptr<InterlockingData> data);
};


#endif //TOPOLITE_INTERLOCKINGSOLVER_CLP_H
