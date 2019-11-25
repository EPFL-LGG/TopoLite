//
// Created by ziqwang on 2019-11-21.
//

#ifndef TOPOLITE_TOPOOBJECT_H
#define TOPOLITE_TOPOOBJECT_H

#include <IO/gluiVar.h>
#include "TopoAssert.h"

class TopoObject {
public:

    TopoObject(){
        varList = nullptr;
    }

    TopoObject(shared_ptr<gluiVarList> _var){
        varList = _var;
    }

    TopoObject(const TopoObject &obj){
        varList = obj.varList;
    }

public:

    shared_ptr<gluiVarList> getVarList(){
        TopoASSERT(varList != nullptr);
        return varList;
    }

    void setVarList(shared_ptr<gluiVarList> var){
        TopoASSERT(var != nullptr);
        varList = var;
    }

public:
    virtual bool checkCoherency(){
        return true;
    }

private:
    shared_ptr<gluiVarList> varList;
};


#endif //TOPOLITE_TOPOOBJECT_H
