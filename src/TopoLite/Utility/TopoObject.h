//
// Created by ziqwang on 2019-11-21.
//

#ifndef TOPOLITE_TOPOOBJECT_H
#define TOPOLITE_TOPOOBJECT_H

#include "IO/InputVar.h"
#include "TopoAssert.h"

class TopoObject {
public:

    TopoObject(){
        varList = nullptr;
    }

    TopoObject(shared_ptr<InputVarList> _var){
        varList = _var;
    }

    TopoObject(const TopoObject &obj){
        varList = obj.varList;
    }

public:

    shared_ptr<InputVarList> getVarList() const{
        TopoASSERT(varList != nullptr);
        return varList;
    }

    void setVarList(shared_ptr<InputVarList> var){
        TopoASSERT(var != nullptr);
        varList = var;
    }

public:
    virtual bool checkCoherency(){
        return true;
    }

private:
    shared_ptr<InputVarList> varList;
};


#endif //TOPOLITE_TOPOOBJECT_H
