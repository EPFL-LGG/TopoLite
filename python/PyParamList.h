//
// Created by ziqwang on 2019-12-15.
//

#ifndef TOPOLITE_PYPARAMLIST_H
#define TOPOLITE_PYPARAMLIST_H

#include "IO/InputVar.h"

class PyParamList{
public:

    shared_ptr<InputVarList> data_;

public:

    PyParamList(){
        data_ = make_shared<InputVarList>();
        InitVar(data_.get());
    }

    PyParamList(shared_ptr<InputVarList> varList){
        data_ = varList;
    }
};

#endif //TOPOLITE_PYPARAMLIST_H
