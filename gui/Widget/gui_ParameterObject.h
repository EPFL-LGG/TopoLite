//
// Created by ziqwang on 06.06.20.
//

#ifndef TOPOLITE_GUI_PARAMETEROBJECT_H
#define TOPOLITE_GUI_PARAMETEROBJECT_H
#include "IO/InputVar.h"
class gui_ParameterObject{
public:
    InputVar *var;
public:
    gui_ParameterObject(InputVar *_var){
        var = _var;
    }

    virtual void update_gui(){}
};

#endif //TOPOLITE_GUI_PARAMETEROBJECT_H
