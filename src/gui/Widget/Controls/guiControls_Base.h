//
// Created by ziqwang on 06.06.20.
//

#ifndef TOPOLITE_GUICONTROLS_BASE_H
#define TOPOLITE_GUICONTROLS_BASE_H
#include "IO/InputVar.h"
class guiControls_Base{
public:
    InputVar *var;
public:
    guiControls_Base(InputVar *_var){
        var = _var;
    }

    virtual void update_gui(){}
};

#endif //TOPOLITE_GUICONTROLS_BASE_H
