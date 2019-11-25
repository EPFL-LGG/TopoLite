//
// Created by ziqwang on 07.03.19.
//

#ifndef TOPOLOCKCREATOR_PYINTERFACE_H
#define TOPOLOCKCREATOR_PYINTERFACE_H

#include "IO/XMLIO.h"
#include "IO/InputVar.h"

#include <iostream>
#include <boost/filesystem.hpp>
#include <fstream>

/*
 * GLOBAL VARIABLES
 */
extern gluiVarList varList; //gluiVar
extern double interactMatrix[16]; // pattern settings
extern shared_ptr<StrucCreator> myStrucCreator; // TI Assembly Creator
extern pugi::xml_document xmldoc;
extern vector<int> pickPartIDs;

int ReadFile(string filename);
int SaveFile(string filename);

#endif //TOPOLOCKCREATOR_PYINTERFACE_H
