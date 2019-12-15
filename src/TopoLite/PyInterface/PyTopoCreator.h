//
// Created by ziqwang on 2019-12-11.
//

#ifndef TOPOLITE_PYTOPOCREATOR_H
#define TOPOLITE_PYTOPOCREATOR_H

#include "IO/XMLIO.h"
#include "PyParamList.h"
#include "PyPolyMesh.h"

class PyTopoCreator{
public:

    string xml_path;
    shared_ptr<XMLData> data;
public:

    PyTopoCreator(const std::string &path):xml_path(path){
        XMLIO Reader;
        data = make_shared<XMLData>();
        Reader.XMLReader(xml_path, *data);
    }

    int numParts(){
        if(data && data->strucCreator && data->strucCreator->struc)
            return data->strucCreator->struc->partList.size();
        else
            return 0;
    }

public:

    vector<PyPolyMesh> getStruc()
    {
        vector<PyPolyMesh> pyPolyMeshes;
        if(data && data->strucCreator && data->strucCreator->struc){
            shared_ptr<Struc> struc = data->strucCreator->struc;
            for(pPart part : struc->partList){
                if(part)
                    pyPolyMeshes.push_back(PyPolyMesh(part->polyMesh, part->atBoundary));
            }
        }
        return pyPolyMeshes;
    }

    PyParamList getParamList(){
        if(data && data->varList){
            return PyParamList(data->varList);
        }
        return PyParamList();
    }
};


#endif //TOPOLITE_PYTOPOCREATOR_H
