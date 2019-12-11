//
// Created by ziqwang on 2019-12-11.
//

#ifndef TOPOLITE_PYTOPOCREATOR_H
#define TOPOLITE_PYTOPOCREATOR_H

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
};


#endif //TOPOLITE_PYTOPOCREATOR_H
