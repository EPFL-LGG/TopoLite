//
// Created by ziqwang on 29.10.18.
//

#ifndef TOPOLOCKCREATOR_GLUIXML_H
#define TOPOLOCKCREATOR_GLUIXML_H

#include "TopoLite/Utility/HelpDefine.h"
#include "IO/IOData.h"

#include <string>
#include <nlohmann/json.hpp>
#include <filesystem>

class JsonIOReader
{
public:

    std::filesystem::path path;
    weak_ptr<IOData> data;


public:

    JsonIOReader(const std::string input_path, shared_ptr<IOData> _data){
        path = input_path;
        data = _data;
    }

public:

    bool read();

    bool readParameter(const nlohmann::json& parameter_json);

    bool readPatternMesh(const nlohmann::json& mesh_json);

    bool readReferenceMesh(const nlohmann::json& mesh_json);

    bool readCrossMesh(const nlohmann::json& mesh_json);

};

#endif //TOPOLOCKCREATOR_GLUIXML_H