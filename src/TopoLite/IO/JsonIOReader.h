//
// Created by ziqwang on 29.10.18.
//

#ifndef TOPOLOCKCREATOR_GLUIXML_H
#define TOPOLOCKCREATOR_GLUIXML_H

#include "Utility/HelpDefine.h"
#include "IO/IOData.h"

#include <string>
#include <nlohmann/json.hpp>

#if defined(GCC_VERSION_LESS_8)
#include <experimental/filesystem>
    using namespace std::experimental::filesystem;
#else
#include <filesystem>
using namespace std::filesystem;
#endif

class JsonIOReader
{
public:

    path input_path;
    weak_ptr<IOData> data;


public:

    JsonIOReader(const std::string _input_path, shared_ptr<IOData> _data){
        input_path = _input_path;
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