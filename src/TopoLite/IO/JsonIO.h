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

// TI Assembly creator

using IgnoreList = std::unordered_map<int, bool> ;


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
};

class JsonIOWriter{
public:
    std::filesystem::path path;
    weak_ptr<IOData> data;

public:

    JsonIOWriter(const std::string output_path, shared_ptr<IOData> _data){
        path = output_path;
        data = _data;
    }

    void write();

public:

    nlohmann::json getParameterJson();
};

#endif //TOPOLOCKCREATOR_GLUIXML_H