//
// Created by ziqwang on 05.06.20.
//

#ifndef TOPOLITE_JSONIOWRITER_H
#define TOPOLITE_JSONIOWRITER_H

#include "TopoLite/Utility/HelpDefine.h"
#include "IO/IOData.h"

#include <string>
#include <nlohmann/json.hpp>
#include <filesystem>


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

#endif //TOPOLITE_JSONIOWRITER_H
