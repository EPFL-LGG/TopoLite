//
// Created by ziqwang on 05.06.20.
//

#ifndef TOPOLITE_JSONIOWRITER_H
#define TOPOLITE_JSONIOWRITER_H

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

class JsonIOWriter{
public:
    path output_path;
    weak_ptr<IOData> data;

public:

    JsonIOWriter(const std::string _output_path, shared_ptr<IOData> _data){
        output_path = _output_path;
        data = _data;
    }

    void write();

public:

    nlohmann::json getParameterJson();
};

#endif //TOPOLITE_JSONIOWRITER_H
