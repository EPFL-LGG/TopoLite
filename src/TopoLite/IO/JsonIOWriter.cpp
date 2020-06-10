//
// Created by ziqwang on 05.06.20.
//

#include "JsonIOWriter.h"
#include <fstream>

nlohmann::json JsonIOWriter::getParameterJson()
{
    nlohmann::json parameter_json;
    InputVarManager manager;

    for(shared_ptr<InputVar> var: data.lock()->varList->varLists){
        std::string name;
        nlohmann::json var_json;
        std::tie(var_json, name) = manager.getJSON(var.get());
        parameter_json[name] = var_json;
    }

    return parameter_json;
}

void JsonIOWriter::write()
{
    nlohmann::json result;
    if(data.lock()){
        //1) parameter
        {
            result["parameter"] = getParameterJson();
        }

        //2) reference surface
        if(data.lock()->reference_surface){
            result["reference surface"] = data.lock()->reference_surface->dump();
        }

        //3) pattern mesh
        if(data.lock()->pattern_mesh){
            result["pattern mesh"] = data.lock()->pattern_mesh->dump();
        }

        //4) cross mesh
        if(data.lock()->cross_mesh){
            result["cross mesh"] = data.lock()->cross_mesh->dump();
        }
    }

    std::ofstream fileout(output_path, std::ofstream::binary);
    std::vector<std::uint8_t> binary_result = nlohmann::json::to_ubjson(result);
    if(fileout){
        fileout.write((char *)binary_result.data(), binary_result.size());
    }
}
