#include "JsonIO.h"
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

    vector<double> textureMatData;
    for(int id = 0; id < 4; id ++){
        for(int jd = 0; jd < 4; jd++){
            textureMatData.push_back(data.lock()->textureMat(id, jd));
        }
    }
    parameter_json["textureMat"] = textureMatData;
    parameter_json["boundary CrossIDs"] = data.lock()->boundary_crossIDs;

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

    std::ofstream fileout(path);
    if(fileout){
        fileout << std::setw(4) << result << std::endl;
    }
}
