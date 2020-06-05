//
// Created by ziqwang on 2020-03-01.
//

#include <catch2/catch.hpp>
#include "IO/JsonIOWriter.h"
#include "IO/XMLIO_backward.h"
#include <filesystem>
using json = nlohmann::json;

TEST_CASE("Check InputVarManager Json Write"){
    shared_ptr<IOData> data = make_shared<IOData>();
    InitVar(data->varList.get());
    InputVarManager manager;
    std::string name;
    json node;
    std::tie(node, name) = manager.getJSON(data->varList->find("tiltAngle"));
    json dict;
    dict[name] = node;
    auto answer = R"({
                        "tiltAngle": {
                            "group": "Block",
                            "label": "Vector Angle",
                            "range": [
                                    0.0,
                                    90.0
                                ],
                            "type": "float",
                            "value": 20.0
                        }
                    })"_json;
    REQUIRE(dict == answer);
}

TEST_CASE("Test Write Parameter"){
    shared_ptr<IOData> data = make_shared<IOData>();
    InitVar(data->varList.get());
    std::filesystem::path jsonFileName(UNITTEST_DATAPATH);
    jsonFileName = jsonFileName / "TopoInterlock/Json/origin.json";
    std::string path = jsonFileName;
    JsonIOWriter writer(path, data);
    writer.getParameterJson().dump(4);
}

TEST_CASE("Test Write Mesh"){
    shared_ptr<IOData> data = make_shared<IOData>();
    InitVar(data->varList.get());
    shared_ptr<PolyMesh<double>> mesh;
    mesh = make_shared<PolyMesh<double>>(data->varList);
    std::filesystem::path dataFolder(UNITTEST_DATAPATH);
    std::filesystem::path filepath = dataFolder / "Mesh/primitives/Icosphere.obj";
    mesh->readOBJModel(filepath.string().c_str(), false);
    mesh->dump().dump(4);
}

TEST_CASE("Test Write CrossMesh"){
    XMLIO_backward xmlio;
    std::filesystem::path xmlFileName(UNITTEST_DATAPATH);
    xmlFileName = xmlFileName / "TopoInterlock/XML/origin.xml";
    IOData data;
    xmlio.XMLReader(xmlFileName, data);
    data.cross_mesh->dump().dump(4);
}

TEST_CASE("Test Write whole json")
{
    XMLIO_backward xmlio;
    std::filesystem::path xmlFileName(UNITTEST_DATAPATH);
    xmlFileName = xmlFileName / "TopoInterlock/XML/origin.xml";
    shared_ptr<IOData> data;
    data = make_shared<IOData>();
    xmlio.XMLReader(xmlFileName, *data);

    std::filesystem::path jsonFileName(UNITTEST_DATAPATH);
    jsonFileName = jsonFileName / "TopoInterlock/Json/origin.json";
    std::string path = jsonFileName;
    JsonIOWriter writer(path, data);
    writer.write();
}