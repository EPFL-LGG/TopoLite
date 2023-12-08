//
// Created by ziqwang on 05.06.20.
//

#include "IO/JsonIOReader.h"
#include <catch2/catch_all.hpp>

TEST_CASE("Test Read json")
{
    shared_ptr<IOData> data = make_shared<IOData>();
    path jsonFileName(UNITTEST_DATAPATH);
    jsonFileName = jsonFileName / "TopoInterlock/Json/origin.json";
    std::string path = jsonFileName;
    JsonIOReader reader(path, data);
    reader.read();

    SECTION("varList"){
        REQUIRE(data->varList->getIntList("boundary_crossIDs").size() == 42);
        REQUIRE(data->varList->getMatrix4d("texturedMat")(0, 0) == Catch::Approx(1.76946));
        REQUIRE(data->varList->getInt("layerOfBoundary") == 1);
    }

    SECTION("pattern mesh"){

    }

//    std::cout << data->varList->getIntList("boundary_crossIDs").size() << std::endl;
//    std::cout << data->varList->getMatrix4d("texturedMat") << std::endl;
}
