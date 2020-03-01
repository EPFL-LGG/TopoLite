//
// Created by ziqwang on 2020-03-01.
//

#include <catch2/catch.hpp>
#include "IO/XMLIO.h"
TEST_CASE("Test_XMLIO"){
    XMLIO IO;
    SECTION("split"){
        vector<std::string> split_strings = IO.split("10, 20, 30, ,", ',');
        REQUIRE(split_strings.size() == 3);
        split_strings = IO.split(",,,,,,,,,", ',');
        REQUIRE(split_strings.size() == 0);
    }

    SECTION("XMLReader_GUISettings"){
        string xmlFileName = UNITTEST_DATAPATH;
        REQUIRE(IO.xmldoc.load_file(xmlFileName.c_str()));
    }
}