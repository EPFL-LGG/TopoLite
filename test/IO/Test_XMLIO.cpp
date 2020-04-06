//
// Created by ziqwang on 2020-03-01.
//

#include <catch2/catch.hpp>
#include "IO/XMLIO.h"
#include "filesystem/path.h"
#include "filesystem/resolver.h"

TEST_CASE("Test_XMLIO"){
    XMLIO IO;
    SECTION("split"){
        vector<std::string> split_strings = IO.split("10, 20, 30, ,", ',');
        REQUIRE(split_strings.size() == 3);
        split_strings = IO.split(",,,,,,,,,", ',');
        REQUIRE(split_strings.size() == 0);
    }

    SECTION("XMLReader_GUISettings")
    {
        Wenzel::filesystem::path xmlFileName(UNITTEST_DATAPATH);
        xmlFileName = xmlFileName / "TopoInterlock/XML/origin.xml";
        REQUIRE(IO.xmldoc.load_file(xmlFileName.str().c_str()));
        XMLData data;
        data.varList = make_shared<InputVarList>();
        InitVar(data.varList.get());
        pugi::xml_node xml_root = IO.xmldoc.child("Documents");
        IO.XMLReader_GUISettings(xml_root, data);
        REQUIRE(data.interactMatrix[0] == 2.344562);
    }

    SECTION("XMLReader")
    {
        Wenzel::filesystem::path xmlFileName(UNITTEST_DATAPATH);
        xmlFileName = xmlFileName / "TopoInterlock/XML/origin.xml";
        XMLData data;
        IO.XMLReader(xmlFileName.str(), data);
        REQUIRE(data.interactMatrix[0] == 2.344562);
    }
}