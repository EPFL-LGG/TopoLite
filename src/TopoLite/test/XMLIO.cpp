//
// Created by ziqwang on 22.02.19.
//

#include "IO/XMLIO.h"
#include <sstream>
#include <boost/algorithm/string.hpp>

#include <catch2/catch.hpp>
#include "Interlocking/ContactGraph.h"

TEST_CASE("Class XMLIO")
{

    SECTION("Read XML")
    {
        XMLIO Reader;
        XMLData data;

        boost::filesystem::path current_path(boost::filesystem::current_path());
        boost::filesystem::path debugxml_filepath;
        if (current_path.filename() == "TopoLite")
        {
            debugxml_filepath = current_path / "data/TopoInterlock/XML/origin.xml";
        }
        else
        {
            debugxml_filepath = current_path / "../data/TopoInterlock/XML/origin.xml";
        }

        REQUIRE(Reader.XMLReader(debugxml_filepath.string(), data) == 1);
        REQUIRE(data.strucCreator->struc->partList.size() == 103);

        pCross cross = data.strucCreator->crossMeshCreator->crossMesh->crossList[0];

        REQUIRE(data.varList->get<bool>("texturedModel"));
        REQUIRE(cross->oriPoints[0]->rotation_angle == 20);
        REQUIRE(cross->oriPoints[1]->rotation_angle == -20);
        REQUIRE(cross->oriPoints[2]->rotation_angle == 20);

        Reader.XMLWriter(debugxml_filepath.string(), data);
    }
}