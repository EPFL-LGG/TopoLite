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
            debugxml_filepath = current_path / "data/TopoInterock/XML/origin.xml";
        }
        else
        {
            debugxml_filepath = current_path / "../data/TopoInterock/XML/origin.xml";
        }

        REQUIRE(Reader.XMLReader(debugxml_filepath.string(), data) == 1);
        Reader.XMLWriter(debugxml_filepath.string(), data);
    }
}