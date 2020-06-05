//
// Created by ziqwang on 25.05.20.
//

#include <catch2/catch.hpp>
#include "IO/XMLIO_backward.h"
#include <filesystem>


TEST_CASE("Read XML"){
    XMLIO_backward xmlio;

    SECTION("XMLReader")
    {
        std::filesystem::path xmlFileName(UNITTEST_DATAPATH);
        xmlFileName = xmlFileName / "TopoInterlock/XML/origin.xml";
        IOData data;
        xmlio.XMLReader(xmlFileName, data);
        data.cross_mesh->writeOBJModel("CrossMesh.obj");
        data.reference_surface->writeOBJModel("Reference.obj");
    }
}
