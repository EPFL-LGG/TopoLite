//
// Created by ziqwang on 25.05.20.
//

#include <catch2/catch_all.hpp>
#include "IO/XMLIO_backward.h"

#if defined(GCC_VERSION_LESS_8)
#include <experimental/filesystem>
    using namespace std::experimental::filesystem;
#else
#include <filesystem>
using namespace std::filesystem;
#endif

TEST_CASE("Read XML"){
    XMLIO_backward xmlio;

    SECTION("XMLReader")
    {
        path xmlFileName(UNITTEST_DATAPATH);
        xmlFileName = xmlFileName / "TopoInterlock/XML/origin.xml";
        IOData data;
        xmlio.XMLReader(xmlFileName, data);
        data.cross_mesh->writeOBJModel("CrossMesh.obj");
        data.reference_surface->writeOBJModel("Reference.obj");
    }
}
