//
// Created by ziqwang on 2020-02-22.
//

#include <catch2/catch.hpp>
#include <CrossMesh/BaseMeshCreator.h>
#include "Mesh/PolyMesh.h"
#include "Mesh/CrossMesh.h"
#include "Mesh/PolyMesh_AABBTree.h"
#include "filesystem/path.h"
#include "filesystem/resolver.h"
#include "IO/XMLIO.h"

TEST_CASE("BaseMeshCreator")
{
    std::shared_ptr<PolyMesh_AABBTree<double>> _polyMesh;
    std::shared_ptr<CrossMesh<double>> _pattern2D;
    BaseMeshCreator<double> baseMeshCreator(_polyMesh, _pattern2D);
    XMLIO IO;

    SECTION("origin.xml"){
        //read xml
        filesystem::path xmlFileName(UNITTEST_DATAPATH);
        xmlFileName = xmlFileName / "TopoInterlock/XML/origin.xml";
        XMLData data;
        IO.XMLReader(xmlFileName.str(), data);

        //read _polyMesh
        filesystem::path surface_objfile(UNITTEST_DATAPATH);
        surface_objfile = surface_objfile / "TopoInterlock/XML/origin_data/origin_Surface.obj";

        bool texturedModel;
        _polyMesh = make_shared<PolyMesh_AABBTree<double>>(data.varList);
        _polyMesh->readOBJModel(surface_objfile.str().c_str(), texturedModel, true);
//        std::shared_ptr<PolyMesh<double>> outBaseMesh2D;
//        std::shared_ptr<CrossMesh<double>> outCrossMesh2D;
//        baseMeshCreator.computeBaseCrossMesh(interactive, outBaseMesh2D, outCrossMesh2D);
    }

}