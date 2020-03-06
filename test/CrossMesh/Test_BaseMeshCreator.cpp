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
#include "CrossMesh/PatternCreator.h"

TEST_CASE("BaseMeshCreator")
{
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVarLite(varList.get());

    std::shared_ptr<PolyMesh_AABBTree<double>> _polyMesh;
    std::shared_ptr<CrossMesh<double>> _pattern2D;

    XMLIO IO;

    SECTION("origin.xml")
    {
        // read xml
        filesystem::path xmlFileName(UNITTEST_DATAPATH);
        xmlFileName = xmlFileName / "TopoInterlock/XML/origin.xml";
        XMLData data;
        IO.XMLReader(xmlFileName.str(), data);

        // read polyMesh
        filesystem::path surface_objfile(UNITTEST_DATAPATH);
        surface_objfile = surface_objfile / "TopoInterlock/XML/origin_data/origin_Surface.obj";

        bool texturedModel;
        _polyMesh = make_shared<PolyMesh_AABBTree<double>>(data.varList);
        _polyMesh->readOBJModel(surface_objfile.str().c_str(), texturedModel, true);

        _polyMesh->buildTexTree();

        // create pattern
        PatternCreator<double> patternCreator(varList);
        //patternCreator.create2DPattern(CROSS_HEXAGON, data.varList->get<int>("patternRadius"), _pattern2D);
        patternCreator.create2DPattern(CROSS_HEXAGON, 50, _pattern2D);

        //build baseMeshCreator
        BaseMeshCreator<double> baseMeshCreator(_polyMesh, _pattern2D);

        // create internal cross
        shared_ptr<PolyMesh<double>> baseMesh2D;
        shared_ptr<CrossMesh<double>> crossMesh;

        Eigen::Matrix4d interactMat;
        interactMat << data.interactMatrix[0], data.interactMatrix[4], data.interactMatrix[8], data.interactMatrix[12],
	    data.interactMatrix[1], data.interactMatrix[5], data.interactMatrix[9], data.interactMatrix[13],
	    data.interactMatrix[2], data.interactMatrix[6], data.interactMatrix[10], data.interactMatrix[14],
	    data.interactMatrix[3], data.interactMatrix[7], data.interactMatrix[11], data.interactMatrix[15];

        SECTION("baseMeshCreator computeBaseCrossMesh")
        {
            baseMeshCreator.computeBaseCrossMesh(interactMat, baseMesh2D, crossMesh);
            crossMesh->writeOBJModel("Pattern/origin.obj");
        }

        SECTION("getTextureCoord"){
            Eigen::Matrix4d textureMat = baseMeshCreator.computeTextureMat(_polyMesh, interactMat);

            // map 2D pattern vertices on 3D input surface
            for(size_t id = 0; id < _pattern2D->vertexList.size(); id++) {
                if (_pattern2D->vertexList[id] == nullptr) continue;

                // 1) get the interactive position of each vertex.
                Vector2d ver_2DCoord = _pattern2D->vertexList[id]->pos.head(2);
                Vector2d tex_2DCoord = baseMeshCreator.getTextureCoord(ver_2DCoord, textureMat);
                _pattern2D->vertexList[id]->pos.x() = tex_2DCoord[0];
                _pattern2D->vertexList[id]->pos.y() = tex_2DCoord[1];
                _pattern2D->vertexList[id]->pos.z() = 0;
            }

            _pattern2D->writeOBJModel("Pattern/texture.obj");
            _polyMesh->getTextureMesh()->writeOBJModel("Pattern/polymesh.obj");
        }
    }

}