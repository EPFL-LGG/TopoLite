//
// Created by ziqwang on 2020-02-22.
//

#include <catch2/catch.hpp>
#include "CrossMesh/BaseMeshCreator.h"
#include "CrossMesh/CrossMeshCreator.h"
#include "Mesh/PolyMesh.h"
#include "Mesh/CrossMesh.h"
#include "Mesh/PolyMesh_AABBTree.h"
#include "IO/XMLIO_backward.h"
#include "CrossMesh/PatternCreator.h"

#if defined(GCC_VERSION_LESS_8)
#include <experimental/filesystem>
    using namespace std::experimental::filesystem;
#else
#include <filesystem>
using namespace std::filesystem;
#endif

TEST_CASE("BaseMeshCreator")
{
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVar(varList.get());

    std::shared_ptr<PolyMesh_AABBTree<double>> _polyMesh;
    std::shared_ptr<CrossMesh<double>> _pattern2D;

    XMLIO_backward IO;
    create_directory("Pattern");

    SECTION("origin.xml")
    {
        // read xml
        path xmlFileName(UNITTEST_DATAPATH);
        xmlFileName = xmlFileName / "TopoInterlock/XML/origin.xml";
        IOData data;
        IO.XMLReader(xmlFileName.c_str(), data);

        // read polyMesh
        path surface_objfile(UNITTEST_DATAPATH);
        surface_objfile = surface_objfile / "TopoInterlock/XML/origin_data/origin_Surface.obj";

        _polyMesh = make_shared<PolyMesh_AABBTree<double>>(data.varList);
        _polyMesh->readOBJModel(surface_objfile.c_str(), true);

        _polyMesh->buildTexTree();

        // create pattern
        PatternCreator<double> patternCreator(varList);
        //patternCreator.create2DPattern(CROSS_HEXAGON, data.varList->get<int>("patternRadius"), _pattern2D);
        patternCreator.create2DPattern(CROSS_HEXAGON, 30, _pattern2D);

        //build baseMeshCreator
        BaseMeshCreator<double> baseMeshCreator(_polyMesh, _pattern2D, data.varList);

        // create internal cross
        shared_ptr<PolyMesh<double>> baseMesh2D;
        shared_ptr<CrossMesh<double>> crossMesh;

        Eigen::Matrix4d interactMat;
        interactMat << data.interactMatrix[0], data.interactMatrix[4], data.interactMatrix[8], data.interactMatrix[12],
	    data.interactMatrix[1], data.interactMatrix[5], data.interactMatrix[9], data.interactMatrix[13],
	    data.interactMatrix[2], data.interactMatrix[6], data.interactMatrix[10], data.interactMatrix[14],
	    data.interactMatrix[3], data.interactMatrix[7], data.interactMatrix[11], data.interactMatrix[15];

        CrossMeshCreator<double> crossMeshCreator(varList);
        crossMeshCreator.setReferenceSurface(_polyMesh);
        Eigen::Matrix4d textureMat = crossMeshCreator.computeTextureMat_backwards_compatible(interactMat);

        SECTION("baseMeshCreator computeBaseCrossMesh")
        {
            data.varList->add(0.2f, "minCrossArea", "");
            baseMeshCreator.computeBaseCrossMesh(interactMat, baseMesh2D, crossMesh, false);
            crossMesh->writeOBJModel("Pattern/origin.obj");
            baseMesh2D->writeOBJModel("Pattern/pattern.obj");
            _polyMesh->getTextureMesh()->writeOBJModel("Pattern/polymesh.obj");
        }

        SECTION("getTextureCoord"){

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
        }
    }

    SECTION("removeDanglingCross")
    {
        _pattern2D = make_shared<CrossMesh<double>>(varList);

        int dXY[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};   // 0---3---8
        int verIDs[4][4] = {{0, 1, 2, 3},                   // | 0 | 3 |
                            {1, 4, 5, 2},                   // 1---2---7
                            {2, 5, 6, 7},                   // | 1 | 2 |
                            {3, 2, 7, 8}};                  // 4---5---6


        shared_ptr<Cross<double>> cross;
        for(int id = 0; id < 4; id++)
        {
            cross = make_shared<Cross<double>>(Cross<double>(varList));
            // vertices position vectors
            cross->push_back(Vector3d(0 + dXY[id][0], 0 + dXY[id][1], 0));
            cross->push_back(Vector3d(1 + dXY[id][0], 0 + dXY[id][1], 0));
            cross->push_back(Vector3d(1 + dXY[id][0], 1 + dXY[id][1], 0));
            cross->push_back(Vector3d(0 + dXY[id][0], 1 + dXY[id][1], 0));
            _pattern2D->push_back(cross);
        }

        //add a dangling cross
        cross = make_shared<Cross<double>>(Cross<double>(varList));
        cross->push_back(Vector3d(2, 2, 0));
        cross->push_back(Vector3d(3, 2, 0));
        cross->push_back(Vector3d(3, 3, 0));
        cross->push_back(Vector3d(2, 3, 0));
        _pattern2D->push_back(cross);
        cross.reset();

        _pattern2D->update();
        BaseMeshCreator<double> baseMeshCreator(_polyMesh, _pattern2D, varList);
        baseMeshCreator.removeDanglingCross(_pattern2D);
        _pattern2D->erase_nullptr();

        //reduce the number of crosses as well as the number of vertices
        REQUIRE(_pattern2D->size() == 4);
        REQUIRE(_pattern2D->vertexList.size() == 9);
    }

    SECTION("recomputeBoundary"){
        _pattern2D = make_shared<CrossMesh<double>>(varList);
        shared_ptr<Cross<double>> cross;
        for(int id = 0; id < 5; id++)
        {
            for(int jd = 0; jd < 5; jd++){
                cross = make_shared<Cross<double>>(Cross<double>(varList));
                // vertices position vectors
                cross->push_back(Vector3d(id, jd, 0));
                cross->push_back(Vector3d(id + 1, jd, 0));
                cross->push_back(Vector3d(id + 1, jd + 1, 0));
                cross->push_back(Vector3d(id, jd + 1, 0));
                _pattern2D->push_back(cross);
            }
        }
        _pattern2D->update();

        REQUIRE(_pattern2D->cross(12)->atBoundary == false);

        varList->add((int)2, "layerOfBoundary", "");
        BaseMeshCreator<double> baseMeshCreator(_polyMesh, _pattern2D, varList);
        baseMeshCreator.recomputeBoundary(_pattern2D);
        REQUIRE(_pattern2D->cross(12)->atBoundary == false);
        REQUIRE(_pattern2D->cross(11)->atBoundary == true);
    }
}
