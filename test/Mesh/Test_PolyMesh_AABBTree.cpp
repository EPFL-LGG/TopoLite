//
// Created by ziqwang on 2020-02-22.
//


#include <catch2/catch.hpp>
#include <TopoLite/Mesh/PolyMesh_AABBTree.h>
using Eigen::Vector3d;
using Eigen::Vector2d;
TEST_CASE("PolyMesh_AABBTree")
{
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVarLite(varList.get());
    shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);

    SECTION("read obj")
    {
        bool texturedModel;
        polyMesh->readOBJModel("../data/Mesh/primitives/Icosphere.obj", texturedModel, true);
        PolyMesh_AABBTree<double> aabbTree(*polyMesh);
        aabbTree.buildTexTree();
        aabbTree.findTexPoint(Vector2d(0.2, 0.2));
    }

    SECTION("one triangle"){
        shared_ptr<_Polygon<double>> poly = make_shared<_Polygon<double>>();
        poly->push_back(Vector3d(0, 0.5, 0), Vector2d(0, 0));
        poly->push_back(Vector3d(0.25, 1, 0), Vector2d(0.5, 0));
        poly->push_back(Vector3d(0.5, 1, 0), Vector2d(0.5, 0.5));
        vector<shared_ptr<_Polygon<double>>> polyLists;
        polyLists.push_back(poly);
        polyMesh->setPolyLists(polyLists);

        PolyMesh_AABBTree<double> aabbTree(*polyMesh);
        aabbTree.buildTexTree();
        REQUIRE(aabbTree.findTexPoint(Vector2d(0.2, 0.2)) != nullptr);
        REQUIRE(aabbTree.findTexPoint(Vector2d(0, 0)) == polyMesh->polyList[0]);
        REQUIRE(aabbTree.findTexPoint(Vector2d(0.6, 0.5)) == nullptr);
    }

    SECTION("two triangles"){
        shared_ptr<_Polygon<double>> poly = make_shared<_Polygon<double>>();
        poly->push_back(Vector3d(0, 0.5, 0), Vector2d(0, 0));
        poly->push_back(Vector3d(0.25, 1, 0), Vector2d(0.5, 0));
        poly->push_back(Vector3d(0.5, 1, 0), Vector2d(0.5, 0.5));
        vector<shared_ptr<_Polygon<double>>> polyLists;
        polyLists.push_back(poly);

        poly = make_shared<_Polygon<double>>();
        poly->push_back(Vector3d(0, 0.5, 0), Vector2d(0, 0));
        poly->push_back(Vector3d(0.25, 1, 0), Vector2d(0.5, 0.5));
        poly->push_back(Vector3d(0.5, 1, 0), Vector2d(0, 0.5));
        polyLists.push_back(poly);

        polyMesh->setPolyLists(polyLists);

        PolyMesh_AABBTree<double> aabbTree(*polyMesh);
        aabbTree.buildTexTree();
        REQUIRE(aabbTree.findTexPoint(Vector2d(0, 0.3)) == polyMesh->polyList[1]);
        REQUIRE(aabbTree.findTexPoint(Vector2d(0.1, 0)) == polyMesh->polyList[0]);
        REQUIRE(aabbTree.findTexPoint(Vector2d(0, 0)) != nullptr);
        REQUIRE(aabbTree.findTexPoint(Vector2d(0.5, 0.5)) != nullptr);
    }

}

TEST_CASE("AABBTree_Line"){
    AABBTree_Line<double> aabb_line;
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVarLite(varList.get());
    shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);

    SECTION("Quad")
    {
        shared_ptr<_Polygon<double>> poly = make_shared<_Polygon<double>>();
        poly->push_back(Vector3d(0, 0, 0), Vector2d(0, 0));
        poly->push_back(Vector3d(1, 0, 0), Vector2d(1, 0));
        poly->push_back(Vector3d(1, 1, 0), Vector2d(1, 1));
        poly->push_back(Vector3d(0, 1, 0), Vector2d(0, 1));
        vector<shared_ptr<_Polygon<double>>> polyLists;
        polyLists.push_back(poly);
        polyMesh->setPolyLists(polyLists);
        AABBTree_Line<double>::MatrixX V;
        AABBTree_Line<double>::MatrixXi F;

        Eigen::VectorXi C;
        polyMesh->convertTexToEigenMesh(V, F, C);
        aabb_line.init(V, F);
        Line<double> line;
        line.point1 = Vector3d(0.5, 0.5, 0);
        line.point2 = Vector3d(-1, -1, 0);
        Vector2d tex;
        aabb_line.findIntersec(line, tex);
        REQUIRE(tex.x() == Approx(0.0));
        REQUIRE(tex.y() == Approx(0.0));

        line.point1 = Vector3d(0.8, 0.8, 0);
        line.point2 = Vector3d(0.7, 0.7, 0);
        aabb_line.findIntersec(line, tex);
        REQUIRE(tex.x() == Approx(0.0));
        REQUIRE(tex.y() == Approx(0.0));

    }

    SECTION("Square has a square hole"){
        shared_ptr<_Polygon<double>> base = make_shared<_Polygon<double>>();
        base->push_back(Vector3d(0, 0, 0), Vector2d(0, 0));
        base->push_back(Vector3d(1, 0, 0), Vector2d(1, 0));
        base->push_back(Vector3d(1, 1, 0), Vector2d(1, 1));
        base->push_back(Vector3d(0, 1, 0), Vector2d(0, 1));

        vector<shared_ptr<_Polygon<double>>> polyLists;
        for(int id = 0; id < 3; id++){
            for(int jd = 0; jd < 3; jd++)
            {
                //remove 1x1 square from a 3x3 square
                if(id == jd && id == 1){
                    continue;
                }

                shared_ptr<_Polygon<double>> poly = make_shared<_Polygon<double>>(*base);
                poly->translatePolygonTex(Vector2d(id, jd));
                polyLists.push_back(poly);
            }
        }

        polyMesh->setPolyLists(polyLists);
        AABBTree_Line<double>::MatrixX V;
        AABBTree_Line<double>::MatrixXi F;

        Eigen::VectorXi C;
        polyMesh->convertTexToEigenMesh(V, F, C);
        aabb_line.init(V, F);


        Line<double> line;
        line.point1 = Vector3d(1.5, 0.5, 0);
        line.point2 = Vector3d(1.5, 3, 0);
        Vector2d tex;
        aabb_line.findIntersec(line, tex);
        REQUIRE(tex.x() == Approx(1.5));
        REQUIRE(tex.y() == Approx(1));

        line.point1 = Vector3d(1.5, 1.5, 0);
        line.point2 = Vector3d(1.5, 3, 0);
        aabb_line.findIntersec(line, tex);
        REQUIRE(tex.x() == Approx(1.5));
        REQUIRE(tex.y() == Approx(2));

        line.point1 = Vector3d(1.5, 2.5, 0);
        line.point2 = Vector3d(1.5, 3, 0);
        aabb_line.findIntersec(line, tex);
        REQUIRE(tex.x() == Approx(1.5));
        REQUIRE(tex.y() == Approx(3));
    }
}