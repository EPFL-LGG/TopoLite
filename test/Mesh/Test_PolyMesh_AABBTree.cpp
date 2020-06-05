//
// Created by ziqwang on 2020-02-22.
//


#include <catch2/catch.hpp>
#include <TopoLite/Mesh/PolyMesh_AABBTree.h>
using Eigen::Vector3d;
using Eigen::Vector2d;
TEST_CASE("PolyMesh_AABBTree - Test findTexPoint")
{
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVar(varList.get());
    shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);

    SECTION("findTexPoint after object read") {
        // This case is more difficult to understand/read as the data inputs are complicated
        polyMesh->readOBJModel("data/Mesh/primitives/Icosphere.obj", true);
        PolyMesh_AABBTree<double> aabbTree(*polyMesh);
        aabbTree.buildTexTree();

        auto res = aabbTree.findTexPoint(Vector2d(0.2, 0.2));
        auto texPoint = res->texs[0];
        REQUIRE(res->getPolyType() == POLY_NONE);
        REQUIRE(texPoint->texCoord == Vector2d(0.208124, 0.214326));
        REQUIRE(texPoint->texID == 11);
    }

    SECTION("findTexPoint for two triangles"){
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
        REQUIRE(aabbTree.findTexPoint(Vector2d(0.0, 0.3)) == aabbTree.polyList[1]);
        REQUIRE(aabbTree.findTexPoint(Vector2d(0.1, 0.0)) == aabbTree.polyList[0]);
        REQUIRE(aabbTree.findTexPoint(Vector2d(0.0, 0.0)) != nullptr);                  // These pts are in p0 and p1
        REQUIRE(aabbTree.findTexPoint(Vector2d(0.5, 0.5)) != nullptr);                  // we don't care which one it is
        REQUIRE(aabbTree.findTexPoint(Vector2d(0.6, 0.5)) == nullptr);                  // outside
    }
}

TEST_CASE("AABBTree_Line Quads - Test findIntersec") {
    AABBTree_Line<double> aabb_line;
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVar(varList.get());
    shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);

    shared_ptr<_Polygon<double>> poly = make_shared<_Polygon<double>>();
    poly->push_back(Vector3d(0, 0, 0), Vector2d(0, 0));
    poly->push_back(Vector3d(1, 0, 0), Vector2d(1, 0));
    poly->push_back(Vector3d(1, 1, 0), Vector2d(1, 1));
    poly->push_back(Vector3d(0, 1, 0), Vector2d(0, 1));
    vector<shared_ptr<_Polygon<double>>> polyLists;
    polyLists.push_back(poly);
    polyMesh->setPolyLists(polyLists);
    AABBTree_Line<double>::MatrixX V, T;
    AABBTree_Line<double>::MatrixXi F;

    polyMesh->convertPosTexToEigenMesh(V, T, F);
    aabb_line.init(V, T, F);
    Line<double> line;

    Vector2d tex;
    Vector3d pos;

    SECTION("Test findIntersec") {

        line.point1 = Vector3d(0.5, 0.5, 0);
        line.point2 = Vector3d(-1, -1, 0);
        aabb_line.findIntersec(line, tex, pos);
        REQUIRE(tex.x() == Approx(0.0));
        REQUIRE(tex.y() == Approx(0.0));

        line.point1 = Vector3d(-0.5, 0.0, 0);
        line.point2 = Vector3d(1.0, 1.5, 0);
        aabb_line.findIntersec(line, tex, pos);
        REQUIRE(tex.x() == Approx(0.0));
        REQUIRE(tex.y() == Approx(0.5));

        line.point1 = Vector3d(0.5, 0.5, 0);
        line.point2 = Vector3d(1.5, 0.0, 0);
        aabb_line.findIntersec(line, tex, pos);
        REQUIRE(tex.x() == 1.00);
        REQUIRE(tex.y() == 0.25);

    }
}

TEST_CASE("AABBTree_Line Quads - Find Square has a square hole"){
    AABBTree_Line<double> aabb_line;
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    InitVar(varList.get());
    shared_ptr<PolyMesh<double>> polyMesh = make_shared<PolyMesh<double>>(varList);

    shared_ptr<_Polygon<double>> base = make_shared<_Polygon<double>>();
    base->push_back(Vector3d(0, 0, 0), Vector2d(0, 0));
    base->push_back(Vector3d(1, 0, 0), Vector2d(1, 0));
    base->push_back(Vector3d(1, 1, 0), Vector2d(1, 1));
    base->push_back(Vector3d(0, 1, 0), Vector2d(0, 1));

    vector<shared_ptr<_Polygon<double>>> polyLists;
    for(int id = 0; id < 3; id++){
        for(int jd = 0; jd < 3; jd++)
        {
            // remove central 1x1 square from a 3x3 square
            if(id == jd && id == 1){
                continue;
            }

            shared_ptr<_Polygon<double>> poly = make_shared<_Polygon<double>>(*base);
            poly->translatePolygon(Vector3d(id, jd, 0));
            poly->translatePolygonTex(Vector2d(id, jd));
            polyLists.push_back(poly);
        }
    }

    polyMesh->setPolyLists(polyLists);
    polyMesh->update();
    AABBTree_Line<double>::MatrixX V, T;
    AABBTree_Line<double>::MatrixXi F;

    polyMesh->convertPosTexToEigenMesh(V, T, F);
    aabb_line.init(V, T, F);

    SECTION("Test findIntersect"){
        Line<double> line;
        line.point1 = Vector3d(1.5, 0.5, 0);
        line.point2 = Vector3d(1.5, 2.9, 0);
        Vector2d tex;
        Vector3d pos;
        aabb_line.findIntersec(line, tex, pos);
        REQUIRE(tex.x() == Approx(1.5));
        REQUIRE(tex.y() == Approx(1));

        line.point1 = Vector3d(1.5, 2.9, 0);
        line.point2 = Vector3d(1.5, 0.5, 0);
        aabb_line.findIntersec(line, tex, pos);
        REQUIRE(tex.x() == Approx(1.5));
        REQUIRE(tex.y() == Approx(2));

        line.point1 = Vector3d(1.5, 3.0, 0);
        line.point2 = Vector3d(1.5, 0.5, 0);
        aabb_line.findIntersec(line, tex, pos);
        REQUIRE(tex.x() == Approx(1.5));
        REQUIRE(tex.y() == Approx(3));

        line.point1 = Vector3d(1.5, 0.5, 0);
        line.point2 = Vector3d(1.5, 3.0, 0);
        aabb_line.findIntersec(line, tex, pos);
        REQUIRE(tex.x() == Approx(1.5));
        REQUIRE(tex.y() == Approx(1));

        line.point1 = Vector3d(1.5, 1.5, 0);
        line.point2 = Vector3d(1.5, 3, 0);
        aabb_line.findIntersec(line, tex, pos);
        REQUIRE(tex.x() == Approx(1.5));
        REQUIRE(tex.y() == Approx(2));

        line.point1 = Vector3d(1.5, 2.5, 0);
        line.point2 = Vector3d(1.5, 3, 0);
        aabb_line.findIntersec(line, tex, pos);
        REQUIRE(tex.x() == Approx(1.5));
        REQUIRE(tex.y() == Approx(3));
    }

    SECTION("Test checkLineBoxIntersec") {
        Line<double> line;
        Line<double> box_line;


        box_line.point1 = Vector3d(0.0, 0.0, 0.0);
        box_line.point2 = Vector3d(1.0, 1.0, 0.0);
        Box<double> box = Box<double>(box_line);

        line.point1 = Vector3d(0.0, 0.0, 0);
        line.point2 = Vector3d(1.0, 0.5, 0);
        REQUIRE(aabb_line.checkLineBoxIntersec(line, box) == true);

        // We are dealing with 2D boxes and lines
        // - If z-axis was taken in account, this test should be false.
        line.point1 = Vector3d(0.0, 0.0, 3.0);
        line.point2 = Vector3d(1.0, 0.5, 3.0);
        REQUIRE(aabb_line.checkLineBoxIntersec(line, box) == true);

        line.point1 = Vector3d(1.1, 0.0, 0);
        line.point2 = Vector3d(2.1, 1.5, 0);
        REQUIRE(aabb_line.checkLineBoxIntersec(line, box) == false);

        line.point1 = Vector3d(-0.9, 2.0, 0);
        line.point2 = Vector3d(-0.4, -1.0, 0);
        REQUIRE(aabb_line.checkLineBoxIntersec(line, box) == false);
    }
    SECTION("Test checkRayBoxIntersec") {/* Same as testing checkLineBoxIntersec}*/}

    SECTION("Test checkLineLineIntersec") {
        Line<double> line1;
        Line<double> line2;
        Vector2d intersec;

        // Intersect
        line1.point1 = Vector3d(0.0, 0.0, 0.0);
        line1.point2 = Vector3d(1.0, 1.0, 0.0);

        line2.point1 = Vector3d(0.0, 1.0, 0);
        line2.point2 = Vector3d(1.0, 0.0, 0);
        bool are_intersecting = aabb_line.checkLineLineIntersec(line1, line2, intersec);

        REQUIRE(are_intersecting);
        REQUIRE(intersec == Vector2d(0.5, 0.5));

        // Special case
        are_intersecting = aabb_line.checkLineLineIntersec(line1, line1, intersec);
        REQUIRE(!are_intersecting);


        // Don't intersect
        line1.point1 = Vector3d(1.0, 0.0, 0.0);
        line1.point2 = Vector3d(1.0, 0.0, 0.0);

        line2.point1 = Vector3d(0.0, 0.0, 0);
        line2.point2 = Vector3d(0.0, 0.0, 0);
        are_intersecting = aabb_line.checkLineLineIntersec(line1, line2, intersec);

        REQUIRE(!are_intersecting);

        // Close to intersect
        line1.point1 = Vector3d(0.0, 0.0, 0.0);
        line1.point2 = Vector3d(1.0, 1.0, 0.0);

        line2.point1 = Vector3d(0.0000000000, 1.000000000, 0);
        line2.point2 = Vector3d(0.4999999999, 0.500000001, 0);
        are_intersecting = aabb_line.checkLineLineIntersec(line1, line2, intersec);

        REQUIRE(!are_intersecting);
    }
}
