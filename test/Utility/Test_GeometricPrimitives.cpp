#include <catch2/catch.hpp>
#include <iostream>
#include "Utility/GeometricPrimitives.h"
using Eigen::Vector3d;
TEST_CASE("GeometricPrimitives Point"){
    Point<double> pt;

    SECTION("initialization"){
        REQUIRE(pt.pos[0] == 0);
    }

    SECTION("copy and construct"){
        pt.pos = Vector3d(1, 2, 3);
        pt.nor = Vector3d(3, 2, 3);

        Point<double> pt2;
        pt2 = pt;
        REQUIRE(pt2.pos[1] == 2);
        REQUIRE(pt2.nor[2] == 3);
    }
}

TEST_CASE("GeometricPrimitives Plane")
{
    Plane<double> plane;

    plane.point = Vector3d(0, 0, 0);
    plane.normal = Vector3d(1, 0 ,0);

    Vector3d vec;
    SECTION("PointPlaneDistance"){
        vec = Vector3d(1, 0 ,0);
        REQUIRE(plane.PointPlaneDistance(vec) == 1);
        vec = Vector3d(-1, 0 ,0);
        REQUIRE(plane.PointPlaneDistance(vec) == 1);
    }

    SECTION("PointPlaneIntersect"){
        vec = Vector3d(1.1e-7, 0 ,0);
        REQUIRE(plane.PointPlaneIntersect(vec) == POINT_PLANE_POSITIVE_SIDE);
        vec = Vector3d(-1.1e-7, 0 ,0);
        REQUIRE(plane.PointPlaneIntersect(vec) == POINT_PLANE_NEGATIVE_SIDE);
        vec = Vector3d(0.9e-7, 0 ,0);
        REQUIRE(plane.PointPlaneIntersect(vec) == POINT_PLANE_INTERSECT);
        vec = Vector3d(-0.9e-7, 0 ,0);
        REQUIRE(plane.PointPlaneIntersect(vec) == POINT_PLANE_INTERSECT);
    }

    SECTION("LinePlaneIntersect"){
        Vector3d pt1(1, 0, 0);
        Vector3d pt2(-2, -1, 0);
        Line<double> line (pt1, pt2);
        REQUIRE(plane.LinePlaneIntersect(line) == LINE_PLANE_INTERSECT);

        pt2 = Vector3d(2, -1, 0);
        line = Line<double>(pt1, pt2);
        REQUIRE(plane.LinePlaneIntersect(line) == LINE_PLANE_POSITIVE_SIDE);
    }

    SECTION("LineIntersectPoint")
    {
        Vector3d pt1(1, 0, 0);
        Vector3d pt2(-2, -1, 0);
        Line<double> line (pt1, pt2);

        Vector3d intersec(0, 0, 0);
        REQUIRE(plane.LineIntersectPoint(line, intersec) == LINE_PLANE_INTERSECT);
        REQUIRE(intersec[0] == 0);
        REQUIRE(intersec[1] ==  Approx(-1.0/3));
    }
}

TEST_CASE("Box"){
    Box<double> box;
    box.minPt = Vector3d(0, 0, 0);
    box.maxPt = Vector3d(1, 1, 1);
    box.GetCenter();
    box.GetSize();
    SECTION("PrintBox"){
        box.PrintBox();
    }

    SECTION("GetCenter"){
        REQUIRE(box.cenPt.x() == 0.5);
        REQUIRE(box.cenPt.y() == 0.5);
        REQUIRE(box.cenPt.z() == 0.5);
    }

    SECTION("GetSize"){
        REQUIRE(box.size.x() == 1);
        REQUIRE(box.size.y() == 1);
        REQUIRE(box.size.z() == 1);
    }

    SECTION("Transform"){
        box.Transform(Vector3d(1, 1.5, 2.5), Vector3d(2, 3, 4));
        REQUIRE(box.cenPt.x() == 2);
        REQUIRE(box.cenPt.y() == 3);
        REQUIRE(box.cenPt.z() == 4.5);
    }

    SECTION("GetQuadArea"){
        box.maxPt = Vector3d(1, 1, 1.1e-7);
        REQUIRE(box.GetQuadArea() == 0);

        box.maxPt = Vector3d(1, 1, 0.9e-7);
        REQUIRE(box.GetQuadArea() == 1);
    }
}

TEST_CASE("Triangle"){
    Triangle<double> tri;

    tri.Init(Vector3d(1, 0, 0), Vector3d(0, 1, 0), Vector3d(0, 0, 1));

    SECTION("IsEqual"){
        REQUIRE(tri.IsEqual(tri) == true);
    }

    SECTION("PrintTriangle"){
        tri.PrintTriangle();
    }

    SECTION("GetBBoxMinPt"){
        Vector3d bbox = tri.GetBBoxMinPt();
        CHECK(bbox.x() == 0);
        CHECK(bbox.y() == 0);
        CHECK(bbox.z() == 0);
    }

    SECTION("GetBBoxMaxPt"){
        Vector3d bbox = tri.GetBBoxMaxPt();
        CHECK(bbox.x() == 1);
        CHECK(bbox.y() == 1);
        CHECK(bbox.z() == 1);
    }

    SECTION("ComputeCenter"){
        CHECK(tri.center.x() == Approx(1.0 / 3));
        CHECK(tri.center.y() == Approx(1.0 / 3));
        CHECK(tri.center.z() == Approx(1.0 / 3));
    }

    SECTION("ComputeArea"){
        CHECK(tri.area == Approx(sqrt(3) / 2));
    }

    SECTION("ComputeNormal"){
        CHECK((tri.normal - Vector3d(1.0/sqrt(3), 1.0/sqrt(3), 1.0/sqrt(3))).norm() == Approx(0));
    }

    SECTION("CorrectNormal"){
        tri.CorrectNormal(Vector3d(-1, -1, -1));
        CHECK((tri.v[0] - Vector3d(1, 0, 0)).norm() == Approx(0));
        CHECK((tri.v[1] - Vector3d(0, 0, 1)).norm() == Approx(0));
        CHECK((tri.v[2] - Vector3d(0, 1, 0)).norm() == Approx(0));
    }

}