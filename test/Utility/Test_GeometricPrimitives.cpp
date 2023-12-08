#include <catch2/catch_all.hpp>
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
    SECTION("computePtPtDistance"){
        vec = Vector3d(1, 0 ,0);
        REQUIRE(plane.computePtPtDistance(vec) == 1);
        vec = Vector3d(-1, 0 ,0);
        REQUIRE(plane.computePtPtDistance(vec) == 1);
    }

    SECTION("computePtPlnIntersec"){
        vec = Vector3d(1.1e-7, 0 ,0);
        REQUIRE(plane.computePtPlnIntersec(vec) == POINT_PLANE_POSITIVE_SIDE);
        vec = Vector3d(-1.1e-7, 0 ,0);
        REQUIRE(plane.computePtPlnIntersec(vec) == POINT_PLANE_NEGATIVE_SIDE);
        vec = Vector3d(0.9e-7, 0 ,0);
        REQUIRE(plane.computePtPlnIntersec(vec) == POINT_PLANE_INTERSECT);
        vec = Vector3d(-0.9e-7, 0 ,0);
        REQUIRE(plane.computePtPlnIntersec(vec) == POINT_PLANE_INTERSECT);
    }

    SECTION("checkLnPlnIntersec"){
        Vector3d pt1(1, 0, 0);
        Vector3d pt2(-2, -1, 0);
        Line<double> line (pt1, pt2);
        REQUIRE(plane.checkLnPlnIntersec(line) == LINE_PLANE_INTERSECT);

        pt2 = Vector3d(2, -1, 0);
        line = Line<double>(pt1, pt2);
        REQUIRE(plane.checkLnPlnIntersec(line) == LINE_PLANE_POSITIVE_SIDE);
    }

    SECTION("computeLnLnIntersec")
    {
        Vector3d pt1(1, 0, 0);
        Vector3d pt2(-2, -1, 0);
        Line<double> line (pt1, pt2);

        Vector3d intersec(0, 0, 0);
        REQUIRE(plane.computeLnLnIntersec(line, intersec) == LINE_PLANE_INTERSECT);
        REQUIRE(intersec[0] == Catch::Approx(0).margin(1E-6));
        REQUIRE(intersec[1] ==  Catch::Approx (-1.0/3));
    }
}

TEST_CASE("Box"){
    Box<double> box;
    box.minPt = Vector3d(0, 0, 0);
    box.maxPt = Vector3d(1, 1, 1);
    box.computeCenter();
    box.computeSize();
    SECTION("print"){
        box.print();
    }

    SECTION("computeCenter"){
        REQUIRE(box.cenPt.x() == 0.5);
        REQUIRE(box.cenPt.y() == 0.5);
        REQUIRE(box.cenPt.z() == 0.5);
    }

    SECTION("computeSize"){
        REQUIRE(box.size.x() == 1);
        REQUIRE(box.size.y() == 1);
        REQUIRE(box.size.z() == 1);
    }

    SECTION("executeTransform"){
        box.executeTransform(Vector3d(1, 1.5, 2.5), Vector3d(2, 3, 4));
        REQUIRE(box.cenPt.x() == 2);
        REQUIRE(box.cenPt.y() == 3);
        REQUIRE(box.cenPt.z() == 4.5);
    }

    SECTION("computeQuadArea"){
        box.maxPt = Vector3d(1, 1, 1.1e-7);
        REQUIRE(box.computeQuadArea() == 0);

        box.maxPt = Vector3d(1, 1, 0.9e-7);
        REQUIRE(box.computeQuadArea() == 1);
    }
}

TEST_CASE("Triangle"){
    Triangle<double> tri;

    tri.init(Vector3d(1, 0, 0), Vector3d(0, 1, 0), Vector3d(0, 0, 1));

    SECTION("checkEqual"){
        REQUIRE(tri.checkEqual(tri) == true);
    }

    SECTION("print"){
        tri.print();
    }

    SECTION("computeBBoxMinPt"){
        Vector3d bbox = tri.computeBBoxMinPt();
        CHECK(bbox.x() == 0);
        CHECK(bbox.y() == 0);
        CHECK(bbox.z() == 0);
    }

    SECTION("computeBBoxMaxPt"){
        Vector3d bbox = tri.computeBBoxMaxPt();
        CHECK(bbox.x() == 1);
        CHECK(bbox.y() == 1);
        CHECK(bbox.z() == 1);
    }

    SECTION("computeCenter"){
        CHECK(tri.computeCenter().x() == Catch::Approx (1.0 / 3));
        CHECK(tri.computeCenter().y() == Catch::Approx (1.0 / 3));
        CHECK(tri.computeCenter().z() == Catch::Approx (1.0 / 3));
    }

    SECTION("computeArea"){
        CHECK(tri.computeArea() == Catch::Approx (sqrt(3) / 2));
    }

    SECTION("computeSignedArea"){
        CHECK(tri.computeSignedArea() == Catch::Approx (0.5));
    }

    SECTION("computeNormal"){
        CHECK((tri.computeNormal() - Vector3d(1.0/sqrt(3), 1.0/sqrt(3), 1.0/sqrt(3))).norm() == Catch::Approx (0));
    }

    SECTION("correctNormal"){
        tri.correctNormal(Vector3d(-1, -1, -1));
        CHECK((tri.v[0] - Vector3d(1, 0, 0)).norm() == Catch::Approx (0));
        CHECK((tri.v[1] - Vector3d(0, 0, 1)).norm() == Catch::Approx (0));
        CHECK((tri.v[2] - Vector3d(0, 1, 0)).norm() == Catch::Approx (0));
    }

}

TEST_CASE("OrientPoint")
{
    OrientPoint<double> oriPt(Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0));
    oriPt.print();
    oriPt.tiltSign = 1;
    oriPt.updateAngle(10.0);
    REQUIRE(oriPt.normal[0] == Catch::Approx (std::cos(10.0 * M_PI / 180)));
    REQUIRE(oriPt.normal[1] == 0);
    REQUIRE(oriPt.normal[2] == Catch::Approx (std::sin(-10.0 * M_PI / 180)));
}