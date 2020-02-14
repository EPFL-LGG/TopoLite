//
// Created by ziqwang on 2020-02-14.
//

#include <catch2/catch.hpp>
#include "Mesh/Polygon.h"

typedef std::vector<Eigen::Vector3d> PolyVector3d;

TEST_CASE("Polygon")
{
    _Polygon<double> poly;

    REQUIRE(poly.vers.empty());

    SECTION("setVertices"){
        PolyVector3d pts;
        pts.push_back(Vector3d(0, 0, 0));
        pts.push_back(Vector3d(1, 0, 0));
        pts.push_back(Vector3d(1, 1, 0));
        pts.push_back(Vector3d(0, 1, 0));

        poly.setVertices(pts);
        REQUIRE(poly.vers.size() == 4);
        REQUIRE((poly.vers[1].pos - Vector3d(1, 0, 0)).norm() == Approx(0));
    }

    SECTION("push_back"){
        poly.push_back(Vector3d(0, 0, 0));
        poly.push_back(Vector3d(1, 0, 0));
        poly.push_back(Vector3d(1, 1, 0));
        poly.push_back(Vector3d(0, 1, 0));

        REQUIRE(poly.vers.size() == 4);
        REQUIRE((poly.vers[1].pos - Vector3d(1, 0, 0)).norm() == Approx(0));
    }

    SECTION("the polygon has a simple input")
    {
        poly.push_back(Vector3d(0, 0, 0));
        poly.push_back(Vector3d(1, 0, 0));
        poly.push_back(Vector3d(1, 1, 0));
        poly.push_back(Vector3d(0, 1, 0));

        SECTION("reverseVertices"){
            poly.reverseVertices();

            REQUIRE(poly.vers.size() == 4);
            REQUIRE((poly.vers[1].pos - Vector3d(1, 1, 0)).norm() == Approx(0));
        }

        SECTION("checkEquality"){
            _Polygon<double> pB;
            pB.push_back(Vector3d(0, 0, 0));
            pB.push_back(Vector3d(1, 0, 0));
            pB.push_back(Vector3d(1, 1, 0));
            pB.push_back(Vector3d(0, 1, 0));
            pB.reverseVertices();

            REQUIRE(poly.checkEquality(pB) == false);

            pB.clear();
            pB.push_back(Vector3d(1, 1, 0));
            pB.push_back(Vector3d(0, 1, 0));
            pB.push_back(Vector3d(0, 0, 0));
            pB.push_back(Vector3d(1, 0, 0));

            REQUIRE(poly.checkEquality(pB) == true);
        }

        SECTION("print"){
            poly.print();
        }

        SECTION("computeCenter"){
            REQUIRE((poly.computeCenter() - Vector3d(0.5, 0.5, 0)).norm() < FLOAT_ERROR_SMALL);
        }

        SECTION("computeNormal"){
            REQUIRE((poly.computeNormal() - Vector3d(0, 0, 1)).norm() < FLOAT_ERROR_SMALL);
        }
    }
}

