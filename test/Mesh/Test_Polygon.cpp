//
// Created by ziqwang on 2020-02-14.
//

#include <catch2/catch.hpp>
#include "Mesh/Polygon.h"
#include "Utility/GeometricPrimitives.h"

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

        SECTION("computeFitedPlaneNormal"){
            REQUIRE((poly.computeFitedPlaneNormal() - Vector3d(0, 0, 1)).norm() < FLOAT_ERROR_SMALL);
        }

        SECTION("computeArea"){
            REQUIRE(poly.computeArea() == Approx(1.0));
        }

        SECTION("computeAverageEdge"){
            REQUIRE(poly.computeAverageEdge() == Approx(1.0));
        }

        SECTION("computeMaxRadius"){
            REQUIRE(poly.computeMaxRadius() == Approx(sqrt(2)/2.0));
        }

        SECTION("computeFrame")
        {
            Vector3d x_axis, y_axis, origin;
            poly.computeFrame(x_axis, y_axis, origin);
            REQUIRE(x_axis.dot(poly.computeNormal()) == Approx(0.0));
            REQUIRE(y_axis.dot(poly.computeNormal()) == Approx(0.0));
        }

        SECTION("convertToTriangles"){
            vector<shared_ptr<Triangle<double>>> tris;
            poly.convertToTriangles(tris);
            REQUIRE(tris.size() == 4);
        }

        SECTION("getPtVerID"){
            REQUIRE(poly.getPtVerID(Vector3d(1, 1, 0)) == 2);
        }

        SECTION("executeTranslation"){
            poly.executeTranslation(Vector3d(1, 1, 1));
            REQUIRE((poly[1] - Vector3d(2, 1, 1)).norm() == Approx(0.0));
        }


    }
}

