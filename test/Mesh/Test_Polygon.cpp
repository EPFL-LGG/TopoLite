//
// Created by ziqwang on 2020-02-14.
//

#include <catch2/catch_all.hpp>
#include "Mesh/Polygon.h"
#include "Utility/GeometricPrimitives.h"
#include <iostream>

typedef std::vector<Eigen::Vector3d> PolyVector3d;
using Eigen::Vector2d;
TEST_CASE("Polygon")
{
    _Polygon<double> poly;

    REQUIRE(poly.size() == 0);

    SECTION("setVertices"){
        PolyVector3d pts;
        pts.push_back(Vector3d(0, 0, 0));
        pts.push_back(Vector3d(1, 0, 0));
        pts.push_back(Vector3d(1, 1, 0));
        pts.push_back(Vector3d(0, 1, 0));

        poly.setVertices(pts);
        REQUIRE(poly.size() == 4);
        REQUIRE((poly.pos(1) - Vector3d(1, 0, 0)).norm() == Catch::Approx(0));
    }

    SECTION("deep copy"){
        PolyVector3d pts;
        pts.push_back(Vector3d(0, 0, 0));
        pts.push_back(Vector3d(1, 0, 0));
        pts.push_back(Vector3d(1, 1, 0));
        pts.push_back(Vector3d(0, 1, 0));

        poly.setVertices(pts);

        _Polygon<double> newpoly;
        newpoly = poly;
        poly.clear();

        REQUIRE((newpoly.pos(1) - Vector3d(1, 0, 0)).norm() == Catch::Approx(0));
    }

    SECTION("push_back"){
        poly.push_back(Vector3d(0, 0, 0));
        poly.push_back(Vector3d(1, 0, 0));
        poly.push_back(Vector3d(1, 1, 0));
        poly.push_back(Vector3d(0, 1, 0));

        REQUIRE(poly.size() == 4);
        REQUIRE((poly.pos(1) - Vector3d(1, 0, 0)).norm() == Catch::Approx(0));
    }

    SECTION("the polygon has a simple input")
    {
        poly.push_back(Vector3d(0, 0, 0));
        poly.push_back(Vector3d(2, 0, 0));
        poly.push_back(Vector3d(2, 2, 0));
        poly.push_back(Vector3d(0, 2, 0));

        SECTION("reverseVertices"){
            poly.reverseVertices();

            REQUIRE(poly.size() == 4);
            REQUIRE((poly.pos(1) - Vector3d(2, 2, 0)).norm() == Catch::Approx(0));
            REQUIRE((poly.normal() - Vector3d(0, 0, -1)).norm() == Catch::Approx(0));
        }

        SECTION("checkEquality"){
            _Polygon<double> pB;
            pB.push_back(Vector3d(0, 0, 0));
            pB.push_back(Vector3d(2, 0, 0));
            pB.push_back(Vector3d(2, 2, 0));
            pB.push_back(Vector3d(0, 2, 0));
            pB.reverseVertices();

            REQUIRE(poly.checkEquality(pB) == false);

            pB.clear();
            pB.push_back(Vector3d(2, 2, 0));
            pB.push_back(Vector3d(0, 2, 0));
            pB.push_back(Vector3d(0, 0, 0));
            pB.push_back(Vector3d(2, 0, 0));

            REQUIRE(poly.checkEquality(pB) == true);
        }

        SECTION("print"){
            poly.print();
        }

        SECTION("computeCenter"){
            REQUIRE((poly.center() - Vector3d(1, 1, 0)).norm()  == Catch::Approx(0.0));
        }

        SECTION("computeNormal"){
            REQUIRE((poly.normal() - Vector3d(0, 0, 1)).norm() == Catch::Approx(0.0));
            poly.reverseVertices();
            REQUIRE((poly.normal() - Vector3d(0, 0, -1)).norm() == Catch::Approx(0.0));
        }

        SECTION("computeFitedPlaneNormal"){
            REQUIRE((poly.computeFitedPlaneNormal() - Vector3d(0, 0, 1)).norm()  == Catch::Approx(0.0));
        }

        SECTION("computeArea"){
            REQUIRE(poly.area() == Catch::Approx(4.0));
        }

        SECTION("computeAverageEdge"){
            REQUIRE(poly.average_edge() == Catch::Approx(2.0));
        }

        SECTION("computeMaxRadius"){
            REQUIRE(poly.max_radius() == Catch::Approx(sqrt(2)));
        }


        SECTION("computeFrame")
        {
            Vector3d x_axis, y_axis, origin;
            poly.computeFrame(x_axis, y_axis, origin);
            REQUIRE(x_axis.dot(poly.normal()) == Catch::Approx(0.0));
            REQUIRE(y_axis.dot(poly.normal()) == Catch::Approx(0.0));
        }

        SECTION("convertToTriangles"){
            vector<shared_ptr<Triangle<double>>> tris;
            poly.triangulateNaive(tris);
            REQUIRE(tris.size() == 4);
        }

        SECTION("getPtVerID"){
            REQUIRE(poly.getPtVerID(Vector3d(2, 2, 0)) == 2);
        }

        SECTION("executeTranslation"){
            poly.translatePolygon(Vector3d(1, 1, 1));
            REQUIRE((poly.pos(1) - Vector3d(3, 1, 1)).norm() == Catch::Approx(0.0));
        }

        SECTION("pos()"){
            // support circular index
            REQUIRE((poly.pos(-1) - Vector3d(0, 2, 0)).norm() == Catch::Approx(0.0));
            REQUIRE((poly.pos(1) - Vector3d(2, 0, 0)).norm() == Catch::Approx(0.0));
            REQUIRE((poly.pos(4) - Vector3d(0, 0, 0)).norm() == Catch::Approx(0.0));
        }
    }

    SECTION("computeBaryCentric"){
        poly.push_back(Vector3d(0, 0, 0), Vector2d(0, 0));
        poly.push_back(Vector3d(2, 0, 0), Vector2d(1.1, 0));
        poly.push_back(Vector3d(2, 2, 0), Vector2d(1.5, 2));
        poly.push_back(Vector3d(0, 2, 0), Vector2d(0, 1.3));

        Vector2d pt(0.1, 0.8);

        vector<double> barycentric = poly.computeBaryCentric(pt);
        Vector2d barypt(0, 0);
        for(int id = 0; id < 4; id++){
            barypt += poly.tex(id) * barycentric[id];
        }
        REQUIRE((barypt - pt).norm() == Catch::Approx(0).margin(1e-7));
    }

    SECTION("computeBaryCentric corner case") {
        poly.push_back(Vector3d(0, 0, 0), Vector2d(0, 0));
        poly.push_back(Vector3d(2, 0, 0), Vector2d(1.1, 0));
        poly.push_back(Vector3d(2, 2, 0), Vector2d(1.5, 2));
        poly.push_back(Vector3d(0, 2, 0), Vector2d(0, 1.3));

        SECTION("(0, 0.8)") {


            Vector2d pt(0, 0.8);

            vector<double> barycentric = poly.computeBaryCentric(pt);
            Vector2d barypt(0, 0);
            for (int id = 0; id < 4; id++) {
                barypt += poly.tex(id) * barycentric[id];
            }
            REQUIRE((barypt - pt).norm() == Catch::Approx(0).margin(1e-7));
        }

        SECTION("(0.8, 0)")
        {
            poly.push_back(Vector3d(0, 0, 0), Vector2d(0, 0));
            poly.push_back(Vector3d(2, 0, 0), Vector2d(1.1, 0));
            poly.push_back(Vector3d(2, 2, 0), Vector2d(1.5, 2));
            poly.push_back(Vector3d(0, 2, 0), Vector2d(0, 1.3));

            Vector2d pt(0.8, 0);

            vector<double> barycentric = poly.computeBaryCentric(pt);
            Vector2d barypt(0, 0);
            for (int id = 0; id < 4; id++) {
                barypt += poly.tex(id) * barycentric[id];
            }
            REQUIRE((barypt - pt).norm() == Catch::Approx(0).margin(1e-7));
        }

        SECTION("(0, 0)")
        {
            poly.push_back(Vector3d(0, 0, 0), Vector2d(0, 0));
            poly.push_back(Vector3d(2, 0, 0), Vector2d(1.1, 0));
            poly.push_back(Vector3d(2, 2, 0), Vector2d(1.5, 2));
            poly.push_back(Vector3d(0, 2, 0), Vector2d(0, 1.3));

            Vector2d pt(0, 0);

            vector<double> barycentric = poly.computeBaryCentric(pt);
            Vector2d barypt(0, 0);
            for (int id = 0; id < 4; id++) {
                barypt += poly.tex(id) * barycentric[id];
            }
            REQUIRE((barypt - pt).norm() == Catch::Approx(0).margin(1e-7));
        }

    }

    SECTION("computeBaryCentric LINE")
    {
        vector<double> barycentric = poly.computeBaryCentric(Vector2d(0, 0), Vector2d(1, 0), Vector2d(0.2, 0));
        REQUIRE(barycentric[0] == Catch::Approx(0.8).margin(1e-7));
        REQUIRE(barycentric[1] == Catch::Approx(0.2).margin(1e-7));

        barycentric = poly.computeBaryCentric(Vector2d(0, 0), Vector2d(1, 0), Vector2d(0, 0));
        REQUIRE(barycentric[0] == Catch::Approx(1).margin(1e-7));
        REQUIRE(barycentric[1] == Catch::Approx(0).margin(1e-7));

        barycentric = poly.computeBaryCentric(Vector2d(0, 0), Vector2d(1, 0), Vector2d(1, 0));
        REQUIRE(barycentric[0] == Catch::Approx(0).margin(1e-7));
        REQUIRE(barycentric[1] == Catch::Approx(1).margin(1e-7));
    }

    SECTION("triangle"){
        PolyVector3d pts;
        pts.push_back(Vector3d(0, 0, 0));
        pts.push_back(Vector3d(1, 0, 0));
        pts.push_back(Vector3d(1, 1, 0));
        pts.push_back(Vector3d(0, 1, 0));
        poly.setVertices(pts);
        vector<shared_ptr<Triangle<double>>> tris;
        poly.edge_at_boundary[0] = true;
        poly.edge_at_boundary[1] = true;
        poly.triangulate(tris);

        REQUIRE(tris.size() == 2);
    }

}

