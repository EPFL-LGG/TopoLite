//
// Created by ziqwang on 2019-12-11.
//
#include "Utility/PolyPolyBoolean.h"
#include <catch2/catch.hpp>
using Eigen::Vector3d;
using Eigen::Vector2d;

void Scale_ListVector3d(vector<Vector3d>& poly, double Scale = 10000){
    for(Vector3d &pt : poly){
        pt *= Scale;
    }
    return;
}

TEST_CASE("Class PolyPolyBoolean")
{
    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVarLite(varList.get());

    PolyPolyBoolean<double> polyBoolean(varList);


    SECTION("clean polygons")
    {
        vector<Vector3d> A;
        A.push_back(Vector3d(0, 0, 0));
        A.push_back(Vector3d(1e-6, 0, 0));
        A.push_back(Vector3d(0.2, 0, 0));
        A.push_back(Vector3d(0.4, 0, 0));
        A.push_back(Vector3d(0.4+1e-6, 0, 0));
        A.push_back(Vector3d(0.8, 0, 0));
        A.push_back(Vector3d(1, 0, 0));
        A.push_back(Vector3d(1, 0.2, 0));
        A.push_back(Vector3d(1, 0.4, 0));
        A.push_back(Vector3d(1, 1, 0));
        A.push_back(Vector3d(0, 1, 0));
        A.push_back(Vector3d(0, 0.8, 0));
        A.push_back(Vector3d(0, 0.2, 0));
        A.push_back(Vector3d(0, 0, 0));
        polyBoolean.cleanPath(A);

        REQUIRE(A.size() == 4);
    }

    SECTION("3 faces merge into 1")
    {
        vector<Vector3d> A;
        A.push_back(Vector3d(0, 0, 0));
        A.push_back(Vector3d(1, 0, 0));
        A.push_back(Vector3d(1, 1, 0));
        A.push_back(Vector3d(0, 1, 0));

        vector<Vector3d> B;
        B.push_back(Vector3d(1, 1, 0));
        B.push_back(Vector3d(2, 1, 0));
        B.push_back(Vector3d(2, 2, 0));
        B.push_back(Vector3d(1, 2, 0));

        vector<Vector3d> C;
        C.push_back(Vector3d(1, 0, 0));
        C.push_back(Vector3d(2, 0, 0));
        C.push_back(Vector3d(2, 1, 0));
        C.push_back(Vector3d(1, 1, 0));

        vector<vector<Vector3d>> polys;
        polys.push_back(A);
        polys.push_back(B);
        polys.push_back(C);

        vector<vector<Vector3d>> polyUnions;
        polyBoolean.computePolygonsUnion(polys, polyUnions);


        REQUIRE(polyUnions.size() == 1);
        REQUIRE(polyUnions[0].size() == 6);
    }

    SECTION("corner case of two polygon have edge edge intersection"){
        vector<vector<float>> A = {{-0.25, -0.3}, {0.15, -0.3}, {0.15, -0.0999999}, {-0.0499999, -0.0999999}, {-0.0499999, 0.0999999}, {0.15, 0.1}, {0.15, 0.3}, {-0.25, 0.3}, };
        vector<vector<float>> B = {{-0.25, -0.0999999}, {-0.0500001, -0.0999999}, {-0.0500002, 0.3}, {0.15, 0.3}, {0.15, -0.3}, {-0.45, -0.3}, {-0.45, -0.1}, {-0.65, -0.1}, {-0.65, 0.1}, {-0.25, 0.1}, };

        vector<Vector3d> PA;
        for(vector<float> pt : A){
            PA.push_back(Vector3d(pt[0], pt[1], 0));
        }

        vector<Vector3d> PB;
        for(vector<float> pt : B){
            PB.push_back(Vector3d(pt[0], pt[1], 0));
        }

        vector<vector<Vector3d>> polyIntersec;
        polyBoolean.computePolygonsIntersection(PA, PB, polyIntersec);

        REQUIRE(polyIntersec.size() == 2);
        REQUIRE(polyIntersec[0].size() == 4);
        REQUIRE(polyIntersec[1].size() == 4);
    }

    SECTION("2 faces Intersec -> triangle")
    {
        vector<Vector3d> A;
        A.push_back(Vector3d(0, 0, 0));
        A.push_back(Vector3d(1, 0, 0));
        A.push_back(Vector3d(1, 0.8, 0));
        A.push_back(Vector3d(1, 1, 0));
        A.push_back(Vector3d(0.8, 1, 0));
        A.push_back(Vector3d(0, 1, 0));

        vector<Vector3d> B;
        B.push_back(Vector3d(0.5, 0.5, 0));
        B.push_back(Vector3d(1.5, 0.5, 0));
        B.push_back(Vector3d(1.5, 1.5, 0));

        vector<Vector3d> polyIntersec;
        polyBoolean.computePolygonsIntersection(A, B, polyIntersec);

        REQUIRE(polyIntersec.size() == 3);
    }

    SECTION("2 faces Intersec -> quad")
    {
        vector<Vector3d> A;
        A.push_back(Vector3d(0, 0, 0));
        A.push_back(Vector3d(1, 0, 0));
        A.push_back(Vector3d(1, 0.8, 0));
        A.push_back(Vector3d(1, 1, 0));
        A.push_back(Vector3d(0.8, 1, 0));
        A.push_back(Vector3d(0, 1, 0));

        vector<Vector3d> B;
        B.push_back(Vector3d(0.4, 0.5, 0));
        B.push_back(Vector3d(1.5, 0.5, 0));
        B.push_back(Vector3d(1.5, 1.5, 0));

        vector<Vector3d> polyIntersec;
        polyBoolean.computePolygonsIntersection(A, B, polyIntersec);

        REQUIRE(polyIntersec.size() == 4);
    }

    SECTION("2 faces Intersec -> triangle")
    {
        vector<Vector3d> A;
        A.push_back(Vector3d(0, 0, 0));
        A.push_back(Vector3d(1, 0, 0));
        A.push_back(Vector3d(1, 0.8, 0));
        A.push_back(Vector3d(1, 1, 0));
        A.push_back(Vector3d(0.8, 1, 0));
        A.push_back(Vector3d(0, 1, 0));

        vector<Vector3d> B;
        B.push_back(Vector3d(0.5, 0.5, 0));
        B.push_back(Vector3d(1.5, 0.5, 0));
        B.push_back(Vector3d(1.5, 1.5, 0));

        vector<Vector3d> polyIntersec;
        polyBoolean.computePolygonsIntersection(A, B, polyIntersec);

        REQUIRE(polyIntersec.size() == 3);
    }

    SECTION("2 faces Intersec -> quad")
    {
        vector<Vector3d> A;
        A.push_back(Vector3d(0, 0, 0));
        A.push_back(Vector3d(1, 0, 0));
        A.push_back(Vector3d(1, 0.8, 0));
        A.push_back(Vector3d(1, 1, 0));
        A.push_back(Vector3d(0.8, 1, 0));
        A.push_back(Vector3d(0, 1, 0));

        vector<Vector3d> B;
        B.push_back(Vector3d(0.4, 0.5, 0));
        B.push_back(Vector3d(1.5, 0.5, 0));
        B.push_back(Vector3d(1.5, 1.5, 0));

        vector<Vector3d> polyIntersec;
        polyBoolean.computePolygonsIntersection(A, B, polyIntersec);

        REQUIRE(polyIntersec.size() == 4);
    }

    SECTION("Scaling Error"){
        vector<Vector3d> A;
        A.push_back(Vector3d(0, 0, 0));
        A.push_back(Vector3d(1, 0, 0));
        A.push_back(Vector3d(1, 0.8, 0));
        A.push_back(Vector3d(1, 1, 0));
        A.push_back(Vector3d(0.8, 1, 0));
        A.push_back(Vector3d(0, 1, 0));
        Scale_ListVector3d(A);

        vector<Vector3d> B;
        B.push_back(Vector3d(0.4, 0.5, 0));
        B.push_back(Vector3d(1.5, 0.5, 0));
        B.push_back(Vector3d(1.5, 1.5, 0));
        Scale_ListVector3d(B);

        vector<Vector3d> polyIntersec;
        polyBoolean.computePolygonsIntersection(A, B, polyIntersec);

        REQUIRE(polyIntersec.size() == 4);
    }


    SECTION("check2DPolygonsIntersection", "intersected 1/8"){

        vector<Vector2d> A;
        A.push_back(Vector2d(0, 0));
        A.push_back(Vector2d(1, 0));
        A.push_back(Vector2d(1, 0.8));
        A.push_back(Vector2d(1, 1));
        A.push_back(Vector2d(0.8, 1));
        A.push_back(Vector2d(0, 1));

        vector<Vector2d> B;
        B.push_back(Vector2d(0.5, 0.5));
        B.push_back(Vector2d(1.5, 0.5));
        B.push_back(Vector2d(1.5, 1.5));

        double area = 0;
        REQUIRE(polyBoolean.check2DPolygonsIntersection(A, B, area) == true);
        REQUIRE(area == Approx(1.0 / 8).margin(1e-6));
    }

    SECTION("check2DPolygonsIntersection", "intersected 3/8"){
        // 1) intersected
        vector<Vector2d> A;
        A.push_back(Vector2d(0, 0));
        A.push_back(Vector2d(1, 0));
        A.push_back(Vector2d(1, 0.8));
        A.push_back(Vector2d(1, 1));
        A.push_back(Vector2d(0.8, 1));
        A.push_back(Vector2d(0, 1));

        vector<Vector2d> B;
        B.push_back(Vector2d(0.5, 0.5));
        B.push_back(Vector2d(0, 1));
        B.push_back(Vector2d(1, 1));
        B.push_back(Vector2d(1, 0.5));

        double area = 0;
        REQUIRE(polyBoolean.check2DPolygonsIntersection(A, B, area) == true);
        REQUIRE(area == Approx(3.0 / 8).margin(1e-6));
    }

    SECTION("check2DPolygonsIntersection Scaling", "intersected"){
        // 1) intersected
        vector<Vector2d> A;
        A.push_back(Vector2d(0, 0));
        A.push_back(Vector2d(1, 0));
        A.push_back(Vector2d(1, 0.8));
        A.push_back(Vector2d(1, 1));
        A.push_back(Vector2d(0.8, 1));
        A.push_back(Vector2d(0, 1));

        vector<Vector2d> B;
        B.push_back(Vector2d(0, 0));
        B.push_back(Vector2d(0, 1000000));
        B.push_back(Vector2d(1000000, 1000000));

        double area = 0;
        REQUIRE(polyBoolean.check2DPolygonsIntersection(A, B, area) == true);
        REQUIRE(area == Approx(1.0 / 2).margin(1e-6));
    }


}