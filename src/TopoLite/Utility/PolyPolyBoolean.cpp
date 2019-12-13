//
// Created by ziqwang on 2019-12-11.
//
#ifndef CATCH2_UNITTEST

#else
#include "PolyPolyBoolean.h"
#include <catch2/catch.hpp>
#include "Mesh/PolyMesh.h"
#include "IO/XMLIO.h"
using pPolyMesh = shared_ptr<PolyMesh>;
using pPolygon = shared_ptr<_Polygon>;

TEST_CASE("Class PolyPolyBoolean")
{
    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVarLite(varList.get());

    PolyPolyBoolean polyBoolean(varList);

    SECTION("3 faces merge into 1")
    {
        vector<Vector3f> A;
        A.push_back(Vector3f(0, 0, 0));
        A.push_back(Vector3f(1, 0, 0));
        A.push_back(Vector3f(1, 1, 0));
        A.push_back(Vector3f(0, 1, 0));

        vector<Vector3f> B;
        B.push_back(Vector3f(1, 1, 0));
        B.push_back(Vector3f(2, 1, 0));
        B.push_back(Vector3f(2, 2, 0));
        B.push_back(Vector3f(1, 2, 0));

        vector<Vector3f> C;
        C.push_back(Vector3f(1, 0, 0));
        C.push_back(Vector3f(2, 0, 0));
        C.push_back(Vector3f(2, 1, 0));
        C.push_back(Vector3f(1, 1, 0));

        vector<vector<Vector3f>> polys;
        polys.push_back(A);
        polys.push_back(B);
        polys.push_back(C);

        vector<vector<Vector3f>> polyUnions;
        polyBoolean.ComputePolygonsUnion(polys, polyUnions);


        REQUIRE(polyUnions.size() == 1);

        //REQUIRE(polyUnions[0].size() == 6);

        std::cout << "[";
        for (int id = 0; id < polyUnions[0].size(); id++) {
                std::cout << "[" << polyUnions[0][id].x << ", " << polyUnions[0][id].y << "], ";
        }
        std::cout << "]" << std::endl;
    }

    SECTION("2 faces Intersec -> triangle")
    {
        vector<Vector3f> A;
        A.push_back(Vector3f(0, 0, 0));
        A.push_back(Vector3f(1, 0, 0));
        A.push_back(Vector3f(1, 0.8, 0));
        A.push_back(Vector3f(1, 1, 0));
        A.push_back(Vector3f(0.8, 1, 0));
        A.push_back(Vector3f(0, 1, 0));

        vector<Vector3f> B;
        B.push_back(Vector3f(0.5, 0.5, 0));
        B.push_back(Vector3f(1.5, 0.5, 0));
        B.push_back(Vector3f(1.5, 1.5, 0));

        vector<Vector3f> polyIntersec;
        polyBoolean.ComputePolygonsIntersection(A, B, polyIntersec);

        REQUIRE(polyIntersec.size() == 3);
    }

    SECTION("2 faces Intersec -> quad")
    {
        vector<Vector3f> A;
        A.push_back(Vector3f(0, 0, 0));
        A.push_back(Vector3f(1, 0, 0));
        A.push_back(Vector3f(1, 0.8, 0));
        A.push_back(Vector3f(1, 1, 0));
        A.push_back(Vector3f(0.8, 1, 0));
        A.push_back(Vector3f(0, 1, 0));

        vector<Vector3f> B;
        B.push_back(Vector3f(0.4, 0.5, 0));
        B.push_back(Vector3f(1.5, 0.5, 0));
        B.push_back(Vector3f(1.5, 1.5, 0));

        vector<Vector3f> polyIntersec;
        polyBoolean.ComputePolygonsIntersection(A, B, polyIntersec);

        REQUIRE(polyIntersec.size() == 4);
    }

    SECTION("2 faces Intersec -> triangle")
    {
        vector<Vector3f> A;
        A.push_back(Vector3f(0, 0, 0));
        A.push_back(Vector3f(1, 0, 0));
        A.push_back(Vector3f(1, 0.8, 0));
        A.push_back(Vector3f(1, 1, 0));
        A.push_back(Vector3f(0.8, 1, 0));
        A.push_back(Vector3f(0, 1, 0));

        vector<Vector3f> B;
        B.push_back(Vector3f(0.5, 0.5, 0));
        B.push_back(Vector3f(1.5, 0.5, 0));
        B.push_back(Vector3f(1.5, 1.5, 0));

        vector<Vector3f> polyIntersec;
        polyBoolean.ComputePolygonsIntersection(A, B, polyIntersec);

        REQUIRE(polyIntersec.size() == 3);
    }

    SECTION("2 faces Intersec -> quad")
    {
        vector<Vector3f> A;
        A.push_back(Vector3f(0, 0, 0));
        A.push_back(Vector3f(1, 0, 0));
        A.push_back(Vector3f(1, 0.8, 0));
        A.push_back(Vector3f(1, 1, 0));
        A.push_back(Vector3f(0.8, 1, 0));
        A.push_back(Vector3f(0, 1, 0));

        vector<Vector3f> B;
        B.push_back(Vector3f(0.4, 0.5, 0));
        B.push_back(Vector3f(1.5, 0.5, 0));
        B.push_back(Vector3f(1.5, 1.5, 0));

        vector<Vector3f> polyIntersec;
        polyBoolean.ComputePolygonsIntersection(A, B, polyIntersec);

        REQUIRE(polyIntersec.size() == 4);
    }
}

#endif