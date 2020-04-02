//
// Created by ziqwang on 14.01.19.
//

#include <catch2/catch.hpp>
#include "Interlocking/ContactGraph.h"
#include "Mesh/PolyMesh.h"

using pPolyMesh = shared_ptr<PolyMesh<double>>;
using pPolygon = shared_ptr<_Polygon<double>>;
using Eigen::Vector3d;

// TODO: @robin move this in utilities + replace all comparisons in other test
/**
 * @brief A good way of comparing floats in C/C++
 *        With this definition, you don't need to rescale epsilon for each comparison as you would normaly do with abs(a-b) <= epsilon
 * @param a
 * @param b
 * @param epsilon
 * @return
 *
 * See: “Donald Knuth - The Art of Computer Programming, Volume II: Seminumerical Algorithms (Addison-Wesley, 1969)”
 */
template <typename Scalar>
bool approximatelyEqual(Scalar a, Scalar b, Scalar epsilon) {
    return (std::abs(a - b) <= (std::max(std::abs(a), std::abs(b)) * epsilon));
}

TEST_CASE("Class ContactGraph") {
    shared_ptr<InputVarList> varList = make_shared<InputVarList>();
    InitVarLite(varList.get());

    pPolygon pA = make_shared<_Polygon<double>>();
    pA->push_back(Vector3d(0, 0, 0));
    pA->push_back(Vector3d(2, 0, 0));
    pA->push_back(Vector3d(2, 2, 0));
    pA->push_back(Vector3d(0, 2, 0));

    /*
     *      |-->--|     * < or > gives the orientation of the polygon
     *   |--|--| B|     * n_k = edge_1,i edge_2,j epsilon_ijk with:
     *   |A |--|--|         * epsilon_ijk = Levi-Civita tensor (https://en.wikipedia.org/wiki/Levi-Civita_symbol).
     *   |-->--|            * i,j,k are coordinates indexes.
     *                      * Einstein notation is used (https://en.wikipedia.org/wiki/Einstein_notation).
     */
    SECTION("load two square A,B. A's normal is [0,0,1], B's normal is [0,0,-1]") {

        pPolygon pB = make_shared<_Polygon<double>>();
        pB->push_back(Vector3d(1, 1, 0));
        pB->push_back(Vector3d(1, 3, 0));
        pB->push_back(Vector3d(3, 3, 0));
        pB->push_back(Vector3d(3, 1, 0));

        pPolyMesh meshA = make_shared<PolyMesh<double>>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh<double>>(varList);
        meshB->polyList.push_back(pB);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);

        vector<bool> atBoundary;
        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 2);
        REQUIRE(graph->edges.size() == 1);

        _Polygon<double> contactPolygon;
        for (const auto &ver: graph->edges[0]->polygons[0]->vers) {
            contactPolygon.push_back(Vector3d(ver->pos[0], ver->pos[1], ver->pos[2]));
        }
        REQUIRE(approximatelyEqual<double>(contactPolygon.area(), 1, 1e-5));
    }

    SECTION("load three squares A,B,C. A's normal is [0,0,1], B's normal is [0,0,-1], C's normal is [0,0,-1]") {
        pPolygon pB = make_shared<_Polygon<double>>();
        pB->push_back(Vector3d(1, 1, 0));
        pB->push_back(Vector3d(1, 3, 0));
        pB->push_back(Vector3d(3, 3, 0));
        pB->push_back(Vector3d(3, 1, 0));

        pPolygon pC = make_shared<_Polygon<double>>();
        pC->push_back(Vector3d(-1, -1, 0));
        pC->push_back(Vector3d(-1,  1, 0));
        pC->push_back(Vector3d( 1,  1, 0));
        pC->push_back(Vector3d( 1, -1, 0));

        pPolyMesh meshA = make_shared<PolyMesh<double>>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh<double>>(varList);
        meshB->polyList.push_back(pB);

        pPolyMesh meshC = make_shared<PolyMesh<double>>(varList);
        meshC->polyList.push_back(pC);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);
        meshes.push_back(meshC);

        vector<bool> atBoundary;
        atBoundary.push_back(false);
        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 3);
        REQUIRE(graph->edges.size() == 2);

    }

    SECTION("load three squares A,B,C. A's normal is [0,0,1], B's normal is [0,0,1], C's normal is [0,0,-1]") {
        pPolygon pB = make_shared<_Polygon<double>>();
        pB->push_back(Vector3d(1, 1, 0));
        pB->push_back(Vector3d(3, 1, 0));
        pB->push_back(Vector3d(3, 3, 0));
        pB->push_back(Vector3d(1, 3, 0));

        pPolygon pC = make_shared<_Polygon<double>>();
        pC->push_back(Vector3d(-1, -1, 0));
        pC->push_back(Vector3d(-1,  1, 0));
        pC->push_back(Vector3d( 1,  1, 0));
        pC->push_back(Vector3d( 1, -1, 0));

        pPolyMesh meshA = make_shared<PolyMesh<double>>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh<double>>(varList);
        meshB->polyList.push_back(pB);

        pPolyMesh meshC = make_shared<PolyMesh<double>>(varList);
        meshC->polyList.push_back(pC);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);
        meshes.push_back(meshC);

        vector<bool> atBoundary;
        atBoundary.push_back(false);
        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 3);
        REQUIRE(graph->edges.size() == 1);
    }

    SECTION("load three squares A,B,C. A's normal is [0,0,1], B's normal is [0,0,1], C's normal is [0,0,1]") {
        pPolygon pB = make_shared<_Polygon<double>>();
        pB->push_back(Vector3d(1, 1, 0));
        pB->push_back(Vector3d(3, 1, 0));
        pB->push_back(Vector3d(3, 3, 0));
        pB->push_back(Vector3d(1, 3, 0));

        pPolygon pC = make_shared<_Polygon<double>>();
        pC->push_back(Vector3d(-1, -1, 0));
        pC->push_back(Vector3d( 1, -1, 0));
        pC->push_back(Vector3d( 1,  1, 0));
        pC->push_back(Vector3d(-1,  1, 0));

        pPolyMesh meshA = make_shared<PolyMesh<double>>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh<double>>(varList);
        meshB->polyList.push_back(pB);

        pPolyMesh meshC = make_shared<PolyMesh<double>>(varList);
        meshC->polyList.push_back(pC);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);
        meshes.push_back(meshC);

        vector<bool> atBoundary;
        atBoundary.push_back(false);
        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 3);
        REQUIRE(graph->edges.empty());
    }

    SECTION("load two square A,B. A's normal is [0,0,1], B's normal is [0,0,1] but centor at Z:-0.0004") {

        pPolygon pB = make_shared<_Polygon<double>>();
        pB->push_back(Vector3d(1, 1, -0.0004));
        pB->push_back(Vector3d(1, 3, -0.0004));
        pB->push_back(Vector3d(3, 3, -0.0004));
        pB->push_back(Vector3d(3, 1, -0.0004));

        pPolyMesh meshA = make_shared<PolyMesh<double>>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh<double>>(varList);
        meshB->polyList.push_back(pB);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);

        vector<bool> atBoundary;
        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 2);
        REQUIRE(graph->edges.size() == 1);

        _Polygon<double> contactPolygon;

        for (const auto& ver: graph->edges[0]->polygons[0]->vers) {
            contactPolygon.push_back(Vector3d(ver->pos[0], ver->pos[1], ver->pos[2]));
        }
        REQUIRE(approximatelyEqual<double>(contactPolygon.area(), 1, 1e-5));
    }

    SECTION("load two square A,B. A's normal is [0,0,1], B's normal is [0,0,-1] but centor at Z:-0.0004") {

        pPolygon pB = make_shared<_Polygon<double>>();
        pB->push_back(Vector3d(1, 1, -0.0004));
        pB->push_back(Vector3d(3, 1, -0.0004));
        pB->push_back(Vector3d(3, 3, -0.0004));
        pB->push_back(Vector3d(1, 3, -0.0004));

        pPolyMesh meshA = make_shared<PolyMesh<double>>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh<double>>(varList);
        meshB->polyList.push_back(pB);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);

        vector<bool> atBoundary;
        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 2);
        REQUIRE(graph->edges.empty());

        _Polygon<double> contactPolygon;
        //A and B are not in contact so we should have the following code.
//        for (const auto& ver: graph->edges[0]->polygons[0]->vers) {
//            contactPolygon.push_back(Vector3d(ver->pos[0], ver->pos[1], ver->pos[2]));
//        }
//        REQUIRE(approximatelyEqual<double>(contactPolygon.area(), 1, 1e-5));
    }

    SECTION("both A B at boundary") {

        pPolygon pB = make_shared<_Polygon<double>>();
        pB->push_back(Vector3d(1, 1, 0.0004));
        pB->push_back(Vector3d(1, 3, 0.0004));
        pB->push_back(Vector3d(3, 3, 0.0004));
        pB->push_back(Vector3d(3, 1, 0.0004));

        pPolyMesh meshA = make_shared<PolyMesh<double>>(varList);
        meshA->polyList.push_back(pA);

        pPolyMesh meshB = make_shared<PolyMesh<double>>(varList);
        meshB->polyList.push_back(pB);

        vector<pPolyMesh> meshes;
        meshes.push_back(meshA);
        meshes.push_back(meshB);

        vector<bool> atBoundary;
        atBoundary.push_back(true);
        atBoundary.push_back(true);

        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(meshes, atBoundary);

        REQUIRE(graph->nodes.size() == 2);
        REQUIRE(graph->edges.empty());
    }


    SECTION("Scaling Error") {
        pPolyMesh A, B;
        A = make_shared<PolyMesh<double>>(varList);
        B = make_shared<PolyMesh<double>>(varList);

        string strA = "../data/Mesh/contact_scaling_bug/01.obj";
        bool texturedModel;
        A->readOBJModel(strA.c_str(), texturedModel, false);

        string strB = "../data/Mesh/contact_scaling_bug/02.obj";
        B->readOBJModel(strB.c_str(), texturedModel, false);

        vector<pPolyMesh> meshes;
        vector<bool> atBoundary;
        meshes.push_back(A);
        meshes.push_back(B);

        atBoundary.push_back(false);
        atBoundary.push_back(false);

        shared_ptr<ContactGraph<double>> graph = make_shared<ContactGraph<double>>(varList);
        graph->buildFromMeshes(meshes, atBoundary);

        pPolyMesh contactMesh;
        graph->getContactMesh(contactMesh);

        A->writeOBJModel("01.obj");
        B->writeOBJModel("02.obj");
        contactMesh->writeOBJModel("debug.obj");
    }

}