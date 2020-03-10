//
// Created by ziqwang on 09.01.19.
//

#ifndef TOPOLOCKCREATOR_QUADTREE_H
#define TOPOLOCKCREATOR_QUADTREE_H

#include <Eigen/Dense>
#include "Mesh/PolyMesh.h"
#include <igl/AABB.h>
#include <igl/boundary_loop.h>

using Eigen::Vector2d;

/*!
 * \brief 2D Kd-Tree, for quickly searching the triangle where given points are located
 */

template<typename Scalar>
class PolyMesh_AABBTree : public PolyMesh<Scalar>
{
public:
    typedef Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
    typedef Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatrixXi;
    typedef Matrix<Scalar, 2, 1> Vector2;
    typedef Matrix<Scalar, 3, 1> Vector3;
    typedef Matrix<Scalar, Eigen::Dynamic, 1> VectorX;
    typedef shared_ptr<_Polygon<Scalar>> pPolygon;

public:

    igl::AABB<MatrixX, 2> texTree;
    igl::AABB<MatrixX, 3> posTree;

    MatrixX pV, tV;
    MatrixXi pF, tF;

    Eigen::VectorXi tC, pC;

public:

    PolyMesh_AABBTree(const PolyMesh<Scalar> &_mesh)
    :PolyMesh<Scalar>::PolyMesh(_mesh.getVarList())
    {
        PolyMesh<Scalar>::polyList = _mesh.polyList;
        PolyMesh<Scalar>::vertexList = _mesh.vertexList;
        PolyMesh<Scalar>::texList = _mesh.texList;
        PolyMesh<Scalar>::texturedModel = _mesh.texturedModel;
    }

public:

    void buildTexTree()
    {
        PolyMesh<Scalar>::convertTexToEigenMesh(tV, tF, tC);
        texTree.init(tV, tF);
    }

    void buildPosTree(){
        PolyMesh<Scalar>::convertPosToEigenMesh(pV, pF, pC);
        posTree.init(pV, pF);
    }

public:

    pPolygon findTexPoint(VectorX pt)
    {
        vector<int> indices = texTree.find(tV, tF, pt.transpose(), false);
        if(!indices.empty()){
            int faceID = tC(indices[0]);
            return PolyMesh<Scalar>::polyList[faceID];
        }
        return nullptr;
    }

    Vector3d findMeshNearestPoint(Vector3 pt);

};

template<typename Scalar>
class AABBTree_Line{
public:
    typedef Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

    typedef Matrix<int, Eigen::Dynamic, Eigen::Dynamic> MatrixXi;

    typedef Matrix<Scalar, 2, 1> Vector2;

    typedef Matrix<Scalar, 3, 1> Vector3;
public:
    struct Node{
        Box<Scalar> box;
        Line<Scalar> line;
        weak_ptr<Node> children[2];
    };

    vector<shared_ptr<Node>> nodes;
    vector<weak_ptr<Node>> roots;
public:

    void init(MatrixX V, MatrixXi F)
    {
        vector<vector<int>> loops;
        igl::boundary_loop(F, loops);

        for(vector<int> loop: loops)
        {
            // create leaves node
            vector<shared_ptr<Node>> loop_nodes;
            for(int id = 0; id < loop.size(); id++)
            {
                Vector2 staPt = V.row(loop[id]);
                Vector2 endPt = V.row(loop[(id + 1) % loop.size()]);
                shared_ptr<Node> node = make_shared<Node>();
                node->line.point1 = Vector3(staPt.x(), staPt.y(), 0);
                node->line.point2 = Vector3(endPt.x(), endPt.y(), 0);
                node->box = Box<double>(node->line);
                loop_nodes.push_back(node);
            }

            // create reset of the tree
            // every two neighboring lines are merged into a big area.
            vector<shared_ptr<Node>> merged_nodes;
            merged_nodes = loop_nodes;
            //if only one node is left, break
            while(merged_nodes.size() > 1)
            {
                //merge every two nodes
                vector<shared_ptr<Node>> parent_nodes;
                for(int id = 0; id < merged_nodes.size() / 2; id++)
                {
                    shared_ptr<Node> node0 = merged_nodes[id * 2];
                    shared_ptr<Node> node1 = merged_nodes[id * 2 + 1];

                    shared_ptr<Node> merge_node = make_shared<Node>();
                    merge_node->box = Box<Scalar>(node0->box, node1->box);
                    merge_node->children[0] = node0;
                    merge_node->children[1] = node1;
                    parent_nodes.push_back(merge_node);
                }

                if(merged_nodes.size() % 2 == 1){
                    parent_nodes.push_back(merged_nodes.back());
                }

                loop_nodes.insert(loop_nodes.end(), parent_nodes.begin(), parent_nodes.end());
                merged_nodes = parent_nodes;
            }

            nodes.insert(nodes.end(), loop_nodes.begin(), loop_nodes.end());
            roots.push_back(loop_nodes.back());
        }
    }

    bool findIntersec(const Line<Scalar> &line, Vector2 &pt)
    {
        Scalar minDist = -1;
        for(int id = 0; id < roots.size(); id++)
        {
            Vector2 intersec;
            if(findIntersec(line, intersec, roots[id].lock())){
                if(minDist == -1 || minDist > (line.point1.head(2) - intersec).norm()){
                    pt = intersec;
                    minDist = (line.point1.head(2) - intersec).norm();
                }
            }
        }

        if(minDist > 0){
            return true;
        }
        else{
            return false;
        }
    }

    bool findIntersec(const Line<Scalar> &line, Vector2 &pt, shared_ptr<Node> node)
    {
        //empty node
        if(node == nullptr){
            return false;
        }

        //leaves
        if(node->children[0].lock() == nullptr
        && node->children[1].lock() == nullptr)
        {
            return checkLineLineIntersec(node->line, line, pt);
        }

        //not leaves
        Scalar minDist = -1;
        for(size_t id = 0; id < 2; id++)
        {
            if(node->children[id].lock() != nullptr
            && checkLineBoxIntersec(line, node->box))
            {
                Vector2 intersec;
                if(findIntersec(line, intersec, node->children[id].lock())){
                    Scalar dist = (line.point1.head(2) - intersec).norm();
                    if(minDist == -1 || minDist > dist)
                    {
                        minDist = dist;
                        pt = intersec;
                    }
                }
            }
        }

        if(minDist != -1){
            return true;
        }
        else{
            return false;
        }
    }

    bool checkLineBoxIntersec(const Line<Scalar> &line, const Box<Scalar> &box)
    {
        Vector2 ray = (line.point2 - line.point1).normalized().head(2);
        if(checkRayBoxIntersec(ray, line.point1.head(2), box)
        || checkRayBoxIntersec(-ray, line.point2.head(2), box))
        {
            return true;
        }
        else{
            return false;
        }
    }

    //https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
    bool checkRayBoxIntersec(const Vector2 &ray, const Vector2 &origin, const Box<Scalar> &box)
    {
        Scalar t0x = (box.minPt[0] - origin[0]) / ray[0];
        Scalar t1x = (box.maxPt[0] - origin[0]) / ray[0];
        Scalar t0y = (box.minPt[1] - origin[1]) / ray[1];
        Scalar t1y = (box.maxPt[1] - origin[1]) / ray[1];
        if(t0x > t1y || t0y > t1x)
        {
            return false;
        }
        else{
            return true;
        }
    }

    //http://flassari.is/2008/11/line-line-intersection-in-cplusplus/
    bool checkLineLineIntersec(const Line<Scalar> &l0, const Line<Scalar> &l1, Vector2 &pt)
    {
        Scalar x1 = l0.point1.x(), x2 = l0.point2.x(), x3 = l1.point1.x(), x4 = l1.point2.x();
        Scalar y1 = l0.point1.y(), y2 = l0.point2.y(), y3 = l1.point1.y(), y4 = l1.point2.y();

        Scalar d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        // If d is zero, there is no intersection
        if (d == 0) return false;

        // Get the x and y
        Scalar pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
        Scalar x = ( pre * (x3 - x4) - (x1 - x2) * post ) / d;
        Scalar y = ( pre * (y3 - y4) - (y1 - y2) * post ) / d;

        // Check if the x and y coordinates are within both lines
        if ( x < min(x1, x2) || x > max(x1, x2) ||
            x < min(x3, x4) || x > max(x3, x4) )
            return false;
        if ( y < min(y1, y2) || y > max(y1, y2) ||
            y < min(y3, y4) || y > max(y3, y4) )
            return false;

        // Return the point of intersection
        pt = Vector2(x, y);
        return true;
    }

};


#endif //TOPOLOCKCREATOR_QUADTREE_H
