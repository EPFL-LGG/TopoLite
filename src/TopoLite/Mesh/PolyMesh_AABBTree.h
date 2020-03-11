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
        Line<Scalar> line2D; //2D texCoords
        Line<Scalar> line3D; //3D vertices
        weak_ptr<Node> children[2];
    };

    vector<shared_ptr<Node>> nodes;
    vector<weak_ptr<Node>> roots;
public:

    void init(MatrixX V, MatrixX T, MatrixXi F)
    {
        vector<vector<int>> loops;
        igl::boundary_loop(F, loops);

        for(vector<int> loop: loops)
        {
            // create leaves node
            vector<shared_ptr<Node>> loop_nodes;
            for(int id = 0; id < loop.size(); id++)
            {
                //create node
                shared_ptr<Node> node = make_shared<Node>();

                //tex 2D
                Vector2 staPt2D = T.row(loop[id]);
                Vector2 endPt2D = T.row(loop[(id + 1) % loop.size()]);
                node->line2D = Line<Scalar>(staPt2D, endPt2D);

                //pos 3D
                Vector3 staPt3D = V.row(loop[id]);
                Vector3 endPt3D = V.row(loop[(id + 1) % loop.size()]);
                node->line3D = Line<Scalar>(staPt3D, endPt3D);

                //box
                node->box = Box<double>(node->line2D);

                //add to the loops
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

    bool findIntersec(const Line<Scalar> &line, Vector2 &tex2D, Vector3& pos3D)
    {
        Scalar minDist = -1;
        for(int id = 0; id < roots.size(); id++)
        {
            Vector2 tmp_tex;
            Vector3 tmp_pos;
            if(findIntersec(line, tmp_tex, tmp_pos, roots[id].lock()))
            {
                Scalar dist = (line.point1.head(2) - tmp_tex).norm();
                if(minDist == -1 || minDist > dist){
                    tex2D = tmp_tex;
                    pos3D = tmp_pos;
                    minDist = dist;
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

    bool findIntersec(const Line<Scalar> &line, Vector2 &tex2D, Vector3& pos3D, shared_ptr<Node> node)
    {
        //empty node
        if(node == nullptr){
            return false;
        }

        //leaves
        if(node->children[0].lock() == nullptr
           && node->children[1].lock() == nullptr)
        {
            if(checkLineLineIntersec(node->line2D, line, tex2D))
            {
                Scalar ratio = (tex2D - node->line2D.point1.head(2)).norm() / (node->line2D.point2 - node->line2D.point1).norm();
                pos3D = node->line3D.point1 + ratio * (node->line3D.point2 - node->line3D.point1);
                return true;
            }
            else{
                return false;
            }
        }

        //not leaves
        Scalar minDist = -1;
        for(size_t id = 0; id < 2; id++)
        {
            //no children, continue
            if(node->children[id].lock() == nullptr) continue;

            //no intersection with box, continue
            if(checkLineBoxIntersec(line, node->children[id].lock()->box) == false) continue;

            Vector2 int_tex2D;
            Vector3 int_pos3D;
            if(findIntersec(line, int_tex2D, int_pos3D, node->children[id].lock())){
                Scalar dist = (line.point1.head(2) - int_tex2D).norm();
                if(minDist == -1 || minDist > dist)
                {
                    minDist = dist;
                    tex2D = int_tex2D;
                    pos3D = int_pos3D;
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
        Scalar tmin = (box.minPt.x() - origin.x()) / ray.x();
        Scalar tmax = (box.maxPt.x() - origin.x()) / ray.x();

        if (tmin > tmax) std::swap(tmin, tmax);

        Scalar tymin = (box.minPt.y() - origin.y()) / ray.y();
        Scalar tymax = (box.maxPt.y() - origin.y()) / ray.y();

        if (tymin > tymax) std::swap(tymin, tymax);

        if ((tmin > tymax) || (tymin > tmax))
            return false;

        return true;
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

    AABBTree_Line<Scalar> lineTree;

    MatrixX pV, tV;
    MatrixXi pF, tF;

    Eigen::VectorXi tC, pC;

public:

    PolyMesh_AABBTree(const PolyMesh<Scalar> &_mesh)
    :PolyMesh<Scalar>::PolyMesh(_mesh.getVarList())
    {
        PolyMesh<Scalar>::polyList = _mesh.polyList;
        PolyMesh<Scalar>::vertexList = _mesh.vertexList;
        PolyMesh<Scalar>::textureList = _mesh.textureList;
        PolyMesh<Scalar>::texturedModel = _mesh.texturedModel;
    }

public:

    void buildTexTree()
    {
        PolyMesh<Scalar>::convertTexToEigenMesh(tV, tF, tC);
        texTree.init(tV, tF);

        MatrixX V, T;
        MatrixXi F;
        PolyMesh<Scalar>::convertPosTexToEigenMesh(V, T, F);
        lineTree.init(V, T, F);
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

    bool findBoundaryIntersec(const Line<Scalar> &line, Vector2 &tex2D, Vector3 &pos3D){
        return lineTree.findIntersec(line, tex2D, pos3D);
    }

    Vector3d findMeshNearestPoint(Vector3 pt);

};




#endif //TOPOLOCKCREATOR_QUADTREE_H
