//
// Created by ziqwang on 09.01.19.
//
// About AABB: https://en.wikipedia.org/wiki/Minimum_bounding_box#Axis-aligned_minimum_bounding_box

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
        Line<Scalar> line2D;         ///> 2D texCoords
        Line<Scalar> line3D;         ///> 3D vertices
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
                // create node
                shared_ptr<Node> node = make_shared<Node>();

                // tex 2D
                Vector2 staPt2D = T.row(loop[id]);
                Vector2 endPt2D = T.row(loop[(id + 1) % loop.size()]);
                node->line2D = Line<Scalar>(staPt2D, endPt2D);

                // pos 3D
                Vector3 staPt3D = V.row(loop[id]);
                Vector3 endPt3D = V.row(loop[(id + 1) % loop.size()]);
                node->line3D = Line<Scalar>(staPt3D, endPt3D);

                // box
                node->box = Box<double>(node->line2D);

                // add to the loops
                loop_nodes.push_back(node);
            }

            // create reset of the tree
            // every two neighboring lines are merged into a big area.
            vector<shared_ptr<Node>> merged_nodes;
            merged_nodes = loop_nodes;
            //if only one node left, break
            while(merged_nodes.size() > 1)
            {
                // merge every two nodes
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

    /**
     * @brief Search for an intersection between a given line and the lines defined in AABB tree
     * @param line
     * @param tex2D
     * @param pos3D
     * @param node
     * @return true/false if an instersection is found
     */
    bool findIntersec(const Line<Scalar> &line, Vector2 &tex2D, Vector3& pos3D)
    {
        Scalar minDist = -1;
        size_t max_id = roots.size();
        for(int id = 0; id < max_id; id++)
        {
            Vector2 tmp_tex;
            Vector3 tmp_pos;
            if(findIntersecFromNode(line, tmp_tex, tmp_pos, roots[id].lock()))
            {
                Scalar dist = (line.point1.head(2) - tmp_tex).norm();
                if(minDist == -1 || minDist > dist){
                    tex2D = tmp_tex;
                    pos3D = tmp_pos;
                    minDist = dist;
                }
            }
        }

        return minDist > 0;
    }

    /**
     * @brief Search for an intersection between a given line and the lines defined in AABB tree from a given node
     * @param line
     * @param tex2D
     * @param pos3D
     * @param node
     * @return true/false if an instersection is found
     */
    bool findIntersecFromNode(const Line<Scalar> &line, Vector2 &tex2D, Vector3& pos3D, shared_ptr<Node> node)
    {
        // empty node
        if(node == nullptr){
            return false;
        }

        // leaves
        if(   node->children[0].lock() == nullptr
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

        // not leaves
        Scalar minDist = -1;
        for(size_t id = 0; id < 2; id++)
        {
            // no children, continue
            if(node->children[id].lock() == nullptr) continue;

            // no intersection with box, continue
            if(checkLineBoxIntersec(line, node->children[id].lock()->box) == false) continue;

            Vector2 int_tex2D;
            Vector3 int_pos3D;
            if(findIntersecFromNode(line, int_tex2D, int_pos3D, node->children[id].lock())){
                Scalar dist = (line.point1.head(2) - int_tex2D).norm();
                if(minDist == -1 || minDist > dist)
                {
                    minDist = dist;
                    tex2D = int_tex2D;
                    pos3D = int_pos3D;
                }
            }
        }
        return minDist != -1;
    }

    /**
     * @brief Find if a 2D line is intersecting with a given box
     * @param line
     * @param box
     * @return
     */
    bool checkLineBoxIntersec(const Line<Scalar> &line, const Box<Scalar> &box)
    {
        Vector2 r = (line.point2 - line.point1).normalized().head(2);
        Vector2 origin = line.point1.head(2);
        return checkRayBoxIntersec( r, origin, box) || checkRayBoxIntersec(-r, origin, box);
    }

    /**
     * @brief Find if there is a ray-box intersection
     * @param ray
     * @param origin
     * @param box
     * @return
     *
     * @see: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
     */
    bool checkRayBoxIntersec(const Vector2 &ray, const Vector2 &origin, const Box<Scalar> &box)
    {
        Scalar txmin = (box.minPt.x() - origin.x()) / ray.x();
        Scalar txmax = (box.maxPt.x() - origin.x()) / ray.x();

        if (txmin > txmax) std::swap(txmin, txmax);

        Scalar tymin = (box.minPt.y() - origin.y()) / ray.y();
        Scalar tymax = (box.maxPt.y() - origin.y()) / ray.y();

        if (tymin > tymax) std::swap(tymin, tymax);

        if ((txmin > tymax) || (tymin > txmax))
            return false;

        return true;
    }

    /**
     * @brief Search for the intersection between two lines
     * @param l0 line1
     * @param l1 line2
     * @param pt intersection coordinates
     * @return
     *
     * @see: http://flassari.is/2008/11/line-line-intersection-in-cplusplus/
     */
    bool checkLineLineIntersec(const Line<Scalar> &l0, const Line<Scalar> &l1, Vector2 &pt)
    {
        Scalar x1 = l0.point1.x(), x2 = l0.point2.x(), x3 = l1.point1.x(), x4 = l1.point2.x();
        Scalar y1 = l0.point1.y(), y2 = l0.point2.y(), y3 = l1.point1.y(), y4 = l1.point2.y();

        Scalar d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

        // [1] - Discard parallel lines.
        if (d == 0)
            return false;

        // [2] - Get the coordinates of the intersection
        Scalar pre = (x1*y2 - y1*x2);
        Scalar post = (x3*y4 - y3*x4);
        Scalar x = ( pre * (x3 - x4) - (x1 - x2) * post ) / d;
        Scalar y = ( pre * (y3 - y4) - (y1 - y2) * post ) / d;

        // [3] - Check if the x and y coordinates are within both lines
        if ( x < min(x1, x2) || x > max(x1, x2) ||
             x < min(x3, x4) || x > max(x3, x4) )
            return false;
        if ( y < min(y1, y2) || y > max(y1, y2) ||
             y < min(y3, y4) || y > max(y3, y4) )
            return false;

        // load the point of intersection
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

    /**
     * @brief
     * @param _mesh
     */
    PolyMesh_AABBTree(const PolyMesh<Scalar> &_mesh)
    :PolyMesh<Scalar>::PolyMesh(_mesh)
    {
    }

public:

    /**
     * @brief build a texture tree
     */
    void buildTexTree()
    {
        PolyMesh<Scalar>::convertTexToEigenMesh(tV, tF, tC);
        texTree.init(tV, tF);

        MatrixX V, T;
        MatrixXi F;
        PolyMesh<Scalar>::convertPosTexToEigenMesh(V, T, F);
        lineTree.init(V, T, F);
    }

    // fixme: Unused function so far
    void buildPosTree(){
        PolyMesh<Scalar>::convertPosToEigenMesh(pV, pF, pC);
        posTree.init(pV, pF);
    }

public:

    /**
     * @brief Search if a point represented by a Vector2d belongs to a given texture polygon.
     *        If the point is on a shared edge between polygons then, one of these polygons is returned
     * @param pt, is a 2 dimension point
     * @return a texture polygon or nullptr
     */
    pPolygon findTexPoint(VectorX pt)
    {
        vector<int> indices = texTree.find(tV, tF, pt.transpose(), false);
        if(!indices.empty()){
            int faceID = tC(indices[0]);
            return PolyMesh<Scalar>::polyList[faceID];
        }
        return nullptr;
    }

    // fixme: Unused function so far
    bool findBoundaryIntersec(const Line<Scalar> &line, Vector2 &tex2D, Vector3 &pos3D){
        return lineTree.findIntersec(line, tex2D, pos3D);
    }

    // todo: implement this
    Vector3d findMeshNearestPoint(Vector3 pt);

};




#endif //TOPOLOCKCREATOR_QUADTREE_H
