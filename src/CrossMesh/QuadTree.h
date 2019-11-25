//
// Created by ziqwang on 09.01.19.
//

#ifndef TOPOLOCKCREATOR_QUADTREE_H
#define TOPOLOCKCREATOR_QUADTREE_H

#include "Mesh/PolyMesh.h"
#include "Mesh/Polygon.h"

#include "Utility/Controls.h"
#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include "Utility/math3D.h"
#include "Utility/PolyPolyTest.h"
#include "Utility/TopoObject.h"

using pPolyMesh =  shared_ptr<PolyMesh>;
using wpPolyMesh =  weak_ptr<PolyMesh>;

using pPolygon   =  shared_ptr<_Polygon>;
using wpPolygon  =  weak_ptr<_Polygon>;

class QuadTreeNode;
using pQuadTreeNode =  shared_ptr<QuadTreeNode>;
using wpQuadTreeNode =  weak_ptr<QuadTreeNode>;

struct QuadTreeNodeData
{
    int faceID;

    wpPolygon polygon;

    double area;
};

class QuadTreeNode
{
public:
    Vector2f BBmin, BBmax;

    vector<pQuadTreeNode> children;

    vector<QuadTreeNodeData> datas;

public:

    bool inside(Vector2f pos)
    {
        if(pos.x < BBmin.x || pos.x > BBmax.x) return false;
        if(pos.y < BBmin.y || pos.y > BBmax.y) return false;
        return true;
    }

    vector<Vector2f> getRectangle()
    {
        vector<Vector2f> rect;
        rect.push_back(Vector2f(BBmin.x, BBmin.y));
        rect.push_back(Vector2f(BBmax.x, BBmin.y));
        rect.push_back(Vector2f(BBmax.x, BBmax.y));
        rect.push_back(Vector2f(BBmin.x, BBmax.y));
        return rect;
    }

    pQuadTreeNode find_child(Vector2f pos){
        if(children.empty()) return nullptr;
        for(auto child : children){
            if(child && child->inside(pos)){
                return child;
            }
        }
        return nullptr;
    }

    void sort(){
        std::sort(datas.begin(), datas.end(), [&](QuadTreeNodeData &a, QuadTreeNodeData &b){
            return a.area > b.area;
        });
    }
};

/*!
 * \brief 2D Kd-Tree, for quickly searching the triangle where given points are located
 */

class QuadTree : public TopoObject{
public:

    wpPolyMesh baseMesh;

    double sizeLimit;

    pQuadTreeNode root;

public:

    QuadTree(pPolyMesh _baseMesh, shared_ptr<gluiVarList> var);

public:

    void buildQuadTree();

    void dfs_buildQuadTree(pQuadTreeNode node);

    void search_point(Vector2f pos, vector<pPolygon> &polyList);
};


#endif //TOPOLOCKCREATOR_QUADTREE_H
