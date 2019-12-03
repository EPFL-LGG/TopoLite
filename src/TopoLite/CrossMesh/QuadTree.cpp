//
// Created by ziqwang on 09.01.19.
//

#include "Mesh/PolyMesh.h"
#include "Mesh/Polygon.h"

#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include "Utility/math3D.h"
#include "QuadTree.h"
#include "tbb/tbb.h"
#include "IO/InputVar.h"

#include "clipper.hpp"
#include "Utility/ConvexHull2D.h"

QuadTree::QuadTree(pPolyMesh _baseMesh, shared_ptr<InputVarList> var)
: TopoObject(var)
{
    baseMesh = _baseMesh;
    sizeLimit = getVarList()->get<float>("crossmesh_quadTree_leafNodeSize");
    buildQuadTree();
}

void QuadTree::buildQuadTree()
{
    root.reset();
    root = make_shared<QuadTreeNode>();
    root->BBmin = Vector2f(-2, -2);
    root->BBmax = Vector2f(2, 2);

    for(int id = 0; id < baseMesh.lock()->polyList.size(); id++)
    {
        pPolygon poly = baseMesh.lock()->polyList[id];
        QuadTreeNodeData quad_data;
        quad_data.polygon = poly;
        quad_data.faceID = id;
        quad_data.area = 0;
        root->datas.push_back(quad_data);
    }

    dfs_buildQuadTree(root);
}

void QuadTree::dfs_buildQuadTree(pQuadTreeNode node)
{

    //std::cout << "Len :\t" << len(node->BBmax - node->BBmin) << ", Size :\t" << node->polygons.size() << std::endl;
    if(node->datas.size() <= 1) return;
    if(len(node->BBmax - node->BBmin) < sizeLimit ) return;

    float minX, midX, maxX;
    float minY, midY, maxY;
    minX = node->BBmin.x; midX = (node->BBmin.x + node->BBmax.x) * 0.5; maxX = node->BBmax.x;
    minY = node->BBmin.y; midY = (node->BBmin.y + node->BBmax.y) * 0.5; maxY = node->BBmax.y;

    pQuadTreeNode child;
    //left_low
    child = make_shared<QuadTreeNode>();
    child->BBmin = Vector2f(minX, minY);
    child->BBmax = Vector2f(midX, midY);
    node->children.push_back(child);

    //left_high
    child = make_shared<QuadTreeNode>();
    child->BBmin = Vector2f(minX, midY);
    child->BBmax = Vector2f(midX, maxY);
    node->children.push_back(child);

    //righ_low
    child = make_shared<QuadTreeNode>();
    child->BBmin = Vector2f(midX, minY);
    child->BBmax = Vector2f(maxX, midY);
    node->children.push_back(child);

    //righ_high
    child = make_shared<QuadTreeNode>();
    child->BBmin = Vector2f(midX, midY);
    child->BBmax = Vector2f(maxX, maxY);
    node->children.push_back(child);

    tbb::parallel_for(tbb::blocked_range<size_t>(0, 4), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t id = r.begin(); id != r.end(); ++id)
        {
            vector<Vector2f> rect = node->children[id]->getRectangle();
            for (int jd = 0; jd < node->datas.size(); jd++)
            {
                pPolygon poly = node->datas[jd].polygon.lock();
                int faceID = node->datas[jd].faceID;
                vector<Vector2f> tex = poly->GetVerticesTex();
                double area = 0;
                if (IsConvexPolygonIntersec(rect, tex, area))
                {
                    QuadTreeNodeData quadData;
                    quadData.polygon = poly;
                    quadData.faceID = faceID;
                    quadData.area = area;
                    node->children[id]->datas.push_back(quadData);
                }
            }

            if (!node->children[id]->datas.empty())
            {
                node->children[id]->sort();
                dfs_buildQuadTree(node->children[id]);
            }
        }
    });

    node->datas.clear();
}

void QuadTree::search_point(Vector2f pos, vector<pPolygon> &polyList)
{
    pQuadTreeNode node = root;
    polyList.clear();

    if(node == nullptr) return;

    while(node){
        pQuadTreeNode child = node->find_child(pos);
        if(child == nullptr) break;
        node = child;
    }

    if(node == nullptr || node->datas.empty()) return;
    else{
        for(auto quadData : node->datas){
            int faceID = quadData.faceID;
            polyList.push_back(baseMesh.lock()->polyList[faceID]);
        }
    }
}

bool QuadTree::IsConvexPolygonIntersec(vector<Vector2f> &polyA, vector<Vector2f> &polyB, double &area){

    double Scale = getVarList()->get<float>("clipper_scale");

    ClipperLib::Path pathA, pathB;
    for(int id = 0; id < polyA.size(); id++){
        int x = polyA[id].x * Scale;
        int y = polyA[id].y * Scale;
        pathA.push_back(ClipperLib::IntPoint(x, y));
    }

    for(int id = 0; id < polyB.size(); id++){
        int x = polyB[id].x * Scale;
        int y = polyB[id].y * Scale;
        pathB.push_back(ClipperLib::IntPoint(x, y));
    }

    if(!ClipperLib::Orientation(pathA)) {
        ClipperLib::ReversePath(pathA);
        //std::cout << "PathA is Wrong" << std::endl;
    }
    if(!ClipperLib::Orientation(pathB))
    {
        ClipperLib::ReversePath(pathB);
        //std::cout << "pathB is Wrong" << std::endl;
    }

    ClipperLib::Clipper solver;
    solver.AddPath(pathA, ClipperLib::ptSubject, true);
    solver.AddPath(pathB, ClipperLib::ptClip, true);
    ClipperLib::Paths path_int;
    solver.Execute(ClipperLib::ctIntersection, path_int, ClipperLib::pftPositive, ClipperLib::pftPositive);

    if(path_int.empty() || path_int.front().empty()) {
        area = 0;
        return false;
    }
    else{
        area = 0;
        for(ClipperLib::Path path : path_int){
            area += ClipperLib::Area(path);
        }
        return true;
    }

}


