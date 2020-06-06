///////////////////////////////////////////////////////////////
//
// ConvexBlock.cpp
//
//   ConvexBlock Model Class
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 15/July/2018
//
///////////////////////////////////////////////////////////////

#include "Utility/HelpDefine.h"
#include "Utility/GeometricPrimitives.h"
#include "Mesh/Cross.h"
#include "Utility/ConvexHull2D.h"
#include "Mesh/PolyMesh.h"
#include "ConvexBlock.h"

//**************************************************************************************//
//                                   Basic Operations
//**************************************************************************************//

template<typename Scalar>
ConvexBlock<Scalar>::ConvexBlock(pCross _cross, Vector2 _cutter_heights)
:TopoObject(_cross->getVarList()), cutter_heights(_cutter_heights), cross(_cross)
{
	polyMesh = nullptr;
}

template<typename Scalar>
ConvexBlock<Scalar>::~ConvexBlock()
{
	clear();
}

template<typename Scalar>
ConvexBlock<Scalar>::ConvexBlock(const ConvexBlock &part)
: TopoObject(part.getVarList())
{
    //cross should be copy in more high level stage
    if(part.polyMesh) polyMesh = make_shared<PolyMesh<Scalar>>(*part.polyMesh);
}

template<typename Scalar>
void ConvexBlock<Scalar>::clear()
{
	polyMesh.reset();
}

template<typename Scalar>
void ConvexBlock<Scalar>::print()
{
	printf("partID: %2d   ", cross.lock()->crossID);
	printf("\n");
}

//**************************************************************************************//
//                                Compute ConvexBlock Geometry 
//**************************************************************************************//

template<typename Scalar>
bool ConvexBlock<Scalar>::compute()
{
	polyMesh.reset();

	computeHyperPlanes();

	computeCorners();

    if(checkGeometry()){
        computeFaces();
        return true;
    }
    else{
        return false;
    }
}

template<typename Scalar>
bool ConvexBlock<Scalar>::checkGeometry()
{
    vector<pHypVertex> valid_corners;
    for (pHypVertex vertex: corners)
    {
        bool is_vertex_valid = true;
        for(pHypPlane plane: hypList)
        {
            if ((vertex->point - plane->point).dot(plane->normal) > FLOAT_ERROR_LARGE)
            {
                is_vertex_valid = false;
                break;
            }
        }
        if(is_vertex_valid){
            valid_corners.push_back(vertex);
        }
    }

    if(valid_corners.empty()){
        corners.clear();
        return false;
    }
    else{
        corners = valid_corners;
        return true;
    }
}

template<typename Scalar>
void ConvexBlock<Scalar>::computeHyperPlanes()
{
    if (cross.lock()->size() < 3)
        return;

    for (size_t id = 0; id < cross.lock()->size(); id++)
    {
        pHypPlane face = make_shared<HypPlane<Scalar>>();
        face->point = cross.lock()->ori(id)->point;
        face->normal = cross.lock()->ori(id)->normal;
        face->planeID = id;
        hypList.push_back(face);
    }

    Vector3 normal = cross.lock()->normal();
    Vector3 center = cross.lock()->center();
    bool boundary = cross.lock()->atBoundary;

    if(boundary || !getVarList()->getBool("only_cut_bdry"))
    {
        if(cutter_heights[0] > 0.0)
        {
            //upper plane cut
            pHypPlane face = make_shared<HypPlane<Scalar>>();
            face->point = center + normal * cutter_heights[0];
            face->normal = normal * (1.0f);
            face->planeID = hypList.size();
            hypList.push_back(face);
        }

        if(cutter_heights[1] > 0.0)
        {
            // lower plane cut
            pHypPlane face = make_shared<HypPlane<Scalar>>();
            face->point = center - normal * cutter_heights[1];
            face->normal = normal * (-1.0f);
            face->planeID = hypList.size();
            hypList.push_back(face);
        }
    }
}

template<typename Scalar>
void ConvexBlock<Scalar>::computeCorners()
{
    for(size_t l0 = 0; l0 < hypList.size(); l0++)
    {
        pHypPlane P0 = hypList[l0];
        for(size_t l1 = l0 + 1; l1 < hypList.size(); l1++)
        {
            pHypPlane P1 = hypList[l1];
            for(size_t l2 = l1 + 1; l2 < hypList.size(); l2++)
            {
                pHypPlane P2 = hypList[l2];

                Eigen::Matrix3d mat;
                mat.row(0) = P0->normal.template cast<double>();
                mat.row(1) = P1->normal.template cast<double>();
                mat.row(2) = P2->normal.template cast<double>();
                if(std::fabs(mat.determinant()) < FLOAT_ERROR_LARGE) continue;

                Eigen::Vector3d D(P0->getD(), P1->getD(), P2->getD());
                pHypVertex vertex = make_shared<HypVertex<Scalar>>();

                vertex->point = (mat.inverse() * D).cast<Scalar>();
                vertex->verID = corners.size();
                vertex->planeIDs[0] = P0->planeID;
                vertex->planeIDs[1] = P1->planeID;
                vertex->planeIDs[2] = P2->planeID;
                corners.push_back(vertex);
            }
        }
    }
}

template<typename Scalar>
void ConvexBlock<Scalar>::computeFaces()
{
    ConvexHull2D<Scalar> convexhull;
    vector<pPolygon> polyList;
    for(pHypPlane plane: hypList)
    {
        vector<Vector3> pointList;
        for(pHypVertex vertex: corners)
        {
            bool is_vertex_on_plane = false;
            for(int kd = 0; kd < 3; kd++){
                if(vertex->planeIDs[kd] == plane->planeID){
                    is_vertex_on_plane = true;
                    break;
                }
            }

            if(is_vertex_on_plane){
                pointList.push_back(vertex->point);
            }
        }

        if(pointList.size() >= 3){
            vector<Vector3> cvx_points;
            convexhull.compute(pointList, plane->normal, cvx_points);
            pPolygon polygon = make_shared<_Polygon<Scalar>>();
            polygon->setVertices(cvx_points);
            polyList.push_back(polygon);
        }
    }
    
    if(polyList.empty()){
        std::cout << "Empty" << std::endl;
    }

    if(!polyList.empty()){
        polyMesh = make_shared<PolyMesh<Scalar>>(getVarList());
        polyMesh->setPolyLists(polyList);
    }
    return;
}


template class ConvexBlock<double>;
