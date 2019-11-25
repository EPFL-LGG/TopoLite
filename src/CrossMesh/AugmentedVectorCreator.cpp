///////////////////////////////////////////////////////////////
//
// Remesh_Own.cpp
//
//  Convert Polygonal Mesh into Cross Mesh
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 02/Feb/2018
//
//
///////////////////////////////////////////////////////////////

#include "time.h"
#include "Utility/HelpDefine.h"
#include "Utility/HelpFunc.h"
#include "Mesh/Polygon.h"
#include "Mesh/PolyMesh.h"
#include "Mesh/HEdgeMesh.h"
#include "Mesh/Cross.h"
#include "Mesh/CrossMesh.h"
#include "AugmentedVectorCreator.h"
#include "CrossConstraint.h"
#include "IO/gluiVar.h"

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

AugmentedVectorCreator::AugmentedVectorCreator()
{

}

AugmentedVectorCreator::~AugmentedVectorCreator()
{

}


//**************************************************************************************//
//                           Convert Polygonal Mesh into Cross Mesh
//**************************************************************************************//

void AugmentedVectorCreator::CreateCrossMesh_Own(pPolyMesh polyMesh, float tiltAngle, pCrossMesh &crossMesh)
{
	/////////////////////////////////////////////////////////////
	// 1. Initialize a cross mesh from the polygonal mesh
	clock_t sta = clock();
	crossMesh.reset();
	crossMesh = make_shared<CrossMesh>();
	InitCrossMesh(polyMesh, crossMesh);
	//std::cout << "----InitCrossMesh:\t" << (float)(clock() - sta) / (CLOCKS_PER_SEC) << std::endl;

	/////////////////////////////////////////////////////////////
	// 2. Build half-edge mesh of the polygonal mesh
	sta = clock();
	pHEdgeMesh hedgeMesh = make_shared<HEdgeMesh>();
	hedgeMesh->InitHEdgeMesh(polyMesh);
	hedgeMesh->BuildHalfEdgeMesh();
	//std::cout << "----InitHEdgeMesh:\t" << (float)(clock() - sta) / (CLOCKS_PER_SEC) << std::endl;

	/////////////////////////////////////////////////////////////
	// 3. Compute neighbors for the cross mesh
	sta = clock();
	ComputeCrossNeighbors(hedgeMesh, crossMesh);
	hedgeMesh.reset();
	//std::cout << "----ComputeCrossNeighbors:\t" << (float)(clock() - sta) / (CLOCKS_PER_SEC) << std::endl;

    if(crossMesh){
        ComputePracticalBoundary(crossMesh);
    }

    /////////////////////////////////////////////////////////////
	// 4. Optimize CrossMesh
	if(aabbTree.lock() != nullptr)
		OptimizeBaseMesh(crossMesh);

	/////////////////////////////////////////////////////////////
	// 5. Compute tilt normals
	sta = clock();
	InitMeshTiltNormals(crossMesh);
	UpdateMeshTiltNormals(crossMesh, tiltAngle);
	//std::cout << "----UpdateMeshTiltNormals:\t" << (float)(clock() - sta) / (CLOCKS_PER_SEC) << std::endl;
}


void AugmentedVectorCreator::CreateCrossMesh_Own(float tiltAngle, pCrossMesh &crossMesh)
{
	clock_t sta = clock();

	if(crossMesh){
        ComputePracticalBoundary(crossMesh);
    }

	/////////////////////////////////////////////////////////////
	// 4. Optimize CrossMesh
	if(aabbTree.lock() != nullptr)
		OptimizeBaseMesh(crossMesh);

	/////////////////////////////////////////////////////////////
	// 5. Compute tilt normals
	sta = clock();
	InitMeshTiltNormals(crossMesh);
	UpdateMeshTiltNormals(crossMesh, tiltAngle);
	//std::cout << "----UpdateMeshTiltNormals:\t" << (float)(clock() - sta) / (CLOCKS_PER_SEC) << std::endl;
}


void AugmentedVectorCreator::InitCrossMesh(pPolyMesh polyMesh, pCrossMesh &crossMesh)
{
	for (int i = 0; i < polyMesh->polyList.size(); i++)
	{
		pPolygon poly = polyMesh->polyList[i];

		shared_ptr<Cross> cross = make_shared<Cross>();
		cross->atBoundary = false;
		cross->crossID = i;

		for (int j = 0; j < poly->vers.size(); j++)
		{
			cross->vers.push_back(poly->vers[j]);
			cross->verIDs.push_back(poly->verIDs[j]);
		}

		cross->ComputeNormal();
		crossMesh->crossList.push_back(cross);
	}

	crossMesh->vertexList = polyMesh->vertexList;
	return;
}

void AugmentedVectorCreator::ComputeCrossNeighbors(pHEdgeMesh hedgeMesh, pCrossMesh crossMesh)
{
	if (crossMesh->crossList.size() < 2) return;

	vector<pHFace> faceList = hedgeMesh->GetFaceList();

	// Find neighboring cross based on the half-edge graph
	for (int i = 0; i < faceList.size(); i++)
	{
		pHFace face = faceList[i];
		shared_ptr<Cross> cross = crossMesh->crossList[i];

		for (int j = 0; j < face->edges.size(); j++)
		{
			if (face->edges[j].lock()->twin.lock() == nullptr)
			{
				cross->neighbors.push_back(weak_ptr<Cross>());
				cross->atBoundary = true;
				continue;
			}

			int neiborFaceID = face->edges[j].lock()->twin.lock()->face.lock()->id;

			if (neiborFaceID < 0 || neiborFaceID >= crossMesh->crossList.size())
			{
				printf("Warning: neiborFaceID = %d is invalid \n", neiborFaceID);
				cross->neighbors.push_back(weak_ptr<Cross>());
				continue;
			}

			//printf("i=%d   j=%d   neiborFaceID: %d \n", i, j, neiborFaceID);

			shared_ptr<Cross> neiborCross = crossMesh->crossList[neiborFaceID];
			cross->neighbors.push_back(neiborCross);
		}
	}
}

void AugmentedVectorCreator::InitMeshTiltNormals(pCrossMesh crossMesh)
{
	for (int i = 0; i < crossMesh->crossList.size(); i++)
	{
		shared_ptr<Cross> cross = crossMesh->crossList[i];

		cross->InitTiltNormals();
	}
}


void AugmentedVectorCreator::UpdateMeshTiltNormals(pCrossMesh crossMesh, float tiltAngle)
{
	vector<pCross> bfsQueue;

	if(crossMesh->crossList.empty())
		return;
	// Find a root for Breadth-First Search Traversal

	shared_ptr<Cross> root;
	for(int id = 0; id < crossMesh->crossList.size(); id++){
		if(!crossMesh->crossList[id]->atBoundary)
        {
            root = crossMesh->crossList[id];
            break;
        }

	}

	if(root == nullptr)
		return;

	root->UpdateTiltNormal_Root(tiltAngle);
	root->isVisited = true;
	bfsQueue.push_back(root);

	//for (int i = 0; i < root->neighbors.size(); i++)
	//{
	//	printf(" i=%d \n", i);
	//	root->neighbors[i]->CorrectTiltNormal(tiltAngleUpper);
	//}

	//return;

	// Start Breadth-First Search Traversal
	while (true)
		// Bug: Segmentation fault here; need to check the size of bfsQueue is non zero; recommand to change the traversal method to iterating all tessellation cells
	{

        if(bfsQueue.empty()) break;

		shared_ptr<Cross> currVisit = bfsQueue[0];
		bfsQueue.erase(bfsQueue.begin());

		// Quit if all crosses have been visited
		bool isAllVisited = true;
		for (int i = 0; i < crossMesh->crossList.size(); i++)
		{
			pCross cross = crossMesh->crossList[i];
			if(cross == nullptr)
                continue;
			if (cross->isVisited == false)
			{
				isAllVisited = false;
				break;
			}
		}

		if (isAllVisited)
		{
			break;
		}

		// Process the neighbors of current cross
		for (int i = 0; i < currVisit->neighbors.size(); i++)
		{
			//printf(" i=%d \n", i);
			shared_ptr<Cross> nextVisit = currVisit->neighbors[i].lock();

			if (nextVisit == NULL)
				continue;

//			if(nextVisit->crossID == 3){
//                printf("stop");
//			}

			if(nextVisit->atBoundary && getVarList()->get<bool>("ground_touch_bdry"))
				continue;

			if (nextVisit->isVisited == true)
				continue;

			nextVisit->UpdateTiltNormal(tiltAngle);

			nextVisit->isVisited = true;

			bfsQueue.push_back(nextVisit);
		}
	}

	if(getVarList()->get<bool>("ground_touch_bdry")){
        for(int id = 0; id < crossMesh->crossList.size(); id++)
        {
            if(crossMesh->crossList[id]->atBoundary)
                crossMesh->crossList[id]->UpdateTiltNormal(tiltAngle);
        }
	}
}

void AugmentedVectorCreator::OptimizeBaseMesh(shared_ptr<CrossMesh> crossMesh)
{
	int iterator = getVarList()->get<int>("shapeop_iterator");
	if(iterator <= 0)
		return;

	int n = crossMesh->vertexList.size();
	int m = crossMesh->crossList.size();
	ShapeOp::Matrix3X position(3, n);
	for(int id = 0; id < n ; id++){
		Vector3f pt = crossMesh->vertexList[id];
		position.col(id) = ShapeOp::Vector3(pt.x, pt.y, pt.z);
	}

	ShapeOp::Solver solver;
	solver.setPoints(position);

	double weight = getVarList()->get<float>("shapeop_planarity");

	//in plane constraint
	{
		std::vector<int> id_vector;
		for(int id = 0; id < m; id++)
		{
			id_vector.clear();
			for(int verID : crossMesh->crossList[id]->verIDs){
				id_vector.push_back(verID);
			}
			shared_ptr<ShapeOp::Constraint> constraint = make_shared<ShapeOp::PlaneConstraint>(id_vector, weight, position);
			solver.addConstraint(constraint);
		}
	}

	//boundary constraint
	weight = getVarList()->get<float>("shapeop_boundary");
	{
		for(int id = 0; id < m; id++){
			shared_ptr<Cross> cross = crossMesh->crossList[id];
			if(cross->atBoundary)
			{
				for(int jd = 0; jd < cross->verIDs.size(); jd++){
					std::vector<int> id_vector;
					int verID = cross->verIDs[jd]; id_vector.push_back(verID);
					ShapeOp::Vector3 pt;
					pt << cross->vers[jd].pos.x, cross->vers[jd].pos.y, cross->vers[jd].pos.z;
					shared_ptr<ShapeOp::ClosenessConstraint> constraint = make_shared<ShapeOp::ClosenessConstraint>(id_vector, weight, position);
					constraint->setPosition(pt);
					solver.addConstraint(constraint);
				}
			}
		}
	}

	//closeness constraint
	weight = getVarList()->get<float>("shapeop_meshFit");
	{
		std::vector<int> id_vector;
		for(int id = 0; id < n; id++)
		{
			id_vector.clear();
			id_vector.push_back(id);
			shared_ptr<ShapeOp::MeshClosenessConstraint> constraint = make_shared<ShapeOp::MeshClosenessConstraint>(id_vector, weight, position);
			constraint->setTree(aabbTree, aabbV, aabbF);
			solver.addConstraint(constraint);
		}
	}

	//bending constraint
	weight  = getVarList()->get<float>("shapeop_bending");
	{
		for(int id = 0; id < m; id++){
			shared_ptr<Cross> cross = crossMesh->crossList[id];
			int nC = cross->verIDs.size();
			for(int jd = 0; jd < cross->neighbors.size(); jd++){
				shared_ptr<Cross> ncross = cross->neighbors[jd].lock();
				if(ncross == nullptr || cross->crossID > ncross->crossID) continue;
				int edgeID = ncross->GetNeighborEdgeID(cross->crossID);
				vector<int> id_vector;
				id_vector.push_back(cross->verIDs[jd]);
				id_vector.push_back(cross->verIDs[(jd + 1) % nC]);
				id_vector.push_back(cross->verIDs[(jd + 2) % nC]);
				id_vector.push_back(ncross->verIDs[(edgeID + 2) % ncross->verIDs.size()]);
				shared_ptr<ShapeOp::BendingConstraint> constraint = make_shared<ShapeOp::BendingConstraint>(id_vector, weight, position, 0, 0.5);
				solver.addConstraint(constraint);
			}
		}
	}

	//regular
	weight  = getVarList()->get<float>("shapeop_regularity");
	{
		for(int id = 0 ;id < m; id++)
		{
			shared_ptr<Cross> cross = crossMesh->crossList[id];
			int num_verIDs = cross->verIDs.size();
			Vector3f center = cross->center;
			Vector3f normal = cross->normal;
			Vector3f x_axis = Vector3f(1, 0, 0) CROSS normal;
			if(len(x_axis) < 1e-4) x_axis = Vector3f(0, 1, 0) CROSS normal;
			Vector3f y_axis = normal CROSS x_axis;
			Eigen::MatrixXd shape(3, num_verIDs);
			vector<int> id_vector;
			for(int jd = 0; jd < num_verIDs; jd ++)
			{
				float angle = M_PI * 2 / num_verIDs * jd;
				Vector3f pt = x_axis * std::cos(angle) + y_axis * std::sin(angle) + center;
				shape.col(jd) << pt.x, pt.y, pt.z;
				id_vector.push_back(cross->verIDs[jd]);
			}
			shared_ptr<ShapeOp::SimilarityConstraint> constraint = make_shared<ShapeOp::SimilarityConstraint>(id_vector, weight, position,true, true, true);
			std::vector<ShapeOp::Matrix3X> shapes;
			shapes.push_back(shape);
			constraint->setShapes(shapes);
			solver.addConstraint(constraint);
		}
	}

	solver.initialize();
	solver.solve(iterator);

	position = solver.getPoints();
	for(int id = 0; id < n ; id++)
	{
		Vector3f &pt = crossMesh->vertexList[id];
		pt.x = position(0, id);
		pt.y = position(1, id);
		pt.z = position(2, id);
	}

	crossMesh->UpdateCrossFromVertexList();
}

void AugmentedVectorCreator::setAABBTree(shared_ptr<igl::AABB<Eigen::MatrixXd, 3>> _aabbTree,
							  std::shared_ptr<Eigen::MatrixXd> V,
							  std::shared_ptr<Eigen::MatrixXi> F) {
	aabbTree = _aabbTree;
	aabbV = V;
	aabbF = F;
}

bool AugmentedVectorCreator::UpdateMeshTiltRange(shared_ptr<CrossMesh> crossMesh)
{
	int m = crossMesh->crossList.size();
	float big_zero_eps = getVarList()->get<float>("big_zero_eps");
	for(int id = 0; id < m; id++)
	{
		shared_ptr<Cross> cross = crossMesh->crossList[id];
		for(int jd = 0; jd < cross->neighbors.size(); jd++){
			shared_ptr<OrientPoint> oriPt = cross->oriPoints[jd];
			Vector3f t = oriPt->rotation_base; t /= len(t);
			Vector3f e = oriPt->rotation_axis; e /= len(e);
			Vector3f y = oriPt->rotation_base CROSS oriPt->rotation_axis; y /= len(y);
			if(oriPt->tiltSign == 1) y *= -1;
			Vector3f p0 = oriPt->point;
			for(int kd = 0; kd < cross->vers.size(); kd++)
			{
				Vector3f pi = cross->vers[kd].pos;
				double yPiP0 = (y) DOT(pi - p0);
				double tPiP0 = (-t) DOT (pi - p0);
				if(std::abs(tPiP0) < big_zero_eps && std::abs(yPiP0) < big_zero_eps)
				{
					continue;
				}
				if(std::abs(tPiP0) < big_zero_eps && yPiP0 > big_zero_eps)
				{
					oriPt->tilt_range = Vector2f(0, 0);
                    oriPt->sided_range = oriPt->tilt_range;
					continue;
				}
				else{

                    double angle = 0;
				    if(yPiP0 > big_zero_eps){
                        angle = 90 - std::atan(yPiP0/tPiP0) * 180 / M_PI;
				    }
				    else if(std::fabs(yPiP0) <= big_zero_eps){
                        angle = 90;
				    }
				    else{
                        angle = std::fabs(std::atan(yPiP0/tPiP0) * 180 / M_PI) + 90;
				    }
                    if(oriPt->tilt_range.y > angle) oriPt->tilt_range.y = angle;
				}
                oriPt->sided_range = oriPt->tilt_range;
			}
		}
	}

	double min_max = 90;
	double max_min = 0;
	for(int id = 0; id < m; id++){
		shared_ptr<Cross> cross = crossMesh->crossList[id];
		for(int jd = 0; jd < cross->neighbors.size(); jd++)
		{
			shared_ptr<Cross> ncross = cross->neighbors[jd].lock();
			if(ncross == nullptr || cross->crossID > ncross->crossID) continue;
			int edgeID = ncross->GetNeighborEdgeID(cross->crossID);
			shared_ptr<OrientPoint> noriPt = ncross->oriPoints[edgeID];
			shared_ptr<OrientPoint> oriPt = cross->oriPoints[jd];
			Vector2f range(0, 0);
			range.x = std::max(oriPt->tilt_range.x, noriPt->tilt_range.x);
			range.y = std::min(oriPt->tilt_range.y, noriPt->tilt_range.y);
			oriPt->tilt_range = range;
			noriPt->tilt_range = range;
			//std::cout << "Cross:\t" << cross->crossID << ",\t OriID:\t" << jd << ",\t Range:\t[" << range.x << ", " << range.y << "]" << std::endl;
			if(min_max > range.y) min_max = range.y;
			if(max_min < range.x) max_min = range.x;
		}
	}
	std::cout <<"Range:\t[" << max_min << ",\t" << min_max << "]"<< std::endl;

	if(getVarList()->get<float>("tiltAngle") < max_min || getVarList()->get<float>("tiltAngle") > min_max)
        return false;
	else
        return true;

}

void AugmentedVectorCreator::ComputePracticalBoundary(shared_ptr<CrossMesh> &crossMesh)
{
    vector<weak_ptr<Cross>> boundaryCross;
    crossMesh->UpdateCrossVertexIndex();
    vector<vector<weak_ptr<Cross>>> &vertexCrossList = crossMesh->vertexCrossList;


    for(int id = 0; id < crossMesh->crossList.size(); id++)
    {
        shared_ptr<Cross> cross = crossMesh->crossList[id];
        cross->atBoundary = false;
    }

    auto setVertexRingBoundary = [&](int verID){
        if(verID >= 0 && verID < vertexCrossList.size())
        {
            for(weak_ptr<Cross> cross: vertexCrossList[verID]){
                if(!cross.lock()->atBoundary){
                    boundaryCross.push_back(cross);
                    cross.lock()->atBoundary = true;
                }
            }
        }
        return;
    };

    for(int id = 0; id < crossMesh->crossList.size(); id++)
    {
        shared_ptr<Cross> cross = crossMesh->crossList[id];
        for(int jd = 0; jd < cross->neighbors.size(); jd ++)
        {
            if(cross->neighbors[jd].lock() == nullptr)
            {
                int verID = cross->verIDs[jd];
                setVertexRingBoundary(verID);
                verID = cross->verIDs[(jd + 1) % cross->verIDs.size()];
                setVertexRingBoundary(verID);
            }
        }
    }

    float minBoundaryEdge = getVarList()->get<float>("minBoundaryEdge");
    for(int id = 0; id < boundaryCross.size(); id++)
    {
        shared_ptr<Cross> cross = boundaryCross[id].lock();
        size_t size = cross->vers.size();
        for(int jd = 0; jd < size; jd++)
        {
            //the neighbor must exists
            if(!cross->neighbors[jd].lock()) continue;

            float edgeLen = len(cross->vers[(jd + 1) % size].pos - cross->vers[jd].pos);
            //if the length of the edge is too small, then include one more part
            if(minBoundaryEdge > edgeLen)
            {
                weak_ptr<Cross> ncross = cross->neighbors[jd];
                int shared_id = cross->GetSharedCross(ncross);

                if(shared_id >= 0 && shared_id < crossMesh->crossList.size()){
                    shared_ptr<Cross> shared_cross = crossMesh->crossList[shared_id];
                    shared_cross->atBoundary = true;
                }
            }
        }
    }

    for(int id = 0; id < crossMesh->crossList.size(); id++) {
        shared_ptr<Cross> cross = crossMesh->crossList[id];
        if (cross == nullptr || cross->atBoundary) continue;
        bool allBoundary = true;
        for (int jd = 0; jd < cross->neighbors.size(); jd++) {
            weak_ptr<Cross> ncross = cross->neighbors[jd];
            if (ncross.lock() != nullptr && ncross.lock()->atBoundary == false) {
                allBoundary = false;
                break;
            }
        }
        cross->atBoundary = allBoundary;
    }
}