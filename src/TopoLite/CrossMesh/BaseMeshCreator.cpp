#include <memory>///////////////////////////////////////////////////////////////
//
// Remesh_Para.cpp
//
//  Convert Parameterized Mesh into Cross Mesh
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 12/Jan/2018
//
//
///////////////////////////////////////////////////////////////


#include "BaseMeshCreator.h"


//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

template <typename Scalar>
BaseMeshCreator<Scalar>::BaseMeshCreator(pPolyMeshAABB _polyMesh,
                                         pCrossMesh _pattern2D)
: TopoObject(_polyMesh?_polyMesh->getVarList():make_shared<InputVarList>())
{
    polyMesh = _polyMesh;
    pattern2D = _pattern2D;
}

template <typename Scalar>
BaseMeshCreator<Scalar>::BaseMeshCreator(shared_ptr<InputVarList> var)
: TopoObject(var)
{

}

template <typename Scalar>
BaseMeshCreator<Scalar>::~BaseMeshCreator()
{

}

//**************************************************************************************//
//                               Compute Lifted 3D Mesh 
//**************************************************************************************//

template <typename Scalar>
void BaseMeshCreator<Scalar>::computeBaseCrossMesh(double *inverTextureMat,
                                                   pPolyMesh &baseMesh2D,
                                                   pCrossMesh &crossMesh)
{
	tbb::tick_count sta = tbb::tick_count::now();

	crossMesh = make_shared<CrossMesh<double>>(getVarList());
	baseMesh2D = make_shared<PolyMesh<double>>(getVarList());

	map_cross2D_3D.clear();
	map_cross3D_2D.clear();

    computeInternalCross(inverTextureMat, baseMesh2D, crossMesh);

//	if(getVarList()->template get<bool>("smooth_bdry"))
//	{
//        ComputeBoundaryCross(inverTextureMat, baseMesh2D, crossMesh);
//		//remove dangling
//		RemoveDanglingCross(crossMesh);
//    }
//
//    if(baseMesh2D){
//        baseMesh2D->setVarList(getVarList());
//    }
//
//	if(crossMesh){
//        crossMesh->setVarList(getVarList());
//        ComputePracticalBoundary(crossMesh);
//    }
//
//	std::cout << "Remesh Para:\t" << (tbb::tick_count::now() - sta).seconds() << std::endl;
}


template <typename Scalar>
void BaseMeshCreator<Scalar>::InitCrossMesh(pPolyMesh polyMesh, pCrossMesh &crossMesh)
{
    for (size_t i = 0; i < polyMesh->polyList.size(); i++)
    {
        pPolygon poly = polyMesh->polyList[i];

        pCross cross = make_shared<Cross>(getVarList());
        cross->atBoundary = false;
        cross->crossID = i;

        for (size_t j = 0; j < poly->vers.size(); j++)
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


template <typename Scalar>
void BaseMeshCreator<Scalar>::RemoveDanglingCross(pCrossMesh crossMesh)
{
	if(crossMesh == nullptr) return;
	if(crossMesh->crossList.size() < 3) return;
	for(auto it = crossMesh->crossList.begin(); it != crossMesh->crossList.end();)
	{
		pCross cross = *it;
		if(cross == nullptr) continue;
		int numNeighbor = 0;
		pCross ncross = nullptr;
		for(size_t jd = 0; jd < cross->neighbors.size(); jd++){
			if(cross->neighbors[jd].lock() == nullptr) continue;
			numNeighbor ++;
			ncross = cross->neighbors[jd].lock();
		}
		if(numNeighbor == 0){
			it = crossMesh->crossList.erase(it);
		}
		else if(numNeighbor == 1){
			ncross->neighbors[ncross->GetNeighborEdgeID(cross->crossID)] = pCross();
			it = crossMesh->crossList.erase(it);
		}
		else{
			it ++;
		}
	}
    crossMesh->updateCrossID();
}

template <typename Scalar>
void BaseMeshCreator<Scalar>::computeInternalCross(double *inverTextureMat,
                                                   pPolyMesh &baseMesh2D,
                                                   pCrossMesh &crossMesh)
{
    //at this stage, baseMes2D and crossMesh are empty.
    if(pattern2D.lock() == nullptr) return;

    //map 2D pattern vertices on 3D input surface
	for(size_t id = 0; id < pattern2D.lock()->vertexList.size(); id++)
	{
        if(pattern2D.lock()->vertexList[id] == nullptr) continue;

        //1) get the interactive position of each vertex.
		Vector2 ver_2DCoord = pattern2D.lock()->vertexList[id]->pos.head(2);
		Vector2 tex_2DCoord = GetTextureCoord(ver_2DCoord, viewSize, inverTextureMat);
		//MultiplyPoint(texCoord, inverTextureMat, texCoord);

		//2) compute the 3D coordinates of the 2D texture vertices
		//by inversing the parametrization mapping
		Vector3 ver_3DCoord;
		if(mapTexPointBackToSurface(tex_2DCoord, ver_3DCoord))
		{
		    //if find its corresponding point on the surface
            int ver_index2D = id;
            int ver_index3D = crossMesh->vertexList.size();

            //create a new pVertex and add it to crossMesh
            pVertex ver3D = make_shared<VPoint<Scalar>>(ver_3DCoord); //coordinate
            ver3D->verID = crossMesh->vertexList.size(); //index
			crossMesh->vertexList.push_back(ver3D);

            map_vertex2D_3D.push_back(ver_index3D);
			map_vertex3D_2D.push_back(ver_index2D);
		}
		else{
		    //use -1 to represent no corresponding point
            int ver_index2D = id;
            int ver_index3D = -1;
			map_vertex2D_3D.push_back(-1);
		}

	}

	//generating all internal crosses
	for(size_t id = 0; id < pattern2D.lock()->size(); id++)
	{
	    if(pattern2D.lock()->cross(id) == nullptr) continue;
        pCross poly2D = pattern2D.lock()->cross(id);

		//create an empty 3D internal cross
		pCross cross3D = make_shared<Cross<Scalar>>(getVarList());
        cross3D->atBoundary = false;
        cross3D->crossID = crossMesh->size();

        //from previous section,
		//we know the 3D corresponding coordinate of each poly2D's vertex.
		//if exist, add it to the cross3D
		//if not, stop
		size_t jd = 0;
		for (; jd < poly2D->vers.size(); jd++)
		{
			int ver_index2D = poly2D->vers[jd]->verID;
			int ver_index3D = map_vertex2D_3D[ver_index2D];
			if(ver_index3D == -1) break;

			pVertex vertex = crossMesh->vertexList[ver_index3D];
			cross3D->vers.push_back(vertex);
		}

		//if all vertex of poly2D is interal.
		if(jd != poly2D->vers.size())
		{
		    int cross_index2D = id;
		    int cross_index3D = cross3D->crossID;

			map_cross2D_3D.push_back(cross_index3D);
			map_cross3D_2D.push_back(cross_index2D);

			crossMesh->push_back(cross3D);

			baseMesh2D->polyList.push_back(make_shared<_Polygon<Scalar>>(*poly2D));  // Cross mesh in the texture UV space
		}
		//if not, currently no corresponding 3D cross.
		else {
			map_cross2D_3D.push_back(-1);
		}
	}

	//build crossMesh connectivity
	crossMesh->createConnectivity();
}


template <typename Scalar>
void BaseMeshCreator<Scalar>::ComputeBoundaryCross(double *inverTextureMat,
									   pPolyMesh &baseMesh2D,
									   pCrossMesh &crossMesh)
{

    float minimumCrossArea = getVarList()->template get<float>("minCrossArea");
	//get all the half_inside pattern2D

	vector<int> half_inside_pattern2D;
	for(size_t id = 0; id < pattern2D.lock()->crossList.size(); id++)
	{
	    if(pattern2D.lock()->crossList[id] == nullptr) continue;
		int cross2ID = pattern2D.lock()->crossList[id]->crossID;
		if(map_cross2D_3D[cross2ID] != -1) continue;
		pCross cross2D = pattern2D.lock()->crossList[id];
		bool has_vertex_in_texture = false;
		for(size_t jd = 0; jd < cross2D->verIDs.size(); jd++)
		{
			if(map_vertex2D_3D[cross2D->verIDs[jd]] != -1)
			{
				has_vertex_in_texture = true;
				break;
			}
		}
		if(has_vertex_in_texture) half_inside_pattern2D.push_back(cross2ID);
	}

	//for each cut the polygon by the mesh boundary
	//here only compute the vertex position of the cut conner


    std::unordered_map<int, vector<int>> patter2D_cut_conner3D, patter2D_cut_conner2D;
	for(size_t id = 0; id < half_inside_pattern2D.size(); id++)
	{

		int cross2ID = half_inside_pattern2D[id];
		pCross cross2D = pattern2D.lock()->crossList[cross2ID];
		if(cross2D == nullptr) continue;

		vector<int> cut_conner3D, cut_conner2D;
		for(size_t jd = 0; jd < cross2D->verIDs.size(); jd++)
		{
			int ver2ID = cross2D->verIDs[jd];
			int next_ver2ID = cross2D->verIDs[(jd + 1) % cross2D->verIDs.size()];
			if((map_vertex2D_3D[ver2ID] == -1) ^ (map_vertex2D_3D[next_ver2ID] == -1))
			{
				pCross ncross2D = cross2D->neighbors[jd].lock();

				//check whether neighbor has computed
				if(ncross2D != nullptr)
				{
					auto find3D = patter2D_cut_conner3D.find(ncross2D->crossID);
					auto find2D = patter2D_cut_conner2D.find(ncross2D->crossID);

					if(find3D != patter2D_cut_conner3D.end()){
						int edgeID = ncross2D->GetNeighborEdgeID(cross2D->crossID);

						cut_conner3D.push_back(find3D->second[edgeID]);
						cut_conner2D.push_back(find2D->second[edgeID]);
						continue;
					}
				}

				//need to compute
				Vector3 sta2D, end2D;
				if(map_vertex2D_3D[ver2ID] != -1){
					sta2D = pattern2D.lock()->vertexList[ver2ID];
					end2D = pattern2D.lock()->vertexList[next_ver2ID];
				}
				else{
					end2D = pattern2D.lock()->vertexList[ver2ID];
					sta2D = pattern2D.lock()->vertexList[next_ver2ID];
				}

				Vector3 pos2D, pos3D;
                if(ComputeBoundaryVertex(inverTextureMat, sta2D, end2D, pos2D, pos3D))
				{
					cut_conner3D.push_back(crossMesh->vertexList.size());
					crossMesh->vertexList.push_back(pos3D);

					cut_conner2D.push_back(baseMesh2D->vertexList.size());
					baseMesh2D->vertexList.push_back(pos2D);
				}
				else {
					cut_conner3D.push_back(-1);
					cut_conner2D.push_back(-1);
				}
			}
			else{
				cut_conner3D.push_back(-1);
				cut_conner2D.push_back(-1);
			}
		}
		patter2D_cut_conner3D[cross2ID] = cut_conner3D;
		patter2D_cut_conner2D[cross2ID] = cut_conner2D;

    }

    //compute the cut cross mesh
	std::unordered_map<int, vector<wpCross>> pattern2D_edge_cross3D;
	std::unordered_map<int, vector<int>> pattern2D_edge_cross3D_edgeIDs;
	for(size_t id = 0; id < half_inside_pattern2D.size(); id++)
	{
		int cross2ID = half_inside_pattern2D[id];
		pCross cross2D = pattern2D.lock()->crossList[cross2ID];

		std::unordered_map<int, bool> vertex_visited;
		int size = cross2D->verIDs.size();
        vector<wpCross> edge_cross3D;
		vector<int> edge_cross3D_edgeIDs;
		edge_cross3D.resize(size, pCross());
		edge_cross3D_edgeIDs.resize(size, -1);
		for(size_t jd = 0; jd < cross2D->verIDs.size(); jd++)
		{
			int ver2ID = cross2D->verIDs[jd];
			int ver3ID = map_vertex2D_3D[ver2ID];
			if(ver3ID == -1) continue;
			if(vertex_visited[jd] == true) continue;

			//iterate anti-clockwise
			int kd = jd;
			while(ver3ID != -1)
			{
				kd = (kd - 1 + size) % size;
				ver3ID = map_vertex2D_3D[cross2D->verIDs[kd]];
			}

			//insert into base2DMesh
			pPolygon polygon = make_shared<_Polygon<Scalar>>();

			//insert into Crossmesh
			pCross cross = make_shared<Cross>(getVarList());
			cross->atBoundary = false;
			cross->crossID = crossMesh->crossList.size();

			//iterate clockwise
			//(1) present kd has conner or not?
			int startID, endID;
			int v3ID = patter2D_cut_conner3D[cross2ID][kd];
			int v2ID = patter2D_cut_conner2D[cross2ID][kd];
			if(v3ID != -1)
			{
				cross->verIDs.push_back(v3ID);
				cross->vers.push_back(_Vertex(crossMesh->vertexList[v3ID]));

				polygon->verIDs.push_back(v2ID);
				polygon->vers.push_back(_Vertex(baseMesh2D->vertexList[v2ID]));

				startID = kd;
			}
			else {
				startID = (kd + 1) % size;
			}

			//(2) include the vertex already in the texture
			kd = (kd + 1) % size;
			ver3ID = map_vertex2D_3D[cross2D->verIDs[kd]];
			while(ver3ID != -1)
			{
				vertex_visited[kd] = true;
				cross->verIDs.push_back(ver3ID);
				cross->vers.push_back(_Vertex(crossMesh->vertexList[ver3ID]));

				polygon->verIDs.push_back(-1);
				polygon->vers.push_back(_Vertex(cross2D->vers[kd].pos));

				kd = (kd + 1) % size;
				ver3ID = map_vertex2D_3D[cross2D->verIDs[kd]];
			}

			//(3) kd - 1 has a conner or not
			kd = (kd - 1 + size) % size;
			v3ID = patter2D_cut_conner3D[cross2ID][kd];
			v2ID = patter2D_cut_conner2D[cross2ID][kd];
			if(v3ID != -1)
			{
				cross->verIDs.push_back(v3ID);
				cross->vers.push_back(_Vertex(crossMesh->vertexList[v3ID]));

				polygon->verIDs.push_back(v2ID);
				polygon->vers.push_back(_Vertex(baseMesh2D->vertexList[v2ID]));

				endID = kd;
			}
			else {
				endID = (kd - 1 + size) % size;
			}

			cross->ComputeCenter();
			cross->ComputeNormal();
			float area = cross->ComputeArea();
			if(cross->vers.size() >= 3 && area >= minimumCrossArea)
			{
				cross->neighbors.resize(cross->vers.size(), pCross());
				crossMesh->crossList.push_back(cross);
				baseMesh2D->polyList.push_back(polygon);
				kd = startID;
				int it = 0;
				do
				{
					edge_cross3D[kd] = cross;
					edge_cross3D_edgeIDs[kd] = it ++;
					kd = (kd + 1) % size;
				}while(kd != (endID + 1) % size);
			}
		}

		pattern2D_edge_cross3D[cross2ID] = edge_cross3D;
		pattern2D_edge_cross3D_edgeIDs[cross2ID] = edge_cross3D_edgeIDs;
	}

	for(size_t id = 0; id < half_inside_pattern2D.size(); id++)
	{
		int cross2ID = half_inside_pattern2D[id];
		pCross cross2D = pattern2D.lock()->crossList[cross2ID];
		for(size_t jd = 0; jd < cross2D->neighbors.size(); jd++)
		{
			pCross ncross2D = cross2D->neighbors[jd].lock();
			pCross cross3D = pattern2D_edge_cross3D[cross2ID][jd].lock();
			if(!ncross2D || !cross3D) continue;

			int ncross2ID = ncross2D->crossID;
			int edgeID = ncross2D->GetNeighborEdgeID(cross2ID);

			int ncross3ID = map_cross2D_3D[ncross2ID];

			//neighbor is a inside polygon
			if(ncross3ID != -1)
			{
				pCross ncross3D = crossMesh->crossList[ncross3ID];
				ncross3D->neighbors[edgeID] = cross3D;
				cross3D->neighbors[pattern2D_edge_cross3D_edgeIDs[cross2ID][jd]] = ncross3D;
			}


			//neighbor is a new created polygon
			auto find_it = pattern2D_edge_cross3D.find(ncross2ID);
			if(ncross3ID == -1 && find_it != pattern2D_edge_cross3D.end())
			{
				pCross ncross3D = find_it->second[edgeID].lock();
				if(ncross3D != nullptr){
					ncross3D->neighbors[pattern2D_edge_cross3D_edgeIDs[ncross2ID][edgeID]] = cross3D;
					cross3D->neighbors[pattern2D_edge_cross3D_edgeIDs[cross2ID][jd]] = ncross3D;
				}
			}

		}
	}
}

template <typename Scalar>
Matrix<Scalar, 2, 1> BaseMeshCreator<Scalar>::GetTextureCoord(Vector2 point, Scalar viewSize, double *interactMat)
{
    Vector2 texCoord;

	texCoord.x() = (point.x() + 0.5f*viewSize) / viewSize;
	texCoord.y() = (point.y() + 0.5f*viewSize) / viewSize;


	Matrix<Scalar, 4, 4> mat;
	mat << interactMat[0], interactMat[4], interactMat[8], interactMat[12],
	       interactMat[1], interactMat[5], interactMat[9], interactMat[13],
	       interactMat[2], interactMat[6], interactMat[10], interactMat[14],
	       interactMat[3], interactMat[7], interactMat[11], interactMat[15];

    texCoord = (mat * Matrix<Scalar, 4, 1>(texCoord.x(), texCoord.y(), 0, 1)).head(2);

    return texCoord;
}

//able to handle polygonal mesh
template <typename Scalar>
bool BaseMeshCreator<Scalar>::mapTexPointBackToSurface(Vector2 ptTexCoord, Vector3 &ptSurfCoord)
{
	if(ptTexCoord.x() < -1.5 || ptTexCoord.x() > 1.5) return false;
	if(ptTexCoord.y() < -1.5 || ptTexCoord.y() > 1.5) return false;
	if(polyMesh.lock() == nullptr) return false;

    pPolygon poly = polyMesh.lock()->findTexPoint(Vector2(ptTexCoord.x(), ptTexCoord.y()));

    if(poly != nullptr){
        vector<Scalar> barycentric = poly->computeBaryCentric(ptTexCoord);
        if(barycentric.size() == poly->size())
        {
            ptSurfCoord = Vector3(0, 0, 0);
            for(int id = 0; id < poly->size(); id++){
                ptSurfCoord += poly->pos(id) * barycentric[id];
            }
            return true;
        }
    }

    return false;
}

template<typename Scalar>
bool BaseMeshCreator<Scalar>::ComputeBoundaryVertex(double *inverTextureMat, Vector3 sta2D, Vector3 end2D, Vector3 &pos2D, Vector3 &pos3D)
{
	Vector3 init_pos3D;
	{
		Vector3 texCoord = GetTextureCoord(sta2D, viewSize);
		MultiplyPoint(texCoord, inverTextureMat, texCoord);
		ComputeSurfaceCoord(polyMesh.lock(), texCoord, init_pos3D);
	}

	pos3D = init_pos3D;
	pos2D = sta2D;
	while((sta2D - end2D).norm() > 1e-4)
	{
		Vector3 mid = (sta2D + end2D) * 0.5f;
		Vector3 texCoord = GetTextureCoord(mid, viewSize);
		MultiplyPoint(texCoord, inverTextureMat, texCoord);
		Vector3 ver3D;
		bool isSuccess = ComputeSurfaceCoord(polyMesh.lock(), texCoord, ver3D);
		if(isSuccess)
		{
			pos3D = ver3D;
			pos2D = mid;
			sta2D = mid;
		}
		else{
			end2D = mid;
		}
	}
	if((init_pos3D - pos3D).norm() < 1e-5){
		return false;
	}
	else{
		return true;
	}
}

template <typename Scalar>
void BaseMeshCreator<Scalar>::ComputePracticalBoundary(pCrossMesh &crossMesh)
{
    vector<wpCross> boundaryCross;
    crossMesh->UpdateCrossVertexIndex();
    vector<vector<wpCross>> &vertexCrossList = crossMesh->vertexCrossList;

    for(size_t id = 0; id < crossMesh->crossList.size(); id++)
    {
        pCross cross = crossMesh->crossList[id];
        cross->atBoundary = false;
    }

    auto setVertexRingBoundary = [&](int verID){
        if(verID >= 0 && verID < vertexCrossList.size())
        {
            for(wpCross cross: vertexCrossList[verID]){
                if(!cross.lock()->atBoundary){
                    boundaryCross.push_back(cross);
                    cross.lock()->atBoundary = true;
                }
            }
        }
        return;
    };

    for(size_t id = 0; id < crossMesh->crossList.size(); id++)
    {
        pCross cross = crossMesh->crossList[id];
        for(size_t jd = 0; jd < cross->neighbors.size(); jd ++)
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

    float minBoundaryEdge = getVarList()->template get<float>("minBoundaryEdge");
    for(size_t id = 0; id < boundaryCross.size(); id++)
    {
        pCross cross = boundaryCross[id].lock();
        size_t size = cross->vers.size();
        for(int jd = 0; jd < size; jd++)
        {
            //the neighbor must exists
            if(!cross->neighbors[jd].lock()) continue;

            float edgeLen = len(cross->vers[(jd + 1) % size].pos - cross->vers[jd].pos);
            //if the length of the edge is too small, then include one more part
            if(minBoundaryEdge > edgeLen)
            {
                wpCross ncross = cross->neighbors[jd];
                int shared_id = cross->GetSharedCross(ncross);

                if(shared_id >= 0 && shared_id < crossMesh->crossList.size()){
                    pCross shared_cross = crossMesh->crossList[shared_id];
                    shared_cross->atBoundary = true;
                }
            }
        }
    }

    for(size_t id = 0; id < crossMesh->crossList.size(); id++) {
        pCross cross = crossMesh->crossList[id];
        if (cross == nullptr || cross->atBoundary) continue;
        bool allBoundary = true;
        for (size_t jd = 0; jd < cross->neighbors.size(); jd++) {
            wpCross ncross = cross->neighbors[jd];
            if (ncross.lock() != nullptr && ncross.lock()->atBoundary == false) {
                allBoundary = false;
                break;
            }
        }
        cross->atBoundary = allBoundary;
    }
}