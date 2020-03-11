///////////////////////////////////////////////////////////////
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
void BaseMeshCreator<Scalar>::computeBaseCrossMesh(Matrix4 interactMat,
                                                   pPolyMesh &baseMesh2D,
                                                   pCrossMesh &crossMesh)
{
	tbb::tick_count sta = tbb::tick_count::now();

	crossMesh = make_shared<CrossMesh<double>>(getVarList());
	baseMesh2D = make_shared<PolyMesh<double>>(getVarList());

	Matrix4 textureMat = computeTextureMat(polyMesh.lock(), interactMat);
    computeInternalCross(textureMat, baseMesh2D, crossMesh);

	if(getVarList()->template get<bool>("smooth_bdry"))
	{
        computeBoundaryCross(textureMat, baseMesh2D, crossMesh);
		//remove dangling
		//RemoveDanglingCross(crossMesh);
    }
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
    // build crossMesh connectivity
    crossMesh->update();
    baseMesh2D->update();
	std::cout << "Remesh Para:\t" << (tbb::tick_count::now() - sta).seconds() << std::endl;
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
void BaseMeshCreator<Scalar>::computeInternalCross(Matrix4 textureMat,
                                                   pPolyMesh &baseMesh2D,
                                                   pCrossMesh &crossMesh)
{
    // at this stage, baseMes2D and crossMesh are empty.
    if(pattern2D.lock() == nullptr) return;

    pattern2D_vertices_on_polyMesh.clear();
    pattern2D_vertices_on_polyMesh.resize(pattern2D.lock()->vertexList.size(), nullptr);

    tbb::concurrent_vector<pVertex> vertexLists;
    // map 2D pattern vertices on 3D input surface
    tbb::parallel_for(tbb::blocked_range<size_t>(0, pattern2D.lock()->vertexList.size()),[&](const tbb::blocked_range<size_t>& r)
    {
        for (size_t id = r.begin(); id != r.end(); ++id)
        //for (size_t id = 0; id != pattern2D.lock()->vertexList.size(); ++id)
        {
            if (pattern2D.lock()->vertexList[id] == nullptr) continue;

            // 1) get the interactive position of each vertex.
            Vector2 ver_2DCoord = pattern2D.lock()->vertexList[id]->pos.head(2);
            Vector2 tex_2DCoord = getTextureCoord(ver_2DCoord, textureMat);

            // 2) compute the 3D coordinates of the 2D texture vertices
            // by inversing the parametrization mapping
            Vector3 ver_3DCoord;
            if (mapTexPointBackToSurface(tex_2DCoord, ver_3DCoord)) {
                // create a new pVertex and add it to crossMesh
                pVertex ver3D = make_shared<VPoint<Scalar>>(ver_3DCoord); // coordinate
                pattern2D_vertices_on_polyMesh[id] = ver3D;
                vertexLists.push_back(ver3D);
            }
        }
    });

    //construct crossMesh's vertexList
    for(pVertex ver3D: vertexLists)
    {
        ver3D->verID = crossMesh->vertexList.size();
        crossMesh->vertexList.push_back(ver3D);
    }

    tbb::concurrent_vector<pCross> internal_cross;
    tbb::concurrent_vector<pPolygon> internal_pattern2D;

	// generating all internal crosses
    tbb::parallel_for(tbb::blocked_range<size_t>(0, pattern2D.lock()->size()),[&](const tbb::blocked_range<size_t>& r)
    {
        for (size_t id = r.begin(); id != r.end(); ++id)
        //for (size_t id = 0; id != pattern2D.lock()->size(); ++id)
        {
            if(pattern2D.lock()->cross(id) == nullptr) continue;
            pCross poly2D = pattern2D.lock()->cross(id);

            // create an empty 3D internal cross
            pCross cross3D = make_shared<Cross<Scalar>>(getVarList());
            cross3D->atBoundary = false;

            // from previous section,
            // we know the 3D corresponding coordinate of each poly2D's vertex.
            // if exist, add it to the cross3D
            // if not, stop
            int num_vertex_inside = 0;
            for (size_t jd = 0; jd < poly2D->vers.size(); jd++)
            {
                int ver_index2D = poly2D->vers[jd]->verID;
                pVertex vertex = pattern2D_vertices_on_polyMesh[ver_index2D];
                if(vertex != nullptr){
                    cross3D->vers.push_back(vertex);
                    num_vertex_inside++;
                }
            }

            // if all vertex of poly2D is interal.
            if(num_vertex_inside == poly2D->vers.size())
            {
                internal_cross.push_back(cross3D);
                internal_pattern2D.push_back(make_shared<_Polygon<Scalar>>(*poly2D));
            }
            else if(num_vertex_inside > 0)
            {
                boundary_pattern2D.push_back(poly2D);
            }
        }
    });

    //append all crosses into crossMesh
    for(pCross cross: internal_cross){
        crossMesh->push_back(cross);
    }

    //append all polygons into baseMesh
    for(pPolygon poly: internal_pattern2D){
        pPolygon rescale_poly = make_shared<_Polygon<Scalar>>(*poly);
        for(pVertex vertex : rescale_poly->vers){
            Vector2 textureCoord = getTextureCoord(vertex->pos.head(2), textureMat);
            vertex->pos.x() = textureCoord.x();
            vertex->pos.y() = textureCoord.y();
            vertex->pos.z() = 0;
        }
        baseMesh2D->polyList.push_back(rescale_poly);
    }
}


template <typename Scalar>
void BaseMeshCreator<Scalar>::computeBoundaryCross(Matrix4 textureMat,
                                                   pPolyMesh &baseMesh2D,
                                                   pCrossMesh &crossMesh)
{
    tbb::concurrent_vector<pPolygon> crossLists;
    tbb::concurrent_vector<pPolygon> poly2DLists;

    // For each boundary cross, we cut it by using the mesh boundary
    tbb::parallel_for(tbb::blocked_range<size_t>(0, boundary_pattern2D.size()),[&](const tbb::blocked_range<size_t>& r)
    {
        //for(size_t id = 0; id < boundary_pattern2D.size(); id++)
        for (size_t id = r.begin(); id != r.end(); ++id){
            vector<bool> lines_inside;
            vector<Line<Scalar>> lines3D;
            vector<Line<Scalar>> lines2D;
            pPolygon poly = boundary_pattern2D[id];
            int N = poly->size();
            for(size_t jd = 0; jd < N; jd++)
            {
                int staID = poly->vers[jd]->verID;
                int endID = poly->vers[(jd + 1) % N]->verID;

                Vector2 sta_pos2D = poly->vers[jd]->pos.head(2);
                sta_pos2D = getTextureCoord(sta_pos2D, textureMat);

                Vector2 end_pos2D = poly->vers[(jd + 1) % N]->pos.head(2);
                end_pos2D = getTextureCoord(end_pos2D, textureMat);

//            std::cout << sta_pos2D.x() << "," << sta_pos2D.y() << std::endl;
//            std::cout << end_pos2D.x() << "," << end_pos2D.y() << std::endl;

                pVertex sta_ver3D = pattern2D_vertices_on_polyMesh[staID];
                pVertex end_ver3D = pattern2D_vertices_on_polyMesh[endID];

                if(sta_ver3D != nullptr && end_ver3D != nullptr){
                    //both are inside
                    lines3D.push_back(Line<Scalar>(sta_ver3D->pos, end_ver3D->pos));
                    lines2D.push_back(Line<Scalar>(sta_pos2D, end_pos2D));
                    lines_inside.push_back(true);
                }
                else if(sta_ver3D == nullptr && end_ver3D == nullptr){
                    //both are outside
                    lines3D.push_back(Line<Scalar>());
                    lines2D.push_back(Line<Scalar>());
                    lines_inside.push_back(false);
                }
                else if(sta_ver3D != nullptr && end_ver3D == nullptr){

                    //sta3d insde, end_ver3D outside
                    //find intersection with the boundary
                    Line<Scalar> line(sta_pos2D, end_pos2D);
                    Vector2 dirct = -(end_pos2D - sta_pos2D) / 100;
                    //Vector2 dirct(0, 0);
                    Vector2 brdy2D; Vector3 brdy3D;

                    //has intersection with boundary
                    //and have correspond position on the surface
                    if(polyMesh.lock()->findTexIntersec(line, brdy2D)
                       && mapTexPointBackToSurface(brdy2D + dirct, brdy3D))
                    {
                        lines3D.push_back(Line<Scalar>(sta_ver3D->pos, brdy3D));
                        lines2D.push_back(Line<Scalar>(sta_pos2D, brdy2D));
                        lines_inside.push_back(true);

                        lines3D.push_back(Line<Scalar>());
                        lines2D.push_back(Line<Scalar>());
                        lines_inside.push_back(false);
                    }
                    else{
                        lines3D.push_back(Line<Scalar>());
                        lines2D.push_back(Line<Scalar>());
                        lines_inside.push_back(false);
                    }

//                std::cout << brdy2D.x() << "," << brdy2D.y() << std::endl;
                }
                else{
                    //sta3d outside, end_ver3D inside
                    //find intersection with the boundary
                    Line<Scalar> line(end_pos2D, sta_pos2D);
                    Vector2 dirct = -(sta_pos2D - end_pos2D) / 100;
                    //Vector2 dirct(0, 0);
                    Vector2 brdy2D; Vector3 brdy3D;

                    //has intersection with boundary
                    //and have correspond position on the surface
                    if(polyMesh.lock()->findTexIntersec(line, brdy2D)
                       && mapTexPointBackToSurface(brdy2D + dirct, brdy3D))
                    {
                        lines2D.push_back(Line<Scalar>());
                        lines3D.push_back(Line<Scalar>());
                        lines_inside.push_back(false);

                        lines2D.push_back(Line<Scalar>(brdy2D, end_pos2D));
                        lines3D.push_back(Line<Scalar>(brdy3D, end_ver3D->pos));
                        lines_inside.push_back(true);
                    }
                    else{
                        lines2D.push_back(Line<Scalar>());
                        lines3D.push_back(Line<Scalar>());
                        lines_inside.push_back(false);
                    }

//                std::cout << brdy2D.x() << "," << brdy2D.y() << std::endl;
                }

//            std::cout << "\n";
            }

            splitIntoConsecutivePolygons(lines3D, lines_inside, crossLists);
            splitIntoConsecutivePolygons(lines2D, lines_inside, poly2DLists);
        }
    });


    //append to crossMesh
    for(pPolygon poly: crossLists){
        pCross cross = make_shared<Cross<Scalar>>(*poly, getVarList());
        crossMesh->push_back(cross);
    }

    //append to baseMesh
    baseMesh2D->polyList.insert(baseMesh2D->polyList.end(), poly2DLists.begin(), poly2DLists.end());

}

template <typename Scalar>
void  BaseMeshCreator<Scalar>::splitIntoConsecutivePolygons(
        const vector<Line<Scalar>> &lines,
        const vector<bool>& insides,
        tbb::concurrent_vector<pPolygon> &polyList)
{
    //split the lines_inside into consecutive true list
    //because it is a loop, the begin and end should be consider separately
    vector<vector<Line<Scalar>>> polyList_lines;
    vector<Line<Scalar>> poly_lines;
    for(int id = 0; id < insides.size(); id++)
    {
        if(insides[id] == false && !poly_lines.empty()){
            polyList_lines.push_back(poly_lines);
            poly_lines.clear();
        }
        if(insides[id] == true){
            poly_lines.push_back(lines[id]);
        }
    }

    //the last chunk
    if(!poly_lines.empty()){
        polyList_lines.push_back(poly_lines);
    }


    if(insides.front() == true
       && insides.back() == true
       && polyList_lines.size() > 1){
        //merge head and tail
        polyList_lines.back().insert(
                polyList_lines.back().end(),
                polyList_lines.front().begin(),
                polyList_lines.front().end());
        polyList_lines.erase(polyList_lines.begin());
    }

    for(vector<Line<Scalar>> poly_lines: polyList_lines)
    {
        pPolygon poly = make_shared<_Polygon<Scalar>>();
        for(size_t id = 0; id < poly_lines.size(); id++){
            poly->push_back(poly_lines[id].point1);
            if(id == poly_lines.size() - 1){
                poly->push_back(poly_lines[id].point2);
            }
        }
        polyList.push_back(poly);
    }
}

template <typename Scalar>
Matrix<Scalar, 2, 1> BaseMeshCreator<Scalar>::getTextureCoord(Vector2 point, Matrix4 textureMat)
{
    Vector2 texCoord;

	texCoord.x() = (point.x() + 0.5f*viewSize) / viewSize;
	texCoord.y() = (point.y() + 0.5f*viewSize) / viewSize;

    texCoord = (textureMat * Matrix<Scalar, 4, 1>(texCoord.x(), texCoord.y(), 0, 1)).head(2);

    return texCoord;
}

template<typename Scalar>
Matrix<Scalar, 4, 4> BaseMeshCreator<Scalar>::computeTextureMat(const pPolyMesh &referenceSurface, Matrix4 interactMat)
{
    //the following want to tranform the points in the surface texture space to the pattern space

    // Compute 2D bounding box of the parameterized surface mesh
    Box<Scalar> texBBox = referenceSurface->texBBox();

    //1) centralize the surface texture
    Matrix4 trans1 = Eigen::Matrix4d::Identity();
    trans1(0, 3) = -0.5*(texBBox.minPt.x() + texBBox.maxPt.x());
    trans1(1, 3) = -0.5*(texBBox.minPt.y() + texBBox.maxPt.y());
    trans1(2, 3) = 0;

    //2) scale 1) into [-0.5, -0.5]x [0.5, 0.5]
    Scalar scale_factor = getVarList()->template get<float>("textureScaleFactor");
    Scalar footScale = scale_factor / max(texBBox.maxPt.x() - texBBox.minPt.x(), texBBox.maxPt.y() - texBBox.minPt.y());
    Matrix4 scale = Eigen::Matrix4d::Identity();
    scale(0, 0) = footScale; scale(1, 1) = footScale;

    //3) tranform 2) by inveInteractMat
    Matrix4 inveInteractMat = interactMat.inverse();
    //Compatible issue with the old data
    //Reason: Since the scale of 2D pattern space is [-1, 1] while the scale of 2D texture space is [0, 1]
    inveInteractMat(0, 3) /= 2;
    inveInteractMat(1, 3) /= 2;
    inveInteractMat(2, 3) /= 2;

    //4) move the 3)'s center into [0.5, 0.5], so that it is within [0, 0] x[1, 1]
    Matrix4 trans2 = Eigen::Matrix4d::Identity();
    trans2(0, 3) = 0.5;
    trans2(1, 3) = 0.5;
    trans2(2, 3) = 0;

    Matrix4 textureMat = trans2 * inveInteractMat * scale * trans1;
    //5) inverse it because we want a transform form pattern space to surface texture space
    return textureMat.inverse();
}


// able to handle polygonal mesh
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
bool BaseMeshCreator<Scalar>::ComputeBoundaryVertex(double *inverTextureMat, Vector3 sta_pos2D, Vector3 end_pos2D, Vector3 &pos2D, Vector3 &pos3D)
{
	Vector3 init_pos3D;
	{
		Vector3 texCoord = GetTextureCoord(sta_pos2D, viewSize);
		MultiplyPoint(texCoord, inverTextureMat, texCoord);
		ComputeSurfaceCoord(polyMesh.lock(), texCoord, init_pos3D);
	}

	pos3D = init_pos3D;
	pos2D = sta_pos2D;
	while((sta_pos2D - end_pos2D).norm() > 1e-4)
	{
		Vector3 mid = (sta_pos2D + end_pos2D) * 0.5f;
		Vector3 texCoord = GetTextureCoord(mid, viewSize);
		MultiplyPoint(texCoord, inverTextureMat, texCoord);
		Vector3 ver3D;
		bool isSuccess = ComputeSurfaceCoord(polyMesh.lock(), texCoord, ver3D);
		if(isSuccess)
		{
			pos3D = ver3D;
			pos2D = mid;
			sta_pos2D = mid;
		}
		else{
			end_pos2D = mid;
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
            // the neighbor must exists
            if(!cross->neighbors[jd].lock()) continue;

            float edgeLen = len(cross->vers[(jd + 1) % size].pos - cross->vers[jd].pos);
            // if the length of the edge is too small, then include one more part
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

