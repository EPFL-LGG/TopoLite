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
                                         pCrossMesh _pattern2D,
                                         shared_ptr<InputVarList> varList)
: TopoObject(varList) {
    polyMesh = _polyMesh;
    pattern2D = _pattern2D;
}

template <typename Scalar>
BaseMeshCreator<Scalar>::BaseMeshCreator(shared_ptr<InputVarList> varList)
: TopoObject(varList) {
    polyMesh = wpPolyMeshAABB();
    pattern2D = wpCrossMesh();
}

template <typename Scalar>
BaseMeshCreator<Scalar>::~BaseMeshCreator()
{

}

//**************************************************************************************//
//                               Compute Lifted 3D Mesh 
//**************************************************************************************//

template <typename Scalar>
void BaseMeshCreator<Scalar>::
computeBaseCrossMesh(Matrix4 textureMat,
                                                   pPolyMesh &baseMesh2D,
                                                   pCrossMesh &crossMesh,
                                                   bool previewMode)
{
	tbb::tick_count sta = tbb::tick_count::now();
	//bool smooth boundary
	bool use_smooth_boundary = getVarList()->template get<bool>("smooth_bdry");

	// allocate mesh memory
	crossMesh = make_shared<CrossMesh<double>>(getVarList());
	baseMesh2D = make_shared<PolyMesh<double>>(getVarList());

//	// compute texture mapping matrix
//	Matrix4 textureMat = computeTextureMat(polyMesh.lock(), interactMat);

	// compute the inner and boundary cross
	computeInternalCross(textureMat, baseMesh2D, crossMesh);
	if(use_smooth_boundary) computeBoundaryCross(textureMat, baseMesh2D, crossMesh);

    // update the vertex ID and build up connectivity of meshes
    crossMesh->update();
    baseMesh2D->update();

    //remove boundary_pattern2D and pattern2D_vertices_on_polyMesh
    boundary_pattern2D.clear();
    pattern2D_vertices_on_polyMesh.clear();

    //polish the boundary of the crossMesh
    if(!previewMode){
        removeSmallCrosses(crossMesh);
        removeDanglingCross(crossMesh);
        crossMesh->erase_nullptr();
        recomputeBoundary(crossMesh);
    }

	std::cout << "Remesh Para:\t" << (tbb::tick_count::now() - sta).seconds() << std::endl;
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
    for(pPolygon poly: internal_pattern2D)
    {
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
                    Vector2 brdy2D; Vector3 brdy3D;

                    //has intersection with boundary
                    //and have correspond position on the surface
                    if(polyMesh.lock()->findBoundaryIntersec(line, brdy2D, brdy3D))
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
                }
                else{
                    //sta3d outside, end_ver3D inside
                    //find intersection with the boundary
                    Line<Scalar> line(end_pos2D, sta_pos2D);
                    Vector2 brdy2D; Vector3 brdy3D;

                    //has intersection with boundary
                    //and have correspond position on the surface
                    if(polyMesh.lock()->findBoundaryIntersec(line, brdy2D, brdy3D))
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
                }
            }

            splitIntoConsecutivePolygons(lines3D, lines_inside, crossLists);
            splitIntoConsecutivePolygons(lines2D, lines_inside, poly2DLists);
        }
    });


    //append to crossMesh
    for(pPolygon poly: crossLists){
        pCross cross = make_shared<Cross<Scalar>>(*poly, getVarList());
        cross->atBoundary = true;
        crossMesh->push_back(cross);
    }

    //append to baseMesh
    baseMesh2D->polyList.insert(baseMesh2D->polyList.end(), poly2DLists.begin(), poly2DLists.end());
}

template<typename Scalar>
void BaseMeshCreator<Scalar>::removeSmallCrosses(BaseMeshCreator::pCrossMesh crossMesh)
{
    Scalar minimum_ratio = getVarList()->template get<float>("minCrossArea");

    //compute the maximum cross Area
    Scalar maximum_crossArea = tbb::parallel_reduce(
            tbb::blocked_range<size_t>(0, crossMesh->size()),
            Scalar(),
            [&](const tbb::blocked_range<size_t>& r, Scalar maximum)->Scalar {
                for(size_t id = r.begin(); id != r.end(); ++id)
                {
                    Scalar area = crossMesh->cross(id)->area();
                    maximum = maximum > area ? maximum : area;
                }
                return maximum;
            },
            [&]( Scalar x, Scalar y )->Scalar {
                return x > y ? x: y;
            }
    );

    for(size_t id = 0; id < crossMesh->size(); id++)
    {
        Scalar area = crossMesh->cross(id)->area();
        if(area < maximum_crossArea * minimum_ratio){
            crossMesh->erase(id);
        }
    }
}

template <typename Scalar>
void BaseMeshCreator<Scalar>::removeDanglingCross(pCrossMesh crossMesh)
{
	if(crossMesh == nullptr) return;
    crossMesh->updateCrossID();

	//cluster all groups into connected components
    mapCrossInt cross_groupID;
    vector<vector<wpCross>> groupCrosses;
    for(size_t id = 0; id < crossMesh->size(); id++){
        pCross cross = crossMesh->cross(id);
        if(cross != nullptr && cross_groupID.find(cross.get()) == cross_groupID.end())
        {
            //add cross into groupCross with ID (groupID)
            int groupID = groupCrosses.size();
            groupCrosses.push_back(vector<wpCross>());
            cross_groupID[cross.get()] = groupID;
            groupCrosses[groupID].push_back(cross);

            //BFS queue
            std::queue<wpCross> crossQueue;
            crossQueue.push(cross);
            while(!crossQueue.empty())
            {
                wpCross u = crossQueue.front(); crossQueue.pop();
                for(wpCross neighbor: u.lock()->neighbors)
                {
                    //if neighbor exists and have not been assigned a ID number
                    //add it to the queue
                    if(neighbor.lock() != nullptr && cross_groupID.find(neighbor.lock().get()) == cross_groupID.end()){
                        crossQueue.push(neighbor.lock());
                        cross_groupID[neighbor.lock().get()] = groupID;
                        groupCrosses[groupID].push_back(neighbor.lock());
                    }
                }
            }
        }
    }

    //select the biggest group and remove others
    //Todo: could have other dangling definition, like number of parts in a cluster is less than 5?
    std::sort(groupCrosses.begin(), groupCrosses.end(), [=](const vector<wpCross> &A, const vector<wpCross> &B){
        return A.size() > B.size();
    });

    for(int id = 1; id < groupCrosses.size(); id++){
        for(wpCross cross: groupCrosses[id]){
            crossMesh->erase(cross.lock()->crossID);
        }
    }
}

template <typename Scalar>
void BaseMeshCreator<Scalar>::recomputeBoundary(pCrossMesh crossMesh)
{
    int iterate_times = std::max(1, getVarList()->template get<int>("layerOfBoundary"));

    //clear all boundary mark
    tbb::parallel_for(tbb::blocked_range<size_t>(0, crossMesh->size()), [&](tbb::blocked_range<size_t> &r) {
        for (size_t id = r.begin(); id != r.end(); id++) {
            crossMesh->cross(id)->atBoundary = false;
        }
    });

    //create iterate_times' layer of boundary crosses
    for(size_t times = 0; times < iterate_times; times++)
    {
        tbb::concurrent_vector<wpCross> boundary_crosses;
        tbb::parallel_for(tbb::blocked_range<size_t>(0, crossMesh->size()), [&](tbb::blocked_range<size_t> &r){
            for(size_t id = r.begin(); id != r.end(); id++)
            {
                pCross cross = crossMesh->cross(id);
                for(wpCross neighbor : cross->neighbors)
                {
                    if(neighbor.lock() == nullptr || neighbor.lock()->atBoundary){
                        boundary_crosses.push_back(cross);
                        break;
                    }
                }
            }
        });

        for(wpCross cross: boundary_crosses){
            cross.lock()->atBoundary = true;
        }
    }
}

//**************************************************************************************//
//                               Utility Function
//**************************************************************************************//

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
    Vector2 texCoord = (textureMat * Matrix<Scalar, 4, 1>(point.x(), point.y(), 0, 1)).head(2);

    return texCoord;
}

//now able to handle polygonal mesh
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

template class BaseMeshCreator<double>;
