//
// Created by ziqwang on 14.01.19.
//


/*************************************************
*
*                  Basic Operation
*
*************************************************/

template<typename Scalar>
ContactGraph<Scalar>::ContactGraph(shared_ptr<InputVarList> varList):TopoObject(varList)
{
}

template<typename Scalar>
ContactGraph<Scalar>::~ContactGraph()
{
    nodes.clear();
    edges.clear();
}

template<typename Scalar>
bool ContactGraph<Scalar>::constructFromPolyMeshes(vector<pPolyMesh> &meshes,
                                           vector<bool> &atBoundary,
                                           double eps)
{

    //0) scale the meshes into united box
    double maxD = 1;
    for (size_t id = 0; id < meshes.size(); id++) {
        pPolyMesh poly = meshes[id];
        if (poly == nullptr) return false;
        for (pPolygon face : poly->polyList) {
            //1.1) construct plane
            plane_contact plane;
            Vector3 nrm = face->normal();
            Vector3 center = face->vers[0]->pos;
            plane.nrm = nrm;

            plane.D = nrm.dot(center);
            maxD = (std::max)(maxD, (double)std::abs(plane.D));
        }
    }

    //1) create contact planes
    vector<plane_contact> planes;
    std::set<plane_contact, plane_contact_compare> setPlanes;

    int groupID = 0;
    for (size_t id = 0; id < meshes.size(); id++)
    {
        pPolyMesh poly = meshes[id];
        if (poly == nullptr)
            return false;
        for (pPolygon face : poly->polyList)
        {
            if(face->vers.size() < 3) continue;

            //1.1) construct plane
            plane_contact plane;
            Vector3 nrm = face->normal();
            Vector3 center = face->vers[0]->pos;
            plane.nrm = Vector3(nrm[0], nrm[1], nrm[2]);
            
            plane.D = nrm.dot(center) / maxD;
            plane.partID = id;
            plane.polygon = face;
            plane.eps = eps;

            //1.2) find groupID
            typename std::set<plane_contact, plane_contact_compare>::iterator find_it = setPlanes.end();
            for(int reverse = -1; reverse <= 1; reverse += 2){
                plane_contact tmp_plane = plane;
                tmp_plane.nrm *= reverse;
                tmp_plane.D *= reverse;
                find_it = setPlanes.find(tmp_plane);
                if(find_it != setPlanes.end()){
                    plane.groupID = (*find_it).groupID;
                    break;
                }
            }

            if(find_it == setPlanes.end()){
                plane.groupID = groupID ++;
                setPlanes.insert(plane);
            }
            
            planes.push_back(plane);
        }
    }

    std::sort(planes.begin(), planes.end(), [&](const plane_contact &A, const plane_contact &B){
        return A.groupID < B.groupID;
    });

     //2) add nodes
     for(size_t id = 0; id < meshes.size(); id++){
         Vector3 centroid = meshes[id]->centroid();
         Scalar volume = meshes[id]->volume();
         pContactGraphNode node = make_shared<ContactGraphNode<Scalar>>(atBoundary[id], centroid, centroid, volume);
         addNode(node);
     }

     //3) find all pairs of contact polygon
     int sta = 0, end = 0;
     vector<pairIJ> planeIJ;
     while(sta < planes.size())
     {
          for(end = sta + 1; end < planes.size(); end++)
          {
              if(planes[sta].groupID != planes[end].groupID)
              {
                  break;
              }
          }
          if (end - sta > 1)
          {
              for (int id = sta; id < end; id++)
              {
                  for(int jd = id + 1; jd < end; jd++)
                  {
                      int partI = planes[id].partID;
                      int partJ = planes[jd].partID;

                      Vector3 nrmI = planes[id].nrm.normalized();
                      Vector3 nrmJ = planes[jd].nrm.normalized();

                      if(partI != partJ
                      && std::abs(nrmI.dot(nrmJ) + 1) < FLOAT_ERROR_LARGE
                      && (!atBoundary[partI] || !atBoundary[partJ]))
                          planeIJ.push_back(pairIJ(id, jd));
                  }
              }
          }
          sta = end;
      }


     //4) parallel compute contacts
     vector<pContactGraphEdge> planeIJEdges;
     planeIJEdges.resize(planeIJ.size());

     tbb::parallel_for(tbb::blocked_range<size_t>(0, planeIJ.size()), [&](const tbb::blocked_range<size_t>& r)
	 {
         for (size_t id = r.begin(); id != r.end(); ++id)
         //for (size_t id = 0; id != planeIJ.size(); ++id)
	 	{

             int planeI = planeIJ[id].first;
             int planeJ = planeIJ[id].second;

             vector<Vector3> polyI = planes[planeI].polygon.lock()->getVertices();
             vector<Vector3> polyJ = planes[planeJ].polygon.lock()->getVertices();
             vector<vector<Vector3>> contactPtLists;

             PolyPolyBoolean<Scalar> ppIntersec(getVarList());

             ppIntersec.computePolygonsIntersection(polyI, polyJ, contactPtLists);
             if (contactPtLists.empty())
                 continue;

             vector<pPolygon> contactPolys;
             for(vector<Vector3> contactPtList: contactPtLists)
             {
                 if(contactPtList.size() >= 3){
                     pPolygon contactPoly = make_shared<_Polygon<Scalar>>();
                     contactPoly->setVertices(contactPtList);
                     contactPolys.push_back(contactPoly);
                 }
             }

             if(!contactPolys.empty()){
                 int partI = planes[planeI].partID;
                 int partJ = planes[planeJ].partID;

                 shared_ptr<ContactGraphEdge<Scalar>> edge = make_shared<ContactGraphEdge<Scalar>>(contactPolys, planes[planeI].nrm);
                 planeIJEdges[id] = edge;
             }
         }
     });

     //5) add contact edges
     for(size_t id = 0; id < planeIJ.size(); id++)
     {
         pContactGraphEdge edge = planeIJEdges[id];
         if(edge != nullptr){
             int planeI = planeIJ[id].first;
             int planeJ = planeIJ[id].second;
             int partI = planes[planeI].partID;
             int partJ = planes[planeJ].partID;
             addContact(nodes[partI], nodes[partJ], edge);
         }
     }

    return true;
}

/*************************************************
*
*                  Graph Operation
*
*************************************************/
template<typename Scalar>
void ContactGraph<Scalar>::addNode(pContactGraphNode _node)
{
    _node->staticID = nodes.size();

    nodes.push_back(_node);

    return;
}

template<typename Scalar>
void ContactGraph<Scalar>::addContact(pContactGraphNode _nodeA, pContactGraphNode _nodeB, pContactGraphEdge _edge)
{

    if (_nodeA->isBoundary && _nodeB->isBoundary)
        return;

    _edge->partIDA = _nodeA->staticID;
    _edge->partIDB = _nodeB->staticID;

    edges.push_back(_edge);

    pair<wpContactGraphNode, wpContactGraphEdge> contactNeighbor;

    contactNeighbor.first = _nodeB;
    contactNeighbor.second = _edge;
    _nodeA->neighbors.push_back(contactNeighbor);

    contactNeighbor.first = _nodeA;
    contactNeighbor.second = _edge;
    _nodeB->neighbors.push_back(contactNeighbor);

    return;
}

template<typename Scalar>
void ContactGraph<Scalar>::finalize()
{
    int dynamicID = 0;
    dynamic_nodes.clear();
    for (pContactGraphNode node : nodes)
    {
        if (!node->isBoundary)
        {
            node->dynamicID = dynamicID++;
            dynamic_nodes.push_back(node);
        }
        else
        {
            node->dynamicID = -1;
        }
    }
}

template<typename Scalar>
void ContactGraph<Scalar>::getContactMesh(pPolyMesh &mesh)
{
    mesh.reset();
    mesh = make_shared<PolyMesh<Scalar>>(getVarList());
    for(pContactGraphEdge edge: edges)
    {
        for(pPolygon poly: edge->polygons)
        {
            mesh->polyList.push_back(poly);
        }
    }

    if(mesh)
        mesh->removeDuplicatedVertices();

//    mesh->ScaleMesh(1.0 / normalized_scale);
//    mesh->TranslateMesh(-normalized_trans);
}