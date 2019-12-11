///////////////////////////////////////////////////////////////
//
// Struc.cpp
//
//   Topological Interlocking Structure
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 16/Oct/2018
//
//
///////////////////////////////////////////////////////////////


#include "Mesh/Cross.h"
#include "Utility/math3D.h"
#include "Utility/HelpDefine.h"
#include "Utility/HelpStruct.h"
#include "Utility/HelpFunc.h"
#include "Mesh/Polygon.h"
#include "Mesh/PolyMesh.h"
#include "Structure/Part.h"
#include "Structure/PartGroup.h"
#include "Structure/Struc.h"
#include "IO/InputVar.h"
#include "Utility/PolyPolyIntersec.h"

#include <unordered_map>
#include <queue>

using std::unordered_map;
using std::queue;

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

Struc::Struc(shared_ptr<InputVarList> var) : TopoObject(var)
{
    numEdgeEdgeContacts = numFaceFaceContacts = 0;
}

Struc::~Struc()
{
	ClearStruc();
}

void Struc::ClearStruc()
{
	partList.clear();

	contactList.clear();

    numEdgeEdgeContacts = numFaceFaceContacts = 0;
}

Struc::Struc(const Struc& _struc) : TopoObject(_struc)
{
    ClearStruc();

    //part Geometry
    for(int id = 0; id < _struc.partList.size(); id++)
    {
        shared_ptr<Part> part = make_shared<Part>(*_struc.partList[id]);
        partList.push_back(part);
    }

    contactList = _struc.contactList;
    parts = _struc.parts;
    innerContactList = _struc.innerContactList;
    minContactArea = _struc.minContactArea;
    avgContactArea = _struc.avgContactArea;
    avgPartSize = _struc.avgPartSize;
    groundPlaneY = _struc.groundPlaneY;
    numFaceFaceContacts = _struc.numFaceFaceContacts;
    numEdgeEdgeContacts = _struc.numEdgeEdgeContacts;

    return;
}

//**************************************************************************************//
//                              Identify Parts (Disk Model)
//**************************************************************************************//

void Struc::ComputeBoundaryParts()
{

	vector<int> BoundPartIDs;

	/*
 	* 1. Basic boundary
	* if part has null neighnor then it should be boundary
 	*/
	{
		for (int i = 0; i < partList.size(); i++)
		{
			shared_ptr<Part> part = partList[i];
			if(part == nullptr || part->polyMesh == nullptr) continue;
			part->atBoundary = part->cross.lock()->atBoundary;
			if(part->atBoundary){
                BoundPartIDs.push_back(part->partID);
			}
		}
	}


	/*
	 * 2. However, sometime the boundary is not connected
	 * First need to make it connected
	 */
	{
		int groupID = 0;
		unordered_map<int, int> partSegmID;
		unordered_map<int ,bool> visited;
		queue<int> Q;

		/*
		 * 2.1 find all segments of boundary
		 * for quad cross mesh, the boundary could be broken
		 * give each boundary part a group ID number
		 */
		for(int partID : BoundPartIDs)
		{
			if(!visited[partID])
			{
				Q.push(partID);
				while(!Q.empty())
				{
					int pUI = Q.front();Q.pop();
					pPart pU = partList[pUI];
					partSegmID[pUI] = groupID;
					for(wpPart _pV : pU->initNeighbors)
					{
						pPart pV = _pV.lock();
						if(pV == NULL) continue;
						if(visited[pV->partID]) continue;
						if(!pV->atBoundary) continue;
						visited[pV->partID] = true;
						Q.push(pV->partID);
					}
				}
				groupID ++;
			}
		}


		/*
		 * 2.2 Find corner parts that can joint two segment together
		 * the criteria for these corner parts that have two neighbor in different segments
		 */
		if(groupID > 1)
		{
			for(pPart part : partList)
			{
				if(part->atBoundary) continue;
				int seg_count = 0;
				std::unordered_map<int, bool> seg_visited;
				for(wpPart _npart : part->initNeighbors)
				{
					pPart npart = _npart.lock();
					if(npart == NULL) continue;
					if(!npart->atBoundary) continue;
					int segID = partSegmID[npart->partID];
					if(seg_visited[segID]) continue;
					seg_visited[segID] = true;
					seg_count ++;
				}
				if(seg_count >= 2)
				{
					BoundPartIDs.push_back(part->partID);
				}
			}
		}

		for(int partID : BoundPartIDs) partList[partID]->atBoundary = true;


	}

	/*
	 * 3. Still, some part would be locked by the boundary part
	 * especially some corner parts cannot be disassembled
	 * we have to put them into the boundary
	 */
	{
		queue<pPart> checkList;
		/*
		 * 3.1 add boundary's one ring neighor into checklist
		 */
		for(int partID : BoundPartIDs)
		{
			for(wpPart _part : partList[partID]->initNeighbors)
			{
				pPart part = _part.lock();
				if(part == nullptr || part->atBoundary) continue;
				checkList.push(part);
			}
		}

		/*
		 * 3.2 continue check all parts in checklists
		 */
		while(!checkList.empty())
		{
			pPart pU = checkList.front(); checkList.pop();
			if(pU->atBoundary || pU->partID >= partList.size()) continue;
			if(CheckPartsDeadLock(pU->partID))
			{
				pU->atBoundary = true;
				BoundPartIDs.push_back(pU->partID);
				for(wpPart _pV : pU->initNeighbors)
				{
					pPart pV = _pV.lock();
					if(pV && pV->atBoundary == false)
						checkList.push(pV);
				}
			}
		}
	}

	int boundaryPartNum = BoundPartIDs.size();
	int assembly = 0;
	parts.clear();
	for(int id = 0; id < partList.size(); id++)
	{
		if(partList[id]->atBoundary){
			partList[id]->assemblyID = -1;
			boundarys.push_back(partList[id]);
		}
		else{
			partList[id]->assemblyID = assembly ++ ;
			parts.push_back(partList[id]);
		}
	}
}

bool Struc::CheckPartsDeadLock(int partID) {

	/*
	 * 1. To check whether part would be locked by the boundary
	 */
	pPart part = partList[partID];
	pCross cross = part->cross.lock();

	/*
	 * 2. get all contacts with boundary
	 */
	vector<Vector3f> normalsList;
	for (int id = 0; id < part->initNeighbors.size(); id++) {
		pPart npart = part->initNeighbors[id].lock();
		if (npart == NULL || !npart->atBoundary) continue;
		normalsList.push_back(cross->oriPoints[id]->normal * -1.0f);
	}

	/*
	 * 3. if no contacts return false
	 */
	if(normalsList.empty()) return false;

//
//	/*
//     * 4. build contact Matrix
//     */
//	int nNor = normalsList.size();
//	std::shared_ptr<ndarray<double, 2>> contactA =
//												std::make_shared<ndarray<double, 2>>(shape(nNor,3),
//			std::function<double(const shape_t<2> &)>(
//					[&](const shape_t<2> & p)
//					{
//						return normalsList[p[0]][p[1]];
//					}));
//	/*
//     * 5. initialize the optimization model
//     */
//
//	Model::t M = new Model("CheckPartAssemblity");
//	auto _M = finally([&]() { M->dispose(); });
//	auto x = M->variable(3, Domain::inRange(-1, 1));
//	auto t = M->variable(nNor, Domain::greaterThan(0.0));
//
//	/*
//     * 6. add constraint
//     */
//	M->constraint("Non-collision Constraints", Expr::sub(Expr::mul(contactA, x),t), Domain::greaterThan(0.0));
//	M -> objective("obj", ObjectiveSense::Maximize, Expr::sum(t));
//	M -> solve();
//
//	/*
//     * 7. solving the linear system and find the correct disassembling direction.
//     */
//	auto result_t = t->level();
//	double sum = 0;
//	for(int id = 0; id < nNor; id++) sum+= (*result_t)[id];
//
//	const double zero_epsilon = 1e-6;
//	if(sum > zero_epsilon)
//	{
//		return false;
//	}
//	else
//	{
//		return true;
//	}

    return false;
}

double Struc::ComputeLowestY()
{
    // Compute structure's lowest point
    float strucMinY = MAX_FLOAT;

    for (int i = 0; i < partList.size(); i++)
    {
        pPart part = partList[i];

        if(part == nullptr || part->polyMesh == nullptr)
            continue;
        
        float partMinY = part->polyMesh->bbox.minPt.y;

        if (partMinY < strucMinY)
        {
            strucMinY = partMinY;
        }
    }
    return strucMinY;
}

void Struc::ComputeGroundY(bool showGround)
{
	// Compute average part size
    ComputeAveragePartSize();

	// Get ground y-coordinate
	groundPlaneY = ComputeLowestY() + 0.1f * avgPartSize;
	getVarList()->set("ground_height", groundPlaneY);

	// Identify parts that touch the ground
	if (showGround)
	{
        ComputeTouchGroundParts();
	}
	else
	{
		ResetTouchGround();
	}
}

void Struc::ComputeTouchGroundParts()
{
	const float ydistThres = 0.08 * avgPartSize;

	for (int i = 0; i < partList.size(); i++)
	{
		pPart part = partList[i];

		float partMinY = part->polyMesh->bbox.minPt.y;
		float partMaxY = part->polyMesh->bbox.maxPt.y;

		if ((groundPlaneY - partMinY) > ydistThres &&
			(partMaxY - groundPlaneY) > ydistThres)
		{
			part->touchGround = true;
		}
		else
        {
            part->touchGround = false;
        }
	}
}

void Struc::ResetTouchGround()
{
	for (int i = 0; i < partList.size(); i++)
	{
		pPart part = partList[i];
		part->touchGround = false;
	}
}

void Struc::RemoveParts(vector<int> partIDs)
{
    if(!partIDs.empty()){
        for(int partID : partIDs)
        {
            if(partID >= partList.size() || partID < 0)
                continue;
            partList[partID]->isRemove = true;
        }

        //remove contact
        for(auto it = contactList.begin(); it != contactList.end();){
            shared_ptr<Contact> contact = *it;
            shared_ptr<Part> partI = partList[contact->partIDA];
            shared_ptr<Part> partJ = partList[contact->partIDB];
            if(partI->isRemove || partJ->isRemove) {
                it = contactList.erase(it);
            }
            else{
            	it ++;
            }
        }

        //remove contact
        for(auto it = innerContactList.begin(); it != innerContactList.end();){
            shared_ptr<Contact> contact = (*it).lock();
            if(contact == nullptr){
				it = innerContactList.erase(it);
				continue;
            }
            shared_ptr<Part> partI = partList[contact->partIDA];
            shared_ptr<Part> partJ = partList[contact->partIDB];
            if(partI->isRemove || partJ->isRemove) {
                it = innerContactList.erase(it);
            }
            else{
            	it++;
            }
        }

        ComputePartContactArea();
    }
}

double Struc::ComputeAveragePartSize()
{
	avgPartSize = 0;

	for (int i = 0; i < partList.size(); i++)
	{
		pPart part = partList[i];
		if(part && part->polyMesh)
		{
            float partSize = len(part->polyMesh->bbox.maxPt - part->polyMesh->bbox.minPt);
            avgPartSize += partSize;
		}
	}

	if(partList.size() != 0){
        avgPartSize /= (float)partList.size();
	}

	return avgPartSize;}

//**************************************************************************************//
//                              Compute Part Contacts
//**************************************************************************************//

void Struc::ComputePartFaceContactsBruteForce()
{
    numFaceFaceContacts = 0;
    std::vector<double> face_area_list;
    for (int i = 0; i < partList.size(); i++)
    {
        shared_ptr<Part> part = partList[i];
        shared_ptr<Cross> cross = partList[i]->cross.lock();
        for (int j = 0; j < part->initNeighbors.size(); j++) {
            shared_ptr<Part> neibor = part->initNeighbors[j].lock();

            if (neibor == NULL || neibor->partID <= part->partID) continue;
            if (part->isRemove || neibor->isRemove) continue;

            //if (part->atBoundary && neibor->atBoundary) continue;
            for (int currFaceID = 0; currFaceID < part->polyMesh->polyList.size(); currFaceID++)
            {
                for (int neiborFaceID = 0; neiborFaceID < neibor->polyMesh->polyList.size(); neiborFaceID++)
                {

                    shared_ptr<_Polygon> currFace = part->polyMesh->polyList[currFaceID];
                    shared_ptr<_Polygon> neiborFace = neibor->polyMesh->polyList[neiborFaceID];

                    if((currFace->normal DOT neiborFace->normal) >= -0.99){
                        continue;
                    }

                    if(std::fabs((currFace->center - neiborFace->center) DOT currFace->normal) >= 0.01){
                        continue;
                    }

                    double projMat[16];
                    double invsProjMat[16];

                    if(currFace == nullptr || neiborFace == nullptr) continue;

                    face_area_list.push_back(currFace->ComputeArea());
                    face_area_list.push_back(neiborFace->ComputeArea());

                    currFace->ComputeProjectMatrixTo2D(projMat, invsProjMat);
                    vector<Vector3f> currFace2D = currFace->ProjectPolygonTo2D(projMat);
                    vector<Vector3f> neiborFace2D = neiborFace->ProjectPolygonTo2D(projMat);

            		PolyPolyIntersec ppIntersec(getVarList());

                    vector<Vector3f> overlapPolyPts;
                    ppIntersec.ComputePolygonsIntersection(currFace2D, neiborFace2D, overlapPolyPts);
                    if( overlapPolyPts.size() == 0 ) continue;

                    vector<Vector3f> contactPoly;
                    ppIntersec.ProjectPolygonTo3D(overlapPolyPts, invsProjMat, contactPoly);

                    Vector3f contactNormal = currFace->normal;

                    bool boundaryContact;
                    if( part->atBoundary && neibor->atBoundary)
                        boundaryContact = true;
                    else
                        boundaryContact = false;

                    pContact contact;
                    if(cross == NULL)
                        contact = make_shared<Contact>(part->partID, neibor->partID, contactNormal, shared_ptr<OrientPoint>(), contactPoly, boundaryContact);
                    else
                        contact = make_shared<Contact>(part->partID, neibor->partID, contactNormal, cross->oriPoints[j], contactPoly, boundaryContact);

                    contactList.push_back(contact);
                    if(!part->atBoundary || !neibor->atBoundary) innerContactList.push_back(contact);

                    numFaceFaceContacts++;
                }
            }
        }
    }

    //average face size
    avgContactArea = 0;
    for(double face_area: face_area_list)
    {
        avgContactArea += face_area;
    }
    avgContactArea /= face_area_list.size();
    ComputePartContactArea();
}

void Struc::ComputePartFaceContacts()
{
	std::vector<double> face_area_list;
    numFaceFaceContacts = 0;
	for (int i = 0; i < partList.size(); i++)
	{
		shared_ptr<Part> part = partList[i];
		shared_ptr<Cross> cross = partList[i]->cross.lock();
		for (int j = 0; j < part->initNeighbors.size(); j++)
		{
			shared_ptr<Part> neibor = part->initNeighbors[j].lock();

			if (neibor == NULL || neibor->partID <= part->partID) continue;
			if (part->isRemove || neibor->isRemove) continue;

			//if (part->atBoundary && neibor->atBoundary) continue;

			int currFaceID = j;
			int neiborFaceID = neibor->cross.lock()->GetNeighborEdgeID(cross->crossID);
			if(neiborFaceID == -1) continue;
			pPolygon currFace = (part->partGeom)->faceList[currFaceID];
			pPolygon neiborFace = (neibor->partGeom)->faceList[neiborFaceID];
			double projMat[16];
			double invsProjMat[16];

			if(currFace == nullptr || neiborFace == nullptr) continue;

			face_area_list.push_back(currFace->ComputeArea());
			face_area_list.push_back(neiborFace->ComputeArea());

			currFace->ComputeProjectMatrixTo2D(projMat, invsProjMat);
			vector<Vector3f> currFace2D = currFace->ProjectPolygonTo2D(projMat);
			vector<Vector3f> neiborFace2D = neiborFace->ProjectPolygonTo2D(projMat);

			PolyPolyIntersec ppIntersec(getVarList());
			vector<Vector3f> overlapPolyPts;
            ppIntersec.ComputePolygonsIntersection(currFace2D, neiborFace2D, overlapPolyPts);
			if( overlapPolyPts.size() == 0 ) continue;

			vector<Vector3f> contactPoly;
			ppIntersec.ProjectPolygonTo3D(overlapPolyPts, invsProjMat, contactPoly);
			Vector3f contactNormal = currFace->normal;

			bool boundaryContact;
			if( part->atBoundary && neibor->atBoundary)
				boundaryContact = true;
			else
				boundaryContact = false;

			pContact contact;
			if(cross == NULL)
				contact = make_shared<Contact>(part->partID, neibor->partID, contactNormal, shared_ptr<OrientPoint>(), contactPoly, boundaryContact);
			else
				contact = make_shared<Contact>(part->partID, neibor->partID, contactNormal, cross->oriPoints[j], contactPoly, boundaryContact);
            numFaceFaceContacts++;
			contactList.push_back(contact);
			if(!part->atBoundary || !neibor->atBoundary) innerContactList.push_back(contact);
		}
	}

    //average face size
    avgContactArea = 0;
    for(double face_area: face_area_list)
    {
        avgContactArea += face_area;
    }
    avgContactArea /= face_area_list.size();
    ComputePartContactArea();
}

void Struc::ComputePartContactArea()
{
    minContactArea = 1 << 30;
    for (int i = 0; i < innerContactList.size(); ++i)
    {
        shared_ptr<Contact> curr_contact = innerContactList[i].lock();
        if(curr_contact->contactType == Contact_FaceFace)
        {
            int ID_A = curr_contact->partIDA;
            int ID_B = curr_contact->partIDB;
//            partList[ID_A] -> contactIDs.push_back(i);
//            partList[ID_B] -> contactIDs.push_back(i);
            _Polygon poly; poly.SetVertices(curr_contact->contactPoly);
            double area = poly.ComputeArea();
            curr_contact->contactArea = area;
            if(minContactArea > area)
            {
                minContactArea = area;
            }
        }
    }
}

void Struc::ComputePartEdgeContact(vector<vector<weak_ptr<Cross>>> &vertexCrossList)
{
    numEdgeEdgeContacts = 0;
	for(int id = 0; id < vertexCrossList.size(); id++)
	{
	    for(int jd = 0; jd < vertexCrossList[id].size(); jd++)
	    {
	        for(int kd = 0; kd < vertexCrossList[id].size(); kd++)
	        {
	            shared_ptr<Cross> crossI = vertexCrossList[id][jd].lock();
	            shared_ptr<Cross> crossJ = vertexCrossList[id][kd].lock();
	            if(crossI && crossJ && crossI->crossID < crossJ->crossID)
	            {
	                //part
	                if(crossI->crossID < 0 || crossI->crossID >= partList.size())
                        continue;
                    if(crossJ->crossID < 0 || crossJ->crossID >= partList.size())
                        continue;

	                shared_ptr<Part> partI = partList[crossI->crossID];
                    shared_ptr<Part> partJ = partList[crossJ->crossID];

                    if(partI == nullptr || partJ == nullptr)
                        continue;

	                //check whether they are neighbor
	                if(crossJ->GetNeighborEdgeID(crossI->crossID) != NONE_ELEMENT) continue;
	                int edgeI = crossI->GetVertexEdgeID(id);
	                int edgeJ = crossJ->GetVertexEdgeID(id);

	                if(edgeI == NONE_ELEMENT || edgeJ == NONE_ELEMENT) continue;
	                if(partI->isRemove || partJ->isRemove) continue;

	                Vector3f pt = crossI->vers[edgeI].pos;
	                int pedgeI = crossI->GetPrevEdgeID(edgeI);
                    int pedgeJ = crossJ->GetPrevEdgeID(edgeJ);

                    Vector3f nI = crossI->oriPoints[edgeI]->normal;
                    Vector3f pnI = crossI->oriPoints[pedgeI]->normal;
                    Vector3f nJ = crossJ->oriPoints[edgeJ]->normal;
                    Vector3f pnJ = crossJ->oriPoints[pedgeJ]->normal;

                    Vector3f ctI = crossI->center;
                    Vector3f n = (nI CROSS pnI) CROSS (nJ CROSS pnJ);
                    //std::cout << len(n) << std::endl;
                    n = n / len(n);
                    if((n DOT (pt - ctI)) < 0) n = -n;

                    bool atBoundary = partI->atBoundary && partJ->atBoundary;
                    shared_ptr<Contact> contact = make_shared<Contact>(partI->partID, partJ->partID, n, pt, atBoundary);
                    contactList.push_back(contact);
                    numEdgeEdgeContacts++;
                    if(!atBoundary) {
                        innerContactList.push_back(contact);
                    }
	            }
	        }
	    }
	}
}


//**************************************************************************************//
//                                   Save OBJ Models
//**************************************************************************************//

void Struc::WriteStructure(const char *folderPath)
{
	vector<int> boundary_part;
    for (int i = 0; i < partList.size(); i++)
    {
        shared_ptr<Part> part = partList[i];
        int assemblyID = part->assemblyID;
        if( part->atBoundary)
        {
            boundary_part.push_back(part->partID);
        }
        else {
            char objFileName[FILE_NAME_LENGTH];
            sprintf(objFileName, "%s/Part_%02d.obj", folderPath, i);
            part->WriteOBJModel(objFileName);
        }
    }

    if( boundary_part.size() > 0 ) {
        char boundaryObjFileName[FILE_NAME_LENGTH];
        sprintf(boundaryObjFileName, "%s/Boundary.obj", folderPath);
        PartGroup boundary_group = PartGroup(partList, boundary_part, getVarList());
        boundary_group.WriteGroupOBJModel(boundaryObjFileName, true);
    }

}

void Struc::WriteStructureWireFrame(const char *folderPath)
{
    vector<int> partIDs;
    vector<shared_ptr<Part>> _partList;
    for (int i = 0; i < partList.size(); i++)
    {
        partIDs.push_back(partList[i]->partID);
        shared_ptr<Part> part = make_shared<Part>(*partList[i]);
        part->polyMesh->TranslateMesh(Vector3f(0, -getVarList()->get<float>("ground_height"), 0));
        int assemblyID = part->assemblyID;
        _partList.push_back(part);
    }

    if(partList.size() > 0 ){
        char objFileName[FILE_NAME_LENGTH];
        sprintf(objFileName, "%s/PartsWire.obj", folderPath);
        PartGroup parts = PartGroup(_partList, partIDs, getVarList());
        parts.WriteGroupOBJWireModel(objFileName);
    }
}

void Struc::WritePartGraph(char *folderPath)
{
	char graphFileName[512];
	sprintf(graphFileName, "%s//PartGraph.txt", folderPath);

	FILE *fp;
	if ((fp = fopen(graphFileName, "w+")) == NULL)
	{
		printf("Error: file not exists! \n");
		return;
	}
	else
	{
		////////////////////////////////////////////////////////
		// Write piece OBJ model name and number

		//fprintf(fp, "# Part ID starts from 0 \n");
		fprintf(fp, "Part Num %d\n", (int)partList.size());

		for (int i = 0; i < partList.size(); i++)
		{
			pPart part = partList[i];
			//char partFileName[FILE_NAME_LENGTH];
			//sprintf(partFileName, "%2d %d", i, part);

			fprintf(fp, "%2d    supp: %d    neighbor %d: [", part->partID, part->atBoundary, (int)part->initNeighbors.size());

			for (int j = 0; j < part->initNeighbors.size(); j++)
			{
				pPart neibor = part->initNeighbors[j].lock();

				if (neibor == NULL)    fprintf(fp, " %2d ", -1);
				else                    fprintf(fp, " %2d ", neibor->partID);
			}

			fprintf(fp, " ] \n");
		}

		fprintf(fp, "\n");

		fclose(fp);
	}
}