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
#include "Utility/Controls.h"
#include "Utility/math3D.h"
#include "Utility/HelpDefine.h"
#include "Utility/HelpStruct.h"
#include "Utility/HelpFunc.h"
#include "Utility/PolyPolyTest.h"
#include "Mesh/Polygon.h"
#include "Mesh/PolyMesh.h"
#include "Structure/Part.h"
#include "Structure/PartGroup.h"
#include "Structure/BodyMobili.h"
#include "Structure/Struc.h"
#include "IO/gluiVar.h"

#include <unordered_map>
#include <queue>

using std::unordered_map;
using std::queue;

extern gluiVarList varList;
extern Vector3f colorTable[18];
extern vector<int> pickPartIDs;

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

Struc::Struc()
{
	partGroup = NULL;
    num_edge_edge = num_face_face = 0;
}

Struc::~Struc()
{
	ClearStruc();
}

void Struc::ClearStruc()
{
	partList.clear();

	partGroup.reset();

	contactList.clear();

    num_edge_edge = num_face_face = 0;
}

Struc::Struc(const Struc& _struc)
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
    minimum_contact_surface = _struc.minimum_contact_surface;
    average_face_area = _struc.average_face_area;
    avgPartSize = _struc.avgPartSize;
    groundY = _struc.groundY;
    num_face_face = _struc.num_face_face;
    num_edge_edge = _struc.num_edge_edge;

    return;
}

void Struc::ClearPartGroup()
{
	partGroup.reset();
}

//**************************************************************************************//
//                              Identify Parts (Disk Model)
//**************************************************************************************//

void Struc::IdentifyBoundaryParts_Disk()
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
			part->atBoundary = part->cross.lock()->atBoundary;
			if(part->atBoundary){
                BoundPartIDs.push_back(part->partID);
			}
		}
//		cout << "Init Boundary: \t\t[";
//		for(int partID : BoundPartIDs)
//		{
//			cout << partID << " ";
//		}
//		cout << "]" << endl;
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

//		cout << "Connect Boundary: \t[";
//		for(int partID : BoundPartIDs)
//		{
//			cout << partID << " ";
//		}
//		cout << "]" << endl;
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
			if(IsLockedByBoundary(pU->partID))
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

//		cout << "Locked Boundary: \t[";
//		for(int partID : BoundPartIDs)
//		{
//			cout << partID << " ";
//		}
//		cout << "]" << endl;
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
			//std::cout << id << ", " << partList[id]->assemblyID << std::endl;
		}
	}

	//printf("\nTotal part: %d   boundary part: %d \n", (int)partList.size(), (int)boundaryPartNum);
}

bool Struc::IsLockedByBoundary(int partID) {

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

double Struc::ComputeGroundHeight()
{
    // Compute structure's lowest point
    float strucMinY = MAX_FLOAT;

    for (int i = 0; i < partList.size(); i++)
    {
        pPart part = partList[i];

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
	AveragePartSize();

	// Get ground y-coordinate
	groundY = ComputeGroundHeight() + 0.1f * avgPartSize;
	varList.set("ground_height", groundY);

	// Identify parts that touch the ground
	if (showGround)
	{
		IdentifyTouchGroundParts_Disk();
	}
	else
	{
		ResetTouchGround();
	}
}

void Struc::IdentifyTouchGroundParts_Disk()
{
	const float ydistThres = 0.08 * avgPartSize;

	for (int i = 0; i < partList.size(); i++)
	{
		pPart part = partList[i];

		float partMinY = part->polyMesh->bbox.minPt.y;
		float partMaxY = part->polyMesh->bbox.maxPt.y;

		if ((groundY - partMinY) > ydistThres &&
			(partMaxY - groundY) > ydistThres)
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

void Struc::AveragePartSize()
{
	avgPartSize = 0;

	for (int i = 0; i < partList.size(); i++)
	{
		pPart part = partList[i];

		float partSize = len(part->polyMesh->bbox.maxPt - part->polyMesh->bbox.minPt);

		//printf("i=%d  partSize: %.3f \n", i, partSize);

		avgPartSize += partSize;
	}

	avgPartSize /= (float)partList.size();

	//printf("AveragePartSize: %3f \n", avgPartSize);
}




//**************************************************************************************//
//                              Compute Part Contacts
//**************************************************************************************//

void Struc::ComputePartFaceContactsBruteForce()
{
    num_face_face = 0;
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
                    vector<Vector3f> overlapPolyPts = ConvexPolygonIntersec(currFace2D, neiborFace2D);

                    if( overlapPolyPts.size() == 0 ) continue;
                    vector<Vector3f> contactPoly = ProjectPolygonTo3D(overlapPolyPts, invsProjMat);
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

                    num_face_face++;
                }
            }
        }
    }

    //average face size
    average_face_area = 0;
    for(double face_area: face_area_list)
    {
        average_face_area += face_area;
    }
    average_face_area /= face_area_list.size();
    ComputePartContactArea();
}


void Struc::ComputePartFaceContacts()
{
	std::vector<double> face_area_list;
    num_face_face = 0;
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
			vector<Vector3f> overlapPolyPts = ConvexPolygonIntersec(currFace2D, neiborFace2D);

//			//if(overlapPolyPts.size() != 6){
//				std::cout << part->partID << "\t" << neibor->partID << "\t" << overlapPolyPts.size() << std::endl;
//			//}

			if( overlapPolyPts.size() == 0 ) continue;
			vector<Vector3f> contactPoly = ProjectPolygonTo3D(overlapPolyPts, invsProjMat);
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
            num_face_face++;
			contactList.push_back(contact);
			if(!part->atBoundary || !neibor->atBoundary) innerContactList.push_back(contact);
		}
	}

    //average face size
    average_face_area = 0;
    for(double face_area: face_area_list)
    {
        average_face_area += face_area;
    }
    average_face_area /= face_area_list.size();
    ComputePartContactArea();
}

void Struc::ComputePartContactArea()
{
    minimum_contact_surface = 1 << 30;
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
            curr_contact->area = area;
            if(minimum_contact_surface > area)
            {
                minimum_contact_surface = area;
            }
        }
    }
}

void Struc::ComputePartEdgeContact(vector<vector<weak_ptr<Cross>>> &vertexCrossList)
{
    num_edge_edge = 0;
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
                    num_edge_edge++;
                    if(!atBoundary) {
                        innerContactList.push_back(contact);
                    }
	            }
	        }
	    }
	}
}

bool Struc::ComputePartPenetration(vector<vector<weak_ptr<Cross>>> &vertexCrossList) {
    for (int id = 0; id < vertexCrossList.size(); id++) {
        for (int jd = 0; jd < vertexCrossList[id].size(); jd++) {
            for (int kd = 0; kd < vertexCrossList[id].size(); kd++) {
                shared_ptr<Cross> crossI = vertexCrossList[id][jd].lock();
                shared_ptr<Cross> crossJ = vertexCrossList[id][kd].lock();
                if (crossI && crossJ && crossI->crossID < crossJ->crossID) {
                    //part
                    if (crossI->crossID < 0 || crossI->crossID >= partList.size())
                        continue;
                    if (crossJ->crossID < 0 || crossJ->crossID >= partList.size())
                        continue;

                    shared_ptr<Part> partI = partList[crossI->crossID];
                    shared_ptr<Part> partJ = partList[crossJ->crossID];

                    if (partI == nullptr || partJ == nullptr)
                        continue;

                    if (ComputePartCSG(partI->partID, partJ->partID)) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool Struc::check_same_poly(vector<Vector3f> poly_A, vector<Vector3f> poly_B) {
	for (int ind_a = 0; ind_a < poly_A.size(); ++ind_a) {
		for (int ind_b = 0; ind_b < poly_B.size(); ++ind_b) {
			if (len(poly_A[ind_a] - poly_B[ind_b]) < FLOAT_ERROR_LARGE)
				return true;
		}
	}
	return false;
}

vector<Vector3f> Struc::MergePoly(vector<Vector3f> poly_A, vector<Vector3f> poly_B)
{
	vector<Vector3f> merged_poly;
	for (int ind_a = 0; ind_a < poly_A.size(); ++ind_a) {
		bool new_vx = true;
		for (int ind_b = 0; ind_b < poly_B.size(); ++ind_b) {
			if (len(poly_A[ind_a] - poly_B[ind_b]) < FLOAT_ERROR_LARGE)
				new_vx = false;
		}
		if (new_vx)	merged_poly.push_back(poly_A[ind_a]);
	}
	for (int ind_b = 0; ind_b < poly_B.size(); ++ind_b)
		merged_poly.push_back(poly_B[ind_b]);
	RearrangePoints(merged_poly);
	return merged_poly;
}

void Struc::ComputePartCutHeight(vector<Vector2f> &heights)
{
	heights.resize(partList.size(), Vector2f(0, 0));
	for(int id = 0; id < contactList.size(); id++){
		shared_ptr<Contact> contact = contactList[id];
		int partIDs[2] = {contact->partIDA, contact->partIDB};
		//partIDA
		for(int partID : partIDs)
		{
			shared_ptr<Part> part = partList[partID];
			Vector3f center = part->cross.lock()->center;
			Vector3f normal = part->cross.lock()->normal;
			normal /= len(normal);

			for(int kd = 0; kd < 2; kd++){
				if(kd == 1) normal = normal * (-1.0f);
				for(Vector3f cpt: contact->contactPoly)
				{
					double dist = (cpt - center) DOT normal;
					if(dist > heights[partID][kd]) heights[partID][kd] = dist;
				}
			}
		}
	}
	return;
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
        PartGroup boundary_group = PartGroup(partList, boundary_part);
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
        part->polyMesh->TranslateMesh(Vector3f(0, -varList.get<float>("ground_height"), 0));
        int assemblyID = part->assemblyID;
        _partList.push_back(part);
    }

    if(partList.size() > 0 ){
        char objFileName[FILE_NAME_LENGTH];
        sprintf(objFileName, "%s/PartsWire.obj", folderPath);
        PartGroup parts = PartGroup(_partList, partIDs);
        parts.WriteGroupOBJWireModel(objFileName);
    }
}


void Struc::WriteStructureForSimulation(const char *folderPath){
	for (int i = 0; i < partList.size(); i++)
	{
		pPart part = partList[i];
		char objFileName[FILE_NAME_LENGTH];
		sprintf(objFileName, "%s/Part_%d.obj", folderPath, i);
		part->WriteOBJModel(objFileName);
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


void Struc::WriteDisassemFiles(char *folderPath, vector<int> doorPartIDs)
{
	/////////////////////////////////////////////////////////////////
	// 1. Assembly parts as a single piece

	vector<int> suppPartIDs;
	for (int i = 0; i < partList.size(); i++)
	{
		pPart part = partList[i];

		if (part->atBoundary == true)
		{
			if (GetElementIndexInList(part->partID, doorPartIDs) == ELEMENT_OUT_LIST)
			{
				suppPartIDs.push_back(part->partID);
			}
		}
	}


	char suppObjFileName[FILE_NAME_LENGTH];
	sprintf(suppObjFileName, "%s\\Supp_Fix.obj", folderPath);

	PartGroup suppGroup = PartGroup(partList, suppPartIDs);
	suppGroup.WriteGroupOBJModel(suppObjFileName);


	/////////////////////////////////////////////////////////////////
	// 2. Door parts as a single piece

	char doorObjFileName[FILE_NAME_LENGTH];
	sprintf(doorObjFileName, "%s\\Supp_Door.obj", folderPath);

	PartGroup doorGroup = PartGroup(partList, doorPartIDs);
	doorGroup.WriteGroupOBJModel(doorObjFileName);


	/////////////////////////////////////////////////////////////////
	// 3. Other parts as individual pieces

	for (int i = 0; i < partList.size(); i++)
	{
		char objFileName[FILE_NAME_LENGTH];
		sprintf(objFileName, "%s\\Part_%02d.obj", folderPath, i);

		if (partList[i]->atBoundary == false)
		{
			partList[i]->WriteOBJModel(objFileName);
		}
	}


	/////////////////////////////////////////////////////////////////
	// 4. Save parts motion file

	WriteMotionFile(folderPath);
}


void Struc::WriteMotionFile(char *folderPath)
{
	char motionFileName[512];
	sprintf(motionFileName, "%s\\animation.motion.txt", folderPath);

	FILE *fp;
	if ((fp = fopen(motionFileName, "w+")) == NULL)
	{
		printf("Error: file not exists! \n");
		return;
	}
	else
	{
		////////////////////////////////////////////////////////
		// Write piece OBJ model name and number

		fprintf(fp, "\n# List of object files with id starts from 1\n");
		fprintf(fp, "Objects %d\n", (int)partMoveList.size() + 2);

		fprintf(fp, "Supp_Door.obj ");

		for (int i = 0; i < partMoveList.size(); i++)
		{
			int partID = partMoveList[i].partID;
			char partFileName[FILE_NAME_LENGTH];
			sprintf(partFileName, "Part_%02d.obj ", partID);

			fprintf(fp, "%s", partFileName);
		}

		fprintf(fp, "Supp_Fix.obj ");

		fprintf(fp, "\n");


		////////////////////////////////////////////////////////
		// Write piece disassembly actions

		const int frameNum = 100;
		const float dist = 3.0f;

		fprintf(fp, "\n# List of actions during the animation\n");

		Vector3f doorMoveVec = dist * Vector3f(0, 0, 1);

		fprintf(fp, "Begin Action %d\n", frameNum);
		fprintf(fp, "Move id %d [%.2f, %.2f, %.2f]\n", 1, doorMoveVec.x, doorMoveVec.y, doorMoveVec.z);
		fprintf(fp, "End\n");

		for (int i = partMoveList.size() - 1; i >= 0; i--)
			//for (int i = 0; i <partMoveList.size(); i++)
		{

			Vector3f moveVector = dist * partMoveList[i].transVec;

			// TODO: may need to make this more flexible
			fprintf(fp, "Begin Action %d\n", frameNum);
			fprintf(fp, "Move id %d [%.2f, %.2f, %.2f]\n", i + 2, moveVector.x, moveVector.y, moveVector.z);
			//fprintf(fp, "Move id %d [%.2f, %.2f, %.2f]\n", i + 1, moveVector.x, moveVector.y, moveVector.z);
			fprintf(fp, "End\n");
		}

		fclose(fp);
	}
}

//**************************************************************************************//
//                                   Test Functions
//**************************************************************************************//

void Struc::Function_PartGroup(vector<int> &pickPartIDs)
{
	if (partList.size() == 0)
		return;

	partGroup.reset();

	/*vector<int> groupPartIDs;
	for (int i = 0; i <partList.size(); i++)
	{
		Part *part = partList[i];

		if( part->atBoundary == false)
		{
			groupPartIDs.push_back(part->partID);
		}
	}*/

	//pickPartIDs = groupPartIDs;

	//partGroup = new PartGroup(partList, groupPartIDs);
	partGroup = make_shared<PartGroup>(partList, pickPartIDs);

	//partGroup->IdentifyGroupBoundary();

	bool isMobile = partGroup->EvaluateGroupMobility();
}

void Struc::Function_Mobility(vector<int> groupPartIDs)
{
	partGroup.reset();
	partGroup = make_shared<PartGroup>(partList, groupPartIDs);

	bool isMobile = partGroup->EvaluateGroupMobility();
}

void Struc::Function_Test()
{
	vector<int> groupPartIDs;

	for(int i=0; i<partList.size(); i++)
	{
		groupPartIDs.push_back( partList[i]->partID );
	}

	partGroup.reset();
	partGroup = make_shared<PartGroup>(partList, groupPartIDs);

	bool isMobile = partGroup->EvaluateGroupMobility();

	/*vector<Vector3f> points;
	points.push_back(Vector3f(-1.0, 0.0, 0));
	points.push_back(Vector3f(0.0, 1.0, 0));
	points.push_back(Vector3f(1.0, 0.0, 0));
	points.push_back(Vector3f(0.0, -1.0, 0));
	points.push_back(Vector3f(-0.1, -0.1, 0));

	vector<Vector3f> normals;
	normals.push_back(Vector3f(-1.0, 0.0, -0.5));
	normals.push_back(Vector3f(0.0, 1.0, 0.5));
	normals.push_back(Vector3f(1.0, 0.0, -0.5));
	normals.push_back(Vector3f(0.0, -1.0, 0.5));
	normals.push_back(Vector3f(-0.5, -1.0, 0.0));

	vector<OrientPoint*> oriPts;
	for (int i = 0; i < points.size(); i++)
	{
		OrientPoint *oriPt = new OrientPoint();
		oriPt->point = points[i];
		oriPt->normal = normals[i];

		oriPts.push_back(oriPt);
	}

	Part *part = new Part(oriPts);
	part->ComputePartGeometry();
	partList.push_back(part);*/
}


#if USE_OPENGL_DRAW

//**************************************************************************************//
//                                 Draw TI Construction
//**************************************************************************************//

void Struc::DrawOriPoints(double worldMatrix[])
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	for (int i = 0; i < partList.size(); i++)
	{
		partList[i]->DrawOriPoints();
	}

	glPopMatrix();
}

void Struc::DrawInnerPolygons(double worldMatrix[])
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	for (int i = 0; i < partList.size(); i++)
	{
		partList[i]->DrawInnerPolygon();
	}

	glPopMatrix();
}

void Struc::DrawGeomFaces(double worldMatrix[])
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	//int i = 0;
	for (int i = 0; i < partList.size(); i++)
	{
		partList[i]->DrawGeomFaces();
	}

	glPopMatrix();
}

void Struc::DrawGeomEdges(double worldMatrix[])
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	for (int i = 0; i < partList.size(); i++)
	{
		partList[i]->DrawGeomEdges();
	}

	glPopMatrix();
}

void Struc::DrawGeomVertices(double worldMatrix[])
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);


	for (int i = 0; i < partList.size(); i++)
	{
		partList[i]->DrawGeomVertices();
	}

//	for(int partID: pickPartIDs){
//		partList[partID]->DrawGeomVertices();
//	}

	glPopMatrix();
}




//**************************************************************************************//
//                                 Draw Part Group Mobility
//**************************************************************************************//

void Struc::DrawGroupMobiliFaces(double worldMatrix[])
{
	if (partGroup == NULL)
		return;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	partGroup->DrawMobiliFaces();

	glPopMatrix();
}

void Struc::DrawGroupMobiliEdges(double worldMatrix[])
{
	if (partGroup == NULL)
		return;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	partGroup->DrawMobiliEdges();

	glPopMatrix();
}

void Struc::DrawGroupMobiliVertices(double worldMatrix[])
{
	if (partGroup == NULL)
		return;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	partGroup->DrawMobiliVertices();

	glPopMatrix();
}

void Struc::DrawGroupMobiliRays(double worldMatrix[])
{
	if (partGroup == NULL)
		return;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	partGroup->DrawMobiliRays();

	glPopMatrix();
}

void Struc::DrawGroupMobiliVectors(double worldMatrix[])
{
	if (partGroup == NULL)
		return;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	partGroup->DrawMobiliVector();

	glPopMatrix();
}

void Struc::DrawGroupMobiliMeshes(double worldMatrix[])
{
	if (partGroup == NULL)
		return;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	partGroup->DrawMobiliMesh();

	glPopMatrix();
}




//**************************************************************************************//
//                                   Draw TI Structure
//**************************************************************************************//

void Struc::DrawStructure(int mode, double worldMatrix[])
{
	if (partList.size() == 0)
		return;

	//int i = 12;

	if(varList.get<bool>("showPickParts"))
	{
		vector<weak_ptr<Part>> partial_parts;
		for (int i = 0; i < pickPartIDs.size(); i++)
		{
		    if(pickPartIDs[i] >= partList.size() || pickPartIDs[i] < 0)
                continue;
			pPart part = partList[pickPartIDs[i]];
			partial_parts.push_back(part);
		}
		DrawParts(mode, worldMatrix, partial_parts);
	}
	else if(varList.get<bool>("showAssembly"))
	{
//		int startID = varList.get<int>("assembling_Kth_part") + 1;
//		vector<weak_ptr<Part>> partial_parts;
//		vector<weak_ptr<Part>> extra_boundaries;
//		for(int i = startID; i < assemblingSequence->orderParts.size(); i++){
//			int partID = assemblingSequence->orderParts[i].lock()->staticID;
//			shared_ptr<Part> part = partList[partID];
//			partial_parts.push_back(part);
//		}
//
//		for(int i = 0; i < assemblingSequence->boundaryParts.size(); i++){
//			int partID = assemblingSequence->boundaryParts[i].lock()->staticID;
//			shared_ptr<Part> part = partList[partID];
//			extra_boundaries.push_back(part);
//		}
//
//		DrawParts(mode, worldMatrix, partial_parts);
//		DrawParts(mode, worldMatrix, boundarys);
//		DrawParts(mode, worldMatrix, extra_boundaries);
//		if(startID >= 0 && startID < assemblingSequence->orderParts.size())
//		{
//			int partID = assemblingSequence->orderParts[startID].lock()->staticID;
//			DrawPartAssemblyCone(worldMatrix, partList[partID], assemblingSequence->orderCones[startID]);
//		}

	}
	else {
		if(!parts.empty()){
			DrawParts(mode, worldMatrix, parts);
			DrawParts(mode, worldMatrix, boundarys);
		}
		else{
			vector<weak_ptr<Part>> partial_parts;
			for(int i = 0; i < partList.size(); i++){
				partial_parts.push_back(partList[i]);
			}
			DrawParts(mode, worldMatrix, partial_parts);
		}

	}
}

void Struc::DrawParts(int mode, double *worldMatrix, const vector<weak_ptr<Part>> &_parts)
{
	for (int i = 0; i < _parts.size(); i++)
    {
		pPart part =  _parts[i].lock();
		if(varList.get<bool>("mult_move") && part->atBoundary == false)
        {
		    if(contactGraph && !contactGraph->translation.isZero())
		    {
		        Eigen::VectorXd &translation = contactGraph->translation;
		        int assemblyID = part->assemblyID;
                if(assemblyID != -1) {
                    Vector3f move(translation[3 * assemblyID], translation[3 * assemblyID + 1], translation[3 * assemblyID + 2]);
                    move *= varList.get<float>("mult_move_scale");
                    part = make_shared<Part>(*_parts[i].lock());
                    part->polyMesh->TranslateMesh(move);
                }
            }
        }

		if(part == nullptr)
            continue;
		if (part->isRemove) continue;

		if (mode == GL_SELECT)
		{
			glLoadName(part->partID);
			glPushName(part->partID);
		}

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadMatrixd(worldMatrix);

		part->DrawPart();

		glPopMatrix();

		if (mode == GL_SELECT)
		{
			glPopName();
		}
	}
}

void Struc::DrawStructureWire(double worldMatrix[])
{
	if (partList.size() == 0)
		return;

	if(varList.get<bool>("showPickParts"))
	{
        vector<weak_ptr<Part>> partial_parts;
        for (int i = 0; i < pickPartIDs.size(); i++)
        {
            if(pickPartIDs[i] >= partList.size() || pickPartIDs[i] < 0)
                continue;
            pPart part = partList[pickPartIDs[i]];
            partial_parts.push_back(part);
        }
        DrawPartsWire(worldMatrix, partial_parts);
	}
	else if(varList.get<bool>("showAssembly"))
	{
//		int startID = varList.get<int>("assembling_Kth_part") + 1;
//		vector<weak_ptr<Part>> partial_parts;
//		vector<weak_ptr<Part>> extra_boundaries;
//		for(int i = startID; i < assemblingSequence->orderParts.size(); i++){
//			int partID = assemblingSequence->orderParts[i].lock()->staticID;
//			shared_ptr<Part> part = partList[partID];
//			partial_parts.push_back(part);
//		}
//
//		for(int i = 0; i < assemblingSequence->boundaryParts.size(); i++){
//			int partID = assemblingSequence->boundaryParts[i].lock()->staticID;
//			shared_ptr<Part> part = partList[partID];
//			extra_boundaries.push_back(part);
//		}
//
//		DrawPartsWire(worldMatrix, partial_parts);
//		DrawPartsWire(worldMatrix, boundarys);
//		DrawPartsWire(worldMatrix, extra_boundaries);

	}
	else {
		if(!parts.empty())
		{
			DrawPartsWire(worldMatrix, parts);
			DrawPartsWire(worldMatrix, boundarys);
		}
		else{
			vector<weak_ptr<Part>> partial_parts;
			for(int i = 0; i < partList.size(); i++){
				partial_parts.push_back(partList[i]);
			}
			DrawPartsWire(worldMatrix, partial_parts);
		}
	}
}

void Struc::DrawPartsWire(double *worldMatrix, const vector<weak_ptr<Part>> &_parts){
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);
	for (int i = 0; i < _parts.size(); i++)
	{
		pPart part = _parts[i].lock();
        if(varList.get<bool>("mult_move") && part->atBoundary == false)
        {
            if(contactGraph && !contactGraph->translation.isZero())
            {
                Eigen::VectorXd &translation = contactGraph->translation;
                int assemblyID = part->assemblyID;
                if(assemblyID != -1){
                    Vector3f move(translation[3 * assemblyID], translation[3 * assemblyID + 1], translation[3 * assemblyID + 2]);
                    move *= varList.get<float>("mult_move_scale");
                    part = make_shared<Part>(*_parts[i].lock());
                    part->polyMesh->TranslateMesh(move);
                }
            }
        }
		if (part == nullptr || part->isRemove)
			continue;

//		if(!pickPartIDs.empty() && pickPartIDs[0] != i)
//			continue;

		part->DrawPartWire(2.0, part->wire_color);
	}
	glPopMatrix();
}

void Struc::DrawStructure3DText(double worldMatrix[])
{
	if (partList.size() == 0)
		return;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	float scale = 0.04f * avgPartSize;
	for (int i = 0; i < partList.size(); i++)
	{
		pPart part = partList[i];

		if(part->isRemove) continue;

		part->DrawPart3DText(scale, 2.0, Vector3f(0.2, 0.2, 0.2));
	}

	glPopMatrix();
}

void Struc::DrawStructureNormals(double worldMatrix[])
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	for (int i = 0; i < partList.size(); i++)
	{
        if(partList[i]->isRemove) continue;
		partList[i]->DrawPartNormals();
	}
//    for (int i = 0; i < pickPartIDs.size(); i++)
//    {
//        partList[pickPartIDs[i]]->DrawPartNormals();
//    }

	glPopMatrix();
}

void Struc::DrawStructureBBox(double worldMatrix[])
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	for (int i = 0; i < partList.size(); i++)
	{
        if(partList[i]->isRemove) continue;
		partList[i]->DrawPartBBox(2.5, Vector3f(0.3, 0.6, 0.3));
	}

	glPopMatrix();
}

void Struc::DrawStructureCentroid(double worldMatrix[])
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	for (int i = 0; i < partList.size(); i++)
	{
        if(partList[i]->isRemove) continue;
		partList[i]->DrawPartCentroid(0.02, Vector3f(0.6, 0.9, 0.9));
	}

	glPopMatrix();
}

void Struc::DrawStructureGraph(double worldMatrix[])
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	glDisable(GL_LIGHTING);
	glLineWidth(3.0);
	glColor3f(0.9, 0.2, 0.9);

	for (int i = 0; i < partList.size(); i++)
	{
		pPart part = partList[i];

		if (part->isRemove) continue;

		Vector3f partCen = part->polyMesh->centroid;

		for (int j = 0; j < part->currNeighbors.size(); j++)
		{
			pPart neibor = part->currNeighbors[j].lock();

			if (neibor == NULL)
				continue;

			Vector3f neiborCen = neibor->polyMesh->centroid;

			glBegin(GL_LINES);
			glVertex3f(partCen.x, partCen.y, partCen.z);
			glVertex3f(neiborCen.x, neiborCen.y, neiborCen.z);
			glEnd();
		}
	}

	glEnable(GL_LIGHTING);
	glLineWidth(1.0);

	glPopMatrix();
}




//**************************************************************************************//
//                                Draw Picked Part(s)
//**************************************************************************************//

void Struc::DrawPickPart(int pickPartID, double worldMatrix[])
{
	if (pickPartID < 0 || pickPartID >= partList.size())
		return;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	pPart pickPart = partList[pickPartID];
	pickPart->DrawPartWire(5.0, Vector3f(0.2, 0.9, 0.2));

	glPopMatrix();
}

void Struc::DrawPickPartGroup(vector<int> pickPartIDs, double worldMatrix[])
{
    Vector3f cyan = Vector3f(0.612,  0.784,  0.953);
    //Vector3f yellow = Vector3f(0.953,  0.886,  0.667);
    //Vector3f green = Vector3f(0.906,  0.973,  0.780);

	for (int i = 0; i < pickPartIDs.size(); i++)
	{
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadMatrixd(worldMatrix);

		int partID = pickPartIDs[i];
		if(partID < 0 || partID >= partList.size())
            continue;
		pPart pickPart = partList[partID];
		pickPart->DrawPartWire(5.0, cyan);

		glPopMatrix();
	}
}

void Struc::DrawDebug_PartGroup(double worldMatrix[])
{
	if( partGroup == NULL )
		return;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	partGroup->DrawOriPoints();

	glPopMatrix();
}




//**************************************************************************************//
//                                   Draw Equilibrium
//**************************************************************************************//


void Struc::DrawStructureContacts(double worldMatrix[])
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	Vector3f red    = Vector3f(230, 184, 185) / 255.0f;
    Vector3f yellow = Vector3f(243, 226, 170) / 255.0f;
    Vector3f purple = Vector3f(192, 140, 233) / 255.0f;

    if(varList.get<bool>("showPickParts"))
    {
        map<int, bool> visited;
        for(int partID: pickPartIDs){
            visited[partID] = true;
        }

        for (int i = 0; i < contactList.size(); i++)
        {
            if(visited[contactList[i]->partIDA] == false) continue;
            if(visited[contactList[i]->partIDB] == false) continue;
            vector<Vector3f> contact_poly = contactList[i]->contactPoly;
            if(contact_poly.size() > 1)
            {
                DrawPolygon(contact_poly, 6.0, red);

                for(int j=0; j<contact_poly.size(); j++ )
                {
                    Vector3f startPt = contact_poly[j];
                    Vector3f endPt = startPt + 0.12f * contactList[i]->normal;

                    DrawLine(startPt, endPt, 6.0f, purple);
                }
            }


            if(varList.get<bool>("showOptGraident") && contactList[i]->versDiff.size() == contact_poly.size())
            {
                glColor3f(0.4f, 0.4f, 0.9f);
                glLineWidth(5);
                glDisable(GL_LIGHTING);
                for(int j = 0; j < contact_poly.size(); j++){
                    glBegin(GL_LINES);
                    Vector3f pt = contact_poly[j];
                    Vector3f pt_gradient = pt + contactList[i]->versDiff[j];
                    glVertex3d(pt.x, pt.y, pt.z);
                    glVertex3d(pt_gradient.x, pt_gradient.y, pt_gradient.z);
                    glEnd();
                }
                glEnable(GL_LIGHTING);
                glLineWidth(1);
            }

        }
    }
    else{
        for (int i = 0; i < contactList.size(); i++)
        {
            vector<Vector3f> contact_poly = contactList[i]->contactPoly;
            if(contact_poly.size() > 1)
            {
                DrawPolygon(contact_poly, 6.0, Vector3f(0.9, 0.3, 0.3));
            }
        }
    }

	glPopMatrix();
}

void Struc::DrawStructureForces(double worldMatrix[])
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	for (int i = 0; i < partList.size(); i++)
	{
		if(pickPartIDs.empty() || pickPartIDs[0] == i)
		{
			shared_ptr<Part> part = partList[i];
			part->DrawContactForces();
		}
	}
	glPopMatrix();
}

void Struc::DrawStructureSupport(double worldMatrix[])
{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	for (int i = 0; i < partList.size(); i++)
	{
		pPart part = partList[i];
		part->DrawSupport();
	}
	glPopMatrix();
}

void Struc::DrawStructureGravityCone(double *worldMatrix)
{
	if(!contactGraph)
		return;

	vector<ContactGraph::SlopeSearchRegion> &gravity_cone = contactGraph->gravity_cone;
	if(gravity_cone.empty())
		return;

	shared_ptr<PolyMesh> slopeMesh;
	slopeMesh = std::make_shared<PolyMesh>();

	 EigenPoint gravity(0, -1, 0), vec;
    // EigenPoint gravity(0, -0.5, 0), vec;

	for(int id = 0; id < gravity_cone.size(); id++){

		pPolygon poly = make_shared<_Polygon>();
		poly->vers.push_back(Vector3f(0, varList.get<float>("ground_height"), 0));

		vec = gravity + gravity_cone[id].va * tan(gravity_cone[id].ta / 180 * M_PI);
		poly->vers.push_back(Vector3f(vec[0], vec[1] + varList.get<float>("ground_height"), vec[2]));

		vec = gravity + gravity_cone[id].vb * tan(gravity_cone[id].tb / 180 * M_PI);
		poly->vers.push_back(Vector3f(vec[0], vec[1] + varList.get<float>("ground_height"), vec[2]));

		slopeMesh->polyList.push_back(poly);
	}

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadMatrixd(worldMatrix);

	//Vector3f color = ColorMapping(0, M_PI/2, contactGraph->minimum_slope / 180 * M_PI);

    Vector3f color = Vector3f(0.612,  0.784,  0.953);

	float mtlAmbient[4] = { color.x, color.y, color.z, 0.4 };
	float mtlDiffuse[4] = { color.x, color.y, color.z, 0.4 };
	float mtlSpecular[4] = { color.x, color.y, color.z, 0.4 };
	float mtlEmission[4] = { color.x, color.y, color.z, 0.4 };

	glPushAttrib(GL_LIGHTING_BIT);
	    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mtlAmbient);
	    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mtlDiffuse);
	    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mtlSpecular);
	    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, mtlEmission);
	slopeMesh->DrawMesh();
	slopeMesh->DrawMesh_Wire(2.0, Vector3f(0.3, 0.3, 0.3));
	glPopAttrib();

	glPopMatrix();
}

void Struc::DrawContactEdgeNormal(double *worldMatrix)
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixd(worldMatrix);

    AveragePartSize();

    Vector3f red    = Vector3f(230, 184, 185) / 255.0f;
    Vector3f yellow = Vector3f(243, 226, 170) / 255.0f;
    Vector3f purple = Vector3f(192, 140, 233) / 255.0f;

    if(varList.get<bool>("showPickParts"))
    {
        map<int, bool> visited;
        for(int partID: pickPartIDs){
            visited[partID] = true;
        }

        for (int i = 0; i < contactList.size(); i++)
        {
            if(visited[contactList[i]->partIDA] == false) continue;
            if(visited[contactList[i]->partIDB] == false) continue;
            vector<Vector3f> contact_poly = contactList[i]->contactPoly;
            if(contact_poly.size() == 1)
            {
                Vector3f startPt = contact_poly[0];
                //Vector3f endPt = startPt + 0.2f * avgPartSize * contactList[i]->normal;
                Vector3f endPt = startPt + 0.13f * contactList[i]->normal;
                DrawLine(startPt, endPt, 8.0f, purple);
            }
        }
    }
    else{
        for (int i = 0; i < contactList.size(); i++)
        {
            vector<Vector3f> contact_poly = contactList[i]->contactPoly;
            if(contact_poly.size() == 1)
            {
                Vector3f startPt = contact_poly[0];
                Vector3f endPt = startPt + 0.2f * avgPartSize * contactList[i]->normal;
                DrawLine(startPt, endPt, 2.0f, purple);
            }
        }
    }

}

void Struc::DrawMultMovement(double *worldMatrix)
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadMatrixd(worldMatrix);


    Vector3f red    = Vector3f(230, 184, 185) / 255.0f;
    Vector3f yellow = Vector3f(243, 226, 170) / 255.0f;
    Vector3f purple = Vector3f(192, 140, 233) / 255.0f;

    for (int i = 0; i < parts.size(); i++) {
        pPart part = parts[i].lock();
        if (varList.get<bool>("mult_move") && part->atBoundary == false) {
            if (contactGraph && !contactGraph->translation.isZero()) {
                Eigen::VectorXd &translation = contactGraph->translation;
                int assemblyID = part->assemblyID;
                if (assemblyID != -1) {
                    Vector3f textLinkPt = part->polyMesh->ComputeExtremeVertex(part->cross.lock()->normal);
                    Vector3f move = Vector3f(translation[3 * assemblyID], translation[3 * assemblyID + 1],
                                  translation[3 * assemblyID + 2]) * 1.5f;
                    DrawLine(textLinkPt, textLinkPt + move, 5.0f, purple);
                }
            }
        }
    }
}




#endif