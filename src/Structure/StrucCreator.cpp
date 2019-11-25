///////////////////////////////////////////////////////////////
//
// StrucCreator.cpp
//
//   Create Topological Interlocking Structure
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 16/Oct/2018
//
//
///////////////////////////////////////////////////////////////
#include <cmath>
#include "Utility/Controls.h"
#include "../Utility/HelpDefine.h"
#include "../Utility/HelpStruct.h"
#include "../Utility/HelpFunc.h"
#include "../Utility/math3D.h"
#include "Mesh/Cross.h"
#include "Mesh/CrossMesh.h"
#include "CrossMesh/CrossMeshCreator.h"
#include "Mesh/PolyMesh.h"
#include "Part.h"
#include "PartGroup.h"
#include "BodyMobili.h"
#include "Struc.h"
#include "StrucCreator.h"
#include "IO/gluiVar.h"
#include "tbb/tbb.h"
#include "CrossMesh/AugmentedVectorCreator.h"
extern Vector3f colorTable[18];
extern gluiVarList varList;
extern vector<int> pickPartIDs;

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

StrucCreator::StrucCreator()
{
	refModel = nullptr;

	myStruc = nullptr;
}

StrucCreator::~StrucCreator()
{
	ClearStruc();
}

void StrucCreator::ClearStruc()
{
	myStruc.reset();
	refModel.reset();
}

StrucCreator::StrucCreator(const StrucCreator &_myStruc)
{
    ClearStruc();

    if(_myStruc.refModel)
        refModel = make_shared<CrossMeshCreator>(*_myStruc.refModel);

    if(_myStruc.myStruc)
    {
        myStruc = make_shared<Struc>(*_myStruc.myStruc);
        for(int id = 0; id < myStruc->partList.size(); id++)
        {
            shared_ptr<Part> part = myStruc->partList[id];
            if(_myStruc.myStruc->partList[id]->cross.lock())
            {
                part->cross = refModel->crossMesh->crossList[_myStruc.myStruc->partList[id]->cross.lock()->crossID];
                part->partGeom->UpdateCross(part->cross.lock());
            }
        }
        UpdateStructureGeometry(false);
    }
}

//**************************************************************************************//
//                                  Create Structure
//**************************************************************************************//

bool StrucCreator::LoadSurface(const char * objFileName)
{
	ClearStruc();
	refModel = make_shared<CrossMeshCreator>();
	if(!refModel->LoadReferenceSurface(objFileName))
	{
		return false;
	}
	return true;
}

int StrucCreator::CreateStructure(bool createCrossMesh,
								  bool texturedModel,
								  double interactMatrix[],
								  bool previewMode)
{

	float tiltAngle = varList.get<float>("tiltAngle");
	float patternID = varList.get<int>("patternID");
	float patternRadius = varList.get<int>("patternRadius");
	float cutUpper = varList.get<float>("cutUpper");
	float cutLower = varList.get<float>("cutLower");
    bool lockAngle = varList.get<bool>("lockTiltAngle");

	tbb::tick_count sta = tbb::tick_count::now();
	if (refModel == nullptr) return 0;

	myStruc.reset();

	////////////////////////////////////////////////////////////////
	// 1. Create a cross mesh from a polygonal mesh
	if ( createCrossMesh && !lockAngle)
	{
		refModel->CreateCrossMesh(texturedModel, tiltAngle, patternID, patternRadius, previewMode, interactMatrix);
	}
	pCrossMesh crossMesh = refModel->crossMesh;
	std::cout << "Compute Cross Mesh:\t" << (tbb::tick_count::now() - sta).seconds() << std::endl;

	////////////////////////////////////////////////////////////////
	// 2. Construct a part (i.e., convex polyhedron) from each 3D polygon

	sta = tbb::tick_count::now();
	myStruc = make_shared<Struc>();

	for (int i = 0; i < crossMesh->crossList.size(); i++)
	{
		pCross cross = crossMesh->crossList[i];
		pPart part = make_shared<Part>(cross);
		part->partID = myStruc->partList.size();
		if (part->CheckLegalGeometry())
		{
			myStruc->partList.push_back(part);
		}
		else
		{
			cout<<"Warning: the tilt normals will not compose legal geometry. Please adjust them"<<endl;
			return 0;
		}
	}

	std::cout << "Check Legal Geometry:\t" << (tbb::tick_count::now() - sta).seconds() << std::endl;

	sta = tbb::tick_count::now();
	tbb::parallel_for(tbb::blocked_range<size_t>(0, myStruc->partList.size()), [&](const tbb::blocked_range<size_t>& r)
	{
		for (size_t id = r.begin(); id != r.end(); ++id)
		{
			shared_ptr<Part> part = myStruc->partList[id];
			part->atBoundary = part->cross.lock()->atBoundary;
			part->ComputePartGeometry( true, Vector2f(cutUpper, cutLower), previewMode);
			if( !previewMode )
			{
				part->polyMesh->UpdateVertices();
				part->Compute3DTextPosition();
			}
		}
	});


//	for (size_t id = 0; id < myStruc->partList.size(); ++id)
//		{
//			shared_ptr<Part> part = myStruc->partList[id];
//			part->atBoundary = part->cross.lock()->atBoundary;
//			part->ComputePartGeometry( true, Vector2f(cutUpper, cutLower), previewMode);
//			if( !previewMode )
//			{
//				part->polyMesh->UpdateVertices();
//				part->Compute3DTextPosition();
//			}
//		}

    if(!previewMode && varList.get<bool>("penetration_check")){
        if(ComputePartPenetration()){
            myStruc.reset();
            return 0;
        }
    }


	std::cout << "Compute Part Geometry:\t" << (tbb::tick_count::now() - sta).seconds() << std::endl;
    std::cout << "Number of Parts:\t" << myStruc->partList.size() << std::endl << std::endl;
	//3. Compute the auxiliary data
	if( !previewMode )
	{
		myStruc->AveragePartSize();
        BuildPartsGraph();
		IdentifyBoundaryParts_Disk();
		myStruc->ComputeGroundY(true);
	}
	return 1;
}

int StrucCreator::UpdateStructureGeometry(bool previewMode)
{
	float cutUpper = varList.get<float>("cutUpper");
	float cutLower = varList.get<float>("cutLower");

	////////////////////////////////////////////////////////////////
	// 2. Construct a part (i.e., convex polyhedron) from each 3D polygon

	pCrossMesh crossMesh = refModel->crossMesh;

	myStruc = make_shared<Struc>();

	// TODO: double check these codes
	for (int i = 0; i < crossMesh->crossList.size(); i++)
	{
		pCross cross = crossMesh->crossList[i];
		pPart part = make_shared<Part>(cross);
		part->partID = myStruc->partList.size();

		if (part->CheckLegalGeometry(true))
		{
			myStruc->partList.push_back(part);
		}
		else
		{
			cout<<"Warning: the tilt normals will not compose legal geometry. Please adjust them"<<endl;
			return 0;
		}
	}

	vector<float> timer;
	timer.resize(6, 0);
	for (int i = 0; i < myStruc->partList.size(); i++)
	{
		shared_ptr<Part> part = myStruc->partList[i];

		part->ComputePartGeometry( true, Vector2f(cutUpper, cutLower), previewMode);

		if( !previewMode )
		{
			part->polyMesh->UpdateVertices();
			part->Compute3DTextPosition();
		}
	}

	if( !previewMode )
	{
		myStruc->AveragePartSize();
		BuildPartsGraph();
		IdentifyBoundaryParts_Disk();
        ComputePartContacts();
	}

	return 1;
}


//**************************************************************************************//
//                              Compute Part Contacts
//**************************************************************************************//

void StrucCreator::BuildPartsGraph()
{
	if (myStruc == NULL)
		return;

	CrossMesh *crossMesh = refModel->crossMesh.get();

	for (int i = 0; i < myStruc->partList.size(); i++)
	{
		pPart part = myStruc->partList[i];
		if(part == nullptr) continue;
        part->initNeighbors.clear();

		int partID = part->partID;

		Cross *cross = crossMesh->crossList[partID].get();

		for (int j = 0; j < cross->neighbors.size(); j++)
		{
			Cross *neiborCross = cross->neighbors[j].lock().get();

			if (neiborCross == NULL)
			{
				part->initNeighbors.push_back(pPart());
			}
			else
			{
				int neiborCrossID = neiborCross->crossID;
				if(neiborCrossID >= myStruc->partList.size())
                    continue;
				part->initNeighbors.push_back(myStruc->partList[neiborCrossID]);
			}
		}
	}

	for (int i = 0; i < myStruc->partList.size(); i++)
	{
		pPart part = myStruc->partList[i];
        if(part == nullptr) continue;
		part->currNeighbors = part->initNeighbors;
	}
}

void StrucCreator::IdentifyBoundaryParts_Disk()
{
	if (myStruc == NULL)
		return;

	myStruc->IdentifyBoundaryParts_Disk();
}

void StrucCreator::ComputePartContacts()
{
    myStruc->contactList.clear();
    myStruc->innerContactList.clear();

    if(varList.get<bool>("faceface_contact")) ComputePartFaceContacts();
    if(varList.get<bool>("edgeedge_contact")) ComputePartEdgeContacts();
}


void StrucCreator::ComputePartFaceContactsBruteForce()
{
    if (myStruc == nullptr) return;
    myStruc->ComputePartFaceContactsBruteForce();
}


void StrucCreator::ComputePartFaceContacts()
{
	if (myStruc == nullptr) return;
    myStruc->ComputePartFaceContacts();
}

void StrucCreator::ComputePartEdgeContacts()
{
	if (refModel == nullptr || myStruc == nullptr || refModel->crossMesh == nullptr) return;

	refModel->crossMesh->UpdateCrossVertexIndex();
	myStruc->ComputePartEdgeContact(refModel->crossMesh->vertexCrossList);
}

bool StrucCreator::ComputePartPenetration() {
    if (refModel == nullptr || myStruc == nullptr || refModel->crossMesh == nullptr)
        return false;

    refModel->crossMesh->UpdateCrossVertexIndex();
    return myStruc->ComputePartPenetration(refModel->crossMesh->vertexCrossList);
}

//**************************************************************************************//
//                                   Save OBJ Models
//**************************************************************************************//

void StrucCreator::WriteStructure(const char *folderPath)
{
	if (myStruc == NULL)
		return;

	myStruc->WriteStructure(folderPath);
}

void StrucCreator::WriteCrossMesh(char *meshFileName)
{
	if( refModel == NULL || refModel->crossMesh == NULL )
		return;

	refModel->crossMesh->WriteOBJModel( meshFileName );
}