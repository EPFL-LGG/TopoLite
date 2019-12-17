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
#include <tbb/tbb.h>

#include "Utility/HelpDefine.h"
#include "Utility/HelpStruct.h"
#include "Utility/HelpFunc.h"
#include "Utility/math3D.h"

#include "Mesh/Cross.h"
#include "Mesh/CrossMesh.h"
#include "Mesh/PolyMesh.h"

#include "CrossMesh/CrossMeshCreator.h"
#include "CrossMesh/AugmentedVectorCreator.h"

#include "Part.h"
#include "PartGroup.h"
#include "Struc.h"
#include "StrucCreator.h"
#include "IO/InputVar.h"

//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

StrucCreator::StrucCreator(shared_ptr<InputVarList> var): TopoObject(var)
{
    crossMeshCreator = nullptr;

	struc = nullptr;
}

StrucCreator::~StrucCreator()
{
	ClearStruc();
}

void StrucCreator::ClearStruc()
{
	struc.reset();

    crossMeshCreator.reset();
}

StrucCreator::StrucCreator(const StrucCreator &_myStruc) : TopoObject(_myStruc)
{
    ClearStruc();

    if(_myStruc.crossMeshCreator)
        crossMeshCreator = make_shared<CrossMeshCreator>(*_myStruc.crossMeshCreator);

    if(_myStruc.struc)
    {
        struc = make_shared<Struc>(*_myStruc.struc);
        for(int id = 0; id < struc->partList.size(); id++)
        {
            shared_ptr<Part> part = struc->partList[id];
            if(_myStruc.struc->partList[id]->cross.lock())
            {
                part->cross = crossMeshCreator->crossMesh->crossList[_myStruc.struc->partList[id]->cross.lock()->crossID];
                part->partGeom->ParseCrossData(part->cross.lock());
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
	crossMeshCreator = make_shared<CrossMeshCreator>(getVarList());
	if(!crossMeshCreator->LoadReferenceSurface(objFileName))
	{
		return false;
	}
	return true;
}

int StrucCreator::CreateStructure(  bool createCrossMesh,
                                    double interactMatrix[],
								    bool previewMode)
{

    float   cutUpper        = getVarList()->get<float>("cutUpper");
    float   cutLower        = getVarList()->get<float>("cutLower");
    bool    lockAngle       = getVarList()->get<bool>("lockTiltAngle");

	tbb::tick_count sta = tbb::tick_count::now();
	if (crossMeshCreator == nullptr) return 0;

	struc.reset();

	////////////////////////////////////////////////////////////////
	// 1. Create a cross mesh from a polygonal mesh
	if(createCrossMesh){
        crossMeshCreator->CreateCrossMesh(previewMode, interactMatrix);
	}

	pCrossMesh crossMesh = crossMeshCreator->crossMesh;
	if(crossMesh == nullptr) return 0;
	std::cout << "Compute Cross Mesh:\t" << (tbb::tick_count::now() - sta).seconds() << std::endl;

	////////////////////////////////////////////////////////////////
	// 2. Construct a part (i.e., convex polyhedron) from each 3D polygon

	sta = tbb::tick_count::now();
	struc = make_shared<Struc>(getVarList());

	for (int i = 0; i < crossMesh->crossList.size(); i++)
	{
		pCross cross = crossMesh->crossList[i];
		pPart part = make_shared<Part>(cross, getVarList());
		part->partID = struc->partList.size();
		if (part->CheckLegalGeometry())
		{
			struc->partList.push_back(part);
		}
		else
		{
			cout<<"Warning: the tilt normals will not compose legal geometry. Please adjust them"<<endl;
			return 0;
		}
	}

	std::cout << "Check Legal Geometry:\t" << (tbb::tick_count::now() - sta).seconds() << std::endl;

	sta = tbb::tick_count::now();
	tbb::parallel_for(tbb::blocked_range<size_t>(0, struc->partList.size()), [&](const tbb::blocked_range<size_t>& r)
	{
		for (size_t id = r.begin(); id != r.end(); ++id)
		{
			shared_ptr<Part> part = struc->partList[id];
			part->atBoundary = part->cross.lock()->atBoundary;
			part->ComputePartGeometry( true, Vector2f(cutUpper, cutLower), previewMode);
			if( !previewMode )
			{
                part->polyMesh->removeDuplicatedVertices();
				part->Compute3DTextPosition();
			}
		}
	});

	std::cout << "Compute Part Geometry:\t" << (tbb::tick_count::now() - sta).seconds() << std::endl;
    std::cout << "Number of Parts:\t" << struc->partList.size() << std::endl << std::endl;

    //3. Compute the auxiliary data
	if( !previewMode )
	{
        struc->ComputeAveragePartSize();
        BuildPartsGraph();
        ComputeBoundaryParts();
        ComputePartContacts();
        struc->ComputeGroundY(true);
	}
	return 1;
}

int StrucCreator::UpdateStructureGeometry(bool previewMode)
{
	float cutUpper = getVarList()->get<float>("cutUpper");
	float cutLower = getVarList()->get<float>("cutLower");

	////////////////////////////////////////////////////////////////
	// 2. Construct a part (i.e., convex polyhedron) from each 3D polygon

	pCrossMesh crossMesh = crossMeshCreator->crossMesh;

	struc = make_shared<Struc>(getVarList());

	// TODO: double check these codes
	for (int i = 0; i < crossMesh->crossList.size(); i++)
	{
		pCross cross = crossMesh->crossList[i];
		pPart part = make_shared<Part>(cross, getVarList());
		part->partID = struc->partList.size();

		if (part->CheckLegalGeometry(true))
		{
			struc->partList.push_back(part);
		}
		else
		{
			cout<<"Warning: the tilt normals will not compose legal geometry. Please adjust them"<<endl;
			return 0;
		}
	}

	vector<float> timer;
	timer.resize(6, 0);
	for (int i = 0; i < struc->partList.size(); i++)
	{
		shared_ptr<Part> part = struc->partList[i];

		part->ComputePartGeometry( true, Vector2f(cutUpper, cutLower), previewMode);

		if( !previewMode )
		{
            part->polyMesh->removeDuplicatedVertices();
			part->Compute3DTextPosition();
		}
	}

	if( !previewMode )
	{
        struc->ComputeAveragePartSize();
		BuildPartsGraph();
        ComputeBoundaryParts();
        ComputePartContacts();
	}

	return 1;
}


//**************************************************************************************//
//                              Compute Part Contacts
//**************************************************************************************//

void StrucCreator::BuildPartsGraph()
{
	if (struc == NULL)
		return;

	CrossMesh *crossMesh = crossMeshCreator->crossMesh.get();
	for (int i = 0; i < struc->partList.size(); i++)
	{
		pPart part = struc->partList[i];
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
				if(neiborCrossID >= struc->partList.size())
                    continue;
				part->initNeighbors.push_back(struc->partList[neiborCrossID]);
			}
		}
	}

	for (int i = 0; i < struc->partList.size(); i++)
	{
		pPart part = struc->partList[i];
        if(part == nullptr) continue;
	}
}

void StrucCreator::ComputeBoundaryParts()
{
	if (struc == NULL)
		return;

    struc->ComputeBoundaryParts();
}

void StrucCreator::ComputePartContacts()
{
    struc->contactList.clear();
    struc->innerContactList.clear();

    if(getVarList()->get<bool>("faceface_contact")) ComputePartFaceContacts();
    if(getVarList()->get<bool>("edgeedge_contact")) ComputePartEdgeContacts();
}

void StrucCreator::ComputePartFaceContactsBruteForce()
{
    if (struc == nullptr) return;
    struc->ComputePartFaceContactsBruteForce();
}

void StrucCreator::ComputePartFaceContacts()
{
	if (struc == nullptr) return;
    struc->ComputePartFaceContacts();
}

void StrucCreator::ComputePartEdgeContacts()
{
	if (crossMeshCreator == nullptr || struc == nullptr || crossMeshCreator->crossMesh == nullptr) return;

	crossMeshCreator->crossMesh->UpdateCrossVertexIndex();
	struc->ComputePartEdgeContact(crossMeshCreator->crossMesh->vertexCrossList);
}


//**************************************************************************************//
//                                   Save OBJ Models
//**************************************************************************************//

void StrucCreator::WriteStructure(const char *folderPath)
{
	if (struc == NULL)
		return;

	struc->WriteStructure(folderPath);
}

void StrucCreator::WriteCrossMesh(char *meshFileName)
{
	if( crossMeshCreator == NULL || crossMeshCreator->crossMesh == NULL )
		return;

	crossMeshCreator->crossMesh->WriteOBJModel( meshFileName );
}