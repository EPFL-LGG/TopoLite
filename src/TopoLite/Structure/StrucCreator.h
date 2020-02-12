///////////////////////////////////////////////////////////////
//
// StrucCreator.h
//
//   Create Topological Interlocking Structure
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 16/Oct/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _STRUC_CREATOR_H
#define _STRUC_CREATOR_H


#include <vector>
#include "TopoLite/Structure/Part.h"
#include "TopoLite/Utility/vec.h"
#include "TopoLite/Utility/GeometricPrimitives.h"
#include "TopoLite/Utility/TopoObject.h"
#include "TopoLite/Structure/Struc.h"
#include "TopoLite/CrossMesh/CrossMeshCreator.h"

using namespace std;

using pCrossMeshCreator = shared_ptr<CrossMeshCreator> ;
using pStruc = shared_ptr<Struc> ;

class StrucCreator : public TopoObject
{

public:

    pCrossMeshCreator   crossMeshCreator;             // Reference surface model
	pStruc              struc;                        // Designed interlocking structure
	double              worldAxesMat[16];
	double              worldMatrix[16];

public:

    StrucCreator(shared_ptr<InputVarList> var);
    ~StrucCreator();
	StrucCreator(const StrucCreator &_myStruc);
	void ClearStruc();

public:
    // Create Structure

	bool LoadSurface(const char* objFileName);
    int CreateStructure(bool createCrossMesh,
                        double interactMatrix[],
                        bool previewMode);
    int UpdateStructureGeometry(bool previewMode);

public:
    // Compute Part Contacts

    void BuildPartsGraph();
	void ComputeBoundaryParts();
	void ComputePartContacts();
	void ComputePartFaceContacts();
    void ComputePartFaceContactsBruteForce();
	void ComputePartEdgeContacts();

public:
    // Save Output Models

	void WriteStructure(const char *folderPath);
    void WriteCrossMesh(char *meshFileName);
};

#endif