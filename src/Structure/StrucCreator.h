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
#include "Structure/Part.h"
#include "Utility/vec.h"
#include "Utility/Controls.h"
#include "Utility/HelpStruct.h"

using namespace std;

class CrossMeshCreator;
class Struc;
typedef shared_ptr<CrossMeshCreator> pModel;
typedef shared_ptr<Struc> pStruc;

class StrucCreator
{

public:

	pModel refModel;                   // Reference surface model

	pStruc myStruc;                    // Designed interlocking structure

public:


    StrucCreator();
    ~StrucCreator();
	StrucCreator(const StrucCreator &_myStruc);
	void ClearStruc();

public: // Create Structure
	bool LoadSurface(const char* objFileName);

    int CreateStructure(bool createCrossMesh,
                        bool texturedModel,
                        double interactMatrix[],
                        bool previewMode);

    int UpdateStructureGeometry(bool previewMode);

public: // Compute Part Contacts
    void BuildPartsGraph();
	void IdentifyBoundaryParts_Disk();
	void ComputePartContacts();
	void ComputePartFaceContacts();
    void ComputePartFaceContactsBruteForce();
	void ComputePartEdgeContacts();
	bool ComputePartPenetration();


public: // Save Output Models

	void WriteStructure(const char *folderPath);
    void WriteCrossMesh(char *meshFileName);
};

#endif