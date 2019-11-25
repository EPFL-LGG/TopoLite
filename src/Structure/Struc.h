///////////////////////////////////////////////////////////////
//
// Struc.h
//
//   Topological Interlocking Structure
//
// by Peng SONG  ( songpenghit@gmail.com )
// 
// 16/Oct/2018
//
//
///////////////////////////////////////////////////////////////


#ifndef _STRUC_H
#define _STRUC_H


#include <vector>
#include "Utility/Controls.h"
#include "../Utility/vec.h"
#include "PartGeom.h"

using namespace std;

struct PartMove 
{
	int partID;
	Vector3f transVec;

	PartMove()
	{
		partID = NONE_PART;
		transVec = Vector3f(0, 0, 0);
	};
};

enum ContactType{
	Contact_FaceFace,
	Contact_FaceEdge,
	Contact_EdgeEdge,
};

struct Contact
{
	int partIDA;                     // PartA with a small ID
	int partIDB;                     // PartB with a large ID

	int assemblyPartIDA;             // PartA new ID without counting boundary parts
	int assemblyPartIDB;             // PartB new ID without counting boundary parts

	Vector3f normal;                 // Normal of partA's contacting face
	vector<Vector3f> contactPoly;    // Contacting region between partA and partB
	weak_ptr<OrientPoint> oript;	 //A to B, orient point of A
	double area = 0;

	bool boundaryContact;            // Contact between boundary parts
	ContactType  contactType;

    vector<Vector3f> versDiff;

	Contact(int _partIDA,
			int _partIDB,
			Vector3f _normal,
			weak_ptr<OrientPoint> _oript,
			vector<Vector3f> _contactPoly,
			bool _boundaryContact)
	{
		partIDA      = _partIDA;
		partIDB      = _partIDB;
		normal       = _normal;
		oript		 = _oript;
		contactPoly  = _contactPoly;
		boundaryContact = _boundaryContact;
		contactType = Contact_FaceFace;
	};

    Contact(int _partIDA,
            int _partIDB,
            Vector3f _normal,
            Vector3f _contactPoly,
            bool _boundaryContact)
    {
        partIDA      = _partIDA;
        partIDB      = _partIDB;
        normal       = _normal;
        contactPoly.push_back(_contactPoly);
        boundaryContact = _boundaryContact;
        contactType = Contact_EdgeEdge;
    }
public:
	void get_norm_fric_for_block(int AssemblyPartID, Vector3f &_normal, Vector3f &_ufric, Vector3f &_vfric)
	{
		_normal = normal; _normal /= len(_normal);
		_ufric = contactPoly[1] - contactPoly[0]; _ufric /= len(_ufric);
		_vfric = normal CROSS _ufric; _vfric /= len(_vfric);

		if(AssemblyPartID == assemblyPartIDA){
			_normal = _normal * (-1.0f);
			_ufric = _ufric * (-1.0f);
			_vfric = _normal CROSS _ufric;
		}
	}
};

typedef shared_ptr<Part> pPart;
typedef weak_ptr<Part> wpPart;
typedef shared_ptr<PartGroup> pPartGroup;
typedef shared_ptr<Contact> pContact;


class Struc 
{
public: //raw data
    vector<pPart> partList;            // Parts in the structure

public:     //generated data

	vector<pContact> contactList;      // Contact between parts

	vector<weak_ptr<Part>> parts; 	   // part not in the boundary;

	vector<weak_ptr<Part>> boundarys;  // part in the boundary

	vector<weak_ptr<Contact>> innerContactList; //Contact except boundary and boundary contact

public:

    int num_edge_edge;
    int num_face_face;

    double minimum_contact_surface;

    double average_face_area;

    float avgPartSize;                 // Average part size

    float groundY;                     // Z-coordinate of the ground

public:

	vector<vector<int>> mergePartGroups;

	vector<int> keyPartsID;

	vector<PartMove> partMoveList;     // Structure disassembly sequence

	pPartGroup partGroup;              // TODO: remove this later

public:
	Struc();
	Struc(const Struc& _struc);
	~Struc();
	void ClearStruc();
	void ClearPartGroup();

	// Identify Parts (Disk Model)
	void IdentifyBoundaryParts_Disk();
	bool IsLockedByBoundary(int partID);
	void ComputeGroundY(bool showGround);
	double ComputeGroundHeight();
	void IdentifyTouchGroundParts_Disk();
	void ResetTouchGround();
	void RemoveParts(vector<int> partIDs);
	void AveragePartSize();

	// Compute Part Contacts
	void ComputePartContactArea();
	void ComputePartFaceContacts();
    bool ComputePartCSG(int partID, int partJD);
    bool ComputePartPenetration(vector<vector<weak_ptr<Cross>>> &vertexCrossList);
	void ComputePartFaceContactsBruteForce();
	void ComputePartEdgeContact(vector<vector<weak_ptr<Cross>>> &vertexCrossList);
	void ComputePartCutHeight(vector<Vector2f> &heights);
	bool check_same_poly(vector<Vector3f> poly_A, vector<Vector3f> poly_B);
	vector<Vector3f> MergePoly(vector<Vector3f> poly_A, vector<Vector3f> poly_B);

	// Part Graph
	void ResetPartsGraph();
	void UpdatePartsGraph();

	// Save Output Models
	void WriteStructure(const char *folderPath);
	void WriteStructureForSimulation(const char *folderPath);
	void WriteStructureWireFrame(const char *folderPath);
	void WritePartGraph(char *folderPath);
	void WriteDisassemFiles(char *folderPath, vector<int> doorPartIDs);
	void WriteMotionFile(char *folderPath);

	// Test Functions
	void Function_Test();
	void Function_PartGroup(vector<int> &pickPartIDs);
	void Function_Mobility(vector<int> groupPartIDs);
	void Function_Assembly();

#if USE_OPENGL_DRAW
public:

	// Draw TI Construction
	void DrawOriPoints(double worldMatrix[]);
	void DrawInnerPolygons(double worldMatrix[]);
	void DrawGeomFaces(double worldMatrix[]);
	void DrawGeomEdges(double worldMatrix[]);
	void DrawGeomVertices(double worldMatrix[]);

	// Draw Part Group Mobility
	void DrawGroupMobiliFaces(double worldMatrix[]);
	void DrawGroupMobiliEdges(double worldMatrix[]);
	void DrawGroupMobiliVertices(double worldMatrix[]);
	void DrawGroupMobiliRays(double worldMatrix[]);
	void DrawGroupMobiliVectors(double worldMatrix[]);
	void DrawGroupMobiliMeshes(double worldMatrix[]);

	// Draw TI Structure
	void DrawStructure(int mode, double worldMatrix[]);
	void DrawStructureWire(double worldMatrix[]);
	void DrawParts(int mode, double *worldMatrix, const vector<weak_ptr<Part>> &_parts);
	void DrawPartsWire(double *worldMatrix, const vector<weak_ptr<Part>> &_parts);
	void DrawStructure3DText(double worldMatrix[]);
	void DrawStructureNormals(double worldMatrix[]);
	void DrawStructureBBox(double worldMatrix[]);
	void DrawStructureCentroid(double worldMatrix[]);
	void DrawStructureGraph(double worldMatrix[]);

	// Draw Picked Part(s)
	void DrawPickPart(int pickPartID, double worldMatrix[]);
	void DrawPickPartGroup(vector<int> pickPartIDs, double worldMatrix[]);
	void DrawDebug_PartGroup(double worldMatrix[]);

	// Draw Equilibrium
	void DrawStructureContacts(double worldMatrix[]);
	void DrawStructureForces(double worldMatrix[]);
	void DrawContactEdgeNormal(double worldMatrix[]);
	void DrawMultMovement(double worldMatrix[]);

	void DrawStructureGravityCone(double worldMatrix[]);
	void DrawStructureSupport(double worldMatrix[]);


#endif
};

#endif