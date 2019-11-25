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
#include "Utility/vec.h"

#include "PartGeom.h"
#include "PartGroup.h"
#include "Utility/TopoObject.h"

using namespace std;

enum ContactType{
	Contact_FaceFace,
	Contact_FaceEdge,
	Contact_EdgeEdge,
};

struct Contact
{
	int                     partIDA;                    // PartA with a small ID
	int                     partIDB;                    // PartB with a large ID
	int                     assemblyPartIDA;            // PartA new ID without counting boundary parts
	int                     assemblyPartIDB;            // PartB new ID without counting boundary parts
	Vector3f                contactNormal;              // Normal of partA's contacting face
	vector<Vector3f>        contactPoly;                // Contacting region between partA and partB
    double                  contactArea = 0;
    ContactType             contactType;
    weak_ptr<OrientPoint>   contactOript;	            //A to B, orient point of A
	bool boundaryContact;                               // Contact between boundary parts


	Contact(int _partIDA,
			int _partIDB,
			Vector3f _normal,
			weak_ptr<OrientPoint> _oript,
			vector<Vector3f> _contactPoly,
			bool _boundaryContact)
	{
		partIDA      = _partIDA;
		partIDB      = _partIDB;
		contactNormal       = _normal;
		contactOript		 = _oript;
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
        contactNormal       = _normal;
        contactPoly.push_back(_contactPoly);
        boundaryContact = _boundaryContact;
        contactType = Contact_EdgeEdge;
    }

public:
	void get_norm_fric_for_block(int AssemblyPartID, Vector3f &_normal, Vector3f &_ufric, Vector3f &_vfric)
	{
		_normal = contactNormal; _normal /= len(_normal);
		_ufric = contactPoly[1] - contactPoly[0]; _ufric /= len(_ufric);
		_vfric = contactNormal CROSS _ufric; _vfric /= len(_vfric);

		if(AssemblyPartID == assemblyPartIDA){
			_normal = _normal * (-1.0f);
			_ufric = _ufric * (-1.0f);
			_vfric = _normal CROSS _ufric;
		}
	}
};

using pPart     =  shared_ptr<Part>;
using wpPart    =  weak_ptr<Part>;
using pContact  =  shared_ptr<Contact>;

class Struc : public TopoObject
{

public:

    vector<pPart>               partList;               // Parts in the structure
	vector<pContact>            contactList;            // Contact between parts
	vector<weak_ptr<Part>>      parts; 	                // part not in the boundary;
	vector<weak_ptr<Part>>      boundarys;              // part in the boundary
	vector<weak_ptr<Contact>>   innerContactList;       //Contact except boundary and boundary contact

public:

    int         numEdgeEdgeContacts;
    int         numFaceFaceContacts;
    double      minContactArea;
    double      avgContactArea;
    float       avgPartSize;                 // Average part size
    float       groundPlaneY;                     // Z-coordinate of the ground

public:

	Struc(shared_ptr<InputVarList> var);
	Struc(const Struc& _struc);
	~Struc();
	void ClearStruc();

public:
    // Compute Part Contacts

    void ComputePartContactArea();
    void ComputePartFaceContacts();
    void ComputePartFaceContactsBruteForce();
    void ComputePartEdgeContact(vector<vector<weak_ptr<Cross>>> &vertexCrossList);

public:
    // Compute Part Basic Info
    double ComputeAveragePartSize();
    double ComputeLowestY();
    bool CheckPartsDeadLock(int partID);

	void ComputeBoundaryParts();
	void ComputeGroundY(bool showGround);
	void ComputeTouchGroundParts();
	void ResetTouchGround();
	void RemoveParts(vector<int> partIDs);

	// Save Output Models
	void WriteStructure(const char *folderPath);
	void WriteStructureWireFrame(const char *folderPath);
	void WritePartGraph(char *folderPath);

private:

    void ComputePolygonsIntersection(   const vector<Vector3f> &polyA,
                                        const vector<Vector3f> &polyB,
                                        vector<Vector3f> &polyInt);

    void ProjectPolygonTo3D(            const vector<Vector3f> &poly,
                                        double projMat[],
                                        vector<Vector3f> &poly3D);
};

#endif