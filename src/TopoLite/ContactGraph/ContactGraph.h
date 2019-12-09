//
// Created by ziqwang on 14.01.19.
//

#ifndef TOPOLOCKCREATOR_CONTACTGRAPH_H
#define TOPOLOCKCREATOR_CONTACTGRAPH_H

#include "ContactGraphNode.h"
#include "PolyhedralCone.h"

#include "Utility/TopoObject.h"

#include <Eigen/SparseCore>
#include <string>
#include <map>

using std::string;
using std::map;

typedef Eigen::SparseMatrix<double> EigenSpMat;

typedef Eigen::Triplet<double> EigenTriple;

typedef shared_ptr<ContactGraphNode> pContactGraphNode;
typedef shared_ptr<ContactGraphEdge> pContactGraphEdge;

typedef weak_ptr<ContactGraphEdge> wpContactGraphEdge;
typedef weak_ptr<ContactGraphNode> wpContactGraphNode;

/*!
 * \brief An graph describe an rigid body system
 */

struct ContactGraphAssemblySequence
{
    vector<weak_ptr<ContactGraphNode>> orderParts;
    vector<PolyhedralCone> orderCones;
    map<ContactGraphNode *, bool> splitParts;
    map<ContactGraphNode *, bool> boundaryParts;

    map<ContactGraphNode *, int> partLayers;
    vector<vector<weak_ptr<ContactGraphNode>>> layerParts;
};

struct SlopeSearchRegion
{
    EigenPoint va, vb; //direction
    double ta, tb; //tilt angle

    EigenPoint vc_;
    double mu_, minimum_, tc_;

    double minimum_possible_tiltAngle();
};

class ContactGraph : public TopoObject{
public:

    Eigen::VectorXd force; //compression & tension force of the structure

    Eigen::VectorXd translation; //multiple direction movement

    vector<SlopeSearchRegion> gravity_cone; //equilibrium cone of gravity

    double minimum_slope;

    bool printEquilibrium;

public:

    vector<shared_ptr<ContactGraphNode>> nodes;

    vector<shared_ptr<ContactGraphEdge>> edges;

public: //automatic generate

    vector<weak_ptr<ContactGraphNode>> dynamic_nodes;


public:

    ContactGraph(shared_ptr<InputVarList> varList);

    ~ContactGraph();

    /*************************************************
     *                  Basic Operation
     *************************************************/

public:

    void addNode(pContactGraphNode _node);

    void addContact(pContactGraphNode _nodeA, pContactGraphNode _nodeB, pContactGraphEdge _edge);

    void initialize();

public:

    /*************************************************
     *             Property Verification
     *************************************************/

public:

    void computeTranslationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size);

    void computeRotationalInterlockingMatrix(vector<EigenTriple> &tri, Eigen::Vector2i &size);

    void get_force_from_norm_fric(Eigen::Vector3d    n,    Eigen::Vector3d   u,  Eigen::Vector3d   v,
                                  Eigen::RowVector2d    &fkx, Eigen::RowVector2d &fky, Eigen::RowVector2d &fkz);

    void get_force_from_norm_fric(Eigen::Vector3d    n,    Eigen::Vector3d   u,  Eigen::Vector3d   v,
                                  Eigen::RowVector4d    &fkx, Eigen::RowVector4d &fky, Eigen::RowVector4d &fkz);

    void get_moment_from_norm_fric_vertex(Eigen::Vector3d n,   Eigen::Vector3d u,   Eigen::Vector3d v, Eigen::Vector3d r,
                                          Eigen::RowVector2d &mx, Eigen::RowVector2d &my, Eigen::RowVector2d &mz);

    void get_moment_from_norm_fric_vertex(Eigen::Vector3d n,   Eigen::Vector3d u,   Eigen::Vector3d v, Eigen::Vector3d r,
                                          Eigen::RowVector4d &mx, Eigen::RowVector4d &my, Eigen::RowVector4d &mz);

    void get_A_j_k(int partID, int edgeID, Eigen::MatrixXd &Ajk, bool withFriction = false);

    void computeEquilibriumMatrix(Eigen::MatrixXd &mat, bool withFriction = false);

    double computeEquilibriumMatrixConditonalNumber();

    virtual bool isTranslationalInterlocking(string &log){ return  true;}

    virtual bool isRotationalInterlocking(string &log){ return  true;}

    virtual bool isEquilibrium(EigenPoint gravity, string &log, bool saveForce){ return  true;}

    virtual double computeSlope(EigenPoint gravity, EigenPoint laterial, double init_angle){return 0.0;}

    virtual double computeSlope(EigenPoint gravity){return 0.0;}

    virtual double computeSlopeUniform(EigenPoint gravity, int n_sample){return 0.0;}

    virtual std::tuple<EigenPoint, EigenPoint> computeSlopeCoordinateSystem(EigenPoint gravity){return std::tuple<EigenPoint, EigenPoint>();}


    /*************************************************
     *              Assembling Sequence
     *************************************************/

public:

    void computeAssemblingSequence(ContactGraphAssemblySequence &sequence, const vector<int> &nonBoundaryParts, int numLayers);

    void computeAsssemblingSequenceByRemovingBoundary(ContactGraphAssemblySequence &sequence, const vector<int> &boundary, int numLayers);

    double computePartMobility(shared_ptr<ContactGraphNode> part, map<ContactGraphNode *, bool> &removeParts, PolyhedralCone &cone);

    void clusterAssemblingLayers(ContactGraphAssemblySequence &sequence, int numLayers);
};


#endif //TOPOLOCKCREATOR_CONTACTGRAPH_H
