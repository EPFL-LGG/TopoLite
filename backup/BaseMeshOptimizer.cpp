//
// Created by ziqwang on 2019-11-25.
//

#include "BaseMeshOptimizer.h"
#include <libShapeOp/src/Solver.h>
#include <libShapeOp/src/Constraint.h>
#include <libShapeOp/src/Common.h>
#include "CrossConstraint.h"

void BaseMeshOptimizer::OptimizeBaseMesh(shared_ptr<CrossMesh> crossMesh)
{
    if(aabbTree.lock() == nullptr) return;

    int iterator = getVarList()->get<int>("shapeop_iterator");
    if(iterator <= 0)
        return;

    int n = crossMesh->vertexList.size();
    int m = crossMesh->crossList.size();
    ShapeOp::Matrix3X position(3, n);
    for(int id = 0; id < n ; id++){
        Vector3f pt = crossMesh->vertexList[id];
        position.col(id) = ShapeOp::Vector3(pt.x, pt.y, pt.z);
    }

    ShapeOp::Solver solver;
    solver.setPoints(position);

    double weight = getVarList()->getFloat("shapeop_planarity");

    //in plane constraint
    {
        std::vector<int> id_vector;
        for(int id = 0; id < m; id++)
        {
            id_vector.clear();
            for(int verID : crossMesh->crossList[id]->verIDs){
                id_vector.push_back(verID);
            }
            shared_ptr<ShapeOp::Constraint> constraint = make_shared<ShapeOp::PlaneConstraint>(id_vector, weight, position);
            solver.addConstraint(constraint);
        }
    }

    //boundary constraint
    weight = getVarList()->getFloat("shapeop_boundary");
    {
        for(int id = 0; id < m; id++){
            shared_ptr<Cross> cross = crossMesh->crossList[id];
            if(cross->atBoundary)
            {
                for(size_t jd = 0; jd < cross->verIDs.size(); jd++){
                    std::vector<int> id_vector;
                    int verID = cross->verIDs[jd]; id_vector.push_back(verID);
                    ShapeOp::Vector3 pt;
                    pt << cross->vers[jd].pos.x, cross->vers[jd].pos.y, cross->vers[jd].pos.z;
                    shared_ptr<ShapeOp::ClosenessConstraint> constraint = make_shared<ShapeOp::ClosenessConstraint>(id_vector, weight, position);
                    constraint->setPosition(pt);
                    solver.addConstraint(constraint);
                }
            }
        }
    }

    //closeness constraint
    weight = getVarList()->getFloat("shapeop_meshFit");
    {
        std::vector<int> id_vector;
        for(int id = 0; id < n; id++)
        {
            id_vector.clear();
            id_vector.push_back(id);
            shared_ptr<ShapeOp::MeshClosenessConstraint> constraint = make_shared<ShapeOp::MeshClosenessConstraint>(id_vector, weight, position);
            constraint->setTree(aabbTree, aabbV, aabbF);
            solver.addConstraint(constraint);
        }
    }

    //bending constraint
    weight  = getVarList()->getFloat("shapeop_bending");
    {
        for(int id = 0; id < m; id++){
            shared_ptr<Cross> cross = crossMesh->crossList[id];
            int nC = cross->verIDs.size();
            for(size_t jd = 0; jd < cross->neighbors.size(); jd++){
                shared_ptr<Cross> ncross = cross->neighbors[jd].lock();
                if(ncross == nullptr || cross->crossID > ncross->crossID) continue;
                int edgeID = ncross->GetNeighborEdgeID(cross->crossID);
                vector<int> id_vector;
                id_vector.push_back(cross->verIDs[jd]);
                id_vector.push_back(cross->verIDs[(jd + 1) % nC]);
                id_vector.push_back(cross->verIDs[(jd + 2) % nC]);
                id_vector.push_back(ncross->verIDs[(edgeID + 2) % ncross->verIDs.size()]);
                shared_ptr<ShapeOp::BendingConstraint> constraint = make_shared<ShapeOp::BendingConstraint>(id_vector, weight, position, 0, 0.5);
                solver.addConstraint(constraint);
            }
        }
    }

    //regular
    weight  = getVarList()->getFloat("shapeop_regularity");
    {
        for(int id = 0 ;id < m; id++)
        {
            shared_ptr<Cross> cross = crossMesh->crossList[id];
            int num_verIDs = cross->verIDs.size();
            Vector3f center = cross->center;
            Vector3f normal = cross->normal;
            Vector3f x_axis = Vector3f(1, 0, 0) CROSS normal;
            if(len(x_axis) < 1e-4) x_axis = Vector3f(0, 1, 0) CROSS normal;
            Vector3f y_axis = normal CROSS x_axis;
            Eigen::MatrixXd shape(3, num_verIDs);
            vector<int> id_vector;
            for(int jd = 0; jd < num_verIDs; jd ++)
            {
                float angle = M_PI * 2 / num_verIDs * jd;
                Vector3f pt = x_axis * std::cos(angle) + y_axis * std::sin(angle) + center;
                shape.col(jd) << pt.x, pt.y, pt.z;
                id_vector.push_back(cross->verIDs[jd]);
            }
            shared_ptr<ShapeOp::SimilarityConstraint> constraint = make_shared<ShapeOp::SimilarityConstraint>(id_vector, weight, position,true, true, true);
            std::vector<ShapeOp::Matrix3X> shapes;
            shapes.push_back(shape);
            constraint->setShapes(shapes);
            solver.addConstraint(constraint);
        }
    }

    solver.initialize();
    solver.solve(iterator);

    position = solver.getPoints();
    for(int id = 0; id < n ; id++)
    {
        Vector3f &pt = crossMesh->vertexList[id];
        pt.x = position(0, id);
        pt.y = position(1, id);
        pt.z = position(2, id);
    }

    crossMesh->UpdateCrossFromVertexList();
}