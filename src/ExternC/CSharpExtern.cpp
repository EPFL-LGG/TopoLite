//
// Created by ziqwang on 2019-10-28.
//

#include "PyInterface/PyInterface.h"
#include "CSharpExtern.h"
#include <string>
#include "Structure/StrucCreator.h"
#include "Optimization/SlopeOpt.h"
#include "Optimization/PolygonCoverOpt.h"
#include "CrossMesh/PatternCreator.h"
#include "PolyMesh/MeshConverter.h"
#include "Structure/Part.h"
/*
 * GLOBAL VARIABLES
 */

gluiVarList varList; //gluiVar
double interactMatrix[16] = {5,0,0,0, 0,5,0,0, 0,0,1,0, 0,0,0,1}; // pattern settings
shared_ptr<StrucCreator> myStrucCreator; // TI Assembly Creator
pugi::xml_document xmldoc;
vector<int> pickPartIDs;

Eigen::MatrixXd V;
Eigen::MatrixXi F;

CSharp_LIBRARY_C_FUNCTION
int readXML(const char *xmlstr)
{
    myStrucCreator.reset();
    myStrucCreator = make_shared<StrucCreator>();
    InitVar(varList);
    std::string filename = xmlstr;
    return ReadFile(filename);
}

CSharp_LIBRARY_C_FUNCTION
int deleteStructure(){
    if(myStrucCreator){
        myStrucCreator.reset();
        return 1;
    }
    else{
        return 0;
    }
}

CSharp_LIBRARY_C_FUNCTION
void refresh() {
    if (myStrucCreator->refModel) {
        if (myStrucCreator->refModel->aabbTree && myStrucCreator->refModel->quadTree) {
            myStrucCreator->CreateStructure(true, true, interactMatrix, false);
        } else {
            myStrucCreator->CreateStructure(true, false, interactMatrix, false);
        }
    }
    return;
}

CSharp_LIBRARY_C_FUNCTION
void preview(){
    if (myStrucCreator->refModel) {
        if (myStrucCreator->refModel->aabbTree && myStrucCreator->refModel->quadTree) {
            myStrucCreator->CreateStructure(true, true, interactMatrix, true);
        } else {
            myStrucCreator->CreateStructure(true, false, interactMatrix, true);
        }
    }
    return;
}

CSharp_LIBRARY_C_FUNCTION
int partNumber(){
    if(myStrucCreator && myStrucCreator->myStruc)
        return myStrucCreator->myStruc->partList.size();
    else
        return 0;
}

CSharp_LIBRARY_C_FUNCTION
bool initMesh(int partID, CMesh *mesh){
    if(myStrucCreator && myStrucCreator->myStruc){
        if(0 <= partID && partID < myStrucCreator->myStruc->partList.size())
        {
            shared_ptr<Part> part = myStrucCreator->myStruc->partList[partID];
            if(part){
                MeshConverter converter;
                converter.convert2EigenMesh(part->polyMesh.get(), V , F);
                mesh->n_vertices = V.rows();
                mesh->n_faces = F.rows();
                std::cout << mesh->n_vertices << std::endl;
                std::cout << mesh->n_faces << std::endl;
                return true;
            }
        }
    }
    return false;
}


CSharp_LIBRARY_C_FUNCTION
bool assignMesh(int partID, CMesh *mesh){
    if(myStrucCreator && myStrucCreator->myStruc){
        if(0 <= partID && partID < myStrucCreator->myStruc->partList.size())
        {
            shared_ptr<Part> part = myStrucCreator->myStruc->partList[partID];
            if(part){
                if(V.rows() != mesh->n_vertices) return false;
                if(F.rows() != mesh->n_faces) return false;
                for(int id = 0; id < V.rows(); id++){
                    mesh->points[id].x = V(id, 0);
                    mesh->points[id].y = V(id, 1);
                    mesh->points[id].z = V(id, 2);
                }
                for(int id = 0; id < F.rows(); id++){
                    mesh->faces[id].a = F(id, 0);
                    mesh->faces[id].b = F(id, 1);
                    mesh->faces[id].c = F(id, 2);
                }
                return true;
            }
        }
    }
    return false;
}

CSharp_LIBRARY_C_FUNCTION
bool isBoundary(int partID)
{
    if(myStrucCreator && myStrucCreator->myStruc)
    {
        if(0 <= partID && partID < myStrucCreator->myStruc->partList.size()) {
            return myStrucCreator->myStruc->partList[partID]->atBoundary;
        }
    }
    return false;
}

CSharp_LIBRARY_C_FUNCTION
float ComputeGroundHeight(){

    if(myStrucCreator && myStrucCreator->myStruc)
    {
        return myStrucCreator->myStruc->ComputeGroundHeight();
    }

    return 0;
}

CSharp_LIBRARY_C_FUNCTION
void setParaDouble(const char *name, double value){
    varList.set(name,  (float)value);
    return;
}