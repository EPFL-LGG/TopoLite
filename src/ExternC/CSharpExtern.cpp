//
// Created by ziqwang on 2019-10-28.
//
#include "CSharpExtern.h"
#include <string>

#include "CrossMesh/PatternCreator.h"
#include "Mesh/MeshConverter.h"
#include "Structure/Part.h"
#include "IO/XMLIO.h"
#include "Structure/StrucCreator.h"

/*
 * GLOBAL VARIABLES
 */


Eigen::MatrixXd V;
Eigen::MatrixXi F;

XMLData* readXML(const char *xmlstr)
{
    XMLData *data = new XMLData();
    XMLIO reader;
    reader.XMLReader(xmlstr, *data);
    return data;
}
//

int deleteStructure(XMLData* data){
    if(data){
        delete data;
        return 1;
    }
    else{
        return 0;
    }
}

void refresh(XMLData* data)
{
    if (data->strucCreator->crossMeshCreator) {
        if (data->strucCreator->crossMeshCreator->aabbTree && data->strucCreator->crossMeshCreator->quadTree) {
            data->strucCreator->CreateStructure(true, true, data->interactMatrix, false);
        } else {
            data->strucCreator->CreateStructure(true, false, data->interactMatrix, false);
        }
    }
    return;
}
//

void preview(XMLData* data)
{
    if (data->strucCreator->crossMeshCreator) {
        if (data->strucCreator->crossMeshCreator->aabbTree && data->strucCreator->crossMeshCreator->quadTree) {
            data->strucCreator->CreateStructure(true, true, data->interactMatrix, true);
        } else {
            data->strucCreator->CreateStructure(true, false, data->interactMatrix, true);
        }
    }
    return;
}


int partNumber(XMLData* data){
    if(data->strucCreator && data->strucCreator->struc)
        return data->strucCreator->struc->partList.size();
    else
        return 0;
}

bool initMesh(int partID, CMesh *mesh, XMLData* data){
    if(data->strucCreator && data->strucCreator->struc){
        if(0 <= partID && partID < data->strucCreator->struc->partList.size())
        {
            shared_ptr<Part> part = data->strucCreator->struc->partList[partID];
            if(part && part->polyMesh){
                MeshConverter converter(data->varList);
                converter.Convert2EigenMesh(part->polyMesh.get(), V , F);
                mesh->n_vertices = V.rows();
                mesh->n_faces = F.rows();
                return true;
            }
        }
    }
    return false;
}


bool assignMesh(int partID, CMesh *mesh, XMLData* data){
    if(data->strucCreator && data->strucCreator->struc){
        if(0 <= partID && partID < data->strucCreator->struc->partList.size())
        {
            shared_ptr<Part> part = data->strucCreator->struc->partList[partID];
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


bool isBoundary(int partID, XMLData* data)
{
    if(data->strucCreator && data->strucCreator->struc)
    {
        if(0 <= partID && partID < data->strucCreator->struc->partList.size()) {
            return data->strucCreator->struc->partList[partID]->atBoundary;
        }
    }
    return false;
}


float ComputeGroundHeight(XMLData* data){

    if(data->strucCreator && data->strucCreator->struc)
    {
        return data->strucCreator->struc->ComputeLowestY();
    }

    return 0;
}


void setParaDouble(const char *name, double value, XMLData* data){
    data->varList->set(name,  (float)value);
    return;
}

