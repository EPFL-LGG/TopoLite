//
// Created by ziqwang on 2019-10-28.
//
#include "CSharpExtern.h"
#include <string>

#include "CrossMesh/PatternCreator.h"

#include "Structure/Part.h"
#include "IO/XMLIO.h"
#include "Structure/StrucCreator.h"

/*
 * GLOBAL VARIABLES
 */


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
    if (data && data->strucCreator && data->strucCreator->crossMeshCreator) {
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
    if (data && data->strucCreator && data->strucCreator->crossMeshCreator) {
        if (data->strucCreator->crossMeshCreator->aabbTree && data->strucCreator->crossMeshCreator->quadTree) {
            data->strucCreator->CreateStructure(true, true, data->interactMatrix, true);
        } else {
            data->strucCreator->CreateStructure(true, false, data->interactMatrix, true);
        }
    }
    return;
}


int partNumber(XMLData* data){
    if(data && data->strucCreator && data->strucCreator->struc)
        return data->strucCreator->struc->partList.size();
    else
        return 0;
}

PolyMeshRhino *initPolyMeshRhino(int partID ,XMLData* data){
    PolyMeshRhino *mesh = NULL;
    if(data && data->strucCreator && data->strucCreator->struc){
        if(0 <= partID && partID < data->strucCreator->struc->partList.size())
        {
            shared_ptr<Part> part = data->strucCreator->struc->partList[partID];
            if(part && part->polyMesh){
                MeshConverter converter(data->varList);
                mesh = new PolyMeshRhino();
                converter.Convert2EigenMesh(part->polyMesh.get(), mesh);
                return mesh;
            }
        }
    }
    return NULL;
}

int isNull(PolyMeshRhino *mesh){
	if (mesh) return 0;
	else return 1;
}

bool deletePolyMeshRhino(PolyMeshRhino *mesh){
    if(mesh){
        delete mesh;
        return true;
    }
    else{
        return false;
    }
}

int getNVertices(PolyMeshRhino *mesh){
    if(mesh){
        return mesh->vertices.size();
    }
    return 0;
}

int getNFaces(PolyMeshRhino *mesh){
    if(mesh){
        return mesh->faces.size();
    }
    return 0;
}

void copyVertexI(PolyMeshRhino *mesh, int vID, float *point){
    if(mesh && vID >= 0 && vID < mesh->vertices.size()){
        point[0] = mesh->vertices[vID][0];
        point[1] = mesh->vertices[vID][1];
        point[2] = mesh->vertices[vID][2];
    }
}

void copyFaceI(PolyMeshRhino *mesh, int fID, int *face){
    if(mesh && fID >= 0 && fID < mesh->faces.size()){
        face[0] = mesh->faces[fID][0];
        face[1] = mesh->faces[fID][1];
        face[2] = mesh->faces[fID][2];
    }
}

int getNVertexGroup(PolyMeshRhino *mesh){
    if(mesh)
    {
        return mesh->verticesGroups.size();
    }
    return 0;
}

int getNFaceGroup(PolyMeshRhino *mesh){
    if(mesh)
    {
        return mesh->facesGroups.size();
    }
    return 0;
}

int getNVertexGroupI(PolyMeshRhino *mesh, int vgID){
    if(mesh && vgID >= 0 && vgID < mesh->verticesGroups.size()){
        return mesh->verticesGroups[vgID].size();
    }
    return 0;
}

int getNFaceGroupI(PolyMeshRhino *mesh, int vfID){
    if(mesh && vfID >= 0 && vfID < mesh->facesGroups.size()){
        return mesh->facesGroups[vfID].size();
    }
    return 0;
}

void copyVertexGroupI(PolyMeshRhino *mesh, int vgID, int* vg){

    //vg should be not null
    if(mesh && vgID >= 0 && vgID < mesh->verticesGroups.size() && vg)
    {
        for(int id = 0; id < mesh->verticesGroups[vgID].size(); id++){
            vg[id] = mesh->verticesGroups[vgID][id];
        }
    }
    return;
}

void copyFaceGroupI(PolyMeshRhino *mesh, int fgID, int* fg){

    //vg should be not null
    if(mesh && fgID >= 0 && fgID < mesh->facesGroups.size() && fg)
    {
        for(int id = 0; id < mesh->facesGroups[fgID].size(); id++){
            fg[id] = mesh->facesGroups[fgID][id];
        }
    }
    return;
}

bool isBoundary(int partID, XMLData* data)
{
    if(data && data->strucCreator && data->strucCreator->struc)
    {
        if(0 <= partID && partID < data->strucCreator->struc->partList.size()) {
            return data->strucCreator->struc->partList[partID]->atBoundary;
        }
    }
    return false;
}


float ComputeGroundHeight(XMLData* data)
{
    if(data && data->strucCreator && data->strucCreator->struc)
    {
        return data->strucCreator->struc->ComputeLowestY();
    }

    return 0;
}


void setParaDouble(const char *name, double value, XMLData* data){
    if(data && data->varList)
        data->varList->set(name,  (float)value);
    return;
}

