///////////////////////////////////////////////////////////////
//
// CrossMesh.cpp
//
//   Cross Mesh
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 15/Oct/2018
//
//
///////////////////////////////////////////////////////////////


#include "Utility/HelpDefine.h"
//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//


template<typename Scalar>
CrossMesh<Scalar>::CrossMesh(std::shared_ptr<InputVarList> var) : TopoObject(var)
{
	baseMesh2D = nullptr;
}

template<typename Scalar>
CrossMesh<Scalar>::~CrossMesh()
{

}

template<typename Scalar>
CrossMesh<Scalar>::CrossMesh(const CrossMesh &_cross)
{
    if(_cross.baseMesh2D) baseMesh2D = make_shared<PolyMesh>(*_cross.baseMesh2D);
    vertexList = _cross.vertexList;

    //boundary_vertexList_index = _cross.boundary_vertexList_index;

    //create cross geometry
    for(int id = 0; id < _cross.crossList.size(); id++)
    {
        shared_ptr<Cross> cross = make_shared<Cross>(*_cross.crossList[id]);
        crossList.push_back(cross);
    }

    //create cross neighbor information
    for(int id = 0; id < crossList.size(); id++)
    {
        shared_ptr<Cross> cross = crossList[id];
        cross->neighbors.clear();
        for(int jd = 0; jd < _cross.crossList[id]->neighbors.size(); jd++)
        {
            shared_ptr<Cross> _ncross = _cross.crossList[id]->neighbors[jd].lock();
            if(_ncross)
            {
                cross->neighbors.push_back(crossList[_ncross->crossID]);
            }
            else{
                cross->neighbors.push_back(_ncross);
            }
        }
    }

    UpdateCrossVertexIndex();
}

template<typename Scalar>
void CrossMesh<Scalar>::Print()
{
	printf("cross num: %lu \n", crossList.size());

	for (int i = 0; i < crossList.size(); i++)
	{
        crossList[i]->Print();
	}
}

template<typename Scalar>
void CrossMesh<Scalar>::SetBaseMesh2D(shared_ptr<PolyMesh> _baseMesh2D)
{
	baseMesh2D = _baseMesh2D;
}

vector<Vector3f> CrossMesh::GetAllVertices()
{
    vector<Vector3f> vertices;

    for (int i = 0; i < crossList.size(); ++i)
    {
        for (int j = 0; j <crossList[i]->vers.size(); ++j)
        {
            Vector3f ver = crossList[i]->vers[j].pos;

            if( GetPointIndexInList(ver, vertices) == ELEMENT_OUT_LIST )
            {
                vertices.push_back( ver );
            }
        }
    }

    return vertices;
}

//**************************************************************************************//
//                                   Update Vertices
//**************************************************************************************//

template<typename Scalar>
void CrossMesh<Scalar>::UpdateCrossFromVertexList()
{
    for (int i = 0; i < crossList.size(); i++)
    {
        shared_ptr<Cross> cross = crossList[i];
        for (int j = 0; j < cross->vers.size(); j++)
        {
            int verID = cross->verIDs[j];
            cross->vers[j].pos = vertexList[verID];
        }
        cross->ComputeCenter();
        cross->ComputeNormal();
    }

    return;
}

template<typename Scalar>
void CrossMesh<Scalar>::UpdateCrossVertexIndex()
{
    vertexList.clear();

    //init vertexList
    for(int id = 0; id < crossList.size(); id++)
    {
        shared_ptr<Cross> cross = crossList[id];
        for(int jd = 0; jd < cross->vers.size(); jd++)
        {
            Vector3f pt = cross->vers[jd].pos;
            cross->verIDs[jd] = vertexList.size();
            vertexList.push_back(pt);
        }
    }

    //init Matrix
    Eigen::MatrixXd V(vertexList.size(), 3);
    for(int id = 0; id < vertexList.size(); id++)
    {
        Vector3f pt = vertexList[id];
        V.row(id) = Eigen::RowVector3d(pt.x, pt.y, pt.z);
    }

    //remove duplicate
    Eigen::MatrixXd SV;
    Eigen::VectorXi SVI, SVJ;
    //   SV  #SV by dim new list of vertex positions
    //   SVI #V by 1 list of indices so SV = V(SVI,:)
    //   SVJ #SV by 1 list of indices so V = SV(SVJ,:)
    igl::remove_duplicate_vertices(V, FLOAT_ERROR_SMALL, SV, SVI, SVJ);

    vertexList.clear();
    for(int id = 0; id < SV.rows(); id++)
    {
        Vector3f pt(SV(id, 0), SV(id, 1), SV(id, 2));
        vertexList.push_back(pt);
    }

    //update cross verIDs
    vertexCrossList.clear();
    vertexCrossList.resize(vertexList.size());
    for(int id = 0; id < crossList.size(); id++)
    {
        shared_ptr<Cross> cross = crossList[id];
        for(int jd = 0; jd < cross->vers.size(); jd++)
        {
            int new_verID = SVJ(cross->verIDs[jd]);
            cross->verIDs[jd] = new_verID;
            vertexCrossList[new_verID].push_back(cross);
        }
    }

//

    return;
}

template<typename Scalar>
Scalar CrossMesh<Scalar>::averageCrossSize() {
    return 0;
}

template<typename Scalar>
void CrossMesh<Scalar>::updateCrossID() {
    for(int id = 0; id < crossList.size(); id++){
        if(crossList[id])
            crossList[id]->crossID = id;
    }
}

template<typename Scalar>
void CrossMesh<Scalar>::TranslateMesh(Vector3f mv) {
    for(int id = 0; id < vertexList.size(); id++){
        vertexList[id] += mv;
    }

    for(int id = 0; id < crossList.size(); id++){
        shared_ptr<Cross> cross = crossList[id];
        for(int jd = 0; jd < cross->vers.size(); jd++){
            cross->vers[jd].pos += mv;
        }
    }
}

shared_ptr<PolyMesh> CrossMesh::getPolyMesh()
{
    shared_ptr<PolyMesh> polymesh = make_shared<PolyMesh>(getVarList());
    for (int i = 0; i < crossList.size(); i++) {
        pPolygon poly = make_shared<_Polygon>(*crossList[i]);
        polymesh->polyList.push_back(poly);
    }

    return polymesh;
}

//**************************************************************************************//
//                                   Save OBJ File
//**************************************************************************************//

template<typename Scalar>
void CrossMesh<Scalar>::WriteOBJModel(const char *objFileName)
{
	FILE *fp;
	if ((fp = fopen(objFileName, "w+")) == NULL)
	{
		printf("Error: file not exists! \n");
		return;
	}
	else
	{
		///////////////////////////////////////////////////////////////////
		// 1. Write the vertex info of the mesh

		//fprintf(fp, "# Wavefront OBJ generated by Peng SONG \n\n");

		vector<Vector3f> vertexList = GetAllVertices();

		fprintf(fp, "# %d vertices \n", (int)vertexList.size());

		for (int i = 0; i < vertexList.size(); i++)
		{
			fprintf(fp, "v %f %f %f \n", vertexList[i].x, vertexList[i].y, vertexList[i].z);
		}
		fprintf(fp, "\n");


		///////////////////////////////////////////////////////////////////
		// 2. Write each polygon info of the mesh

		fprintf(fp, "# %d faces \n", (int)crossList.size());

		for (int i = 0; i < crossList.size(); i++)
		{
			pPolygon poly = crossList[i];

			//fprintf(fp, "f ");
			//for (int j = 0; j < poly->verIDs.size(); j++)
			//{
			//	//Since the index in OBJ file starting from 1 instead of 0, we need to add 1 to each index
			//	fprintf(fp, " %d", poly->verIDs[j] + 1);
			//}
			//fprintf(fp, "\n");

			fprintf(fp, "f ");
			for (int j = 0; j < poly->vers.size(); j++)
			{
				int verID = GetPointIndexInList(poly->vers[j].pos, vertexList);
				//Since the index in OBJ file starting from 1 instead of 0, we need to add 1 to each index
				fprintf(fp, " %d", verID + 1);
			}
			fprintf(fp, "\n");

		}
		fprintf(fp, "\n");

		fclose(fp);
	}
}




