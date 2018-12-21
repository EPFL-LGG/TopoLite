///////////////////////////////////////////////////////////////
//
// PolyMesh.cpp
//
//   Polygonal Mesh
//
// by Peng SONG ( songpenghit@gmail.com )
// 
// 09/Jan/2018
//
//
///////////////////////////////////////////////////////////////

#include "Polygon.h"
#include "PolyMesh.h"


//**************************************************************************************//
//                                   Initialization
//**************************************************************************************//

PolyMesh::PolyMesh()
{

}

PolyMesh::~PolyMesh()
{
	ClearMesh();
}



void PolyMesh::ClearMesh()
{
	for (int i = 0; i < polyList.size(); i++)
	{
		delete polyList[i];
	}
	polyList.clear();
}

void PolyMesh::PrintMesh()
{
//	printf("vertex num: %lu \n", vertexList.size());
//	for (int i = 0; i < vertexList.size(); i++)
//	{
//		Vector3d ver = vertexList[i];
//		printf("%9.6f  %9.6f  %9.6f \n", ver[0], ver[1], ver[2]);
//	}
//	printf("\n");
//
//	printf("polygon num: %lu \n", polyList.size());
//	for (int i = 0; i < polyList.size(); i++)
//	{
//		_Polygon *poly = polyList[i];
//		for(int j=0; j<poly->verIDs.size(); j++)
//		{
//			printf("%d ", poly->verIDs[j]);
//		}
//		printf("\n");
//	}
//	printf("\n");
}




//**************************************************************************************//
//                                  Mesh Operations
//**************************************************************************************//

void PolyMesh::ComputeBBox()
{
	double minX, minY, minZ, maxX, maxY, maxZ;
	minX = minY = minZ = MAX_FLOAT;
	maxX = maxY = maxZ = MIN_FLOAT;

	for (int i = 0; i < vertexList.size(); i++)
	{
		Vector3d ver = vertexList[i];

		if (maxX < ver[0])  maxX = ver[0];
		if (minX > ver[0])  minX = ver[0];

		if (maxY < ver[1])  maxY = ver[1];
		if (minY > ver[1])  minY = ver[1];

		if (maxZ < ver[2])  maxZ = ver[2];
		if (minZ > ver[2])  minZ = ver[2];
	}

	bbox.minPt = Vector3d(minX, minY, minZ);
	bbox.maxPt = Vector3d(maxX, maxY, maxZ);
	bbox.cenPt = 0.5f*(bbox.minPt + bbox.maxPt);
	bbox.size = bbox.maxPt - bbox.minPt;
}

Box PolyMesh::ComputeTextureBBox()
{
	Box texBBox;

	double minU, minV, maxU, maxV;
	minU = minV = MAX_FLOAT;
	maxU = maxV = MIN_FLOAT;

	for (int i = 0; i < texCoordList.size(); i++)
	{
		Vector2d texCoord = texCoordList[i];

		if (maxU < texCoord[0])   maxU = texCoord[0];
		if (minU > texCoord[0])   minU = texCoord[0];

		if (maxV < texCoord[1])   maxV = texCoord[1];
		if (minV > texCoord[1])   minV = texCoord[1];
	}

	texBBox.minPt = Vector3d(minU, minV, 0);
	texBBox.maxPt = Vector3d(maxU, maxV, 0);
	return texBBox;
}

void PolyMesh::NormalizeMesh()
{
	ComputeBBox();

	double scale = 2.0f / _MAX(bbox.size[0], _MAX(bbox.size[1], bbox.size[2]));
	Vector3d trans = -bbox.cenPt;

	for (int i = 0; i < vertexList.size(); ++i)
	{
		Vector3d origVer = vertexList[i];

		vertexList[i] = (origVer+trans) * scale;
	}

	for (int i = 0; i < polyList.size(); i++)
	{
		for (int j = 0; j < polyList[i]->vers.size(); ++j)
		{
			Vector3d origVer = polyList[i]->vers[j].pos;

			polyList[i]->vers[j].pos = (origVer+trans) * scale;
		}
	}
}




//**************************************************************************************//
//                                  Volume and Centroid
//**************************************************************************************//

void PolyMesh::ComputeVolume()
{
//	// Convert into triangular mesh
//	MeshConverter meshConverter;
//	vector<Triangle*> triList = meshConverter.Convert2TriMesh(this);
//
//	// Compute volume of the triangular mesh
//	volume = ComputeVolume(triList);
//
//	// Release the memory
//	for (int i = 0; i < triList.size(); i++)
//	{
//		delete triList[i];
//	}
//	triList.clear();
}
/////////////////////////////////////////////////////////////////////
// Compute model volume following Dr Robert Nurnberg's method at:
// http://wwwf.imperial.ac.uk/~rn/centroid.pdf

double PolyMesh::ComputeVolume(vector<Triangle*> triList)
{
	double volume = 0;

	// Accumulate volume value for each triangle
	for (int i = 0; i < triList.size(); i++)
	{
		Triangle *tri = triList[i];

		Vector3d ver0 = tri->v[0];
		Vector3d crossVec = -1.0f * (tri->v[2] - tri->v[0]).cross(tri->v[1] - tri->v[0]);
		double dotP = ver0.dot(crossVec);

		volume += dotP;
	}

	volume = volume / 6.0f;

	return volume;
}

void PolyMesh::ComputeCentroid()
{
//	// Convert into triangular mesh
//	MeshConverter meshConverter;
//	vector<Triangle*> triList = meshConverter.Convert2TriMesh(this);
//
//	// Compute centroid of the triangular mesh
//	centroid = ComputeCentroid(triList);
//
//	// Release the memory
//	for (int i = 0; i < triList.size(); i++)
//	{
//		delete triList[i];
//	}
//	triList.clear();
}
/////////////////////////////////////////////////////////////////////
// Compute model centroid following Dr Robert Nurnberg's method at:
// http://wwwf.imperial.ac.uk/~rn/centroid.pdf

Vector3d PolyMesh::ComputeCentroid(vector<Triangle*> triList)
{
	Vector3d centroid = Vector3d(0, 0, 0);

	// Save the 3 major axes
	Vector3d axes[3];
	axes[0] = Vector3d(1, 0, 0);
	axes[1] = Vector3d(0, 1, 0);
	axes[2] = Vector3d(0, 0, 1);

	// Accumulate centroid value for each major axes
	for (int i = 0; i < 3; i++)
	{
		Vector3d axis = axes[i];

		for (int j = 0; j < triList.size(); j++)
		{
			Triangle *tri = triList[j];
			Vector3d crossVec = -1.0f * (tri->v[2] - tri->v[0]).cross(tri->v[1] - tri->v[0]);

			centroid[i] += (1 / 24.0f) * (crossVec.dot(axis)) *
				(pow((tri->v[0] + tri->v[1]).dot(axis), 2) +
				pow((tri->v[1] + tri->v[2]).dot(axis), 2) +
				pow((tri->v[2] + tri->v[0]).dot(axis), 2));
		}
	}

	// Compute volume and centroid
	double volume = ComputeVolume(triList);
	centroid = centroid / (2.0f*volume);

	return centroid;
}

void PolyMesh::ComputeLowestPt()
{
	Vector3d down_direc (0.0, -1.0, 0.0);
	lowestPt = ComputeExtremeVertex(down_direc);
}

Vector3d PolyMesh::ComputeExtremeVertex(Vector3d rayDir)
{
	Vector3d extremVer;

	// Push back all vertices, edge middle points, and face center points
	vector<Vector3d> pointList = vertexList;
	for (int i = 0; i < polyList.size(); i++)
	{
		_Polygon *poly = polyList[i];
	
		for (int j = 0; j < poly->vers.size(); j++)
		{
			Vector3d staPt = poly->vers[j].pos;
			Vector3d endPt = poly->vers[(j + 1) / poly->vers.size()].pos;

			Vector3d edgeMidPt = (staPt + endPt) / 2.0f;

			pointList.push_back(edgeMidPt);
		}

		pointList.push_back(poly->center);
	}

	// Find the extreme point with the largest distance along ray direction and the shortest distance along perpendicular direction
	const double weight = 0.3f;
	double maxDist = MIN_FLOAT;
	for (int i = 0; i < pointList.size(); i++)
	{
		Vector3d ver = pointList[i];

		double verDist = ver.norm();
		double rayDist = ver.dot(rayDir);
		double perpDist = sqrt(verDist*verDist - rayDist*rayDist);


		double dist = rayDist - weight*perpDist;

		if (dist > maxDist)
		{
			maxDist = dist;
			extremVer = ver;
		}
	}
	return extremVer;
}




//**************************************************************************************//
//                                   Transform Mesh
//**************************************************************************************//

void PolyMesh::RotateMesh(Vector3d rotCenter, Vector3d rotAxis, double rotAngle)
{
//	for (int i = 0; i < vertexList.size(); ++i)
//	{
//		Vector3d &ver = vertexList[i];
//
//		ver = RotateVector(rotCenter, rotAxis, rotAngle, ver);
//	}
//
//	for (int i = 0; i < polyList.size(); ++i)
//	{
//		_Polygon *poly = polyList[i];
//
//		for (int j = 0; j < poly->vers.size(); ++j)
//		{
//			Vector3d &ver = poly->vers[j].pos;
//
//			ver = RotateVector(rotCenter, rotAxis, rotAngle, ver);
//		}
//	}
}




//**************************************************************************************//
//                                   Read Save OBJ File
//**************************************************************************************//

bool PolyMesh::ReadOBJModel(const char *fileName)
{
////	FILE *fp;
////
////	///////////////////////////////////////////////////////
////	// 1. File does not exist
////
////	if ((fp = fopen(fileName, "r")) == NULL)
////	{
////		printf("Error: file doe not exist! \n");
////		fclose(fp);
////		return false;
////	}
////
////
////	///////////////////////////////////////////////////////
////	// 2. File extension type is not correct
////
////	string datType("obj");
////	string fileType = GetFileType(fileName);
//
//	if (fileType.compare(datType) != 0)
//	{
//		printf("Warning: The input file should be in OBJ format. \n");
//
//		fclose(fp);
//		return false;
//	}
//
//
//	/////////////////////////////////////////////////////
//	// 3. Read vertices
//
//	char *tok;
//	char temp[128];
//	char pLine[512];
//
//	fseek(fp, 0, SEEK_SET);
//
//	while (fgets(pLine, 512, fp))
//	{
//		if (pLine[0] == 'v' && pLine[1] == ' ')
//		{
//			// Read vertex position
//			double nvv[3];
//			tok = strtok(pLine, " ");
//			for (int i = 0; i < 3; i++)
//			{
//				tok = strtok(NULL, " ");
//				strcpy(temp, tok);
//				temp[strcspn(temp, " ")] = 0;
//				nvv[i] = (double)atof(temp);
//			}
//
//			Vector3d verPos = Vector3d(nvv[0], nvv[1], nvv[2]);
//
//			vertexList.push_back(verPos);
//		}
//	}
//
//
//	/////////////////////////////////////////////////////
//	// 4. Read texture coordinates
//
//	fseek(fp, 0, SEEK_SET);
//
//	int verID = 0;
//	while (fgets(pLine, 512, fp))
//	{
//		if (pLine[0] == 'v' && pLine[1] == 't' && pLine[2] == ' ')
//		{
//			// Read vertex texture coordinate
//			double nvv[2];
//			tok = strtok(pLine, " ");
//			for (int i = 0; i < 2; i++)
//			{
//				tok = strtok(NULL, " ");
//				strcpy(temp, tok);
//				temp[strcspn(temp, " ")] = 0;
//				nvv[i] = (double)atof(temp);
//			}
//
//			Vector2d texCoord = Vector2d(nvv[0], nvv[1]);
//			//printf("tex: %.6f %.6f \n", texCoord[0], texCoord[1]);
//
//			texCoordList.push_back(texCoord);
//		}
//	}
//
//
//	/////////////////////////////////////////////////////
//	// 5. Read facets
//
//	char temp2[128];
//
//	fseek(fp, 0, SEEK_SET);
//
//	while (fgets(pLine, 512, fp))
//	{
//		char *pTmp = pLine;
//
//		if (pTmp[0] == 'f' && pLine[1] == ' ')
//		{
//			_Polygon *poly = new _Polygon();
//
//			tok = strtok(pLine, " ");
//			while ((tok = strtok(NULL, " ")) != NULL)
//			{
//				strcpy(temp, tok);
//
//				temp[strcspn(temp, "/")] = 0;
//				int verPosID = (int)strtol(temp, NULL, 10) - 1;
//
//				//printf(" ");
//				////printf(" id: %d \n", id);
//
//				// Note: this could be caused "" at the end of each line
//				if (verPosID < 0)
//					break;
//
//				strcpy(temp2, tok);
//				//printf(temp2);
//				//printf(" ");
//
//				// TODO: parse the text using a better way
//				for (int i = 0; i < strcspn(temp, "/")+1; i++)
//				{
//					temp2[i] = '0';
//				}
//				//printf(temp2);
//				//printf(" ");
//				int texCoordID = (double)atof(temp2) - 1;
//
//				//printf(" ver: %d    tex: %d \n", verPosID, texCoordID);
//
//				_Vertex ver;
//				if (texCoordID >= 0)
//				{
//					ver = _Vertex(vertexList[verPosID], texCoordList[texCoordID]);
//				}
//				else
//				{
//					ver = _Vertex(vertexList[verPosID]);
//				}
//
//				//ver.texCoord = texCoordList[texCoordID];
//
//				poly->vers.push_back(ver);
//
//				poly->verIDs.push_back(verPosID);
//			}
//
//			if (poly->vers.size() >= 3)
//			{
//				poly->ComputeCenter();
//				poly->ComputeNormal();
//
//				//for (int j = 0; j < poly->vers.size(); j++)
//				//{
//				//	printf("j=%d \n", j);
//				//	printf("pos: [%.3f %.3f %.3f] \n", poly->vers[j].pos[0], poly->vers[j].pos[1], poly->vers[j].pos[2]);
//				//	printf("tex: [%.3f %.3f] \n", poly->vers[j].texcoord[0], poly->vers[j].texcoord[1]);
//				//}
//
//				polyList.push_back(poly);
//			}
//			else
//			{
//				delete poly;
//				printf("Warning: The model face should have at least three vertices. \n");
//			}
//		}
//	}
//
//
//	/////////////////////////////////////////////////////
//	// 5. Print vertices and facets
//
//	//PrintMesh();
//
//	fclose(fp);
//
//	return true;
}


void PolyMesh::WriteOBJModel(const char *objFileName)
{
//	FILE *fp;
//	if ((fp = fopen(objFileName, "w+")) == NULL)
//	{
//		printf("Error: file not exists! \n");
//		return;
//	}
//	else
//	{
//		///////////////////////////////////////////////////////////////////
//		// 1. Compute vertices and vertexIDs in case they are missing
//
//		UpdateVertices();
//
//
//		///////////////////////////////////////////////////////////////////
//		// 2. Write the vertex info of the mesh
//
//		//fprintf(fp, "# Wavefront OBJ generated by Peng SONG \n\n");
//
//		fprintf(fp, "# %d vertices \n", (int)vertexList.size());
//
//		for (int i = 0; i < vertexList.size(); i++)
//		{
//			fprintf(fp, "v %f %f %f \n", vertexList[i][0] , vertexList[i][1], vertexList[i][2] );
//		}
//		fprintf(fp, "\n");
//
//
//		///////////////////////////////////////////////////////////////////
//		// 3. Write each polygon info of the mesh
//
//		fprintf(fp, "# %d faces \n", (int)polyList.size());
//
//		for (int i = 0; i < polyList.size(); i++)
//		{
//			_Polygon *poly = polyList[i];
//
//			//fprintf(fp, "f ");
//			//for (int j = 0; j < poly->verIDs.size(); j++)
//			//{
//			//	//Since the index in OBJ file starting from 1 instead of 0, we need to add 1 to each index
//			//	fprintf(fp, " %d", poly->verIDs[j] + 1);
//			//}
//			//fprintf(fp, "\n");
//
//			fprintf(fp, "f ");
//			for (int j = 0; j < poly->vers.size(); j++)
//			{
//				int verID = GetPointIndexInList(poly->vers[j].pos, vertexList);
//				//Since the index in OBJ file starting from 1 instead of 0, we need to add 1 to each index
//				fprintf(fp, " %d", verID + 1);
//			}
//			fprintf(fp, "\n");
//
//		}
//		fprintf(fp, "\n");
//
//		fclose(fp);
//	}
}

void PolyMesh::UpdateVertices()
{
//	vertexList.clear();
//	for (int i = 0; i < polyList.size(); i++)
//	{
//		_Polygon *poly = polyList[i];
//
//		for (int j = 0; j < poly->vers.size(); j++)
//		{
//			Vector3d ver = poly->vers[j].pos;
//
//			if( GetPointIndexInList(ver, vertexList) == ELEMENT_OUT_LIST)
//			{
//				vertexList.push_back(ver);
//			}
//		}
//	}
//
//	for (int i = 0; i < polyList.size(); i++)
//	{
//		_Polygon *poly = polyList[i];
//
//
//		poly->verIDs.clear();  // Note: verIDs mostly should be empty
//
//		for (int j = 0; j < poly->vers.size(); j++)
//		{
//			Vector3d ver = poly->vers[j].pos;
//
//			int index = GetPointIndexInList(ver, vertexList);
//
//			if (index == ELEMENT_OUT_LIST)
//			{
//				//vertexList.push_back(ver);
//
//				//poly->verIDs.push_back(vertexList.size()-1);
//				printf("Warning: the vertex should be in the list. \n");
//			}
//			else
//			{
//				poly->verIDs.push_back(index);
//			}
//		}
//	}
}




//**************************************************************************************//
//                                     Draw Mesh
//**************************************************************************************//

void PolyMesh::DrawMesh()
{
	// Draw the polygons
	for (int i = 0; i < polyList.size(); i++)
	{
		glBegin(GL_POLYGON);

		Vector3d faceNor = polyList[i]->normal;
		glNormal3f(faceNor[0], faceNor[1], faceNor[2]);

		for (int j = 0; j < polyList[i]->vers.size(); j++)
		{
			Vector3d vertexPos = polyList[i]->vers[j].pos;
			glVertex3f(vertexPos[0], vertexPos[1], vertexPos[2]);
		}
		glEnd();
	}

	glDisable(GL_LIGHTING);

}

void PolyMesh::DrawMesh_Texture()
{
	// Draw the polygons
	for (int i = 0; i < polyList.size(); i++)
	{
		glBegin(GL_POLYGON);

		Vector3d faceNor = polyList[i]->normal;
		glNormal3f(faceNor[0], faceNor[1], faceNor[2]);

		for (int j = 0; j < polyList[i]->vers.size(); j++)
		{
			Vector3d vertexPos = polyList[i]->vers[j].pos;
			Vector2d texCoord = polyList[i]->vers[j].texCoord;

			glTexCoord2f(texCoord[0], texCoord[1]);
			glVertex3f(vertexPos[0], vertexPos[1], vertexPos[2]);
		}
		glEnd();
	}
}

void PolyMesh::DrawMesh_Wire(double width, Vector3d color)
{
//	double epsilon = 1e-6;
//	double projMat[16];
//	glGetFloatv(GL_PROJECTION_MATRIX, projMat);
//
//	glDisable(GL_LIGHTING);
//	glColor3f(color[0], color[1], color[2]);
//	glLineWidth(width);
//
//	// Move the mesh toward the eye a bit to avoid z-fighting issue
//	glMatrixMode(GL_PROJECTION);
//	glPushMatrix();
//	glLoadIdentity();
//	glTranslatef(0.0f, 0.0f, -epsilon);
//	glMultMatrixf(projMat);
//
//	// Draw the polygons
//	for (int i = 0; i < polyList.size(); i++)
//	{
//		glBegin(GL_LINE_LOOP);
//		for (int j = 0; j < polyList[i]->vers.size(); j++)
//		{
//			Vector3d vertexPos = polyList[i]->vers[j].pos;
//			glVertex3f(vertexPos[0], vertexPos[1], vertexPos[2]);
//		}
//		glEnd();
//	}
//
//	glPopMatrix();
//	glMatrixMode(GL_MODELVIEW);
//
//	glLineWidth(1.0);
//	glEnable(GL_LIGHTING);
//
//	//////////////////////////////////////////////////////////////////
//	// 1. Temporary code for debugging
//
//	//DrawWireCuboid(bbox.minPt, bbox.maxPt, 2.0, Vector3d(0, 0.9, 0));
//
//	//DrawMesh_Debug();
//
//	//DrawMesh_Vertex(10.0, Vector3d(0.1,0.9,0.1));
}

void PolyMesh::DrawMesh_Vertex(double size, Vector3d color)
{
//	glDisable(GL_LIGHTING);
//	glPointSize(size);
//	glColor3f(color[0], color[1], color[2]);
//
//	for (int i = 0; i < vertexList.size(); i++)
//	{
//		Vector3d point = vertexList[i];
//
//		glBegin(GL_POINTS);
//		glVertex3f(point[0], point[1], point[2]);
//		glEnd();
//	}
//
//	glPointSize(1.0);
//	glEnable(GL_LIGHTING);
}

void PolyMesh::DrawMesh_Normal(Vector3d color)
{
//	const double normalLen = 0.6f;
//
//	glDisable(GL_LIGHTING);
//	glPointSize(12.0);
//	glLineWidth(2.0);
//	glColor3f(color[0], color[1], color[2]);
//
//	for (int i = 0; i < polyList.size(); i++)
//	{
//		Vector3d point = polyList[i]->center;
//		Vector3d normal = polyList[i]->normal;
//		Vector3d endPt = point + normalLen * normal;
//
//		glBegin(GL_POINTS);
//		glVertex3f(point[0], point[1], point[2]);
//		glEnd();
//
//		glBegin(GL_LINES);
//		glVertex3f(point[0], point[1], point[2]);
//		glVertex3f(endPt[0], endPt[1], endPt[2]);
//		glEnd();
//	}
//
//	glPointSize(1.0);
//	glLineWidth(1.0);
//	glEnable(GL_LIGHTING);
}

void PolyMesh::DrawMesh_Debug()
{
//	glDisable(GL_LIGHTING);
//	glColor3f(0.9, 0.1, 0.1);
//	glLineWidth(4.0);
//
//	glBegin(GL_LINES);
//	glVertex3f( 0.300,  0.173,  0.000);
//	glVertex3f( 0.350,  0.202, -0.082);
//
//	glVertex3f( 0.300, -0.173,  0.000);
//	glVertex3f( 0.250, -0.144, -0.082);
//
//	glVertex3f( 0.000, -0.346,  0.000);
//	glVertex3f( 0.000, -0.404, -0.082);
//
//	glVertex3f(-0.300,  0.173,  0.000);
//	glVertex3f(-0.350,  0.202, -0.082);
//
//	glVertex3f( 0.000,  0.346,  0.000);
//	glVertex3f( 0.000,  0.289, -0.082);
//
//	glVertex3f(-0.300, -0.173,  0.000);
//	glVertex3f(-0.250, -0.144, -0.082);
//
//	glVertex3f( 0.300, -0.173,  0.000);
//	glVertex3f( 0.274, -0.158, -0.095);
//
//	glVertex3f( 0.300,  0.173,  0.000);
//	glVertex3f( 0.326,  0.188, -0.095);
//
//	glVertex3f( 0.000,  0.346,  0.000);
//	glVertex3f( 0.000,  0.316, -0.095);
//
//	glVertex3f( 0.000, -0.346,  0.000);
//	glVertex3f( 0.000, -0.377, -0.095);
//
//	glVertex3f(-0.300,  0.173,  0.000);
//	glVertex3f(-0.326,  0.188, -0.095);
//
//	glVertex3f(-0.300, -0.173,  0.000);
//	glVertex3f(-0.274, -0.158, -0.095);
//	glEnd();
//
//	glLineWidth(1.0);
//	glEnable(GL_LIGHTING);
}

void PolyMesh::DrawMesh_Footprint()
{
//    for (int i=0; i<polyList.size(); i++)
//    {
//        _Polygon *poly = polyList[i];
//
//        glColor3f(0.1, 0.1, 0.1);
//        glBegin(GL_POLYGON);
//        for(int j=0; j<poly->vers.size(); j++)
//        {
//            Vector2d texCoord = poly->vers[j].texCoord;
//
//            glVertex2f(texCoord[0], texCoord[1]);
//        }
//        glEnd();
//    }
}




//**************************************************************************************//
//                               Draw Mesh Property
//**************************************************************************************//

void PolyMesh::DrawMeshBBox(double width, Vector3d color)
{
//	const Vector3d offset(0.0015, 0.0015, 0.0015);
//
//	Vector3d minPoint = bbox.minPt - offset;
//	Vector3d maxPoint = bbox.maxPt + offset;
//	DrawWireCuboid(minPoint, maxPoint, width, color);
}

void PolyMesh::DrawMeshCentroid(double size, Vector3d color)
{
//	DrawSphere(centroid, size, Vector3d(0.1, 0.1, 0.1), color, color, Vector3d(0.1, 0.1, 0.1));
}




//**************************************************************************************//
//                               Draw Mesh to Buffer
//**************************************************************************************//

GLubyte* PolyMesh::DrawMeshOffline(double viewW, double viewH, GLint iWidth, GLint iHeight)
{
//	GLubyte *imgBytes = new GLubyte[iWidth*iHeight * 4];
//	for (int i = 0; i < iWidth*iHeight * 4; i++)
//	{
//		imgBytes[i] = 255;
//	}
//
//	for (int i = 0; i < polyList.size(); i++)
//	{
//		_Polygon *poly = polyList[i];
//
//		for (int j = 0; j < poly->vers.size(); j++)
//		{
//			Vector3d staPt = poly->vers[j].pos;
//			Vector3d endPt = poly->vers[(j + 1) % poly->vers.size()].pos;
//
//			Vector3i staPixelPos;
//			Vector3i endPixelPos;
//
//			bool getStaPixelPos = GetPixelLocation(staPt, viewW, viewH, iWidth, iHeight, staPixelPos);
//			bool getEndPixelPos = GetPixelLocation(endPt, viewW, viewH, iWidth, iHeight, endPixelPos);
//
//			if (getStaPixelPos == false || getEndPixelPos == false)
//			{
//				break;
//			}
//
//			vector<Vector3i> linePts = Rasterize2DLine(staPixelPos, endPixelPos);
//
//			for (int k = 0; k < linePts.size(); k++)
//			{
//				//int index = linePts[k][0] * iHeight + linePts[k][1];
//				int index = linePts[k][1] * iWidth + linePts[k][0];
//
//				if (index < 0 || index > iWidth*iHeight)
//				{
//					printf("index = %d  \n", index);
//					break;
//				}
//
//				imgBytes[4 * index] = 255;
//				imgBytes[4 * index + 1] = 0;
//				imgBytes[4 * index + 2] = 0;
//			}
//		}
//	}
//
//	return imgBytes;
}

vector<Vector3i> PolyMesh::Rasterize2DLine(Vector3i staPixelPos, Vector3i endPixelPos)
{
//	int x0 = staPixelPos[0];
//	int y0 = staPixelPos[1];
//
//	int x1 = endPixelPos[0];
//	int y1 = endPixelPos[1];
//
//	int dx, dy, steps, k;
//	//int **line_point;
//	double xIncrement, yIncrement, x, y;
//
//	dx = x1 - x0;   dy = y1 - y0;
//	x = (double)x0;  y = (double)y0;
//	if (fabs((double)dx) > fabs((double)dy))   steps = (int)fabs((double)dx);
//	else                                     steps = (int)fabs((double)dy);
//
//	vector<Vector3i> linePts;
//
//	Vector3i pt0;
//	pt0[0] = (int)(x0 + 0.5);
//	pt0[1] = (int)(y0 + 0.5);
//	pt0[2] = 0;
//
//	linePts.push_back(pt0);
//
//	xIncrement = double(dx) / double(steps);
//	yIncrement = double(dy) / double(steps);
//
//
//	for (k = 0; k < steps; k++)
//	{
//		x = x + xIncrement;
//		y = y + yIncrement;
//
//		Vector3i pt;
//		pt[0] = (int)(x + 0.5);
//		pt[1] = (int)(y + 0.5);
//		pt[2] = 0;
//
//		linePts.push_back(pt);
//	}
//
//	return linePts;
}

bool PolyMesh::GetPixelLocation(Vector3d crossPt, double viewW, double viewH, GLint iWidth, GLint iHeight, Vector3i &pixelPos)
{
//	if (crossPt[0] >= 0.5f*viewW || crossPt[0] <= -0.5f*viewW ||
//		crossPt[1] >= 0.5f*viewH || crossPt[1] <= -0.5f*viewH)
//	{
//		return false;
//	}
//
//	int u = ((crossPt[0] + 0.5f*viewW) / viewW) * (iWidth - 1);
//	int v = ((crossPt[1] + 0.5f*viewH) / viewH) * (iHeight - 1);
//
//	pixelPos = Vector3i(u, v, 0);
//
//	return true;
}
