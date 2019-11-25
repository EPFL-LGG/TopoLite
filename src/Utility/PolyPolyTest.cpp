///////////////////////////////////////////////////////////////
//
// PolyPolyTest.cpp
//
//   Intersection Test between Two Planar Convex Polygons, following the approach in:
//     https://www.swtestacademy.com/intersection-convex-polygons-algorithm/
//
// by Song Peng ( songpenghit@gmail.com )
//
// 21/July/2018
//
//
///////////////////////////////////////////////////////////////

#include "HelpDefine.h"
#include "HelpFunc.h"
#include "PolyPolyTest.h"
#include "clipper.hpp"
#include "ConvexHull2D.h"
#include "IO/gluiVar.h"
extern gluiVarList varList;

//**************************************************************************************//
//                        Compute overlap of two 2D polygons
//**************************************************************************************//

bool IsConvexPolygonIntersec(vector<Vector2f> &polyA, vector<Vector2f> &polyB, double &area){

    double Scale = varList.get<float>("clipper_scale");

    ClipperLib::Path pathA, pathB;
    for(int id = 0; id < polyA.size(); id++){
        int x = polyA[id].x * Scale;
        int y = polyA[id].y * Scale;
        pathA.push_back(ClipperLib::IntPoint(x, y));
    }

    for(int id = 0; id < polyB.size(); id++){
        int x = polyB[id].x * Scale;
        int y = polyB[id].y * Scale;
        pathB.push_back(ClipperLib::IntPoint(x, y));
    }

    if(!ClipperLib::Orientation(pathA)) {
        ClipperLib::ReversePath(pathA);
        //std::cout << "PathA is Wrong" << std::endl;
    }
    if(!ClipperLib::Orientation(pathB))
    {
        ClipperLib::ReversePath(pathB);
        //std::cout << "pathB is Wrong" << std::endl;
    }

    ClipperLib::Clipper solver;
    solver.AddPath(pathA, ClipperLib::ptSubject, true);
    solver.AddPath(pathB, ClipperLib::ptClip, true);
    ClipperLib::Paths path_int;
    solver.Execute(ClipperLib::ctIntersection, path_int, ClipperLib::pftPositive, ClipperLib::pftPositive);

    if(path_int.empty() || path_int.front().empty()) {
        area = 0;
        return false;
    }
    else{
        area = 0;
        for(ClipperLib::Path path : path_int){
            area += ClipperLib::Area(path);
        }
        return true;
    }

}

vector<Vector3f> ConvexPolygonIntersec(vector<Vector3f> polyA, vector<Vector3f> polyB){
    _Polygon PA; PA.SetVertices(polyA);
    _Polygon PB; PB.SetVertices(polyB);

    Vector3f x_axis, y_axis, origin;
    PA.ComputeFrame(x_axis, y_axis, origin);

    float Scale = varList.get<float>("clipper_scale");
    vector<Vector3i> intPA = PA.ProjectToNormalPlane(x_axis, y_axis, origin, Scale);
    vector<Vector3i> intPB = PB.ProjectToNormalPlane(x_axis, y_axis, origin, Scale);

    ClipperLib::Path pathA, pathB;
    for(int id = 0; id < intPA.size(); id++){
        int x = intPA[id].x;
        int y = intPA[id].y;
        pathA.push_back(ClipperLib::IntPoint(x, y));
    }

    for(int id = 0; id < intPB.size(); id++){
        int x = intPB[id].x;
        int y = intPB[id].y;
        pathB.push_back(ClipperLib::IntPoint(x, y));
    }

    ClipperLib::Clipper solver;
    solver.AddPath(pathA, ClipperLib::ptSubject, true);
    solver.AddPath(pathB, ClipperLib::ptClip, true);
    ClipperLib::Paths path_int;
    solver.Execute(ClipperLib::ctIntersection, path_int, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

    vector<Vector3f> polyInt;
    if(path_int.empty()) return polyInt;

    for(ClipperLib::IntPoint pt : path_int[0]){
        float x = pt.X / Scale;
        float y = pt.Y / Scale;
        Vector3f pos = x_axis * x + y_axis * y + origin;
        polyInt.push_back(pos);
    }

    vector<Vector3f> polySimplest;
    bool doAgain = true;
    float big_zero_eps = varList.get<float>("big_zero_eps");
    while(doAgain)
    {
        doAgain = false;
        polySimplest.clear();
        int N = polyInt.size();
        for(int id = 0; id < polyInt.size(); id++)
        {
            Vector3f ppt = polyInt[(id - 1 + N) % N];
            Vector3f pt =  polyInt[id];
            Vector3f npt = polyInt[(id + 1) % N];
            Vector3f tA = ppt - pt;
            Vector3f tB = npt - pt;
            if(len(tA) < big_zero_eps || len(tB) < big_zero_eps) {
                doAgain = true; continue;
            }
            double cross_product = len(tA CROSS tB)/len(tA)/len(tB);
            if(cross_product < big_zero_eps) {
                doAgain = true; continue;
            }
            polySimplest.push_back(pt);
        }
        polyInt = polySimplest;
    }
    return polyInt;
}

vector<Vector3f> PolyPolyIntersect(vector<Vector3f> polyA, vector<Vector3f> polyB)
{
    vector<Vector3f> overlapPolyPts;

    for (int i = 0; i < polyA.size(); i++)
    {
        // Note: avoid shared
        if ( GetPointIndexInList(polyA[i], polyB) == ELEMENT_OUT_LIST &&
             IsPointInPolygon(polyA[i], polyB) == true )
        {
            overlapPolyPts.push_back( polyA[i] );
        }
    }

    for (int i = 0; i < polyB.size(); i++)
    {
        if ( GetPointIndexInList(polyB[i], polyA) == ELEMENT_OUT_LIST &&
             IsPointInPolygon(polyB[i], polyA) == true )
        {
            overlapPolyPts.push_back( polyB[i] );
        }
    }

    if( overlapPolyPts.size() == 0 )
        return overlapPolyPts;
    overlapPolyPts.clear();
    for (int i = 0; i < polyA.size(); i++)
    {
        // Note: avoid shared
        if (IsPointInPolygon(polyA[i], polyB) == true )
        {
            overlapPolyPts.push_back( polyA[i] );
        }
    }

    for (int i = 0; i < polyB.size(); i++)
    {
        if (GetPointIndexInList(polyB[i], overlapPolyPts) == ELEMENT_OUT_LIST && 
            IsPointInPolygon(polyB[i], polyA) == true )
        {
            overlapPolyPts.push_back( polyB[i] );
        }
    }

    //printf("overlapPtNum: %d \n", overlapPolyPts.size());

    for (int i = 0, next = 1; i < polyA.size(); i++, next = (i + 1 == polyA.size()) ? 0 : i + 1)
    {
        vector<Vector3f> intersectPts = GetIntersectionPoints(polyA[i], polyA[next], polyB);

        for (int j = 0; j < intersectPts.size(); j++)
        {
            if (GetPointIndexInList(intersectPts[j], overlapPolyPts) == ELEMENT_OUT_LIST)
                overlapPolyPts.push_back( intersectPts[j] );
        }
    }

    return overlapPolyPts;
}

bool IsPointInPolygon(Vector3f point, vector<Vector3f> polygon)
{
    int i;
    int j;
    bool result = false;
    for (i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
    {
        if ((polygon[i].y > point.y) != (polygon[j].y > point.y) &&
            (point.x < (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x))
        {
            result = !result;
        }
    }
    return result;
}

bool IsEqual(double d1, double d2)
{
    const double EquityTolerance = 0.0000001;
    return abs(d1-d2) <= EquityTolerance;
}

vector<Vector3f> GetIntersectionPoints(Vector3f l1p1, Vector3f l1p2, vector<Vector3f> polygon)
{
    vector<Vector3f> intersectPts;

    for (int i = 0; i < polygon.size(); i++)
    {
        int next = (i + 1 == polygon.size()) ? 0 : i + 1;

        Vector3f intersectPt;
        bool isSuccess = GetIntersectionPoint(l1p1, l1p2, polygon[i], polygon[next], intersectPt);

        if ( isSuccess)
        {
            intersectPts.push_back( intersectPt);
        }
    }

    return intersectPts;
}

bool GetIntersectionPoint(Vector3f l1p1, Vector3f l1p2, Vector3f l2p1, Vector3f l2p2, Vector3f &intersectPt)
{
    double A1 = l1p2.y - l1p1.y;
    double B1 = l1p1.x - l1p2.x;
    double C1 = A1 * l1p1.x + B1 * l1p1.y;

    double A2 = l2p2.y - l2p1.y;
    double B2 = l2p1.x - l2p2.x;
    double C2 = A2 * l2p1.x + B2 * l2p1.y;

    //lines are parallel
    double det = A1 * B2 - A2 * B1;
    if (IsEqual(det, 0))
    {
        return false; //parallel lines
    }
    else
    {
        double x = (B2 * C1 - B1 * C2) / det;
        double y = (A1 * C2 - A2 * C1) / det;
        bool online1 =    ((_MIN(l1p1.x, l1p2.x) < x || IsEqual(_MIN(l1p1.x, l1p2.x), x))
                           && (_MAX(l1p1.x, l1p2.x) > x || IsEqual(_MAX(l1p1.x, l1p2.x), x))
                           && (_MIN(l1p1.y, l1p2.y) < y || IsEqual(_MIN(l1p1.y, l1p2.y), y))
                           && (_MAX(l1p1.y, l1p2.y) > y || IsEqual(_MAX(l1p1.y, l1p2.y), y))
        );
        bool online2 =    ((_MIN(l2p1.x, l2p2.x) < x || IsEqual(_MIN(l2p1.x, l2p2.x), x))
                           && (_MAX(l2p1.x, l2p2.x) > x || IsEqual(_MAX(l2p1.x, l2p2.x), x))
                           && (_MIN(l2p1.y, l2p2.y) < y || IsEqual(_MIN(l2p1.y, l2p2.y), y))
                           && (_MAX(l2p1.y, l2p2.y) > y || IsEqual(_MAX(l2p1.y, l2p2.y), y))
        );

        if (online1 && online2)
        {
            intersectPt = Vector3f(x, y, 0);
            return true;
        }
    }
    return false; //intersection is at out of at least one segment.
}




//**************************************************************************************//
//                          Post-process the overlap polygon
//**************************************************************************************//

void RearrangePoints(vector<Vector3f> &points)
{
    Vector3f cenPt = Vector3f(0,0,0);

    for(int i=0; i<points.size(); i++)
    {
        cenPt += points[i];
    }

    cenPt.x /= points.size();
    cenPt.y /= points.size();

    vector<float> angles;
    for (int i = 0; i < points.size(); i++)
    {
        float angle =  atan2(points[i].y - cenPt.y, points[i].x - cenPt.x);
        angles.push_back( angle );
    }

    vector<int> indices = BubbleSort(angles, true);

    vector<Vector3f> newPoints;
    for (int i = 0; i <  indices.size(); i++)
    {
        int index = indices[i];
        Vector3f point = points[index];

        newPoints.push_back( point);
    }

    points = newPoints;
}

vector<Vector3f> ProjectPolygonTo3D(vector<Vector3f> poly, double projMat[])
{
    vector<Vector3f> poly3D;

    for (int i = 0; i < poly.size(); i++)
    {
        Vector3f ver3D;
        MultiplyPoint( poly[i], projMat, ver3D);

        poly3D.push_back( ver3D );
    }

    return poly3D;
}

