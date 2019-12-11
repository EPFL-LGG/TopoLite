#ifndef POLYPLOYINTERSEC_H
#define POLYPLOYINTERSEC_H

#include <clipper.hpp>
#include "HelpFunc.h"
#include "TopoObject.h"

class PolyPolyIntersec: TopoObject
{
public:
    PolyPolyIntersec(shared_ptr<InputVarList> varList) : TopoObject(varList){

    }
    
public:
    void ComputePolygonsIntersection(
        const vector<Vector3f> &polyA,
        const vector<Vector3f> &polyB,
        vector<Vector3f> &polyIntsec)
    {
        polyIntsec.clear();

        _Polygon PA;
        PA.SetVertices(polyA);
        _Polygon PB;
        PB.SetVertices(polyB);

        Vector3f x_axis, y_axis, origin;
        PA.ComputeFrame(x_axis, y_axis, origin);
        float Scale = getVarList()->get<float>("clipper_scale");
        vector<Vector3i> intPA = PA.ProjectToNormalPlane(x_axis, y_axis, origin, Scale);
        vector<Vector3i> intPB = PB.ProjectToNormalPlane(x_axis, y_axis, origin, Scale);

        ClipperLib::Path pathA, pathB;
        for (int id = 0; id < intPA.size(); id++)
        {
            int x = intPA[id].x;
            int y = intPA[id].y;
            pathA.push_back(ClipperLib::IntPoint(x, y));
        }

        for (int id = 0; id < intPB.size(); id++)
        {
            int x = intPB[id].x;
            int y = intPB[id].y;
            pathB.push_back(ClipperLib::IntPoint(x, y));
        }

        ClipperLib::Clipper solver;
        solver.AddPath(pathA, ClipperLib::ptSubject, true);
        solver.AddPath(pathB, ClipperLib::ptClip, true);
        ClipperLib::Paths path_int;
        solver.Execute(ClipperLib::ctIntersection, path_int, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

        if (path_int.empty())
            return;

        for (ClipperLib::IntPoint pt : path_int[0])
        {
            float x = pt.X / Scale;
            float y = pt.Y / Scale;
            Vector3f pos = x_axis * x + y_axis * y + origin;
            polyIntsec.push_back(pos);
        }

        vector<Vector3f> polySimplest;
        bool doAgain = true;
        float big_zero_eps = getVarList()->get<float>("big_zero_eps");
        while (doAgain)
        {
            doAgain = false;
            polySimplest.clear();
            int N = polyIntsec.size();
            for (int id = 0; id < polyIntsec.size(); id++)
            {
                Vector3f ppt = polyIntsec[(id - 1 + N) % N];
                Vector3f pt = polyIntsec[id];
                Vector3f npt = polyIntsec[(id + 1) % N];
                Vector3f tA = ppt - pt;
                Vector3f tB = npt - pt;
                if (len(tA) < big_zero_eps || len(tB) < big_zero_eps)
                {
                    doAgain = true;
                    continue;
                }
                double cross_product = len(tA CROSS tB) / len(tA) / len(tB);
                if (cross_product < big_zero_eps)
                {
                    doAgain = true;
                    continue;
                }
                polySimplest.push_back(pt);
            }
            polyIntsec = polySimplest;
        }
        return;
    }

    void ProjectPolygonTo3D(const vector<Vector3f> &poly, double *projMat, vector<Vector3f> &poly3D)
    {
        poly3D.clear();

        for (int i = 0; i < poly.size(); i++)
        {
            Vector3f ver3D;
            MultiplyPoint(poly[i], projMat, ver3D);
            poly3D.push_back(ver3D);
        }

        return;
    }
};

#endif