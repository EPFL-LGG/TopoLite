#ifndef POLYPLOYINTERSEC_H
#define POLYPLOYINTERSEC_H

#include <clipper.hpp>
#include "HelpFunc.h"
#include "TopoObject.h"
#include "Mesh/Polygon.h"

class PolyPolyBoolean: TopoObject
{
public:
    PolyPolyBoolean(shared_ptr<InputVarList> varList) : TopoObject(varList){

    }
    
public:
    void ComputePolygonsUnion(
            vector<vector<Vector3f>> &polys,
            vector<vector<Vector3f>> &polysUnion
            )
    {
        polysUnion.clear();
        if(polys.empty())return;

        vector<_Polygon> Ps;
        for(int id = 0; id < polys.size(); id++)
        {
            _Polygon PA;
            PA.SetVertices(polys[id]);
            Ps.push_back(PA);
        }

        Vector3f x_axis, y_axis, origin;
        Ps[0].ComputeFrame(x_axis, y_axis, origin);
        float Scale = getVarList()->get<float>("clipper_scale");
        vector<ClipperLib::Path> clipperPaths;

        for(int id = 0; id < Ps.size(); id++)
        {
            vector<Vector3i> intPA = Ps[id].ProjectToNormalPlane(x_axis, y_axis, origin, Scale);
            Ps[id].ComputeNormal();
            ClipperLib::Path pathA;
            for (int jd = 0; jd < intPA.size(); jd++)
            {
                int x = intPA[jd].x;
                int y = intPA[jd].y;
                pathA.push_back(ClipperLib::IntPoint(x, y));
            }

            if(pathA.empty()) continue;

            ClipperLib::ClipperOffset offsetter;
            offsetter.AddPath(pathA, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
            ClipperLib::Paths off_sol;
            offsetter.Execute(off_sol, 10.0);
            TopoASSERT(off_sol.empty() == false);
            if(!off_sol.empty())
                clipperPaths.push_back(off_sol[0]);
        }

        ClipperLib::Clipper solver;
        solver.AddPaths(clipperPaths, ClipperLib::ptSubject, true);
        ClipperLib::Paths path_union;
        solver.Execute(ClipperLib::ctUnion, path_union, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

        if (path_union.empty())
            return;

        for(ClipperLib::Path path : path_union)
        {
            vector<Vector3f> polyUnion;
            for (ClipperLib::IntPoint pt : path)
            {
                float x = pt.X / Scale;
                float y = pt.Y / Scale;
                Vector3f pos = x_axis * x + y_axis * y + origin;
                polyUnion.push_back(pos);
            }
            cleanPath(polyUnion);
            polysUnion.push_back(polyUnion);
        }

        return;
    }

    void ComputePolygonsIntersection(
            const vector<Vector3f> &polyA,
            const vector<Vector3f> &polyB,
            vector<Vector3f> &polyIntsec)
    {
        vector<vector<Vector3f>> polylists;
        ComputePolygonsIntersection(polyA, polyB, polylists);
        if(!polylists.empty()){
            polyIntsec = polylists[0];
        }

        return;
    }

    void printPolygon(vector<Vector3f> poly){
        std::cout << "{";
        for (int id = 0; id < poly.size(); id++) {
            std::cout << "{" << poly[id].x << ", " << poly[id].y << "}, ";
        }
        std::cout << "}\n";
    }

    void ComputePolygonsIntersection(
        const vector<Vector3f> &polyA,
        const vector<Vector3f> &polyB,
        vector<vector<Vector3f>> &polyIntsec)
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

        for (int id = 0; id < intPB.size(); id++) {
            int x = intPB[id].x;
            int y = intPB[id].y;
            pathB.push_back(ClipperLib::IntPoint(x, y));
        }

        ClipperLib::Clipper solver;
        solver.AddPath(pathA, ClipperLib::ptSubject, true);
        solver.AddPath(pathB, ClipperLib::ptClip, true);
        ClipperLib::Paths path_int;
        solver.StrictlySimple(true);
        solver.Execute(ClipperLib::ctIntersection, path_int, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);
        ClipperLib::ClipperOffset offset;
        offset.AddPaths(path_int, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        offset.Execute(path_int, -100);
        ClipperLib::SimplifyPolygons(path_int);
        if (path_int.empty())
            return;

        for(int id = 0; id < path_int.size(); id++)
        {
            vector<Vector3f> polylist;
            for (ClipperLib::IntPoint pt : path_int[id])
            {
                float x = pt.X / Scale;
                float y = pt.Y / Scale;
                Vector3f pos = x_axis * x + y_axis * y + origin;
                polylist.push_back(pos);
            }
            cleanPath(polylist);
            polyIntsec.push_back(polylist);
        }

        return;
    }

    void cleanPath(vector<Vector3f> &polyIntsec)
    {
        vector<Vector3f> polySimplest;
        bool doAgain = true;
        float big_zero_eps = FLOAT_ERROR_LARGE;


        int N = polyIntsec.size();
        //remove duplicate points first
        for (int id = 0; id < polyIntsec.size(); id++) {

            Vector3f ppt = polyIntsec[(id - 1 + N) % N];
            Vector3f pt = polyIntsec[id];
            Vector3f npt = polyIntsec[(id + 1) % N];
            Vector3f tA = ppt - pt;
            Vector3f tB = npt - pt;
            if (len(tA) < big_zero_eps)
            {
                doAgain = true;
                continue;
            }
            else {
                polySimplest.push_back(pt);
            }
        }

        //remove points in a same line
        polyIntsec = polySimplest;
        polySimplest.clear();
        N = polyIntsec.size();
        for(int id = 0; id < polyIntsec.size(); id++){
            Vector3f ppt = polyIntsec[(id - 1 + N) % N];
            Vector3f pt = polyIntsec[id];
            Vector3f npt = polyIntsec[(id + 1) % N];
            Vector3f tA = ppt - pt;
            Vector3f tB = npt - pt;
            double cross_product = len(tA CROSS tB) / len(tA) / len(tB);
            if (cross_product > big_zero_eps) {
                polySimplest.push_back(pt);
            }
        }
        polyIntsec = polySimplest;
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