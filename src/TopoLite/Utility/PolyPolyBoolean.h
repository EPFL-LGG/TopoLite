#ifndef POLYPLOYINTERSEC_H
#define POLYPLOYINTERSEC_H

#include "clipper.hpp"
#include <cmath>
#include <Eigen/Dense>

#include "TopoObject.h"
#include "HelpDefine.h"
using Eigen::Matrix;
template<typename Scalar>
class PolyPolyBoolean: TopoObject
{

    typedef Matrix<Scalar, 3 ,1> Vector3;
    typedef Matrix<Scalar, 2 ,1> Vector2;

    typedef vector<Vector3> PolyVector3;
    typedef vector<vector<Vector3>> PolysVector3;

    typedef vector<Vector2> PolyVector2;
    typedef vector<vector<Vector2>> PolysVector2;

public:
    PolyPolyBoolean(shared_ptr<InputVarList> varList) : TopoObject(varList){

    }
    
public:
    void computePolygonsUnion(PolysVector3 &polys, PolysVector3 &polysUnion);

    void computePolygonsIntersection(const PolyVector3 &polyA, const PolyVector3 &polyB, PolyVector3 &polyIntsec);

    void computePolygonsIntersection(const PolyVector3 &polyA, const PolyVector3 &polyB, PolysVector3 &polyIntsec);

    bool check2DPolygonsIntersection(const PolyVector2 &polyA, const PolyVector2 &polyB, Scalar &area);

public:

    void printPolygon(const PolyVector3& poly);

    void cleanPath(PolyVector3 &polyIntsec);

    void computeFrame(const PolyVector3 &poly, Vector3 &xaxis, Vector3 &yaxis, Vector3 &origin);

    Scalar computeScale(const PolysVector3 &poly, Vector3 &xaxis, Vector3 &yaxis, Vector3 &origin);

    Scalar computeScale(const PolysVector2 &poly);

    Vector3 computeNormal(const PolyVector3 &poly);

    Vector3 computeCenter(const PolyVector3 &poly);

    ClipperLib::Path projectToNormalPlane(const PolyVector3 &poly, Vector3 xaxis, Vector3 yaxis, Vector3 origin, Scalar Scale);

    PolyVector3 projectTo3D(const ClipperLib::Path &path, Vector3 xaxis, Vector3 yaxis, Vector3 origin, Scalar scale);
};

template<typename Scalar>
void PolyPolyBoolean<Scalar>::computePolygonsUnion(PolyPolyBoolean::PolysVector3 &polys,
                                                   PolyPolyBoolean::PolysVector3 &polysUnion) {
    polysUnion.clear();
    if(polys.empty())return;

    Vector3 x_axis, y_axis, origin;
    computeFrame(polys[0], x_axis, y_axis, origin);

    Scalar Scale = computeScale(polys, x_axis, y_axis, origin);

    vector<ClipperLib::Path> clipperPaths;

    for(size_t id = 0; id < polys.size(); id++)
    {
        ClipperLib::Path path;
        path = projectToNormalPlane(polys[id], x_axis, y_axis, origin, Scale);

        ClipperLib::ClipperOffset offsetter;
        offsetter.AddPath(path, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
        ClipperLib::Paths off_sol;
        offsetter.Execute(off_sol, 10.0);
        TopoASSERT(off_sol.empty() == false);
        if(!off_sol.empty()) clipperPaths.push_back(off_sol[0]);
    }

    ClipperLib::Clipper solver;
    solver.AddPaths(clipperPaths, ClipperLib::ptSubject, true);
    ClipperLib::Paths path_union;
    solver.Execute(ClipperLib::ctUnion, path_union, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

    if (path_union.empty())
        return;

    for(ClipperLib::Path path : path_union)
    {
        PolyVector3 polyUnion = projectTo3D(path, x_axis, y_axis, origin, Scale);
        cleanPath(polyUnion);
        polysUnion.push_back(polyUnion);
    }

    return;
}

template<typename Scalar>
void PolyPolyBoolean<Scalar>::computePolygonsIntersection(const PolyPolyBoolean::PolyVector3 &polyA,
                                                          const PolyPolyBoolean::PolyVector3 &polyB,
                                                          PolyPolyBoolean::PolyVector3 &polyIntsec) {
    PolysVector3 polylists;
    computePolygonsIntersection(polyA, polyB, polylists);
    if(!polylists.empty()){
        polyIntsec = polylists[0];
    }

    return;
}

template<typename Scalar>
void PolyPolyBoolean<Scalar>::printPolygon(const PolyPolyBoolean::PolyVector3 &poly) {
    std::cout << "{";
    for (size_t id = 0; id < poly.size(); id++) {
        std::cout << "{" << poly[id].x << ", " << poly[id].y << "}, ";
    }
    std::cout << "}\n";
}

template<typename Scalar>
void PolyPolyBoolean<Scalar>::computePolygonsIntersection(const PolyPolyBoolean::PolyVector3 &polyA,
                                                          const PolyPolyBoolean::PolyVector3 &polyB,
                                                          PolyPolyBoolean::PolysVector3 &polyIntsec) {
    polyIntsec.clear();

    Vector3 x_axis, y_axis, origin;
    computeFrame(polyA, x_axis, y_axis, origin);

    PolysVector3 polys;
    polys.push_back(polyA);
    polys.push_back(polyB);

    Scalar Scale = computeScale(polys, x_axis, y_axis, origin);

    ClipperLib::Path pathA, pathB;
    pathA = projectToNormalPlane(polyA, x_axis, y_axis, origin, Scale);
    pathB = projectToNormalPlane(polyB, x_axis, y_axis, origin, Scale);

    ClipperLib::Clipper solver;
    solver.AddPath(pathA, ClipperLib::ptSubject, true);
    solver.AddPath(pathB, ClipperLib::ptClip, true);
    ClipperLib::Paths path_int;
    solver.StrictlySimple(true);
    solver.Execute(ClipperLib::ctIntersection, path_int, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);
    ClipperLib::ClipperOffset offset;
    offset.AddPaths(path_int, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
    offset.Execute(path_int, -10);
    ClipperLib::SimplifyPolygons(path_int);
    if (path_int.empty())
        return;

    for(size_t id = 0; id < path_int.size(); id++)
    {
        PolyVector3 polylist = projectTo3D(path_int[id], x_axis, y_axis, origin, Scale);
        cleanPath(polylist);
        polyIntsec.push_back(polylist);
    }

    return;
}

template<typename Scalar>
bool PolyPolyBoolean<Scalar>::check2DPolygonsIntersection(const PolyPolyBoolean::PolyVector2 &polyA,
                                                          const PolyPolyBoolean::PolyVector2 &polyB,
                                                          Scalar &area){
    PolysVector2 polys;
    polys.push_back(polyA);
    polys.push_back(polyB);
    Scalar Scale = computeScale(polys);

    ClipperLib::Path pathA, pathB;
    for(size_t id = 0; id < polyA.size(); id++){
        int x = polyA[id].x() * Scale;
        int y = polyA[id].y() * Scale;
        pathA.push_back(ClipperLib::IntPoint(x, y));
    }

    for(size_t id = 0; id < polyB.size(); id++){
        int x = polyB[id].x() * Scale;
        int y = polyB[id].y() * Scale;
        pathB.push_back(ClipperLib::IntPoint(x, y));
    }

    if(!ClipperLib::Orientation(pathA))
    {
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

    if(path_int.empty() || path_int.front().empty())
    {
        area = 0;
        return false;
    }
    else{
        area = 0;
        for(ClipperLib::Path path : path_int){
            area += ClipperLib::Area(path) / Scale / Scale;
        }
        return true;
    }
}

template<typename Scalar>
void PolyPolyBoolean<Scalar>::cleanPath(PolyPolyBoolean::PolyVector3 &polyIntsec)
{
    PolyVector3 polySimplest;
    bool doAgain = true;
    float big_zero_eps = FLOAT_ERROR_LARGE;


    int N = polyIntsec.size();
    //remove duplicate points first
    for (size_t id = 0; id < polyIntsec.size(); id++) {

        Vector3 ppt = polyIntsec[(id - 1 + N) % N];
        Vector3 pt = polyIntsec[id];
        Vector3 npt = polyIntsec[(id + 1) % N];
        Vector3 tA = ppt - pt;
        Vector3 tB = npt - pt;
        if (tA.norm() < big_zero_eps)
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
    for(size_t id = 0; id < polyIntsec.size(); id++){
        Vector3 ppt = polyIntsec[(id - 1 + N) % N];
        Vector3 pt = polyIntsec[id];
        Vector3 npt = polyIntsec[(id + 1) % N];
        Vector3 tA = ppt - pt;
        Vector3 tB = npt - pt;
        double cross_product = (tA.cross(tB)).norm() / tA.norm() / tB.norm();
        if (cross_product > big_zero_eps) {
            polySimplest.push_back(pt);
        }
    }
    polyIntsec = polySimplest;
}

template<typename Scalar>
void PolyPolyBoolean<Scalar>::computeFrame(const PolyPolyBoolean::PolyVector3 &poly, PolyPolyBoolean::Vector3 &xaxis,
                                           PolyPolyBoolean::Vector3 &yaxis, PolyPolyBoolean::Vector3 &origin) {
    Vector3 normal = computeNormal(poly);
    Vector3 center = computeCenter(poly);

    xaxis = normal.cross(Vector3(1, 0, 0));
    if(xaxis.norm() < FLOAT_ERROR_LARGE)
        xaxis = normal.cross(Vector3(0, 1, 0));
    xaxis.normalize();

    yaxis = normal.cross(xaxis);
    yaxis.normalize();

    origin = center;

    return;
}

template<typename Scalar>
Matrix<Scalar, 3, 1> PolyPolyBoolean<Scalar>::computeNormal(const PolyPolyBoolean::PolyVector3 &poly)
{
    Vector3 normal(0, 0 ,0), center(0, 0, 0);
    center = computeCenter(poly);

    for(int id = 0; id < (int)(poly.size()) - 1; id++){
        normal += (poly[id] - center).cross(poly[id + 1] - center);
    }

    if(normal.norm() < FLOAT_ERROR_LARGE)
        return Vector3(0, 0, 0);

    return normal.normalized();
}

template<typename Scalar>
Matrix<Scalar, 3, 1> PolyPolyBoolean<Scalar>::computeCenter(const PolyPolyBoolean::PolyVector3 &poly)
{
    Vector3 center(0, 0, 0);

    for(size_t id = 0; id < poly.size(); id++)
    {
        center += poly[id];
    }
    if(!poly.empty())
        center /= poly.size();
    else
        center = Vector3(0, 0, 0);

    return center;
}

template<typename Scalar>
ClipperLib::Path
PolyPolyBoolean<Scalar>::projectToNormalPlane(  const PolyPolyBoolean::PolyVector3 &poly,
                                                PolyPolyBoolean::Vector3 xaxis,
                                                PolyPolyBoolean::Vector3 yaxis,
                                                PolyPolyBoolean::Vector3 origin,
                                                Scalar Scale) {
    ClipperLib::Path path;
    for(size_t id = 0; id < poly.size(); id++)
    {
        Vector3 pos = poly[id];
        int x = (int)((pos - origin).dot(xaxis) * Scale);
        int y = (int)((pos - origin).dot(yaxis) * Scale);
        path.push_back(ClipperLib::IntPoint(x, y));
    }

    return path;
}

template<typename Scalar>
vector<Matrix<Scalar, 3, 1>>
PolyPolyBoolean<Scalar>::projectTo3D(const ClipperLib::Path &path,
                                     PolyPolyBoolean::Vector3 xaxis,
                                     PolyPolyBoolean::Vector3 yaxis,
                                     PolyPolyBoolean::Vector3 origin,
                                     Scalar Scale) {

    PolyVector3 poly;
    for(size_t id = 0; id < path.size(); id++)
    {
        float x = path[id].X / Scale;
        float y = path[id].Y / Scale;
        Vector3 pos = xaxis * x + yaxis * y + origin;
        poly.push_back(pos);
    }

    return poly;
}

template<typename Scalar>
Scalar PolyPolyBoolean<Scalar>::computeScale(const PolyPolyBoolean::PolysVector3 &polys, PolyPolyBoolean::Vector3 &xaxis,
                                           PolyPolyBoolean::Vector3 &yaxis, PolyPolyBoolean::Vector3 &origin){

    Scalar Scale = 1;
    int maxdigit = 0;
    for(PolyVector3 poly:polys)
    {
        for(size_t id = 0; id < poly.size(); id++)
        {
            Vector3 pos = poly[id];
            Scalar x = std::abs((pos - origin).dot(xaxis));
            Scalar y = std::abs((pos - origin).dot(yaxis));
            int digit = std::floor(std::max(std::log10(x) + 1, std::log10(y) + 1));
            maxdigit = std::max(digit, maxdigit);
        }
    }

    Scale = std::max(Scale, (Scalar)(std::pow(10, std::max(0, 8 - maxdigit))));
    return Scale;
}

template<typename Scalar>
Scalar PolyPolyBoolean<Scalar>::computeScale(const PolyPolyBoolean::PolysVector2 &polys) {

    Scalar Scale = 1;
    int maxdigit = 0;

    for(PolyVector2 poly:polys)
    {
        for(size_t id = 0; id < poly.size(); id++)
        {
            Vector2 pos = poly[id];
            Scalar x = pos.x();
            Scalar y = pos.y();
            int digit = std::floor(std::max(std::log10(x) + 1, std::log10(y) + 1));
            maxdigit = std::max(digit, maxdigit);
        }
    }

    Scale = std::max(Scale, std::pow(10, std::max(0, 8 - maxdigit)));
    return Scale;
}


#endif
