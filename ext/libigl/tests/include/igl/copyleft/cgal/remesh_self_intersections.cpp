#include <test_common.h>
#include <Eigen/Dense>

#include <igl/copyleft/cgal/remesh_self_intersections.h>
#include <igl/copyleft/cgal/RemeshSelfIntersectionsParam.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

TEST(RemeshSelfIntersections, CubeWithFold) {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    test_common::load_mesh("cube_with_fold.ply", V, F);

    typedef CGAL::Exact_predicates_exact_constructions_kernel K;
    typedef Eigen::Matrix<K::FT, Eigen::Dynamic, Eigen::Dynamic> MatrixXe;

    MatrixXe VV;
    Eigen::MatrixXi FF, IF;
    Eigen::VectorXi J, IM;
    igl::copyleft::cgal::RemeshSelfIntersectionsParam param;
    igl::copyleft::cgal::remesh_self_intersections(V, F, param, VV, FF, IF, J, IM);
}
