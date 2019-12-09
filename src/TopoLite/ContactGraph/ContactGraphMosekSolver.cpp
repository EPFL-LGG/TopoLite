//
// Created by ziqwang on 14.01.19.
//

#include "ContactGraphMosekSolver.h"
#include <tbb/tbb.h>
#include <fstream>
#include "IO/gluiVar.h"
#include <Eigen/QR>
#include "Utility/HelpFunc.h"
#include "Utility/vec.h"
extern gluiVarList varList;
Eigen::MatrixXd RImatrix;
ContactGraphMosekSolver::ContactGraphMosekSolver()
{
    printEquilibrium = true;
    force.setZero();

    mosek_rbeForce_upperBound = varList.get<float>("mosek_rbeForce_upperBound");
    mosek_rbe_eps = varList.get<float>("mosek_rbe_eps");
    mosek_rotational_interlocking_eps = mosek_translational_interlocking_eps = varList.get<float>("mosek_interlocking_eps");
    mosek_intpntCoTolRelGap =  varList.get<float>("mosek_intpntCoTolRelGap");

    slope_binarySearch_eps = varList.get<float>("slope_binarySearch_eps");
    slope_bound_eps = varList.get<float>("slope_bound_eps");
    slope_num_uniform_sample = 16;
    slope_num_init_sample = varList.get<int>("slope_num_init_sample");

    small_zero_eps = varList.get<float>("small_zero_eps");
}

void ContactGraphMosekSolver::computeTranslationalInterlockingMatrix(mosek::fusion::Matrix::t &mat, Eigen::Vector2i &size)
{
    vector<EigenTriple> triplist;
    ContactGraph::computeTranslationalInterlockingMatrix(triplist, size);

    size_t n_value = triplist.size();
    shared_ptr<monty::ndarray<int, 1>> row_index (new monty::ndarray<int,1>(monty::shape(n_value), [triplist](ptrdiff_t i) {return triplist[i].row();}));
    shared_ptr<monty::ndarray<int, 1>> col_index (new monty::ndarray<int,1>(monty::shape(n_value), [triplist](ptrdiff_t i) {return triplist[i].col();}));
    shared_ptr<monty::ndarray<double, 1>> m_content (new monty::ndarray<double,1>(monty::shape(n_value), [triplist](ptrdiff_t i) {return triplist[i].value();}));
    mat = mosek::fusion::Matrix::sparse(size[0], size[1], row_index, col_index, m_content);
    return;
}

void ContactGraphMosekSolver::computeRotationalInterlockingMatrix(mosek::fusion::Matrix::t &mat, Eigen::Vector2i &size) {
    vector<EigenTriple> triplist;
    ContactGraph::computeRotationalInterlockingMatrix(triplist, size);

    size_t n_value = triplist.size();
    shared_ptr<monty::ndarray<int, 1>> row_index (new monty::ndarray<int,1>(monty::shape(n_value), [triplist](ptrdiff_t i) {return triplist[i].row();}));
    shared_ptr<monty::ndarray<int, 1>> col_index (new monty::ndarray<int,1>(monty::shape(n_value), [triplist](ptrdiff_t i) {return triplist[i].col();}));
    shared_ptr<monty::ndarray<double, 1>> m_content (new monty::ndarray<double,1>(monty::shape(n_value), [triplist](ptrdiff_t i) {return triplist[i].value();}));
    mat = mosek::fusion::Matrix::sparse(size[0], size[1], row_index, col_index, m_content);
    Eigen::SparseMatrix<double> spmat(size[0], size[1]);
    spmat.setFromTriplets(triplist.begin(), triplist.end());
    RImatrix = Eigen::MatrixXd(spmat);
    return;
}

bool ContactGraphMosekSolver::isTranslationalInterlocking(string &log)
{
    /////////////////////////////////////////////////////////
    // 1. Build linear programming matrix
    Eigen::Vector2i size;
    mosek::fusion::Matrix::t A;
    computeTranslationalInterlockingMatrix(A, size);

    /////////////////////////////////////////////////////////
    // 2. Set Constraints

    mosek::fusion::Model::t M = new mosek::fusion::Model("isTranslationalInterlocking");
    auto _M = monty::finally([&]() { M->dispose(); });
    auto x = M->variable(size[1], mosek::fusion::Domain::unbounded());
    auto t = M->variable(size[0], mosek::fusion::Domain::inRange(0.,10.));

    M -> constraint("no collision constraint", mosek::fusion::Expr::sub(mosek::fusion::Expr::mul(A, x), t), mosek::fusion::Domain::greaterThan(0.0));

    /////////////////////////////////////////////////////////
    // 3. Solve the linear programming

    M -> objective("obj", mosek::fusion::ObjectiveSense::Maximize, mosek::fusion::Expr::sum(t));
    M -> setSolverParam("intpntCoTolRelGap", mosek_intpntCoTolRelGap);
    //M ->setLogHandler([=](const std::string &msg) { std::cout << msg << std::flush; });
    M -> solve();

    if(M->getProblemStatus(mosek::fusion::SolutionType::Interior) != mosek::fusion::ProblemStatus::PrimalAndDualFeasible)
    {
        return -1;
    }



    /////////////////////////////////////////////////////////
    // 4. Analysis optimization result
    auto aux_var = t->level();
    double aux_sum = 0;
    for(auto& num : *aux_var) {
        aux_sum += num;
    }

    translation = Eigen::VectorXd();
    if (aux_sum > mosek_translational_interlocking_eps){
        translation = Eigen::VectorXd::Zero(size[1]);
        for(int id = 0; id < size[1]; id++){
            translation(id) = (*x->level())[id];
        }
        translation /= translation.norm();
//        for(int id = 0; id < dynamic_nodes.size(); id++){
//            std::cout << dynamic_nodes[id].lock()->staticID << ":\t";
//            std::cout << translation(3 * id) << ", " << translation(3 * id + 1) << ", " <<  translation(3 * id + 2) << std::endl;
//        }
        return false;
    }
    return true;
}

bool ContactGraphMosekSolver::isRotationalInterlocking(string &log) {
    /////////////////////////////////////////////////////////
    // 1. Build linear programming matrix
    Eigen::Vector2i size;
    mosek::fusion::Matrix::t A;
    computeRotationalInterlockingMatrix(A, size);

    /////////////////////////////////////////////////////////
    // 2. Set Constraints

    mosek::fusion::Model::t M = new mosek::fusion::Model("isRotationalInterlocking");
    auto _M = monty::finally([&]() { M->dispose(); });
    auto x = M->variable(size[1], mosek::fusion::Domain::unbounded());
    auto t = M->variable(size[0], mosek::fusion::Domain::inRange(0.,10.));

    M -> constraint("no collision constraint", mosek::fusion::Expr::sub(mosek::fusion::Expr::mul(A, x), t), mosek::fusion::Domain::greaterThan(0.0));
    if(dynamic_nodes.size() == nodes.size()){
        //for closed surface
        std::cout << "fixed constraint activate" << std::endl;
        M->constraint("fixed constraint", x->slice(0, 5), mosek::fusion::Domain::equalsTo(0.0f));
    }

    /////////////////////////////////////////////////////////
    // 3. Solve the linear programming

    M -> objective("obj", mosek::fusion::ObjectiveSense::Maximize, mosek::fusion::Expr::sum(t));
    M->setSolverParam("optimizer", "dualSimplex");
    M->setSolverParam("basisRelTolS", 1e-15);
    //M ->setLogHandler([&](const std::string &msg) { std::cout << msg << std::endl; });
    M -> solve();

    if(M->getProblemStatus(mosek::fusion::SolutionType::Basic) != mosek::fusion::ProblemStatus::PrimalAndDualFeasible)
    {
        return -1;
    }

    /////////////////////////////////////////////////////////
    // 4. Analysis optimization result
    auto aux_var = t->level();
    double aux_sum = 0;
    for(auto& num : *aux_var) aux_sum += num;
    if (aux_sum > mosek_rotational_interlocking_eps)
        return false;
    return true;
}

bool ContactGraphMosekSolver::isEquilibrium( EigenPoint gravity, string &log, bool saveForce)
{
    double friction_coeff = varList.get<float>("mosek_rbe_friction_coeff");
    if(friction_coeff < varList.get<float>("big_zero_eps")){
        /*
     * Get equilibrium matrix
     */
        Eigen::MatrixXd Aeq;
        computeEquilibriumMatrix(Aeq);

//    std::ofstream fout;
//    fout.open("EQmatrix.txt");
//    Eigen::IOFormat matlabFormat(6, 0, ", ", ";\n", "[", "]", "[", "]");
//    fout << Aeq.format(matlabFormat) << std::endl;
//    fout.close();

        int m = Aeq.rows();
        int n = Aeq.cols();

        /*
         * Init Optimization Problem
         */
        mosek::fusion::Model::t M = new mosek::fusion::Model("isEquilibrium");
        auto _M = monty::finally([&]() { M->dispose(); });
        //auto f = M->variable(n, mosek::fusion::Domain::inRange(0, mosek_rbeForce_upperBound));
        auto f = M->variable(n, mosek::fusion::Domain::greaterThan(0.0));

        /*
         * Convert MatrixXf to Mosek:Matrix
         */

        shared_ptr<monty::ndarray<double,2>> Aeq_ndarray = std::make_shared<monty::ndarray<double, 2>>(monty::shape(m,n),
                                                                                                       std::function<double(const monty::shape_t<2> &)>(
                                                                                                               [&](const monty::shape_t<2> & p)
                                                                                                               {
                                                                                                                   return Aeq(p[0], p[1]);
                                                                                                               }));
        mosek::fusion::Matrix::t Mosek_Aeq = mosek::fusion::Matrix::sparse(Aeq_ndarray);
        /*
         * Add Aeq constraint
         * Aeq * f - Gravity = 0
         */
        shared_ptr<monty::ndarray<double,1>> gravity_ndarray = std::make_shared<monty::ndarray<double,1>>(m);
        for(int id = 0; id < m / 6; id++){
            (*gravity_ndarray)[id * 6]     = gravity[0] * dynamic_nodes[id].lock()->mass;
            (*gravity_ndarray)[id * 6 + 1] = gravity[1] * dynamic_nodes[id].lock()->mass;
            (*gravity_ndarray)[id * 6 + 2] = gravity[2] * dynamic_nodes[id].lock()->mass;
            (*gravity_ndarray)[id * 6 + 3] = 0;
            (*gravity_ndarray)[id * 6 + 4] = 0;
            (*gravity_ndarray)[id * 6 + 5] = 0;
        }
        M->constraint("Aeq", mosek::fusion::Expr::add(mosek::fusion::Expr::mul(Mosek_Aeq, f), gravity_ndarray), mosek::fusion::Domain::equalsTo(0.0));

        /*
         * Add positive fni+, fni-
         */
        for(int id = 0; id < n/2; id++){
            M->constraint("f+^" + std::to_string(id), f->index(id * 2)    ,  mosek::fusion::Domain::greaterThan(0.0));

            if(!varList.get<bool>("showCompression")){
                M->constraint("f-^" + std::to_string(id), f->index(id * 2 + 1),  mosek::fusion::Domain::greaterThan(0));
            }
            else{
                M->constraint("f-^" + std::to_string(id), f->index(id * 2 + 1),  mosek::fusion::Domain::equalsTo(0));
            }
        }

        /*
        * Objective function
        */
        shared_ptr<monty::ndarray<int,1>> pick_indexs = std::make_shared<monty::ndarray<int,1>>(n / 2);
        for(int id = 0, jd = 0; id < n / 2; id++){
            if(!varList.get<bool>("showCompression")){
                (*pick_indexs)[jd ++] =  2 * id + 1;
            }
            else{
                (*pick_indexs)[jd ++] =  2 * id;
            }
        }
        auto f_neg = f->pick(pick_indexs);
        auto t = M->variable(1, mosek::fusion::Domain::greaterThan(0.0));
        M->constraint("QP", mosek::fusion::Expr::vstack(t, f_neg), mosek::fusion::Domain::inQCone(n / 2 + 1));
        M->objective("QP_obj", mosek::fusion::ObjectiveSense::Minimize, t);
        M->setSolverParam("intpntCoTolRelGap", mosek_intpntCoTolRelGap);

        M->solve();
        if(M->getProblemStatus(mosek::fusion::SolutionType::Interior) != mosek::fusion::ProblemStatus::PrimalAndDualFeasible)
        {
            return false;
        }

        Eigen::VectorXd Vgravity(m);
        double value =  M->dualObjValue();
        if(saveForce){
            force.setZero();
            force.resize(n);
            for(int id = 0; id < n; id ++){
                force(id) = (*f->level())[id];
            }
            for(int id = 0; id < m / 6; id++){
                Vgravity[id * 6] =     gravity[0] * dynamic_nodes[id].lock()->mass;
                Vgravity[id * 6 + 1] = gravity[1] * dynamic_nodes[id].lock()->mass;
                Vgravity[id * 6 + 2] = gravity[2] * dynamic_nodes[id].lock()->mass;
                Vgravity[id * 6 + 3] = 0;
                Vgravity[id * 6 + 4] = 0;
                Vgravity[id * 6 + 5] = 0;
            }

            //double value = (*t->level())[0];
            if(printEquilibrium) std::cout << "Equilibrium : " << value << ", Aeq f + gravity:" << (Aeq * force + Vgravity).norm() << std::endl;
        }
        else{
            if(printEquilibrium) std::cout << "Equilibrium : " << value << std::endl;
        }

        //if(value > 0.1)
        if(value > varList.get<float>("mosek_rbe_eps") && !varList.get<bool>("showCompression"))
            return  false;
        else
            return  true;
    }
    else{
        /*
        * Get equilibrium matrix
        */
        Eigen::MatrixXd Aeq;
        computeEquilibriumMatrix(Aeq, true);

            std::ofstream fout;
    fout.open("EQmatrix.txt");
    //Eigen::IOFormat matlabFormat(6, 0, ", ", ";\n", "[", "]", "[", "]");
    //fout << Aeq.format(matlabFormat) << std::endl;
        fout << Aeq << std::endl;
    fout.close();

        int m = Aeq.rows();
        int n = Aeq.cols();
        /*
         * Init Optimization Problem
         */
        mosek::fusion::Model::t M = new mosek::fusion::Model("isEquilibrium");
        auto _M = monty::finally([&]() { M->dispose(); });
        auto f = M->variable(n);

        /*
         * Convert MatrixXf to Mosek:Matrix
         */

        shared_ptr<monty::ndarray<double,2>> Aeq_ndarray = std::make_shared<monty::ndarray<double, 2>>(monty::shape(m,n),
                                                                                                       std::function<double(const monty::shape_t<2> &)>(
                                                                                                               [&](const monty::shape_t<2> & p)
                                                                                                               {
                                                                                                                   return Aeq(p[0], p[1]);
                                                                                                               }));
        mosek::fusion::Matrix::t Mosek_Aeq = mosek::fusion::Matrix::sparse(Aeq_ndarray);
        /*
         * Add Aeq constraint
         * Aeq * f - Gravity = 0
         */

        shared_ptr<monty::ndarray<double,1>> gravity_ndarray = std::make_shared<monty::ndarray<double,1>>(m);
        for(int id = 0; id < m / 6; id++){
            (*gravity_ndarray)[id * 6]     = gravity[0] * dynamic_nodes[id].lock()->mass;
            (*gravity_ndarray)[id * 6 + 1] = gravity[1] * dynamic_nodes[id].lock()->mass;
            (*gravity_ndarray)[id * 6 + 2] = gravity[2] * dynamic_nodes[id].lock()->mass;
            (*gravity_ndarray)[id * 6 + 3] = 0;
            (*gravity_ndarray)[id * 6 + 4] = 0;
            (*gravity_ndarray)[id * 6 + 5] = 0;
        }
        M->constraint("Aeq", mosek::fusion::Expr::add(mosek::fusion::Expr::mul(Mosek_Aeq, f), gravity_ndarray), mosek::fusion::Domain::equalsTo(0.0));

        /*
         * Add positive fni+, fni-
         */
        for(int id = 0; id < n/4; id++)
        {
            M->constraint("f+^" + std::to_string(id), f->index(id * 4)    ,  mosek::fusion::Domain::greaterThan(0));

            if(!varList.get<bool>("showCompression")){
                M->constraint("f-^" + std::to_string(id), f->index(id * 4 + 1),  mosek::fusion::Domain::greaterThan(0));
            }
            else{
                M->constraint("f-^" + std::to_string(id), f->index(id * 4 + 1),  mosek::fusion::Domain::equalsTo(0));
            }

            M->constraint("friction_u-" + std::to_string(id),
                          mosek::fusion::Expr::sub(
                                  mosek::fusion::Expr::mul(f->index(id * 4), friction_coeff),
                                  f->index(id * 4 + 2)), mosek::fusion::Domain::greaterThan(0));
            M->constraint("friction_u+" + std::to_string(id),
                          mosek::fusion::Expr::add(
                                  mosek::fusion::Expr::mul(f->index(id * 4), friction_coeff),
                                  f->index(id * 4 + 2)), mosek::fusion::Domain::greaterThan(0));
            M->constraint("friction_v-" + std::to_string(id),
                          mosek::fusion::Expr::sub(
                                  mosek::fusion::Expr::mul(f->index(id * 4), friction_coeff),
                                  f->index(id * 4 + 3)), mosek::fusion::Domain::greaterThan(0));
            M->constraint("friction_v+" + std::to_string(id),
                          mosek::fusion::Expr::add(
                                  mosek::fusion::Expr::mul(f->index(id * 4), friction_coeff),
                                  f->index(id * 4 + 3)), mosek::fusion::Domain::greaterThan(0));
        }

        /*
        * Objective function
        */
        shared_ptr<monty::ndarray<int,1>> pick_indexs = std::make_shared<monty::ndarray<int,1>>(n/4);
        for(int id = 0, jd = 0; id < n/4; id++){
            if(!varList.get<bool>("showCompression")){
                (*pick_indexs)[jd ++] =  4 * id + 1;
            }
            else{
                (*pick_indexs)[jd ++] =  4 * id;
            }
        }

        auto f_neg = f->pick(pick_indexs);
        auto t = M->variable(1, mosek::fusion::Domain::greaterThan(0.0));
        M->constraint("QP", mosek::fusion::Expr::vstack(t, f_neg), mosek::fusion::Domain::inQCone(n/4 + 1));
        M->objective("QP_obj", mosek::fusion::ObjectiveSense::Minimize, t);
        //M->setLogHandler([&](const std::string &msg) { std::cout << msg << std::endl;});
        M->setSolverParam("intpntCoTolRelGap", mosek_intpntCoTolRelGap);

        M->solve();
        if(M->getProblemStatus(mosek::fusion::SolutionType::Interior) != mosek::fusion::ProblemStatus::PrimalAndDualFeasible)
        {
            return false;
        }


        double value =  M->dualObjValue();
        if(saveForce){
            Eigen::VectorXd Vgravity(m);
            force.setZero();
            force.resize(n / 2);
            double min_force = std::numeric_limits<double>::max();
            double max_force = -std::numeric_limits<double>::max();
            for(int id = 0; id < n / 4; id ++){
                force(2 * id) = (*f->level())[ 4 * id];
                force(2 * id + 1) = (*f->level())[ 4 * id + 1];
                if(min_force > (*f->level())[ 4 * id]) min_force = (*f->level())[ 4 * id];
                if(max_force < (*f->level())[ 4 * id]) max_force = (*f->level())[ 4 * id];
            }
            for(int id = 0; id < m / 6; id++){
                Vgravity[id * 6] =     gravity[0] * dynamic_nodes[id].lock()->mass;
                Vgravity[id * 6 + 1] = gravity[1] * dynamic_nodes[id].lock()->mass;
                Vgravity[id * 6 + 2] = gravity[2] * dynamic_nodes[id].lock()->mass;
                Vgravity[id * 6 + 3] = 0;
                Vgravity[id * 6 + 4] = 0;
                Vgravity[id * 6 + 5] = 0;
            }

            if(printEquilibrium) std::cout << "Equilibrium : " << value << ", max F = " << max_force << ", min F = " << min_force << std::endl;
        }
        else{
            if(printEquilibrium) std::cout << "Equilibrium : " << value << std::endl;

        }

        //if(value > 0.1)
        if(value > varList.get<float>("mosek_rbe_eps") && !varList.get<bool>("showCompression"))
            return  false;
        else
            return  true;
    }

}

double ContactGraphMosekSolver::computeSlope(EigenPoint gravity, EigenPoint laterial, double init_angle)
{
    printEquilibrium = false;
    bool isUpperBoundSet = false;
    double low = init_angle;
    double high = std::max(M_PI_2 / 90.0, low);

    gravity.normalized();
    laterial.normalized();

    string log;
    force.setZero();
    while(!isUpperBoundSet && high < M_PI_2)
    {
        EigenPoint test_gravity = gravity * std::cos(high) + laterial * std::sin(high);
        if(isEquilibrium(test_gravity, log, false))
        {
            low = high;
            high = std::min(M_PI_2, high * 2);
        }
        else{
            isUpperBoundSet = true;
            break;
        }
    }

    double tiltDegree = low / M_PI * 180;
    double slope_binarySearch_eps_rad = slope_binarySearch_eps / 180 * M_PI;
    while((high - low) > slope_binarySearch_eps_rad){
        float mid = (low + high) / 2;
        //EigenPoint test_direction = gravity * std::cos(mid) + laterial * std::sin(mid);
        EigenPoint test_direction = gravity + laterial * std::tan(mid);
        if(isEquilibrium(test_direction, log, false)){
            tiltDegree = mid / M_PI * 180;
            low = mid;
        }
        else{
            high = mid;
        }
    }
    return tiltDegree;
}

double ContactGraphMosekSolver::computeSlopeUniform(EigenPoint gravity, int n_sample)
{
    string log;
    bool equilibrium = isEquilibrium(gravity, log, false);
    if(!equilibrium)
        return  0;

    /*
     * 1. Compute local u, v directions perpendicular to the gravity
     */

    gravity.normalized();

    EigenPoint u_axis, v_axis;
    u_axis = EigenPoint(0, 0, 1).cross(gravity);
    if((u_axis).norm() < small_zero_eps) u_axis = EigenPoint(1, 0, 0).cross(gravity);
    u_axis.normalized();
    v_axis = gravity.cross(u_axis);
    v_axis.normalized();

    /*
     * 2. Compute the tile angle in several initial directions
     */

    vector<EigenPoint> directions; directions.resize(n_sample);
    vector<double> tiltAngles; tiltAngles.resize(n_sample);

    tbb::parallel_for(tbb::blocked_range<size_t>(0, n_sample),
                      [&](const tbb::blocked_range<size_t>& r) {
                          for (size_t i = r.begin(); i != r.end(); ++i)
                          {
                              directions[i] =  v_axis * std::sin((double ) i / n_sample * 2 * M_PI) + u_axis * std::cos((double )i / n_sample * 2 * M_PI);
                              tiltAngles[i] = computeSlope(gravity, directions[i], 0);
                          }
                      });
//    for(int i = 0; i < n_sample; ++i){
//        directions[i] =  v_axis * std::sin((double ) i / n_sample * 2 * M_PI) + u_axis * std::cos((double )i / n_sample * 2 * M_PI);
//        tiltAngles[i] = computeSlope(gravity, directions[i], 0);
//    }

    double minimum = std::numeric_limits<double>::max();
//    for(int id = 0; id < tiltAngles.size(); id++){
//        if(minimum > tiltAngles[id]) minimum = tiltAngles[id];
//    }

    gravity_cone.clear();
    //add initial region to search queue
    //compute the lower bound for the minimum tilt angle
    for(int id = 0; id < n_sample; id++)
    {
        std::cout << "( " << id << "):\t[" << directions[id][0] << ", " << directions[id][1] << ", " << directions[id][2] << "],\t";
        std::cout << tiltAngles[id] << std::endl;

        SlopeSearchRegion region;
        region.va = directions[id];
        region.vb = directions[(id + 1) % n_sample];
        region.ta = tiltAngles[id];
        region.tb = tiltAngles[(id + 1) % n_sample];

        double ra = tan(region.ta / 180 * M_PI);
        double rb = tan(region.tb / 180 * M_PI);

        Vector2f xa = (float)ra * Vector2f(region.va.dot(u_axis), region.va.dot(v_axis));
        Vector2f xb = (float)rb * Vector2f(region.vb.dot(u_axis), region.vb.dot(v_axis));
        Vector2f xc;
        double tilt = PointSegmentDistance(Vector2f(0, 0), xa, xb, xc);
        tilt = atan(tilt) * 180 / M_PI;
        if(minimum > tilt)
            minimum = tilt;
        gravity_cone.push_back(region);
    }

    return minimum;
}

double ContactGraphMosekSolver::computeSlope(EigenPoint gravity) {
    /*
 * 0. Compute whether structure is equlibrium in gravity
 */
    string log;
    bool equilibrium = isEquilibrium(gravity, log, false);
    if(!equilibrium)
        return  0;

    /*
     * 1. Compute local u, v directions perpendicular to the gravity
     */

    gravity.normalized();

    EigenPoint u_axis, v_axis;
    u_axis = EigenPoint(0, 0, 1).cross(gravity);
    if((u_axis).norm() < small_zero_eps) u_axis = EigenPoint(1, 0, 0).cross(gravity);
    u_axis.normalized();
    v_axis = gravity.cross(u_axis);
    v_axis.normalized();

    /*
     * 2. Compute the tile angle in several initial directions
     */

    int n_sample = slope_num_init_sample;
    vector<EigenPoint> directions; directions.resize(n_sample);
    vector<double> tiltAngles; tiltAngles.resize(n_sample);

    tbb::parallel_for(tbb::blocked_range<size_t>(0, n_sample),
                      [&](const tbb::blocked_range<size_t>& r) {
                          for (size_t i = r.begin(); i != r.end(); ++i)
                          {
                              directions[i] =  v_axis * std::sin((double ) i / n_sample * 2 * M_PI) + u_axis * std::cos((double )i / n_sample * 2 * M_PI);
                              tiltAngles[i] = computeSlope(gravity, directions[i], 0);
                          }
    });
//    for (size_t i = 0; i < n_sample; ++i){
//        directions[i] =  v_axis * std::sin((double ) i / n_sample * 2 * M_PI) + u_axis * std::cos((double )i / n_sample * 2 * M_PI);
//        tiltAngles[i] = computeSlope(gravity, directions[i], 0);
//    }

    std::vector<SlopeSearchRegion> regions;
    //add initial region to search queue
    //compute the lower bound for the minimum tilt angle
    for(int id = 0; id < n_sample; id++)
    {
        SlopeSearchRegion region;
        region.va = directions[id];
        region.vb = directions[(id + 1) % n_sample];
        region.ta = tiltAngles[id];
        region.tb = tiltAngles[(id + 1) % n_sample];
        regions.push_back(region);
    }


    float stop_length_difference = 1e-4;
    vector<SlopeSearchRegion> validRegions; //remove those region exceed the bounds
    gravity_cone.clear();

    Eigen::Vector2d bound(std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    do
        {
        bound[0] = std::numeric_limits<double>::max();

        for(int id = 0; id < regions.size(); id++)
        {
            double maximum = std::min(regions[id].ta, regions[id].tb);
            if(bound[1] > maximum) bound[1] = maximum;

            double minimum = regions[id].minimum_possible_tiltAngle();
            if(bound[0] > minimum) bound[0] = minimum;
        }

        if(regions.empty() || (bound[1] - bound[0] < slope_bound_eps))
        {
            gravity_cone.insert(gravity_cone.end(), regions.begin(), regions.end());
            break;
        }


        validRegions.clear();
        for(int id = 0; id < regions.size(); id++)
        {
            if(regions[id].minimum_ < bound[1] && (regions[id].vb - regions[id].va).norm() > stop_length_difference)
                validRegions.push_back(regions[id]);
            else{
                gravity_cone.push_back(regions[id]);
            }
        }

        tbb::parallel_for(tbb::blocked_range<size_t>(0, validRegions.size()),
                          [&](const tbb::blocked_range<size_t>& r) {
                              for (size_t i = r.begin(); i != r.end(); ++i)
                              {
                                  validRegions[i].tc_ = computeSlope(gravity, validRegions[i].vc_, validRegions[i].minimum_ / 180 * M_PI);
                              }
                          });
//        for (size_t i = 0; i < validRegions.size(); ++i)
//        {
//            validRegions[i].tc_ = computeSlope(gravity, validRegions[i].vc_, validRegions[i].minimum_ / 180 * M_PI);
//        }

        regions.clear();

        for(int id = 0; id < validRegions.size(); id++)
        {
            SlopeSearchRegion region;
            if(validRegions[id].mu_ >= small_zero_eps && validRegions[id].mu_ <= 1 - small_zero_eps)
            {
                region.va = validRegions[id].va;
                region.vb = validRegions[id].vc_;
                region.ta = validRegions[id].ta;
                region.tb = validRegions[id].tc_;
                regions.push_back(region);

                region.va = validRegions[id].vc_;
                region.vb = validRegions[id].vb;
                region.ta = validRegions[id].tc_;
                region.tb = validRegions[id].tb;
                regions.push_back(region);
            }
        }
        std::cout << "[" << bound[0] << ", " << bound[1] << "]" << std::endl;
    }while(!regions.empty() && (bound[1] - bound[0] > slope_bound_eps));
    minimum_slope = bound[1];
    return bound[1];
}

std::tuple<EigenPoint, EigenPoint> ContactGraphMosekSolver::computeSlopeCoordinateSystem(EigenPoint gravity)
{
    EigenPoint u_axis, v_axis;
    u_axis = EigenPoint(0, 0, 1).cross(gravity);
    if((u_axis).norm() < small_zero_eps) u_axis = EigenPoint(1, 0, 0).cross(gravity);
    u_axis.normalized();
    v_axis = gravity.cross(u_axis);
    v_axis.normalized();
    return std::tuple(u_axis, v_axis);
}