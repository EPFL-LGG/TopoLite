//
// Created by ziqwang on 21.03.19.
//

#ifndef TOPOLOCKCREATOR_DERIVATIVETEST_H
#define TOPOLOCKCREATOR_DERIVATIVETEST_H

extern shared_ptr<StrucCreator> myStrucCreator; // TI Assembly Creator
struct DerivativeTest
{
private:
    double step;
    shared_ptr<StrucCreator> strucCreator;
    shared_ptr<StrucDiff> strucDiff;

public:
    double value;
    vector<double> derivative;
    vector<double> dx;

public:
    DerivativeTest(int _seed, double _step)
    {
        srand(_seed);
        step = _step;
        if(myStrucCreator)
        {
            strucCreator = make_shared<StrucCreator>(*myStrucCreator);
            strucDiff = make_shared<StrucDiff>(strucCreator);
        }
    }

public:
    void resetStructure()
    {
        strucCreator.reset();
        strucDiff.reset();
        if(myStrucCreator)
        {
            strucCreator = make_shared<StrucCreator>(*myStrucCreator);
            strucDiff = make_shared<StrucDiff>(strucCreator);
        }
    }

    void randomStructure()
    {
        if(strucCreator)
        {
            int n = strucCreator->variables.size();

            Eigen::VectorXd delta = Eigen::VectorXd::Zero(n);
            for (int id = 0; id < n; id++)
            {
                delta(id) = rand() % 10000 - 5000;
            }
            delta.normalize();
            delta *= step;
            dx.clear();
            for(int id = 0; id < n; id++){
                dx.push_back(delta(id));
            }

            shared_ptr<OrientPoint> pt;
            for (int id = 0; id < n; id++)
            {
                pt = strucCreator->variables[id].first.lock();
                pt->rotation_angle += delta(id) * pt->tiltSign;
                pt->normal = RotateNormal(pt->rotation_base, pt->rotation_axis, pt->rotation_angle);

                pt = strucCreator->variables[id].second.lock();
                pt->rotation_angle += delta(id) * pt->tiltSign;
                pt->normal = RotateNormal(pt->rotation_base, pt->rotation_axis, pt->rotation_angle);
            }

            strucCreator->UpdateStructureGeometry(false);
            strucDiff.reset();
            strucDiff = make_shared<StrucDiff>(strucCreator);
        }
    }

public:

    bool computeVolumeDerivative()
    {
        if(strucCreator)
        {
            value = 0;
            derivative.clear();
            return strucDiff->pybind11_unittest_volume(value, derivative);
        }
        return false;
    }

    bool computeContactAreaDerivative()
    {
        if(strucCreator)
        {
            value = 0;
            derivative.clear();
            return strucDiff->pybind11_unittest_contact(value, derivative);
        }
        return false;
    }

    bool computeOrientedPointDerivative()
    {
        if(strucCreator)
        {
            value = 0;
            derivative.clear();
            return strucDiff->pybind11_unittest_oriPoint(value, derivative);
        }
        return false;
    }

    bool computeVertexDerivative()
    {
        if(strucDiff)
        {
            value = 0;
            derivative.clear();
            return strucDiff->pybind11_unittest_vertex(value, derivative);
        }
        return false;
    }

    bool computeTensionEnergyDerivative(float x, float y, float z)
    {
        if(strucDiff)
        {
            value = 0;
            derivative.clear();
            return strucDiff->pybind11_unittest_tension(Vector3f(x,y,z), value, derivative);
        }
        return false;
    }
    bool computeContactNormalDerivative(){
        if(strucDiff)
        {
            value = 0;
            derivative.clear();
            return strucDiff->pybind11_unittest_contactnormal(value, derivative);
        }
        return false;

    }

    double checkEquilibriumMatrix(){
        if(strucDiff)
        {
            strucCreator->myStruc->ComputeContactGraph();
            strucCreator->myStruc->contactGraph->initialize();
            Eigen::MatrixXd eq1, eq2;
            strucCreator->myStruc->contactGraph->computeEquilibriumMatrix(eq1);
            strucDiff->pybind11_unittest_getEquilibriumMatrix(eq2);
            std::ofstream fout1, fout2;
            fout1.open("eq1.txt");
            fout2.open("eq2.txt");
            fout1 << eq1 << std::endl;
            fout2 << eq2 << std::endl;
            fout1.close();
            fout2.close();
//            for(int id = 0; id < eq1.rows(); id++)
//            {
//                for(int jd = 0; jd < eq1.cols(); jd++)
//                {
//                    double difference = std::abs(eq1(id, jd) - eq2(id, jd));
//                    if(difference > 1e-6)
//                    {
//                        std::cout << id << ", "<< jd << ", " << eq1(id, jd) << "," << eq2(id, jd) << std::endl;
//                    }
//                }
//            };
            return (eq1 - eq2).norm();
        }
        return -1;
    }

private:
    Vector3f RotateNormal(Vector3f normal, Vector3f rotAxis, float rotAngle)
    {
        rotAngle = rotAngle / 180 * M_PI;
        Eigen::Matrix3d mat, ux;
        ux <<   0, -rotAxis.z, rotAxis.y,
                rotAxis.z, 0, -rotAxis.x,
                -rotAxis.y, rotAxis.x, 0;
        mat = Eigen::Matrix3d::Identity() * (double)std::cos(rotAngle) + ux * (double)std::sin(rotAngle);
        Eigen::Vector3d inNormal(normal[0], normal[1], normal[2]);
        Eigen::Vector3d outNormal;
        outNormal = mat * inNormal;
        return Vector3f(outNormal[0], outNormal[1], outNormal[2]);
    }
};

#endif //TOPOLOCKCREATOR_DERIVATIVETEST_H
