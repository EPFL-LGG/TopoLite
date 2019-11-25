//
// Created by ziqwang on 21.03.19.
//

#ifndef TOPOLOCKCREATOR_PATTERNSERIESGENERATOR_H
#define TOPOLOCKCREATOR_PATTERNSERIESGENERATOR_H
#include "Utility/vec.h"
#include <Eigen/Dense>
#include "Structure/StrucCreator.h"
extern shared_ptr<StrucCreator> myStrucCreator; // TI Assembly Creator
extern double interactMatrix[16];

struct PatternSeriesGenerator
{
private:
    vector<shared_ptr<StrucCreator>> strucCreators;
    Eigen::MatrixXd interactMat;

public:
    vector<double> rotation_angle;
    vector<double> translation_x;
    vector<double> translation_y;
    vector<double> scale_factor;

public:
    void PatternSeriesGenerator(vector<double> &_angles, vector<double> &_xs, vector<double> &_ys, vector<double> &_scale)
    {
        rotation_angle = _angles;
        translation_x = _xs;
        translation_y = _ys;
        scale_factor = _scale;
        interactMat = MatrixXd::Zero(4, 4);

        for(int ix = 0; ix < 4; ix++){
            for(int iy = 0;iy < 4; iy++){
                interactMat(ix, iy) = interactMatrix[iy * 4 + ix];
            }
        }
    }

    void seriesGeneration()
    {
        for(double angle : rotation_angle)
        {
            for(double x: translation_x)
            {
                for(double y: translation_y)
                {
                    for(double s: scale_factor)
                    {
                        Eigen::MatrixXd transformMat = transformMatrix(angle, x, y, s);
                        transformMat = transformMat * interactMat;
                        shared_ptr<strucCreator> strucCreator = make_shared<strucCreator>(*myStrucCreator);
                        double interactDouble[16];
                        eige2double(transformMat, interactDouble);
                        strucCreator->CreateStructure(true, true, interactDouble, false);
                        strucCreators.push_back(strucCreator);
                    }
                }
            }
        }
    }

    vector<bool> checkInterlocking()
    {
        for(int id = 0; id < strucCreators.size(); id++)
        {
            
        }
    }

private:

    void eige2double(Eigen::MatrixXd matIN, double *matOUT)
    {
        for(int ix = 0; ix < 4; ix++)
        {
            for(int iy = 0;iy < 4; iy++)
            {
                interactMatrix[iy * 4 + ix] = interactMat(ix, iy);
            }
        }
    }

    Eigen::MatrixXd transformMatrix(double angle, double x, double y, double s) {
        Eigen::MatrixXd mat = Eigen::Matrix::Identity(4);
        mat(1, 1) = mat(0, 0) = s * cos(angle);
        mat(0, 1) = s * sin(angle);
        mat(1, 0) = -s * sin(angle);
        mat(0, 3) = x * s;
        mat(1, 3) = y * s;
        return mat;
    }
};

#endif //TOPOLOCKCREATOR_PATTERNSERIESGENERATOR_H
