//
// Created by ziqwang on 03.04.20.
//

#ifndef TOPOLITE_GUI_MATH_H
#define TOPOLITE_GUI_MATH_H

#include <Eigen/Dense>
#include <nanogui/vector.h>

inline Eigen::Vector3f project(const Eigen::Vector3f &obj,
                        const Eigen::Matrix4f &model,
                        const Eigen::Matrix4f &proj,
                        const nanogui::Vector2i &viewportSize) {
    Eigen::Vector4f tmp;
    tmp << obj, 1;

    tmp = model * tmp;

    tmp = proj * tmp;

    tmp = tmp.array() / tmp(3);
    tmp = tmp.array() * 0.5f + 0.5f;
    tmp(0) = tmp(0) * viewportSize.x();
    tmp(1) = tmp(1) * viewportSize.y();

    return tmp.head(3);
}

inline Eigen::Vector3f unproject(const Eigen::Vector3f &win,
                          const Eigen::Matrix4f &model,
                          const Eigen::Matrix4f &proj,
                          const nanogui::Vector2i &viewportSize) {
    Eigen::Matrix4f Inverse = (proj * model).inverse();

    Eigen::Vector4f tmp;
    tmp << win, 1;
    tmp(0) = tmp(0) / viewportSize.x();
    tmp(1) = tmp(1) / viewportSize.y();
    tmp = tmp.array() * 2.0f - 1.0f;

    Eigen::Vector4f obj = Inverse * tmp;
    obj /= obj(3);

    return obj.head(3);
}

inline Eigen::Matrix4f lookAt(const Eigen::Vector3f &eye,
                       const Eigen::Vector3f &center,
                       const Eigen::Vector3f &up) {
    Eigen::Vector3f f = (center - eye).normalized();
    Eigen::Vector3f s = f.cross(up).normalized();
    Eigen::Vector3f u = s.cross(f);

    Eigen::Matrix4f Result = Eigen::Matrix4f::Identity();
    Result(0, 0) = s(0);
    Result(0, 1) = s(1);
    Result(0, 2) = s(2);
    Result(1, 0) = u(0);
    Result(1, 1) = u(1);
    Result(1, 2) = u(2);
    Result(2, 0) = -f(0);
    Result(2, 1) = -f(1);
    Result(2, 2) = -f(2);
    Result(0, 3) = -s.transpose() * eye;
    Result(1, 3) = -u.transpose() * eye;
    Result(2, 3) = f.transpose() * eye;
    return Result;
}

inline Eigen::Matrix4f ortho(const float left, const float right, const float bottom,
                      const float top, const float zNear, const float zFar) {
    Eigen::Matrix4f Result = Eigen::Matrix4f::Identity();
    Result(0, 0) = 2.0f / (right - left);
    Result(1, 1) = 2.0f / (top - bottom);
    Result(2, 2) = -2.0f / (zFar - zNear);
    Result(0, 3) = -(right + left) / (right - left);
    Result(1, 3) = -(top + bottom) / (top - bottom);
    Result(2, 3) = -(zFar + zNear) / (zFar - zNear);
    return Result;
}

inline Eigen::Matrix4f frustum(const float left, const float right, const float bottom,
                        const float top, const float nearVal,
                        const float farVal) {
    Eigen::Matrix4f Result = Eigen::Matrix4f::Zero();
    Result(0, 0) = (2.0f * nearVal) / (right - left);
    Result(1, 1) = (2.0f * nearVal) / (top - bottom);
    Result(0, 2) = (right + left) / (right - left);
    Result(1, 2) = (top + bottom) / (top - bottom);
    Result(2, 2) = -(farVal + nearVal) / (farVal - nearVal);
    Result(3, 2) = -1.0f;
    Result(2, 3) = -(2.0f * farVal * nearVal) / (farVal - nearVal);
    return Result;
}

inline Eigen::Matrix4f scale(const Eigen::Matrix4f &m, const Eigen::Vector3f &v) {
    Eigen::Matrix4f Result;
    Result.col(0) = m.col(0).array() * v(0);
    Result.col(1) = m.col(1).array() * v(1);
    Result.col(2) = m.col(2).array() * v(2);
    Result.col(3) = m.col(3);
    return Result;
}

inline Eigen::Matrix4f translate(const Eigen::Matrix4f &m, const Eigen::Vector3f &v) {
    Eigen::Matrix4f Result = m;
    Result.col(3) = m.col(0).array() * v(0) + m.col(1).array() * v(1) +
                    m.col(2).array() * v(2) + m.col(3).array();
    return Result;
}


#endif //TOPOLITE_GUI_MATH_H
