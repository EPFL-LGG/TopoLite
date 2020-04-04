//
// Created by ziqwang on 03.04.20.
//

#ifndef TOPOLITE_ARCBALL_H
#define TOPOLITE_ARCBALL_H
#include <Eigen/Dense>
#include <nanogui/vector.h>

/**
 * \struct Arcball glutil.h nanogui/glutil.h
 *
 * \brief Arcball helper class to interactively rotate objects on-screen.
 */
struct Arcball {
    Arcball(float speedFactor = 2.0f)
            : mActive(false), mLastPos(Eigen::Vector2i::Zero()), mSize(Eigen::Vector2i::Zero()),
              mQuat(Eigen::Quaternionf::Identity()),
              mIncr(Eigen::Quaternionf::Identity()),
              mSpeedFactor(speedFactor) { }

    Arcball(const Eigen::Quaternionf &quat)
            : mActive(false), mLastPos(Eigen::Vector2i::Zero()), mSize(Eigen::Vector2i::Zero()),
              mQuat(quat),
              mIncr(Eigen::Quaternionf::Identity()),
              mSpeedFactor(2.0f) { }

    Eigen::Quaternionf &state() { return mQuat; }

    void setState(const Eigen::Quaternionf &state) {
        mActive = false;
        mLastPos = Eigen::Vector2i::Zero();
        mQuat = state;
        mIncr = Eigen::Quaternionf::Identity();
    }

    void setSize(Eigen::Vector2i size) { mSize = size; }
    const Eigen::Vector2i &size() const { return mSize; }
    void setSpeedFactor(float speedFactor) { mSpeedFactor = speedFactor; }
    float speedFactor() const { return mSpeedFactor; }
    bool active() const { return mActive; }

    void button(Eigen::Vector2i pos, bool pressed) {
        mActive = pressed;
        mLastPos = pos;
        if (!mActive)
            mQuat = (mIncr * mQuat).normalized();
        mIncr = Eigen::Quaternionf::Identity();
    }

    bool motion(Eigen::Vector2i pos) {
        if (!mActive)
            return false;

        /* Based on the rotation controller form AntTweakBar */
        float invMinDim = 1.0f / mSize.minCoeff();
        float w = (float) mSize.x(), h = (float) mSize.y();

        float ox = (mSpeedFactor * (2*mLastPos.x() - w) + w) - w - 1.0f;
        float tx = (mSpeedFactor * (2*pos.x()      - w) + w) - w - 1.0f;
        float oy = (mSpeedFactor * (h - 2*mLastPos.y()) + h) - h - 1.0f;
        float ty = (mSpeedFactor * (h - 2*pos.y())      + h) - h - 1.0f;

        ox *= invMinDim; oy *= invMinDim;
        tx *= invMinDim; ty *= invMinDim;

        Eigen::Vector3f v0(ox, oy, 1.0f), v1(tx, ty, 1.0f);
        if (v0.squaredNorm() > 1e-4f && v1.squaredNorm() > 1e-4f) {
            v0.normalize(); v1.normalize();
            Eigen::Vector3f axis = v0.cross(v1);
            float sa = std::sqrt(axis.dot(axis)),
                    ca = v0.dot(v1),
                    angle = std::atan2(sa, ca);
            if (tx*tx + ty*ty > 1.0f)
                angle *= 1.0f + 0.2f * (std::sqrt(tx*tx + ty*ty) - 1.0f);
            mIncr = Eigen::AngleAxisf(angle, axis.normalized());
            if (!std::isfinite(mIncr.norm()))
                mIncr = Eigen::Quaternionf::Identity();
        }
        return true;
    }

    Eigen::Matrix4f matrix() const {
        Eigen::Matrix4f result2 = Eigen::Matrix4f::Identity();
        result2.block<3,3>(0, 0) = (mIncr * mQuat).toRotationMatrix();
        return result2;
    }

protected:
    bool mActive;
    Eigen::Vector2i mLastPos;
    Eigen::Vector2i mSize;
    Eigen::Quaternionf mQuat, mIncr;
    float mSpeedFactor;
};

Eigen::Vector3f project(const Eigen::Vector3f &obj,
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

Eigen::Vector3f unproject(const Eigen::Vector3f &win,
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

Eigen::Matrix4f lookAt(const Eigen::Vector3f &eye,
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

Eigen::Matrix4f ortho(const float left, const float right, const float bottom,
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

Eigen::Matrix4f frustum(const float left, const float right, const float bottom,
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

Eigen::Matrix4f scale(const Eigen::Matrix4f &m, const Eigen::Vector3f &v) {
    Eigen::Matrix4f Result;
    Result.col(0) = m.col(0).array() * v(0);
    Result.col(1) = m.col(1).array() * v(1);
    Result.col(2) = m.col(2).array() * v(2);
    Result.col(3) = m.col(3);
    return Result;
}

Eigen::Matrix4f translate(const Eigen::Matrix4f &m, const Eigen::Vector3f &v) {
    Eigen::Matrix4f Result = m;
    Result.col(3) = m.col(0).array() * v(0) + m.col(1).array() * v(1) +
                    m.col(2).array() * v(2) + m.col(3).array();
    return Result;
}


#endif //TOPOLITE_ARCBALL_H
