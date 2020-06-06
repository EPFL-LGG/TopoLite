//
// Created by ziqwang on 03.04.20.
//

#ifndef TOPOLITE_GUI_ARCBALL_H
#define TOPOLITE_GUI_ARCBALL_H

#include <Eigen/Dense>
#include <nanogui/vector.h>
#include "gui_Math.h"

/**
 * \struct Arcball glutil.h nanogui/glutil.h
 *
 * \brief Arcball helper class to interactively rotate objects on-screen.
 */
struct gui_Arcball {
    gui_Arcball(float speedFactor = 2.0f)
            : mActive(false), mLastPos(Eigen::Vector2i::Zero()), mSize(Eigen::Vector2i::Zero()),
              mQuat(Eigen::Quaternionf::Identity()),
              mIncr(Eigen::Quaternionf::Identity()),
              mSpeedFactor(speedFactor) { }

    gui_Arcball(const Eigen::Quaternionf &quat)
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
#endif //TOPOLITE_GUI_ARCBALL_H
