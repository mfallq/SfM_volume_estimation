#pragma once

#include "Matrix.h"

namespace cvl {

class Pose
{
public:    
    Pose();

    Pose(const cvl::Matrix3d& R);
    Pose(const cvl::Vector3d& t);
    Pose(const cvl::Matrix3d& R, const cvl::Vector3d& t);
    Pose(const cvl::Vector4d& q, const cvl::Vector3d& t);
    Pose(const cvl::Matrix34d& P);
    Pose(const cvl::Matrix4d& P);

    ~Pose();

    /**
     * @brief getRRef
     * @return a pointer to the first element of the quaternion in the pose
     */
    double* getRRef();
    /**
     * @brief getTRef
     * @return a pointer to the first element of the translation in the pose
     */
    double* getTRef();
    void setT(const cvl::Vector3d& tt);
    void setQuaternion(const cvl::Vector4d& qq);


    cvl::Vector3d operator*(const cvl::Vector3d& ins) const;

    Pose operator*(const Pose& rhs) const;
    Pose inverse() const;
    double angleDistance(const Pose& p) const;

    cvl::Vector4d getQuaternion() const;
    cvl::Matrix3d getR() const;
    cvl::Matrix3d rotation() const;

    cvl::Vector3d getT() const;
    cvl::Vector3d translation() const;
    cvl::Vector3d& translation();

    cvl::Matrix34d get3x4() const;
    cvl::Matrix4d get4x4() const;
    double getAngle() const;

    cvl::Matrix3d getEssentialMatrix() const;

    bool noRotation() const;
    bool isIdentity() const;
    /// returns true if no value is strange
    bool isnormal() const;

    void scaleT(double scale);

    void normalize();
    cvl::Vector3d rotate(const cvl::Vector3d& x) const;
    void rotateInPlace(cvl::Vector3d& x) const;

private:
    cvl::Vector4d q; // Rotation quaternion. Coefficients are ordered as (scalar, i, j, k)
    cvl::Vector3d t;
};

using RigidTransform = Pose;

} // cvl::
