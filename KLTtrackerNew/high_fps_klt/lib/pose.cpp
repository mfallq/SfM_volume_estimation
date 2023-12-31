#include <cassert>
#include "pose.h"
#include "rotation_helpers.h"
#include "geometry_tools.h"

namespace cvl {

Pose::Pose()
{
    q = cvl::Vector4d(1, 0, 0, 0);
    t = cvl::Vector3d(0, 0, 0);
}

Pose::Pose(const cvl::Vector3d& t)
{
    q = cvl::Vector4d(1, 0, 0, 0);
    this->t = t;
}

Pose::Pose(const cvl::Matrix3d& R)
{
    cvl::Matrix3d r = normalizeRotationMatrix(R);
    q = getRotationQuaternion(r);
    t = cvl::Vector3d(0, 0, 0);
}

Pose::Pose(const cvl::Matrix3d& R, const cvl::Vector3d& t)
{
    q = getRotationQuaternion(R);
    this->t = t;
}

Pose::Pose(const cvl::Vector4d& q, const cvl::Vector3d& t)
{
    this->q = q;
    this->t = t;
}

Pose::Pose(const cvl::Matrix34d& P)
{
    q = getRotationQuaternion(P.topLeftCorner<3, 3>());
    t = cvl::Vector3d(P(0,3), P(1,3), P(2,3));
}

Pose::Pose(const cvl::Matrix4d& P)
{
    double s = 1.0 / P(3,3);
    q = s * getRotationQuaternion(P.topLeftCorner<3,3>());
    t = s * cvl::Vector3d(P(0, 3), P(1, 3), P(2, 3));
}

double* Pose::getRRef()
{
    return q.data();
}

double* Pose::getTRef()
{
    return t.data();
}

void Pose::setT(const cvl::Vector3d& t)
{
    this->t = t;
}

void Pose::setQuaternion(const cvl::Vector4d& q)
{
    this->q = q;
    (this->q).normalize();
}

cvl::Matrix3d Pose::getR() const
{
    return getRotationMatrix(getQuaternion());
}

cvl::Matrix3d Pose::rotation() const
{
    return getRotationMatrix(getQuaternion());
}

bool Pose::noRotation() const
{
    return (q(0) == 1);
}

bool Pose::isIdentity() const
{
    return ((q(0) == 1) && (t(0) == 0) && (t(1) == 0) && (t(2) == 0));
}

cvl::Vector3d Pose::getT() const
{
    return t;
}

cvl::Vector3d Pose::translation() const
{
    return t;
}

cvl::Vector3d& Pose::translation()
{
    return t;
}

void Pose::scaleT(double scale)
{
    t *= scale;
}

cvl::Matrix34d Pose::get3x4() const
{
    cvl::Matrix3d R = getR();
    cvl::Vector3d t = getT();
#ifdef USE_EIGEN
    cvl::Matrix34d P;
    P << R, t;
#else
    cvl::Matrix34d P(
        R(0, 0), R(0, 1), R(0, 2), t(0),
        R(1, 0), R(1, 1), R(1, 2), t(1),
        R(2, 0), R(2, 1), R(2, 2), t(2));
#endif
    return P;
}

cvl::Matrix4d Pose::get4x4() const
{
    cvl::Matrix3d R = getR();
    cvl::Vector3d t = getT();
#ifdef USE_EIGEN
    cvl::Matrix4d T;
    T <<
        R(0,0), R(0,1), R(0,2), t(0),
        R(1,0), R(1,1), R(1,2), t(1),
        R(2,0), R(2,1), R(2,2), t(2),
        0, 0, 0, 1;
#else
    cvl::Matrix4d T(
        R(0, 0), R(0, 1), R(0, 2), t(0),
        R(1, 0), R(1, 1), R(1, 2), t(1),
        R(2, 0), R(2, 1), R(2, 2), t(2),
        0, 0, 0, 1);
#endif
    return T;
}

double Pose::getAngle() const
{
    return 2 * std::acos(q(0));
}

Pose::~Pose()
{
}

cvl::Matrix3d Pose::getEssentialMatrix() const
{
    cvl::Vector3d t = getT().normalized();
    cvl::Matrix3d E = crossMatrix(t) * getR();
    return E / E.cwiseAbs().maxCoeff();
}

double Pose::angleDistance(const Pose& p) const
{
    Pose delta = inverse()*p;
    return delta.getAngle();
}

Pose Pose::inverse() const
{
    assert(isnormal());
    cvl::Matrix3d Ri = getR().transpose();// implies normalization
    cvl::Vector3d ti = -Ri*getT();
    return Pose(Ri, ti);
}

cvl::Vector4d Pose::getQuaternion() const
{
    cvl::Vector4d qr = q;
    if (qr(0) < 0)
        qr *= -1;
    return qr;
}

void Pose::normalize()
{
    assert(fabs(q.norm() - 1) < 1e-5);
    q.normalize();
    if (q(0) < 0)
        q *= -1;
}

cvl::Vector3d Pose::rotate(const cvl::Vector3d& in) const
{
    const double t2 = q(0) * q(1);
    const double t3 = q(0) * q(2);
    const double t4 = q(0) * q(3);
    const double t5 =-q(1) * q(1);
    const double t6 = q(1) * q(2);
    const double t7 = q(1) * q(3);
    const double t8 =-q(2) * q(2);
    const double t9 = q(2) * q(3);
    const double t1 =-q(3) * q(3);

    cvl::Vector3d out;

    out(0) = (2 * ((t8 + t1) * in(0) + (t6 - t4) * in(1) + (t3 + t7) * in(2)) + in(0));
    out(1) = (2 * ((t4 + t6) * in(0) + (t5 + t1) * in(1) + (t9 - t2) * in(2)) + in(1));
    out(2) = (2 * ((t7 - t3) * in(0) + (t2 + t9) * in(1) + (t5 + t8) * in(2)) + in(2));

    return out;
}

void Pose::rotateInPlace(cvl::Vector3d& v) const
{
    v = rotate(v);
}

cvl::Vector3d Pose::operator*(const cvl::Vector3d& ins) const
{
    return rotate(ins) + t;
}

bool Pose::isnormal() const
{
    cvl::Vector4d q = getQuaternion();
    if (!q.allFinite()) {
        return false;
    }

    cvl::Vector3d t = getT();
    if (std::isnan(t.squaredNorm())) {
        return false;
    }

    if (std::abs(q.norm() - 1) > 1e-5) {
        return false;
    }

    return true;
}

Pose Pose::operator*(const Pose& rhs) const
{
    cvl::Matrix3d R = getR();
    cvl::Vector3d t = getT();
    cvl::Matrix3d Rr = rhs.getR();
    cvl::Vector3d tr = rhs.getT();

    cvl::Matrix3d R_ = R*Rr;
    cvl::Vector3d t_ = R*tr + t;

    return Pose(R_, t_);
}

} // cvl::