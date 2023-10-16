#include "ProjectionModel.h"

namespace iqmatic
{
namespace camera
{
    PinholeProjectionModel::PinholeProjectionModel()
    {
        K = Matrix3d(1, 0, 0, 0, 1, 0, 0, 0, 1);
    }

    PinholeProjectionModel::PinholeProjectionModel(Matrix3d K_)
    {
        K = K_;
        K_inv = K_.inverse();
    }

    void PinholeProjectionModel::projectToImage(const Vector3d& X, Vector2d& y)
    {
        Vector3d x;
        x = K * X;
        y(0) = x(0) / x(2);
        y(1) = x(1) / x(2);
    }

    void PinholeProjectionModel::projectToImage(const vector<Vector3d>& X, vector<Vector2d>& y)
    {
        y.resize(X.size());
        for(uint i = 0; i < X.size(); ++i) {
            projectToImage(X[i], y[i]);
        }
    }

    void PinholeProjectionModel::imageToIdealPlane(const Vector2d& y, Vector2d& x, bool& finite)
    {
        Vector3d y_hom(y(0), y(1), 1);
        Vector3d x_hom = K_inv * y_hom;
        if(x_hom(2) < 1e-9) {
            x(0) = x_hom(0);
            x(1) = x_hom(1);
            finite = false;
        } else {
            x(0) = x_hom(0) / x_hom(2);
            x(1) = x_hom(1) / x_hom(2);
            finite=true;
        }
    }

    void PinholeProjectionModel::imageToIdealPlane(const vector<Vector2d>& y, vector<Vector2d>& x, vector<bool>& finite)
    {
        x.resize(y.size());
        finite.resize(y.size());
        bool finite_=true;
        for(uint i = 0; i < y.size(); ++i) {
            imageToIdealPlane(y[i], x[i], finite_);
            finite[i]=finite_;
        }
    }

    void PinholeProjectionModel::imageToOpticRay(const Vector2d& y, Vector3d& r)
    {
        Vector3d y_hom(y(0), y(1), 1);
        r = K_inv * y_hom;
        r.normalize();
    }

    void PinholeProjectionModel::imageToOpticRay(const vector<Vector2d>& y, vector<Vector3d>& r)
    {
        r.resize(y.size());
        for(uint i = 0; i < y.size(); ++i) {
            imageToOpticRay(y[i], r[i]);
        }
    }

    void PinholeProjectionModel::setK(Matrix3d K_)
    {
        K = K_;
        K_inv = K_.inverse();
    }

    Matrix3d PinholeProjectionModel::getK()
    {
        return K;
    }

    double PinholeProjectionModel::getFocalLength() {
        return K(0,0);
    }
} // camera
} // iqmatic