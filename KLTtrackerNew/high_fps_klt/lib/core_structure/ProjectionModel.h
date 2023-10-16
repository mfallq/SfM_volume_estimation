#ifndef VISION_CAMERA_DISTORTIOMODEL
#define VISION_CAMERA_DISTORTIOMODEL

#include "../iqmatic_defs.h"


using std::string;
using std::vector;


namespace iqmatic
{

namespace camera
{

    class ProjectionModel
    {
    public:
        virtual ~ProjectionModel(){};
        virtual double getFocalLength()=0;
        virtual void projectToImage(const Vector3d& X, Vector2d& y) = 0;
        virtual void projectToImage(const vector<Vector3d>& X, vector<Vector2d>& y) = 0;
        virtual void imageToIdealPlane(const Vector2d& y, Vector2d& x, bool& finite) = 0;
        virtual void imageToIdealPlane(const vector<Vector2d>& y, vector<Vector2d>& x, vector<bool>& finite) = 0;
        virtual void imageToOpticRay(const Vector2d& y, Vector3d& r) = 0;
        virtual void imageToOpticRay(const vector<Vector2d>& y, vector<Vector3d>& r) = 0;
    };

    class PinholeProjectionModel : public ProjectionModel
    {
    public:
        PinholeProjectionModel();
        PinholeProjectionModel(Matrix3d K_);
        virtual ~PinholeProjectionModel(){};

        void projectToImage(const Vector3d& X, Vector2d& y);
        void projectToImage(const vector<Vector3d>& X, vector<Vector2d>& y);
        void imageToIdealPlane(const Vector2d& y, Vector2d& x, bool& finite);
        void imageToIdealPlane(const vector<Vector2d>& y, vector<Vector2d>& x, vector<bool>& finite);
        void imageToOpticRay(const Vector2d& y, Vector3d& r);
        void imageToOpticRay(const vector<Vector2d>& y, vector<Vector3d>& r);
        void setK(Matrix3d K_);
        Matrix3d getK();
        double getFocalLength();

    private:
        Matrix3d K;
        Matrix3d K_inv;
    };

} //camera
} //iqmatic

#endif // VISION_CAMERA_DISTORTIOMODEL
