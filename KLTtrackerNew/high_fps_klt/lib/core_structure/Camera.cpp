#include "Camera.h"

namespace iqmatic {
    namespace camera {
#if 0
        Camera::Camera()
        {
            image_size = cv::Size(1600, 1200);
            intrinsics = new PinholeProjectionModel(Matrix3x3d(400, 0, 320, 0, 400, 240, 0, 0, 1));
            proj_type = ProjectionModelType::pinhole;
            extrinsics = RigidTransform();
            extrinsics.setT(Vector3d(0, 0, 0));
            extrinsics.setQuaternion(cvl::Vector4<double>(1, 0, 0, 0));
        }
#endif
        Camera::Camera(ProjectionModelType tp, const Matrix3d& K, const vector<double>& dist)
        {
            proj_type = tp;
            switch (tp) {
                case ProjectionModelType::pinhole:
                    intrinsics = new PinholeProjectionModel(K);
                    break;
                case ProjectionModelType::brown:
                case ProjectionModelType::fisheye:
                default:
                    intrinsics = new PinholeProjectionModel(Matrix3d(400, 0, 320, 0, 400, 240, 0, 0, 1));
            }
        }

        Camera::Camera(ProjectionModelType tp, const Matrix3d& K, const vector<double>& dist, const RigidTransform& ext)
                : Camera(tp, K, dist)
        {
            setExtrinsics(ext);
        }

        Camera::Camera(string config) {
        }

        const RigidTransform& Camera::getExtrinsics()
        {
            return extrinsics;
        }

        void Camera::setExtrinsics(const RigidTransform &T_) {
            extrinsics = T_;
        }

        void Camera::setImageSize(int width, int height)
        {
            image_size = cv::Size(width, height);
        }

        void Camera::setImageSize(cv::Size size)
        {
            image_size = size;
        }

        cv::Size Camera::getImageSize()
        {
            return image_size;
        }

        ProjectionModelType Camera::getType() {
            return proj_type;
        }

        void Camera::projectWorldToImage(const Vector3d &X, Vector2d &y, bool &in_frame) {
            Vector3d X_c;
            //std::cout << "calling the right camera function" << std::endl;
            transformWorldToCamera(X, X_c);
            projectCameraToImage(X_c, y, in_frame);
        }

        void Camera::projectWorldToImage(const vector<Vector3d> &X, vector<Vector2d> &y, vector<bool> &in_frame) {
            vector<Vector3d> X_c;
            transformWorldToCamera(X, X_c);
            projectCameraToImage(X_c, y, in_frame);
        }

        void Camera::transformWorldToCamera(const Vector3d &X_w, Vector3d &X_c) {
            X_c = extrinsics * X_w;
        }

        void Camera::transformWorldToCamera(const vector<Vector3d> &X_w, vector<Vector3d> &X_c)
        {
            X_c.resize(X_w.size());

            for (uint i = 0; i < X_w.size(); ++i) {
                X_c[i] = extrinsics * X_w[i];
            }
        }

        bool Camera::isInFrame(const cvl::Vector3d& X, const cvl::Vector2d& y)
        {
            if (X(2) < 0) return false; // Behind the camera
            return (y(0) > 0 && y(0) < image_size.width &&
                    y(1) > 0 && y(1) < image_size.height);
        }

        void Camera::projectCameraToImage(const Vector3d &X, Vector2d &y, bool &in_frame)
        {
            intrinsics->projectToImage(X, y);
            in_frame = isInFrame(X, y);
        }



        void Camera::projectCameraToImage(const vector<Vector3d> &X, vector<Vector2d> &y, vector<bool> &in_frame)
        {
            intrinsics->projectToImage(X, y);

            in_frame.resize(y.size());
            for (uint i = 0; i < y.size(); ++i) {
                in_frame[i] = isInFrame(X[i], y[i]);
            }
        }

        void Camera::imageToIdealPlane(const Vector2d &y, Vector2d &x, bool &finite) {
            intrinsics->imageToIdealPlane(y, x, finite);
        }

        void Camera::imageToIdealPlane(const vector<Vector2d> &y, vector<Vector2d> &x, vector<bool> &finite) {
            intrinsics->imageToIdealPlane(y, x, finite);
        }

        void Camera::imageToOpticRay(const Vector2d &y, Vector3d &r) {
            intrinsics->imageToOpticRay(y, r);
        }

        void Camera::imageToOpticRay(const vector<Vector2d> &y, vector<Vector3d> &r) {
            intrinsics->imageToOpticRay(y, r);
        }

        void Camera::projectWorldToIdeal(const Vector3d &X, Vector2d &x, bool &finite) {
            Vector3d X_c;
            transformWorldToCamera(X, X_c);
            projectCameraToIdeal(X_c, x, finite);
        }

        void Camera::projectWorldToIdeal(const vector<Vector3d> &X, vector<Vector2d> &x, vector<bool> &finite) {
            vector<Vector3d> X_c;
            transformWorldToCamera(X, X_c);
            projectCameraToIdeal(X_c, x, finite);
        }

        void Camera::projectCameraToIdeal(const Vector3d &X, Vector2d &x, bool &finite) {
            if (X(2) < 1e-6) {
                x(0) = X(0);
                x(1) = X(1);
                finite = false;
            } else {
                x(0) = X(0) / X(2);
                x(1) = X(1) / X(2);
                finite = true;
            }
        }

        void Camera::projectCameraToIdeal(const vector<Vector3d> &X, vector<Vector2d> &x, vector<bool> &finite) {
            x.resize(X.size());
            finite.resize(X.size());
            bool finite_;
            for (uint i = 0; i < X.size(); ++i) {
                projectCameraToIdeal(X[i], x[i], finite_);
                finite[i] = finite_;
            }
        }

        double Camera::getFocalLength() {
            return intrinsics->getFocalLength();
        }
    } // iqmatic
} // camera
