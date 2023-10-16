#ifndef VISION_CAMERA
#define VISION_CAMERA

#include <opencv2/opencv.hpp>

#include "../iqmatic_defs.h"
#include "ProjectionModel.h"

using std::string;
using std::vector;
using cvl::Vector2d;
using cvl::Vector3d;

namespace iqmatic {

    namespace camera {
        enum ProjectionModelType {
            pinhole = 0, brown = 1, kannala = 2, fisheye = 2
        };

        class Camera {
        public:
#if 0
            /**
             * @brief Instantiate a standard pinhole camera with VGA resolution and 60 degrees fov
             */
            Camera();
#endif

            /**
             * @brief Instantiate a camera using the specified projection model, intrinsics and
             * distortion.
             */
            Camera(ProjectionModelType tp, const Matrix3d& K, const vector<double>& dist);

            /**
             * @brief Instantiate a camera using the specified projection model, intrinsics, extrinsics and
             * distortion.
             */
            Camera(ProjectionModelType tp, const Matrix3d& K, const vector<double>& dist, const RigidTransform& ext);

            /**
             * @brief Instantiate a camera using the parametery given in the configuration file in
             * input
             */
            Camera(string config);

            ~Camera() { };

            const RigidTransform& getExtrinsics();

            void setExtrinsics(const RigidTransform &T_);

            /// Set image size [pixels]
            void setImageSize(int width, int height);

            /// Set image size [pixels]
            void setImageSize(cv::Size size);

            /// Get the image size [pixels]
            cv::Size getImageSize();

            ProjectionModelType getType();

            // projects 3d points in the world coordinate system to the image plane
            void projectWorldToImage(const Vector3d &X, Vector2d &y, bool &in_frame);

            void projectWorldToImage(const vector<Vector3d> &X, vector<Vector2d> &y, vector<bool> &in_frame);

            // projects 3d points in the world coordinate system to the ideal image plane
            void projectWorldToIdeal(const Vector3d &X, Vector2d &y, bool &finite);

            void projectWorldToIdeal(const vector<Vector3d> &X, vector<Vector2d> &y, vector<bool> &finite);

            // transforms 3d points in the world coordinate system to the camera coordinate system
            void transformWorldToCamera(const Vector3d &X_w, Vector3d &X_c);

            void transformWorldToCamera(const vector<Vector3d> &X_w, vector<Vector3d> &X_c);

            // Check that a projected point is in front of the camera and inside the viewing frustum.
            bool isInFrame(const cvl::Vector3d& X, const cvl::Vector2d& y);

            // projects 3d points in the camera coordinate system to the image plane
            void projectCameraToImage(const Vector3d &X, Vector2d &y, bool &in_frame);

            void projectCameraToImage(const vector<Vector3d> &X, vector<Vector2d> &y, vector<bool> &in_frame);

            // projects 3d points in the world coordinate system to the ideal image plane
            void projectCameraToIdeal(const Vector3d &X, Vector2d &y, bool &finite);

            void projectCameraToIdeal(const vector<Vector3d> &X, vector<Vector2d> &y, vector<bool> &finite);

            // maps points from the image plane to the ideal image plane
            void imageToIdealPlane(const Vector2d &y, Vector2d &x, bool &finite);

            void imageToIdealPlane(const vector<Vector2d> &y, vector<Vector2d> &x, vector<bool> &finite);

            // computes the optic rays corresponding to points in the image plane
            void imageToOpticRay(const Vector2d &y, Vector3d &r);

            void imageToOpticRay(const vector<Vector2d> &y, vector<Vector3d> &r);

            double getFocalLength();


            const cv::Size &getImage_size() const {
                return image_size;
            }

            void setImage_size(const cv::Size &image_size) {
                Camera::image_size = image_size;
            }

        private:
            // Extrinsics parameters map 3D points from system coordinate system to camera coordinate
            // system
            RigidTransform extrinsics;

            ProjectionModel *intrinsics;

            ProjectionModelType proj_type;

            cv::Size image_size;
        };
    } //camera
} //iqmatic

#endif // VISION_CAMERA

