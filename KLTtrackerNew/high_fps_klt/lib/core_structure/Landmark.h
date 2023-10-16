//
// Created by iqmatic on 2016-03-15.
//

#ifndef IQMATIC_LANDMARK_H
#define IQMATIC_LANDMARK_H

#include <memory>
#include <deque>
#include "../common.h"
#include <opencv2/core/mat.hpp>
#include "../iqmatic_defs.h"
//#include "../utilities.h"
#include "PoseGraph_defs.h"
#include "Frame.h"

namespace iqmatic {
	/*
struct GTObservation
{
    SimLandmarkID pcl_id; // Synthetic point cloud index
    LandmarkID lm_id;   // Landmark index
    cvl::Vector3d world; // pcl coordinates in the world frame
    cvl::Vector3d vehicle; // pcl coordinates in the vehicle frame
    cvl::Vector2d image; // image coordinates [pixels]
    cvl::Vector2d ideal; // ideal image coordinates
    cvl::RigidTransform T_cw;

    GTObservation()
    {
    }

    GTObservation(SimLandmarkID pcl_id, LandmarkID lm_id, const cvl::Vector3d& world_pos,
        const cvl::Vector3d& vehicle_pos, const cvl::Vector2d& image_pos, const cvl::Vector2d& ideal_pos)
    {
        this->pcl_id = pcl_id;
        this->lm_id = lm_id;
        this->world = world_pos;
        this->vehicle = vehicle_pos;
        this->image = image_pos;
        this->ideal = ideal_pos;
    }
};*/

class Observation
{
protected:
    Observation();

public:
    static sObservation create();
    static sObservation create(Timepoint *tp, Frame *frame, sLandmark lm, cv::Point2f position, cv::Point2f n_position,
                               cv::Mat desc = cv::Mat());

    Timepoint *tp;
    Frame *frame;
    uint64 landmark_ID;
    sLandmark landmark;

	cv::Point2f position;
	cv::Point2f normalized_position;
    cv::Mat descriptor;
};


class Landmark
{
protected:
    Landmark();

public:
    ~Landmark();

    static sLandmark create();

    // Resets the landmark counter, returns previous counter value
    static uint64 init();

    void appendObservation(sObservation obs, bool is_in_keyframe = false);

    //void appendGTObservation(const GTObservation& gt_obs);

    uint64 trackLength();

    uint64 getID() const
    {
        return ID;
    }

    sObservation kf_observation; // The observation that belongs to the latest keyframe
    Trajectory track;
   // std::vector<GTObservation> gt_track;
    Vector3d world_position;
    static uint64 landmark_counter;
    static bool initialized;

private:
    uint64 ID;

    };


} //iqmatic
#endif //IQMATIC_LANDMARK_H
