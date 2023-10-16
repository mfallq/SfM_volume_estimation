//
// Created by iqmatic on 2016-03-15.
//

#include "Landmark.h"

namespace iqmatic {

uint64 Landmark::landmark_counter = 0;
bool Landmark::initialized = false;

sLandmark Landmark::create()
{
    sLandmark slm = std::shared_ptr<Landmark>(new Landmark());
    return slm;
}

uint64 Landmark::init()
{
    uint64 tmp = landmark_counter;
    landmark_counter = 0;
    initialized = true;
    return tmp;
}

Landmark::Landmark()
{
    if (!Landmark::initialized)
        init();
    ID = Landmark::landmark_counter += 1;
    world_position = Vector3d(0,0,0);
}

Landmark::~Landmark()
{
}

void Landmark::appendObservation(sObservation obs, bool is_in_keyframe)
{
    assert(obs->landmark.get() == this && "Cannot add an observation of a different landmark");
    track.push_front(obs);
    if (is_in_keyframe || this->track.size() == 1) {
        this->kf_observation = obs;
    }
}
/*
void Landmark::appendGTObservation(const GTObservation& gt_obs)
{
    gt_track.push_back(gt_obs);
}
*/
uint64 Landmark::trackLength()
{
    return track.size();
}

sObservation Observation::create(Timepoint *tp, Frame *frame, sLandmark lm, cv::Point2f position,
	cv::Point2f n_position, cv::Mat desc)
{
    sObservation obs = Observation::create();

    obs->tp = tp;
    obs->frame = frame;
    obs->position = position;
    obs->normalized_position = n_position;
    obs->descriptor = desc;
    obs->landmark_ID = lm->getID();
    obs->landmark = lm;

    return obs;
}

sObservation Observation::create()
{
    return std::shared_ptr<Observation>(new Observation());
}

Observation::Observation()
{
}

} // iqmatic::