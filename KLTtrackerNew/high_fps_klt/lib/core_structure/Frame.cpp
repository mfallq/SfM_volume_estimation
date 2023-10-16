//
// Created by iqmatic on 2016-03-15.
//

#include "Frame.h"

namespace iqmatic {

	void Frame::newObservation(sLandmark lm, const cv::Point2f& position, cv::Mat descriptor)
{
	cv::Point2f npos;
   // bool finite;

    //camera->imageToIdealPlane(position, npos, finite);
    sObservation obs = Observation::create(this->tp, this, lm, position, npos, descriptor);
    lm->appendObservation(obs);

    //observations.push_back(obs);
    uint64 lmid = obs->landmark_ID;
    observation_map[lmid] = obs;
}

void Frame::addObservation(sObservation obs)
{
    //observations.push_back(obs);
    observation_map[obs->landmark_ID] = obs;
}
/*
void Frame::addGTObservation(const GTObservation& gt_obs)
{
    gt_observations[gt_obs.lm_id] = gt_obs;
}
*/
sFrame Frame::create(const cv::Mat &gray, const cv::Mat &rgb)
{	
    return std::shared_ptr<Frame>(new Frame(gray, rgb));
}

void Frame::removeObservation(sObservation obs)
{
    observation_map.erase(obs->landmark_ID);
}

} // iqmatic::