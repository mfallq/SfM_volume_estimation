#pragma once

#include <memory>
#include <deque>

using SimLandmarkID = int64;
using LandmarkID = int64;

namespace iqmatic {

class Timepoint;
using sTimepoint = std::shared_ptr<Timepoint>;

namespace camera {
class Camera;
}
using sCamera = std::shared_ptr<camera::Camera>;

class Frame;
using sFrame = std::shared_ptr<Frame>;

class Landmark;
using sLandmark = std::shared_ptr<Landmark>;

class Observation;
using sObservation = std::shared_ptr<Observation>;
using Trajectory = std::deque<sObservation>;

} // iqmatic::
