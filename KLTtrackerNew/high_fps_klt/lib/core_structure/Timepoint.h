//
// Created by iqmatic on 2016-03-15.
//

#ifndef IQMATIC_TIMEPOINT_H
#define IQMATIC_TIMEPOINT_H

#include "Frame.h"
#include "Landmark.h"
//#include "../sources/DataToken.h"

namespace iqmatic {

    class Timepoint {

    public:

        Timepoint(std::vector<sFrame> frames_);
        Timepoint(std::vector<sFrame> frames_, uint64 timestamp_);

        ~Timepoint() { }


        static sTimepoint create(std::vector<sFrame> frames_);
        static sTimepoint create(std::vector<sFrame> frames_, uint64 timestamp_);

        const std::vector<sFrame> &getFrames() const {
            return frames;
        }

        std::map<uint64, sLandmark>& getLandmarks()
        {
            return landmarks;
        }

        void addLandmark(sLandmark lm);

        uint64 getTimestamp() const {
            return timestamp;
        }

        void setTimestamp(uint64 timestamp_) {
            timestamp = timestamp_;
        }
		/*
        const RigidTransform &getT_vw() const {
            return T_vw;
        }

        RigidTransform& getMutableT_vw()
        {
            return T_vw;
        }

        void setT_vw(const RigidTransform &T_vw_) {
            T_vw = T_vw_;
        }
		*/

    private:

        std::vector<sFrame> frames;
        std::map<uint64, sLandmark> landmarks;
        uint64 timestamp;
        //RigidTransform T_vw;


    };

} //iqmatic
#endif //IQMATIC_TIMEPOINT_H
