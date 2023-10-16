//
// Created by iqmatic on 2016-03-15.
//

#include "Timepoint.h"

iqmatic::Timepoint::Timepoint(vector<iqmatic::sFrame> frames_) {
    frames = frames_;
    for (sFrame f : frames_){
        for (auto obs : f->getObservation_map()){
            addLandmark(obs.second->landmark);
        }
    }
}

iqmatic::Timepoint::Timepoint(vector<iqmatic::sFrame> frames_, uint64 timestamp_) : Timepoint(frames_) {
    timestamp = timestamp_;
}

void iqmatic::Timepoint::addLandmark(iqmatic::sLandmark lm) {
    landmarks[lm->getID()] = lm;
}

iqmatic::sTimepoint iqmatic::Timepoint::create(vector<iqmatic::sFrame> frames_) {
    sTimepoint t = std::shared_ptr<Timepoint>(new Timepoint(frames_));
    for (sFrame f : t->getFrames()){
        f->setTp(t.get());
    }
    return t;
}

iqmatic::sTimepoint iqmatic::Timepoint::create(vector<iqmatic::sFrame> frames_, uint64 timestamp_) {
    sTimepoint t = std::shared_ptr<Timepoint>(new Timepoint(frames_, timestamp_));
    for (sFrame f : t->getFrames()){
        f->setTp(t.get());
    }
    return t;
}
