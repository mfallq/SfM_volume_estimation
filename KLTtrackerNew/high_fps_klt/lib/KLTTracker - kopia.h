#ifndef TRACKERS_KLTTRACKER
#define TRACKERS_KLTTRACKER

#include <iostream>
#include <iomanip>
#include <vector>
#include <utility>
#include <sstream>

#include "opencv2/opencv.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/cudaoptflow.hpp"
//#include "opencv2/cudaimgproc.hpp"
#include "../../SensorData.h"
#include "definitions.h"
#include "core_structure\Frame.h"
#include "core_structure\Landmark.h"
#include "core_structure\PoseGraph_defs.h"
#include "iqmatic_defs.h"

using std::vector;
using std::cout;
using std::endl;



namespace vision {

namespace trackers
{

class KLTTracker
{
protected:
    
    MotionField mf;
    PointList prev_pts, curr_pts;
	
    cv::Mat prev_im, curr_im;
    ImagePyramid prev_pyr, curr_pyr;

    cv::Size win_size;
    uint pyr_lvl;
    uint max_iter;
    uint target_npts;
    uint frame_num, keyframe_num;
	double fps, totalFrames, epipolarThresh;
	float min_pt_d;
	//double timestamp;	

	uint extendMotionField();
	void generateObservations();
	//Added
	void keyPoints();
	double medianDisplacement();
	double medianThresh;
	string videoPath;
	// To store landmark IDs
	std::map<int, std::vector<int>> IDs;
	Track key_points;
	MotionField key_mf;

	// Iqmatic data
	std::vector<std::map<uint, cv::Point2f>> initial_pts;	
	std::vector<iqmatic::Frame> key_frames;
	iqmatic::Frame current_frame;	
	
public:
    KLTTracker();
    ~KLTTracker();
	void saveKeyFrames();
	void displayKeyFrames();
    MotionField getMotionField();
	std::vector<iqmatic::Frame> getKeyFrames(){ return key_frames; };
	MotionField& getKeyMotionField() { return key_mf; };
	std::map<int, std::vector<int>>& getIDs() { return IDs; };	
	double getTotalNumberOfFrames(){ return totalFrames; };

    void drawTracks(cv::Mat& out, uint max_length, float scaling_factor);
    //void drawSideBySide(cv::Mat& out, float scaling_factor);
    //void drawDiff(cv::Mat& out, float scaling_factor);
	//Added
	bool newFrame();
	void process(SensorData& SF, double imageScaling, double NoFrames = 0);
	void geometricConstraint();

};

#ifdef WITH_CUDA

class KLTTracker_cuda : public KLTTracker
{
protected:
    cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> klt;
    cv::Ptr<cv::cuda::CornersDetector> gftt;
    cv::cuda::GpuMat prev_im_cu, curr_im_cu;

public:
    KLTTracker_cuda();
    ~KLTTracker_cuda();
    void newImage(cv::Mat& in);
    uint extendMotionField();
    void morePoints();
};

#else

class KLTTracker_cuda : public KLTTracker
{
public:
    KLTTracker_cuda();
    ~KLTTracker_cuda();
};

#endif // WITH_CUDA
} //trackers

} //vision
#endif // TRACKERS_KLTTRACKER
