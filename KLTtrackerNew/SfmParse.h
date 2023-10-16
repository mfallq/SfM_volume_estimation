#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#pragma once

class View
{
public:
	View(int& v_id_, std::string& filename_, cv::Mat& R_, cv::Mat& c_);
	~View();
	int i_id, v_id, p_id;
	std::string filename;
	cv::Mat intrinsic, t, R;
protected:

	//std::map<int, std::pair<cv::Mat, cv::Mat>> poses;
};

class Structure
{
public:
	Structure(cv::Point3f voxel_point_, std::map<int, cv::Point2f> observations_);
	~Structure();		
	cv::Point3f point;
	std::map<int, cv::Point2f> observations;
	//std::vector<cv::Point2f> observations;
	std::string color = "None";
protected:
	
	
};

class Intrinsic
{
public:
	Intrinsic(int intr_id_, float focal_, cv::Point2f princp_, float width_, float height_);
	~Intrinsic();
	int intr_id;
	//Focal length
	float focal;
	//Principal point & width, height
	//float u0, v0;
	cv::Point2f princp;
	float width, height;
	cv::Mat K; 
protected:	
	
};

class Sfm
{
public:
	//Constructor
	Sfm();
	Sfm(const std::string& filename_);
	std::vector<Structure> structures;
	std::vector<Intrinsic> intrinsics;
	std::vector<View> views;
	std::string root_path;
	//Destructor
	~Sfm();
protected:
	void loadSfmJson();
	void unpack_intrinsics();
	void unpack_structure();
	void unpack_views();	
	void unpack_poses();
	std::map<int, std::pair<cv::Mat, cv::Mat>> poses;

	std::string filename;

};