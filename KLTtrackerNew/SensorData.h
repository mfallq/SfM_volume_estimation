#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#pragma once

class SensorData
{
public:
	SensorData(const std::string& sensorPath_, const std::string& videoPath_);
	~SensorData();	
	SensorData();
	//Process data	
	cv::Mat SensorData::applyTranslations(bool write=false);	
	void syncImuWithVideo();	
	//double getImuScale();
	//Gets
	std::string getVideoPath(){ return videoPath; };
	double sampling_rate = 100; //In hertz, default 100, need get
	void writeToFile(std::string fileName, std::vector<cv::Mat> data, bool write);
	std::vector<cv::Mat> getPositions();
protected:
	//Functions to form data-matrices from log-file
	void getNextLineAndSplitIntoTokens(std::istream& str);
	cv::Mat formDataMatrix(std::vector<std::string> input);
	//Functions to use sensor data
	void integrateAccelerometer(bool write);	
	void estimateBias(bool flag=false, double seconds = 1);
	void SensorData::cameraToWorldRotation(cv::Mat a);
	cv::Mat g = (cv::Mat_<double>(3, 1) << 0,9.82,0); //0,0,9.82);
	//Data members for storing rotation and translation
	std::vector<cv::Mat> raw_acc, IMU_rotations, IMU_acc, IMU_velocity, IMU_positions, SFM_rotations, SFM_translations;
	std::string videoPath, sensorPath;	
	cv::Mat Acc_data, Gyro_data, Gps_data;
	cv::Mat Acc_bias, Gyro_bias, R_init;
	double t0; //sampling_rate=100; //Timepoint for first sample
	
};

