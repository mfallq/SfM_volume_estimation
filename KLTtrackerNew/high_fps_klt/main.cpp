#include <iostream>
#include <iomanip>
#include <vector>
#include <utility>
#include <sstream>
#include <string>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include "lib/KLTTracker.h"
#include "../SensorData.h"
#include "../SfmParse.h"
#include "../SpaceCarving.h"
//using std::vector;
using namespace cv;
using namespace std;


int main(int argc, char* argv[])
{	
	int NoFrames = 0;
	double imageScaling = 1;	//0.8 < imageScaling < 1.0
	bool writeToFile = false, visualize = true;
	//to test imu:
	//const string sensorPath = "D:/Lagring/Plugg/Examensarbete/Data/SensorFusion/liggandes_20160518T143016.txt"; //OK
	//const string sensorPath = "D:/Lagring/Plugg/Examensarbete/Data/SensorFusion/liggandesner_20160519T152542.txt";
	//const string sensorPath = "D:/Lagring/Plugg/Examensarbete/Data/SensorFusion/sidan_20160518T143044.txt";
	//const string sensorPath = "D:/Lagring/Plugg/Examensarbete/Data/SensorFusion/upp_20160518T143116.txt";
	//const string sensorPath = "D:/Lagring/Plugg/Examensarbete/Data/SensorFusion/rak_20160527T143622.txt";//OK
	//const string sensorPath = "D:/Lagring/Plugg/Examensarbete/Data/SensorFusion/R_20160608T143434.txt"; //some drift, prob due to bias
	//const string videoPath = "D:/Lagring/Plugg/Examensarbete/Data/20160527_143435.mp4"; //test by using *_timestamp+1.mp4
		
	//Gravel pile-scene 1-1 38.9ton +-50kg  
	//const string sensorPath = "../Data/sensorLog_20160527T111147.txt"; // Large bias drift
	//const string videoPath = "../Data/20160527_111152.mp4"; 
	//Pile 1-2 - counter clockwise 
	//const string sensorPath = "../Data/sensorLog_20160527T111309.txt"; 
	//const string videoPath = "../Data/20160527_111314.mp4";	 
	
	// Pile 2-1 20.1ton +-50kg 
	//const string sensorPath = "../Data/sensorLog_20160527T111932.txt";
	//const string videoPath = "../Data/20160527_111936.mp4";
	// Pile 2-2  
	const string sensorPath = "../Data/sensorLog_20160527T112038.txt"; 
	const string videoPath = "../Data/20160527_112043.mp4"; 

	
	// Load files
	SensorData IMU = SensorData(sensorPath, videoPath);		
	cv::Mat pos = IMU.applyTranslations(writeToFile);	
	//cout << "End pos [IMU]: " << pos << endl;	
	//Tracking		
	vision::trackers::KLTTracker klt = vision::trackers::KLTTracker(IMU, imageScaling, NoFrames, visualize);
	klt.process();
	klt.saveKeyFrames();
	

	// Now run SfM
	cout << "Now run OpenMVG SfM. Then export excintrics to json file with this program. Then proceed tocalculate volume in the MatLab script" << endl;	
	//system("pause");	
	//const string sfmPath = "../Data/images/sfm-excintrics/extrinsictesting.json";
	//const string sfmPath = "../Data/images/pile1-2-OK/pile1-2extrinsic.json";
	const string sfmPath = "../Data/images/pile2-2-OK/pile2-2extrinsic.json";
	Sfm sfm_scene = Sfm(sfmPath);
	SpaceCarving voxelization = SpaceCarving(sfm_scene);

	return 0;

} // main

