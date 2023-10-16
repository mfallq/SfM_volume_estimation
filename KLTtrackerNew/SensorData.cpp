#include "SensorData.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include "jsoncons\json.hpp"
// for convenience
using jsoncons::json;
using jsoncons::json_deserializer;
using jsoncons::wjson;
using namespace cv;
using namespace std;
//test
#include <iomanip>   //setprecision on cout

SensorData::SensorData(const string& sensorPath_, const string& videoPath_)
{
	sensorPath.assign(sensorPath_);
	videoPath.assign(videoPath_);
	std::ifstream file(sensorPath);
	
	if (file.good())
	{				
		SensorData::getNextLineAndSplitIntoTokens(file);		
		//std::cout << "SensorFusion file loaded. " << endl;
	}
	else
	{
		cout << "Error loading sensordata: is the path correct? " << endl;
	}
}


SensorData::SensorData()
{}
SensorData::~SensorData()
{}

std::vector<cv::Mat> SensorData::getPositions()
{	
	return IMU_positions;
}

void SensorData::getNextLineAndSplitIntoTokens(std::istream& str)
{
	bool first = true;
	std::vector<std::string> AccVec, GyroVec, GpsVec;
	std::string					line,cell, buf;
	char						c;
	std::size_t found;
	while (str)
	{			
		std::getline(str, line);
		std::stringstream			lineStream(line);	
		
		vector<string> tokens; // Create vector to hold our words		

		while (std::getline(lineStream, cell))
		{
			if (first)
			{	// We need to store first timestamp (not a nice solution right here..)
				first = false;
				stringstream ss(cell);
				(ss >> buf);
				tokens.push_back(buf);
				t0 = stoi(tokens[0]);	//Internal timepoint for first sample			
			}			

			// Find the type of sensor response, NEEDS UPDATE TO FIND STRING
				
			found = cell.find("ACC");
			if (found != std::string::npos)
			{
				AccVec.push_back(cell);
				break;
			}
			found = cell.find("RAW");
			if (found != std::string::npos)
			{
				GyroVec.push_back(cell);
				break;
			}
			found = cell.find("GPS");
			if (found != std::string::npos)
			{
				GpsVec.push_back(cell);
				break;
			}	
		}
	}
	if (GpsVec.size()>1)
		Gps_data = SensorData::formDataMatrix(GpsVec);
	if (AccVec.size()>1)
		Acc_data = SensorData::formDataMatrix(AccVec);
	if (GyroVec.size()>1)
		Gyro_data = SensorData::formDataMatrix(GyroVec);	
}

Mat SensorData::formDataMatrix(std::vector<std::string> input)
{
	size_t rows = input.size()-1;
	Mat resultMat(rows, 4, DataType<double>::type);	
	for (size_t i = 0; i < rows; i++)
	{
		string str(input[i]);
		string buf; // Have a buffer string
		stringstream ss(str); // Insert the string into a stream

		vector<string> tokens; // Create vector to hold our words
		
		while (ss >> buf)
			tokens.push_back(buf);
		
		resultMat.at<double>(i, 0) = (stoi(tokens[0])-t0)/1000; // Timestamp in ms->seconds		
		resultMat.at<double>(i, 1) = stod(tokens[2]); // X (Skip type and go to readings)
		resultMat.at<double>(i, 2) = stod(tokens[3]); // Y
		resultMat.at<double>(i, 3) = stod(tokens[4]); // Z
	}
	return resultMat;
}

// Used to remove samples recorded before the video (not needed when capturing sensor+camera at the same time)
void removeSamples(cv::Mat &data, double t_diff)
{ //Data: 4xN [T X Y Z], now remove samples up to t_diff 		
	double timestamp, time0=0;	
	if (!data.empty())
	{	
	for (int i = 0; i < data.rows; i++)
	{		
		timestamp = data.at<double>(i, 0);
		if (timestamp > t_diff ) 
		{		
			// Remove samples 
			time0 = timestamp;
			data = data.colRange(0, data.cols).rowRange(i, data.rows).clone();
			break;
		}			
	}	
	data.col(0) -= time0;
	}	
}
//Time_sync
void SensorData::syncImuWithVideo()
{ // Sync Acc_data, Gyro_data and Gps_data with video [0 X Y Z]
	const char * vPath(videoPath.c_str());
	const char * sPath(sensorPath.c_str());
	
	char drive[_MAX_DRIVE];
	char dir[_MAX_DIR];
	char fname[_MAX_FNAME];
	char ext[_MAX_EXT];
	_splitpath_s(vPath, drive, dir, fname, ext);

	//Find time stamps of video and IMU
	char t_c0[_MAX_FNAME], t_imu0[_MAX_FNAME];
	char * pch;
	// Video file separates time with '_'
	pch = strrchr(fname, '_');		
	int index = pch - fname+1 ;
	//printf("Last occurence of '_' found at %d \n", index); 
	memmove(fname, fname + index, strlen(fname));
	strcpy(t_c0 ,fname);	
	
	// Sensor file separates time with 'T'
	_splitpath_s(sPath, drive, dir, fname, ext);	
	pch = strrchr(fname, 'T');	
	index = pch - fname + 1;	
	memmove(fname, fname + index, strlen(fname));
	strcpy(t_imu0, fname);
	
	//Time difference between IMU and C recordings:
	double t_diff = atof(t_imu0) - atof(t_c0);	// in seconds
	
	//Remove t_diff samples from start of IMU recordings and recordings past video sequence:	
	if (t_diff > 0) 
		cerr << "IMU recordings missing for first frames! Please start recording sensordata before video." << endl;
	else
	{		
	//Remove samples before video recording
		t_diff = abs(t_diff);		
		
		if (t_diff != 0)
		{	
			//Remove initial rotation from first second of readins (Should also calculate bias)
			estimateBias(true, 1);
			//cout << "t_diff: " << t_diff << endl;
			//Then remove samples
			removeSamples(Acc_data, t_diff);			
			removeSamples(Gps_data, t_diff);			
			removeSamples(Gyro_data, t_diff);				
		}
		else
			estimateBias(false);
	}			
	
}
// Implementation based on Hannes Ovrén@liu CRISP-code
cv::Mat fastIntegrate(cv::Mat gyro_data, float dt) //Gyro data and dt = time between samples
{
	//def integrate_gyro_quaternion_uniform(np.ndarray[DTYPE_t, ndim = 2] gyro_data, np.float dt) :
	//NB : Quaternion q = [a, n1, n2, n3], scalar first	
	int N = gyro_data.rows;
	cv::Mat q_list(N, 4, DataType<double>::type);	

	//Iterate over all(except first)
	//unsigned int i, j;
	double wx, wy, wz;
	double q0, q1, q2, q3;

	//cdef np.ndarray[DTYPE_t, ndim = 1] qnew = np.zeros((4, ))
	cv::Mat qnew;
	double qnorm;
	double dt_half = dt / 2.0;

	// Initial rotation
	q0 = 1.0;
	q1 = q2 = q3 = 0.0;

	for (int i = 0; i < N; i++)
	{
		wx = gyro_data.at<double>(i, 1);
		wy = gyro_data.at<double>(i, 2);
		wz = gyro_data.at<double>(i, 3);
		
		q_list.at<double>(i, 0) = q0 + dt_half * (-wx*q1 - wy*q2 - wz*q3);
		q_list.at<double>(i, 1) = q1 + dt_half * (q0*wx + q2*wz - wy*q3);
		q_list.at<double>(i, 2) = q2 + dt_half * (wy*q0 - wz*q1 + wx*q3);
		q_list.at<double>(i, 3) = q3 + dt_half * (wz*q0 + wy*q1 - wx*q2);

		// Normalize
		qnorm = sqrt(pow(q_list.at<double>(i, 0), 2) + pow(q_list.at<double>(i, 1), 2) + pow(q_list.at<double>(i, 2), 2) + pow(q_list.at<double>(i, 3), 2));
		for (int j = 0; i < 4; i++)
		{
			if (qnorm != 0)
			{
				q_list.at<double>(i, j) /= qnorm;
			}
			// New prev values
			q0 = q_list.at<double>(i, 0);
			q1 = q_list.at<double>(i, 1);
			q2 = q_list.at<double>(i, 2);
			q3 = q_list.at<double>(i, 3);
		}
	}
	return q_list;
}

cv::Mat quat_to_rotation_matrix(cv::Mat q)
{/*
Convert unit quaternion to rotation matrix
Parameters
------------ -
q : (4, 1) mat
	Unit quaternion, scalar as first element

	Returns
	----------------
R : (3, 3) mat
	Rotation matrix
*/
	//q = q.flatten();
	//assert q.size == 4
	//assert_almost_equal(np.linalg.norm(q), 1.0, err_msg = "Not a unit quaternion!")
	double q0 = q.at<double>(0);
	double q1 = q.at<double>(1);
	double q2 = q.at<double>(2);
	double q3 = q.at<double>(3);
	double qq0 = pow(q0, 2);
	double qq1 = pow(q1, 2);
	double qq2 = pow(q2, 2);
	double qq3 = pow(q3, 2);

	Mat R = (Mat_<double>(3, 3) << qq0 + qq1 - qq2 - qq3, 2 * q1 * q2 - 2 * q0 * q3, 2 * q1 * q3 + 2 * q0 * q2, 2 * q1 * q2 + 2 * q0 * q3, qq0 - qq1 + qq2 - qq3, 2 * q2 * q3 - 2 * q0 * q1, 2 * q1 * q3 - 2 * q0 * q2, 2 * q2 * q3 + 2 * q0 * q1, qq0 - qq1 - qq2 + qq3);
	
	return R;
}

cv::Mat integrate_gyro_quaternion(cv::Mat gyro_data, cv::Mat Gyro_bias)
{/*
Integrate angular velocities to rotations

Parameters
-------------- -
gyro_ts : ndarray
		  Timestamps
	  gyro_data : (3, N) ndarray
				  Angular velocity measurements

				  Returns
				  -------------- -
			  rotations : (4, N) ndarray
						  Rotation sequence as unit quaternions(first element scalar)

*/
//NB : Quaternion q = [a, n1, n2, n3], scalar first
	//gyro_ts.shape[0]
	cv::Mat gyro_ts = gyro_data.col(0); // Timestamps in first col	
	gyro_data = gyro_data.colRange(1, gyro_data.cols); // Data [x y z]	
	int N = gyro_ts.rows;
	cv::Mat q_list; // Nx4 quaternion list	
	q_list = Mat::zeros(gyro_ts.rows, 4, DataType<double>::type);	
	//cv::Mat q_list(N, 4, DataType<double>::type);
	//q_list.col(0) = 1; //Initial rotation [1, 0, 0, 0] i.e. no rotation
	q_list.at<double>(0, 0) = 1;
	//q_list.row(0) = (Mat_<double>(1, 4) << 1, 0, 0, 0);
	
// Iterate over all(except first)
	cv::Mat w, qprev, qnew, qtemp, A;
	double dt, qnorm;	
	for (int i = 1; i < N; i++) //N-1
	{
		w = gyro_data.row(i)-Gyro_bias;		
		dt = gyro_ts.at<double>(i) - gyro_ts.at<double>(i - 1);
		
		qprev = q_list.row(i - 1);
		
		transpose(qprev, qprev);			//Row vector, needs transpose	
		
		double w0 = w.at<double>(0);
		double w1 = w.at<double>(1);
		double w2 = w.at<double>(2);	
		
		A = (Mat_<double>(4, 4) << 0, -w0, -w1, -w2, w0, 0, w2,-w1, w1,	-w2, 0, w0, w2, w1, -w0, 0);		
		
		qnew = (Mat::eye(4, 4, CV_64F) + (dt / 2.0) * A)*(qprev);	//.dot(qprev) in crisp	
		
		qnorm = norm(qnew,NORM_L2);
	
		if (qnorm != 0)
		{
			qnew /= qnorm;
		}			
		q_list.at<double>(i, 0) = qnew.at<double>(0);
		q_list.at<double>(i, 1) = qnew.at<double>(1);
		q_list.at<double>(i, 2) = qnew.at<double>(2);
		q_list.at<double>(i, 3) = qnew.at<double>(3);
		
	}
	return q_list;
}


void SensorData::writeToFile(string fileName, vector<cv::Mat> data,bool write)
{
	cv::Mat position, prev_position;
	prev_position = (Mat_<double>(1, 3) << 0, 0, 0); // Origin in first pos

	int N = data.size();

	string saveName, entry;
	const char * path(videoPath.c_str());
	const char * textPath;
	char drive[_MAX_DRIVE];
	char dir[_MAX_DIR];
	char fname[_MAX_FNAME];
	char ext[_MAX_EXT];
	FILE * pFile;

	_splitpath_s(path, drive, dir, fname, ext);
	saveName = drive + string(dir) + fileName; 
	// Convert to const char *
	textPath = (saveName.c_str());
	remove(textPath);

	int progress = 0;
	cout << "\n";
	for (int i = 0; i < N; i++)//
	{
		progress = int((100.0*(i + 1) / (N)));
		std::cout << "Writing " << fileName << " file: " << progress << " %\r";
		position = data[i]; // + prev_position

		textPath = (saveName.c_str());
		pFile = fopen(textPath, "a");  //opens and writes to file
		if (pFile != NULL)
		{
			entry = std::to_string(position.at<double>(0)) + " " + std::to_string(position.at<double>(1)) + " " + std::to_string(position.at<double>(2)) + "\n";
			textPath = entry.c_str();
			fputs(textPath, pFile);
			fclose(pFile);
		}// close file
		prev_position = position;
	}
}

//Simple implementation of integration
cv::Mat integration(std::vector<cv::Mat> function, double t_curr, double t_next)
{
	double dt = abs(t_next - t_curr);
	
	cv::Mat result(1, 4, DataType<double>::type);
	
	result.at<double>(0, 0) = t_curr; 
	unsigned int N = function.size();

	if (N == 1) //Acceleration as input, simple integration:
	{
		//Simple "block"-integration	
		result.at<double>(0, 1) = function[0].at<double>(0)*dt;
		result.at<double>(0, 2) = function[0].at<double>(1)*dt;
		result.at<double>(0, 3) = function[0].at<double>(2)*dt;
	}
	else
	{ // Velocity as input; integrate response up until now:
		result.at<double>(0, 1) = 0;
		result.at<double>(0, 2) = 0;
		result.at<double>(0, 3) = 0;		
		for (int i = 0; i < N; i++)
		{
			result.at<double>(0, 1) += function[i].at<double>(0,1);
			result.at<double>(0, 2) += function[i].at<double>(0,2);
			result.at<double>(0, 3) += function[i].at<double>(0,3);
		}		
		result.at<double>(0, 1) *= dt;
		result.at<double>(0, 2) *= dt;
		result.at<double>(0, 3) *= dt;
	}	
	
	return result;
	
}

std::vector<cv::Mat> trapzIntegration(std::vector<cv::Mat> function, std::vector<double> dt_vec)
{
	double period=0,dt=0;	
	unsigned int N = function.size(), counter = 0;
	cv::Mat result(1, 3, DataType<double>::type);
	std::vector<cv::Mat> resultVec, tempVec;
	// First result (0,0,0)
	
	result.at<double>(0) = 0;
	result.at<double>(1) = 0;
	result.at<double>(2) = 0;

	resultVec.push_back(result.clone());

	tempVec.push_back(result.clone());
	for (int i = 1; i < N; i++)
	{
		dt = dt_vec.at(i);
		period += dt;
		counter++;


		result.at<double>(0) = resultVec[i - 1].at<double>(0) + dt*((function[i - 1].at<double>(0) + function[i].at<double>(0)) / 2);
		result.at<double>(1) = resultVec[i - 1].at<double>(1) + dt*((function[i - 1].at<double>(1) + function[i].at<double>(1)) / 2);
		result.at<double>(2) = resultVec[i - 1].at<double>(2) + dt*((function[i - 1].at<double>(2) + function[i].at<double>(2)) / 2);

		resultVec.push_back(result.clone());


		/*

		result.at<double>(0) = tempVec[i - 1].at<double>(0) + dt*((function[i - 1].at<double>(0) + function[i].at<double>(0)) / 2);
		result.at<double>(1) = tempVec[i - 1].at<double>(1) + dt*((function[i - 1].at<double>(1) + function[i].at<double>(1)) / 2);
		result.at<double>(2) = tempVec[i - 1].at<double>(2) + dt*((function[i - 1].at<double>(2) + function[i].at<double>(2)) / 2);

		tempVec.push_back(result.clone());

		if (period>0.1)
		{
			for (int k = 0; k < counter; k++)
			{
				resultVec.push_back(result.clone());
			}
			counter = 0;
			period = 0;
			tempVec[i - 1].at<double>(0) = 0;
			tempVec[i - 1].at<double>(1) = 0;
			tempVec[i - 1].at<double>(2) = 0;
		}*/
	}
	return resultVec;
}

std::vector<cv::Mat> blockIntegration(std::vector<cv::Mat> function, std::vector<double> dt_vec)
{
	double dt;
	unsigned int N = function.size();
	cv::Mat result(1, 3, DataType<double>::type);
	std::vector<cv::Mat> resultVec;

	result.at<double>(0) = 0;
	result.at<double>(1) = 0;
	result.at<double>(2) = 0;

	for (int i = 0; i < N; i++)
	{		
		dt = dt_vec.at(i);
		result.at<double>(0) += function[i].at<double>(0)*dt; 
		result.at<double>(1) += function[i].at<double>(1)*dt;
		result.at<double>(2) += function[i].at<double>(2)*dt;

		resultVec.push_back(result.clone());
	}
	return resultVec;
}

std::vector<cv::Mat> doubleIntegration(std::vector<cv::Mat> function, std::vector<double> dt_vec)
{
	double dt = 0;
	unsigned int N = function.size();
	cv::Mat result(1, 3, DataType<double>::type);
	std::vector<cv::Mat> resultVec,tempVec;

	result.at<double>(0) = 0;
	result.at<double>(1) = 0;
	result.at<double>(2) = 0;

	for (int i = 0; i < N; i++)
	{
		dt = dt_vec.at(i);
		result.at<double>(0) += function[i].at<double>(0)*dt;
		result.at<double>(1) += function[i].at<double>(1)*dt;
		result.at<double>(2) += function[i].at<double>(2)*dt;

		tempVec.push_back(result.clone());
	}
	
	dt = 0;
	result.at<double>(0) = 0;
	result.at<double>(1) = 0;
	result.at<double>(2) = 0;
	for (int i = 0; i < N; i++)
	{
		dt += dt_vec.at(i);
		
		result.at<double>(0) = tempVec[i].at<double>(0)*dt;
		result.at<double>(1) = tempVec[i].at<double>(1)*dt;
		result.at<double>(2) = tempVec[i].at<double>(2)*dt;

		resultVec.push_back(result.clone());
	}
	return resultVec;
}



//Use first second to estimate and then remove bias from acc and gyro
void SensorData::estimateBias(bool flag, double seconds)
{
	Acc_bias = (Mat_<double>(1, 3) << 0, 0, 0);
	Gyro_bias = (Mat_<double>(1, 3) << 0, 0, 0);

	cv::Mat gtranspose;
	if (flag)
	{//Estimate bias on first seconds
		int N = (int)(seconds*sampling_rate);		
		//Remove gravity (UPDATE NEEDED: rotation?)
		for (int i = 1; i < N; i++)
		{			
			Acc_bias = Acc_bias + Acc_data.row(i).colRange(1, Acc_data.cols).clone();
			Gyro_bias = Gyro_bias + Gyro_data.row(i).colRange(1, Gyro_data.cols).clone();			
		}
		//Mean
		Gyro_bias /= N;
		Acc_bias /= N;		
		
		// Estimate initial rotation of phone (stored in R_init)
		cameraToWorldRotation(Acc_bias.clone());
		Acc_bias = (Mat_<double>(1, 3) << 0, 0, 0);
		transpose(Acc_bias, Acc_bias);		
		//transpose(Gyro_bias, Gyro_bias);
		//cout << "Acc_bias: " << Acc_bias << endl;//<< "\nGyro_bias: " << Gyro_bias << "\nR_init: " << R_init << endl;		
		//cout << "R*Acc_bias: " << R_init*Acc_bias << endl;
		//cout << "R*Acc_bias - g: " << R_init*Acc_bias - g << endl;
		//NOT CORRECT: Acc_bias =  R_init*Acc_bias - g;
		//Gyro_bias = R_init*Gyro_bias;
	}
	else
	{	
		Acc_bias = (Mat_<double>(1, 3) << 0, 0, 0);
		Gyro_bias = (Mat_<double>(1, 3) << 0, 0, 0);
		Mat I = Mat::eye(3, 3, CV_64F);
		R_init = I;
	}	
	//cout << "Acc_bias: " << Acc_bias <<  "\nR_init: " << R_init << "\nGyro_bias: " << Gyro_bias << endl;
	cout << "Using initial rotation matrix: \n" << R_init << endl;
}

void SensorData::cameraToWorldRotation(Mat a)
{ // Find R that rotates unit vector a to unit vector b and store into class member R_init
	Mat b, v;	

	double sinangle, cosangle;
	normalize(a, a);	
	normalize(g, b);
	transpose(a, a);	
	
	v = a.cross(b);	
	sinangle = norm(v);
	cosangle = a.dot(b);
	
	Mat vx = (Mat_<double>(3, 3) << 0, -v.at<double>(2), v.at<double>(1), v.at<double>(2), 0, -v.at<double>(0), -v.at<double>(1), v.at<double>(0), 0);
	Mat vx2, sinangle2;	
	
	vx2 = vx*vx;
	pow(sinangle, 2, sinangle2);	
	//R = I + vx +  vx ^ 2(1 - cosangle) / sinangle ^ 2;
	Mat I = Mat::eye(3, 3, CV_64F);
	R_init = I + vx + vx2*(1-cosangle)/sinangle2;
	//return R;
}

// Use R for each recording of accelerometer to remove gravity vector
void SensorData::integrateAccelerometer(bool write)
{ // To find:
	//For each reading we have: a_world-adj(t) = R*a(t) - g
	// Start by finding the Rotations for all readings:	
	cv::Mat R, R_transpose, q_list, q, a, a_imu, a_world; 
	std::vector<double> dt_vec;
	std::vector<cv::Mat> velocity, positions;
	double t_curr, t_next;	
	int N;
	int Ngyro = Gyro_data.rows;
	int Nacc = Acc_data.rows;
	if (Nacc > Ngyro)
		N = Ngyro;
	else
		N = Nacc;
	//First in form of quaternions from gyro-readings:
	q_list = integrate_gyro_quaternion(Gyro_data, Gyro_bias);	
	int progress = 0;	
	
	for (int i = 0; i < N-1; i++)
	{	
		progress = int((100.0*(i+1) / (N-1)));
		std::cout << "Using IMU-data to find rotations and translations of cellphone: " << progress << " %\r";
		//Then a sample is to be converted to rotation matrix:
		q = q_list.row(i);		
		R = quat_to_rotation_matrix(q);
		//transpose(R, R);		
		// Extract Acc recording, remove bias and Rotate it with R		
		a = Acc_data.row(i).colRange(1, Acc_data.cols).clone(); 	
		
		transpose(a, a_imu);		
		//a_imu = R_init*a_imu;
				
		if (0)
		{
			cout << "raw acc: " << a << endl;
			cout << "R_init*a_imu - Acc_bias: " << R_init*a_imu - Acc_bias << endl;
			cout << "R*a_imu - g: " << R*a_imu - g << endl;
		}
		//R_init*R*a_imu
		a_world = R_init*R*a_imu - g - Acc_bias; //TEST: R might be broken
		
		//FOR TESTING: acc and integration
		//a_world = (Mat_<double>(1, 3) << 2, 2, 2);
		//cout << "a_world: " << a_world << endl;

		// Save data for Double-integrate
		t_curr = Acc_data.at<double>(i, 0);
		t_next = Acc_data.at<double>(i+1, 0);		
		dt_vec.push_back(t_next - t_curr);
		
		//dt_vec.push_back(1);
		raw_acc.push_back(a.clone()); //Raw acc from sensors
		IMU_acc.push_back(a_world.clone()); //Transformed acc
		IMU_rotations.push_back(R.clone());	
	}
	velocity = trapzIntegration(IMU_acc, dt_vec);	
	//velocity = blockIntegration(IMU_acc, dt_vec);		
	
	if (write)
	{
		writeToFile("rawacceleration.txt", raw_acc, write);
		writeToFile("acceleration.txt", IMU_acc, write);
		writeToFile("velocity.txt", velocity, write);
	}

	positions = trapzIntegration(velocity, dt_vec);
	//positions = blockIntegration(velocity, dt_vec);

	//positions = doubleIntegration(IMU_acc, dt_vec);	
	// Save the [timestamp pos] vector and rotation		
	IMU_positions = positions;
}

cv::Mat SensorData::applyTranslations(bool write)
{	// Sync IMU and video and integrate accelerometer
	syncImuWithVideo();
	
	// Then integrate accelerometer
	integrateAccelerometer(write);

	// Data init
	cv::Mat position, prev_position;
	prev_position = (Mat_<double>(1, 3) << 0, 0, 0); // Origin in first pos
	position = (Mat_<double>(1, 3) << 0, 0, 0);

	int N = IMU_positions.size();
	
	string saveName, entry;
	const char * path(videoPath.c_str());
	const char * textPath;
	char drive[_MAX_DRIVE];
	char dir[_MAX_DIR];
	char fname[_MAX_FNAME];
	char ext[_MAX_EXT];
	FILE * pFile;
	if (write)
	{
	_splitpath_s(path, drive, dir, fname, ext);
	saveName = drive + string(dir) + "positions.txt";
	// Convert to const char *
	textPath = (saveName.c_str());
	remove(textPath);
	}	
	int progress = 0;
	cout << "\n";
	for (int i = 0; i < N; i++)//
	{
		progress = int((100.0*(i) / (N)));
		std::cout << "Writing IMU-positions to file: " << progress << " %\r";
				
		position = IMU_positions[i];
		if (write)
		{
			textPath = (saveName.c_str());
			pFile = fopen(textPath, "a");  //opens and writes to file
			if (pFile != NULL)
				{							
				entry = std::to_string(position.at<double>(0)) + " " + std::to_string(position.at<double>(1)) + " " + std::to_string(position.at<double>(2)) + "\n";
				textPath = entry.c_str();
				fputs(textPath, pFile);					
				fclose(pFile);
				}// close file 			
		}
		prev_position = position;
	
	}
	return position;
}

