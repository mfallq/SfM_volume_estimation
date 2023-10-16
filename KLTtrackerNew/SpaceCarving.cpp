#include "SpaceCarving.h"
#include "SfmParse.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <windows.h>
#include <typeinfo>       // std::bad_cast

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"


#include <vtkSmartPointer.h>
#include <vtkStructuredPoints.h>
#include <vtkPointData.h>
#include <vtkPLYWriter.h>
#include <vtkFloatArray.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkMarchingCubes.h>
#include <vtkCleanPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>

#include <vtkPLYReader.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkInteractorStyleTrackballCamera.h>

using namespace cv;
using namespace std;

SpaceCarving::SpaceCarving(Sfm sfm_scene_)
{	
	try
	{
		sfm_scene = sfm_scene_;
		// Parse data as needed		
		//All images has the same width and height
		IMG_WIDTH = sfm_scene.intrinsics[0].width;
		IMG_HEIGHT = sfm_scene.intrinsics[0].height;
		//N_cameras = sfm_scene.views.size();
		N_cameras = sfm_scene.views.size(); //kass 30+,50 icke 70,90
		
		cout << "Number of views: " << N_cameras << endl;
		K = cv::Mat::eye(3, 3, CV_32F);
		K.at<float>(0, 0) = sfm_scene.intrinsics[0].focal; /* fx */
		K.at<float>(1, 1) = sfm_scene.intrinsics[0].focal; /* fy */
		K.at<float>(0, 2) = sfm_scene.intrinsics[0].princp.x; /* cx */
		K.at<float>(1, 2) = sfm_scene.intrinsics[0].princp.y; /* cy */
	
		//Specify voxel dim
		VOXEL_DIM = 128;
		VOXEL_SIZE = VOXEL_DIM*VOXEL_DIM*VOXEL_DIM;
		VOXEL_SLICE = VOXEL_DIM*VOXEL_DIM;
		OUTSIDE = 0;	
		
		SpaceCarve();
	}
	catch (std::bad_cast& bc)
	{
		std::cerr << "bad_cast caught: " << bc.what() << '\n';
	}

}

string ExePath() {
	char buffer[MAX_PATH];
	GetModuleFileName(NULL, buffer, MAX_PATH);
	string::size_type pos = string(buffer).find_last_of("\\/");
	return string(buffer).substr(0, pos);
}

void SpaceCarving::exportModel(char *filename, vtkPolyData *polyData) {

	/* exports 3d model in ply format */
	vtkSmartPointer<vtkPLYWriter> plyExporter = vtkSmartPointer<vtkPLYWriter>::New();
	plyExporter->SetFileName(filename);
	plyExporter->SetInputData(polyData);
	plyExporter->Update();
	plyExporter->Write();
	cout << "Model exported" << endl;
}


coord SpaceCarving::project(camera cam, voxel v) {

	coord im;

	/* project voxel into camera image coords */
	float z = cam.P.at<float>(2, 0) * v.xpos +
		cam.P.at<float>(2, 1) * v.ypos +
		cam.P.at<float>(2, 2) * v.zpos +
		cam.P.at<float>(2, 3);

	im.y = (cam.P.at<float>(1, 0) * v.xpos +
		cam.P.at<float>(1, 1) * v.ypos +
		cam.P.at<float>(1, 2) * v.zpos +
		cam.P.at<float>(1, 3)) / z;

	im.x = (cam.P.at<float>(0, 0) * v.xpos +
		cam.P.at<float>(0, 1) * v.ypos +
		cam.P.at<float>(0, 2) * v.zpos +
		cam.P.at<float>(0, 3)) / z;

	return im;
}

void SpaceCarving::renderModel(float fArray[], startParams params) {	
	
	/* create vtk visualization pipeline from voxel grid (float array) */
	vtkSmartPointer<vtkStructuredPoints> sPoints = vtkSmartPointer<vtkStructuredPoints>::New();
	sPoints->SetDimensions(VOXEL_DIM, VOXEL_DIM, VOXEL_DIM);
	sPoints->SetSpacing(params.voxelDepth, params.voxelHeight, params.voxelWidth);
	sPoints->SetOrigin(params.startZ, params.startY, params.startX);
	sPoints->AllocateScalars(VTK_FLOAT,3); //SetScalarTypeToFloat();

	vtkSmartPointer<vtkFloatArray> vtkFArray = vtkSmartPointer<vtkFloatArray>::New();
	vtkFArray->SetNumberOfValues(VOXEL_SIZE);
	vtkFArray->SetArray(fArray, VOXEL_SIZE, 1);	

	sPoints->GetPointData()->SetScalars(vtkFArray);
	sPoints->GetPointData()->Update(); //TVEK!
	
	/* create iso surface with marching cubes algorithm */
	vtkSmartPointer<vtkMarchingCubes> mcSource = vtkSmartPointer<vtkMarchingCubes>::New();
	mcSource->SetInputData(sPoints);//SetInputConnection(sPoints->GetProducerPort());
	mcSource->SetNumberOfContours(1);
	mcSource->SetValue(0, 0.5);
	mcSource->Update();
	
	/* recreate mesh topology and merge vertices */
	vtkSmartPointer<vtkCleanPolyData> cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
	cleanPolyData->SetInputConnection(mcSource->GetOutputPort());
	cleanPolyData->Update();

	exportModel("D:/Lagring/Plugg/Examensarbete/Data/model.ply", mcSource->GetOutput()); //cleanPolyData->GetOutput() //mcSource->GetOutput()
	/* usual render stuff */	
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();	
	renderer->SetBackground(.45, .45, .9);	
	renderer->SetBackground2(.0, .0, .0);
	renderer->GradientBackgroundOn();

	vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);	

	vtkSmartPointer<vtkRenderWindowInteractor> interactor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	interactor->SetRenderWindow(renderWindow);

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputConnection(cleanPolyData->GetOutputPort());

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);

	/* visible light properties */
	actor->GetProperty()->SetSpecular(0.15);
	actor->GetProperty()->SetInterpolationToPhong();
	renderer->AddActor(actor);

	renderWindow->SetWindowName("Model of scene");
	renderWindow->Render();

	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
		vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New(); //like paraview

	interactor->SetInteractorStyle(style);

	interactor->Start();
}

void SpaceCarving::carve(float fArray[], startParams params, camera cam) {

	cv::Mat silhouette, distImage, sample;
	cv::Canny(cam.Silhouette, silhouette, 0, 255); //Edges
	cv::bitwise_not(silhouette, silhouette); //Invert bits
	cv::distanceTransform(silhouette, distImage, CV_DIST_L2, 3);
	
	//resize(cam.Silhouette, sample, sample.size(), 0.5, 0.5);
	//cv::imshow("cam.Silhouette", sample);
	
	//cout << cam.t << endl;
	resize(distImage, sample, sample.size(), 0.5, 0.5);
	cv::imshow("distImage", sample);
	char k = cv::waitKey(5);

	for (int i = 0; i < VOXEL_DIM; i++) {
		for (int j = 0; j < VOXEL_DIM; j++) {
			for (int k = 0; k < VOXEL_DIM; k++) {

				/* calc voxel position inside camera view frustum */
				voxel v;
				v.xpos = params.startX + i * params.voxelWidth;
				v.ypos = params.startY + j * params.voxelHeight;
				v.zpos = params.startZ + k * params.voxelDepth;
				v.value = 1.0f;
				
				coord im = project(cam, v);
				float dist = -1.0f;
				
				/* test if projected voxel is within image coords */
				if (im.x > 0 && im.y > 0 && im.x < IMG_WIDTH && im.y < IMG_HEIGHT) {
					dist = distImage.at<float>(im.y, im.x);						
					if (cam.Silhouette.at<uchar>(im.y, im.x) == OUTSIDE) {
						dist *= -1.0f;
					}
				}

				if (dist < fArray[i*VOXEL_SLICE + j*VOXEL_DIM + k]) {
					fArray[i*VOXEL_SLICE + j*VOXEL_DIM + k] = dist;
					
				}

			}
		}
	}

}
camera SpaceCarving::CameraFromView(int i, cv::Mat& current_img)
{
	camera c;
	cv::Mat Rt, hsv, sample, silhouette, scene, bg;
	/* silhouette */	
	cv::cvtColor(current_img, hsv, CV_BGR2HSV);
	cv::inRange(hsv, cv::Scalar(60, 0, 0), cv::Scalar(255, 255, 255), bg); 
	cv::inRange(current_img, cv::Scalar(35, 35, 35), cv::Scalar(120, 120, 120), silhouette);	

	silhouette -= bg;	

	int erosion_size = 1, dilation_size = 1;
	// Erosion and dilation to remove background objects
	Mat e_element = getStructuringElement(MORPH_ELLIPSE,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));	

	Mat d_element = getStructuringElement(MORPH_ELLIPSE,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	/// Apply the dilation operation to fill holes in pile
	//dilate(silhouette, silhouette, d_element);
	// Erode 
	erode(silhouette, silhouette, e_element);
	// Another dilation to get a solid silhouette
	dilation_size = 4;
	d_element = getStructuringElement(MORPH_ELLIPSE,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	dilate(silhouette, silhouette, d_element);
	//cvtColor(current_img, silhouette, CV_BGR2GRAY);
	//threshold(silhouette, bg, 170, 255, 0);
	//threshold(silhouette, silhouette, 30, 255, 0);	
	
	//silhouette = scene;
	//resize(silhouette, sample, sample.size(), 0.5, 0.5);
	//cv::imshow("silhouette efter dilation", sample);	

	silhouette.rowRange(2*IMG_HEIGHT/3, IMG_HEIGHT).setTo(Scalar(0));
	silhouette.colRange(0, 1).setTo(Scalar(0));
	silhouette.colRange(IMG_WIDTH-1, IMG_WIDTH).setTo(Scalar(0));
	
	//resize(silhouette, sample, sample.size(), 0.5, 0.5);
	cv::imshow("silhouette efter dilation", current_img);
	//char k = waitKey(5);

	c.Image = current_img;
	c.K = K;
	c.R = sfm_scene.views[i].R;
	//transpose(c.R, c.R);
	c.t = sfm_scene.views[i].t;		
	hconcat(c.R, c.t, Rt);	
	c.P = K * Rt; 
	c.Silhouette = silhouette; // silhouette - bg;
	c.Silhouette = c.Silhouette.clone();
	//cout << "t: " << sfm_scene.views[i].t << endl;

	return c;
}

camera TestDino(int i, cv::Mat& current_img)
{
	camera c;
	cv::Mat P,Ktemp,R,t,hsv, sample, silhouette, bg;

	/* silhouette */
	cv::cvtColor(current_img, hsv, CV_BGR2HSV);
	//cv::imshow("curr hsv", bg);
	//cv::inRange(hsv, cv::Scalar(105, 100, 100), cv::Scalar(120, 255, 255), bg);	
	//cv::inRange(silhouette, cv::Scalar(15, 100, 100), cv::Scalar(25, 255, 255), silhouette);
	cv::inRange(hsv, cv::Scalar(0, 0, 30), cv::Scalar(25, 255, 255), silhouette);
	int erosion_size = 1, dilation_size = 4;
	// Erosion and dilation to remove background objects
	Mat e_element = getStructuringElement(MORPH_ELLIPSE,
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),
		Point(erosion_size, erosion_size));

	Mat d_element = getStructuringElement(MORPH_ELLIPSE,
		Size(2 * dilation_size + 1, 2 * dilation_size + 1),
		Point(dilation_size, dilation_size));
	/// Apply the dilation operation
	dilate(silhouette, silhouette, d_element);
	//erode(silhouette, silhouette, e_element);
	//dilate(silhouette, silhouette, d_element);
	
	cv::imshow("current_img", current_img);	
	//cv::imshow("bg", bg);		
	//cv::imshow("testdino sil", silhouette);
	char k = waitKey(6);
	
	//cvtColor(current_img, silhouette, CV_BGR2GRAY);
	//threshold(silhouette, bg, 170, 255, 0);
	//threshold(silhouette, silhouette, 30, 255, 0);
	
	if (i == 0)
	{		
		P = (Mat_<float>(3, 4) << 3.99235687564161,	39.4176809830138, -0.763289879714919, 3.95917550891323,
			-14.4302310113271, -0.941441580237717, -27.4509701085667,-14.4294334377681,
			0.0122492403549385,-0.000145746037561476,-0.000569307087309742,0.0122493586975179);
	}	
	else if (i == 1)
	{
		P = (Mat_<float>(3, 4) << 10.7732491138691,38.1264946071842,-0.763289879714919,3.95917550891323,
			-14.3746180623658,1.57741397558573,-27.4509701085667,-14.4294334377681,
			0.0120380326410739,-0.00226955971786568,-0.000569307087309742,0.0122493586975179);
	}
	else if (i == 2)
	{
		P = (Mat_<float>(3, 4) << 17.2347705752451, 35.6742928552171, -0.763289879714919, 3.95917550891323,
			-13.8817985491694, 4.05136376169108, -27.4509701085667, -14.4294334377681,
			0.0114604857495905, -0.00432693859026821, -0.000569307087309742, 0.0122493586975179);
	}
	else if (i == 3)
	{
		P = (Mat_<float>(3, 4) << 23.1648466976242, 32.1415986686521, -0.763289879714919, 3.95917550891323,
			-12.9679613636550, 6.39920755719158, -27.4509701085667, -14.4294334377681,
			0.0105355674014520, -0.00625035604665700, -0.000569307087309742, 0.0122493586975179);
	}
	else if (i == 4)
	{
		P = (Mat_<float>(3, 4) << 28.4115710175871, 27.6129520300480, -0.763289879714919, 3.95917550891323,
			-11.6543721542474, 8.56116166063744, -27.4509701085667, -14.4294334377681,
			0.00928513675591998, -0.00799070501297492, -0.000569307087309742, 0.0122493586975179);
	}
	c.P = P;
	decomposeProjectionMatrix(P, Ktemp, R, t);
	c.Image = current_img;
	c.K = Ktemp;
	c.R = R;
	//transpose(c.R, c.R);
	c.t = t;	
	
	c.Silhouette = silhouette; //- bg;
	c.Silhouette = c.Silhouette.clone();
	//cout << "t: " << sfm_scene.views[i].t << endl;

	return c;
}

map<int, float> FindModel(vector<camera> cameras)
{
	map<int,float> bblims;
	float xmin, ymin, zmin, xmax, ymax, zmax;
	
	xmin = cameras[0].t.at<float>(0);
	ymin = cameras[0].t.at<float>(1);
	zmin = cameras[0].t.at<float>(2);

	xmax = cameras[0].t.at<float>(0);
	ymax = cameras[0].t.at<float>(1);
	zmax = cameras[0].t.at<float>(2);
	
	for (int i = 0; i < cameras.size(); i++)
	{
		xmin = min(xmin, cameras[i].t.at<float>(0));
		ymin = min(ymin, cameras[i].t.at<float>(1));
		zmin = min(zmin, cameras[i].t.at<float>(2));

		xmax = max(xmax, cameras[i].t.at<float>(0));
		ymax = max(ymax, cameras[i].t.at<float>(1));
		zmax = max(zmax, cameras[i].t.at<float>(2));
	}
	bblims[0] = xmin;
	bblims[1] = xmax;
	bblims[2] = ymin;
	bblims[3] = ymax;
	bblims[4] = zmin;
	bblims[5] = zmax;
	cout << bblims[0] << endl;
	cout << bblims[1] << endl;
	cout << bblims[2] << endl;
	cout << bblims[3] << endl;
	cout << bblims[4] << endl;
	cout << bblims[5] << endl;
	return bblims;
}

void SpaceCarving::SpaceCarve()
{
	int progress;
	/* acquire camera images, silhouettes and camera matrix */
	std::vector<camera> cameras;

	for (int i = 0; i < N_cameras; i+=50) { // 	 i+=50

		progress = int((100.0*(i) / (N_cameras - 1)));
		std::cout << "Loading cameras on " << N_cameras << " views: " << progress << " %\r";
		/* camera image */
		std::stringstream simg;		
		simg << "../Data/images/" << sfm_scene.views[i].filename;
		cv::Mat img = cv::imread(simg.str());
		//TESTA DINO-SCEN			
		/*
		simg << "../Data/Kalibrering och testing/Dino/viff.00"<<i<<".ppm";				
		cv::Mat img = cv::imread(simg.str());		
		IMG_WIDTH = img.cols;
		IMG_HEIGHT = img.rows;
		*/
		if (!img.data)                              // Check for invalid input
		{
			cout << "Could not open or find the images for space carving at " << simg.str() << std::endl;
			break;
		}	
		camera c = CameraFromView(i, img);		
		//camera c = TestDino(i, img);
		cameras.push_back(c);		
	}
	
	map<int, float> bblims = FindModel(cameras);
	/* bounding box dimensions of object */
	float xmin = bblims[0], ymin = bblims[2], zmin = bblims[4]; //-6.21639, ymin = -10.2796, zmin = -14.0349; //xmin = -0.1, ymin = -0.0, zmin = -0.8; // 
	float xmax = bblims[1], ymax = bblims[3], zmax = bblims[5]; //xmax = 0.1, ymax = 0.08, zmax = -0.5; //

	float bbwidth = std::abs(xmax - xmin)*1.15;
	float bbheight = std::abs(ymax - ymin)*1.15;
	float bbdepth = std::abs(zmax - zmin)*1.05;

	startParams params;
	params.startX = 0.0f;// xmin - std::abs(xmax - xmin)*0.15;
	params.startY = 0.0f;//ymin - std::abs(ymax - ymin)*0.15;
	params.startZ = 0.0f;
	params.voxelWidth = bbwidth / VOXEL_DIM;
	params.voxelHeight = bbheight / VOXEL_DIM;
	params.voxelDepth = bbdepth / VOXEL_DIM;
	
	/* 3 dimensional voxel grid */
	float *fArray = new float[VOXEL_SIZE];
	std::fill_n(fArray, VOXEL_SIZE, 5000.0f); //1000.0f
	progress = 0;
	
	
	/* carving model for every given camera image */
	for (int i = 0; i < cameras.size(); i++) { 
		progress = int((100.0*(i) / (cameras.size() - 1)));
		std::cout << "Performing space carving on the loaded " << cameras.size() << " views: " << progress << " %\r";		
		carve(fArray, params, cameras.at(i));
		/*
		cv::Mat sample;
		resize(cameras.at(i).Silhouette, sample, sample.size(), 0.5, 0.5);
		cv::imshow("Original scene", sample);
		cout << "cp" << cameras.at(i).t << endl;
		char k = cv::waitKey(10);
		*/
	}
	
	/* show example of segmented image */
	cv::Mat sample;
	resize(cameras.at(0).Image, sample, sample.size(), 0.5, 0.5);
	cv::imshow("Original scene", sample);
	char k = cv::waitKey(10);
	resize(cameras.at(0).Silhouette, sample, sample.size(), 0.5, 0.5);
	cv::imshow("Scene Silhouette", sample);
	k = cv::waitKey(10);	
	
	renderModel(fArray, params);

	//return 0;
}

SpaceCarving::~SpaceCarving()
{
}
