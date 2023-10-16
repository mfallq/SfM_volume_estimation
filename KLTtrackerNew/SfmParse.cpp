#include "SfmParse.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <tuple>
#include "jsoncons\json.hpp"
// for convenience
using jsoncons::json;
using jsoncons::json_deserializer;
using jsoncons::wjson;
using namespace cv;
using namespace std;

//-------------------------SFM---------------------
Sfm::Sfm()
{}
Sfm::Sfm(const std::string& filename_)
{
	filename = filename_;
	loadSfmJson();
}

// Load OpenMVG sfm excintrics
void Sfm::loadSfmJson()
{	
	//http://danielaparker.github.io/jsoncons/
	std::ifstream file(filename);

	if (file.good())
	{						
		std::cout << "Reading data from sfm scene. " << endl;
		unpack_structure();
		unpack_poses();
		unpack_views();		
		unpack_intrinsics();	
	}
	else
		cout << "Couldn't load SfM-file at: " << filename << endl;
}

Sfm::~Sfm()
{
}

//-------------------------VIEW---------------------
View::View(int& v_id_, std::string& filename_, Mat& R_, Mat& c_)
{
	v_id = v_id_;
	filename = filename_;
	R = R_;	
	t = c_;	
}
void Sfm::unpack_poses()
{	
	//std::map<int, std::pair<cv::Mat, cv::Mat>> poses;
	pair<Mat, Mat> pose;
	Mat R_temp, c_temp;
	json root;
	std::ifstream file(filename);
	file >> root;
	json poses_data = root["extrinsics"];
	json jroot_path = root["root_path"];
	root_path = jroot_path.as<string>();
	int pose_id;
	// Iterate over the elements of extrinsics
	for (auto idata : poses_data.elements())
	{
		json jp_id = idata["key"];
		pose_id = jp_id.as<int>();
		
		json pdata = idata["value"];
		json jp_rotation = pdata["rotation"];
		json jp_center = pdata["center"];
	
		//Structure (3d-point)		
		c_temp = (cv::Mat_<float>(3, 1) << jp_center[0].as<float>(), jp_center[1].as<float>(), jp_center[2].as<float>());
		R_temp = (cv::Mat_<float>(3, 3) << jp_rotation[0][0].as<float>(), jp_rotation[0][1].as<float>(), jp_rotation[0][2].as<float>(), jp_rotation[1][0].as<float>(), jp_rotation[1][1].as<float>(), jp_rotation[1][2].as<float>(), jp_rotation[2][0].as<float>(), jp_rotation[2][1].as<float>(), jp_rotation[2][2].as<float>());
		
		pose = std::make_pair(R_temp, c_temp);
		poses[pose_id] = pose;
	}
	cout << "Poses unpacked." << " \r";
	
}

void Sfm::unpack_views()
{	
	json root;
	std::ifstream file(filename);
	file >> root;
	string img_name;
	json views_data = root["views"];
	int v_id, i_id, p_id;
	
	// Iterate over the elements of views
	for (auto it : views_data.elements())
	{		
		json vdata = it["value"]["ptr_wrapper"]["data"];
		json jv_id = vdata["id_view"];
		v_id = jv_id.as<int>();
	
		json ji_id = vdata["id_intrinsic"];
		i_id = ji_id.as<int>(); //not used atm
		
		//intr = self.intrinsics[i_id] ?
		json jfilename = vdata["filename"];
		img_name = jfilename.as<string>();
		
		//try:
		json jp_id = vdata["id_pose"];
		p_id = jp_id.as<int>();
		
		// R kanske behöver transponeras!
		pair<Mat,Mat> pose = poses[p_id];

		views.push_back(View(v_id, img_name, pose.first, pose.second));
		//Create view object per pose
	}
	cout << "Views unpacked." << " \r";
}

View::~View()
{

}
//-------------------------INTRINSIC---------------------
Intrinsic::Intrinsic(int intr_id_, float focal_, cv::Point2f princp_, float width_, float height_)
{
	intr_id = intr_id_;
	focal = focal_;
	princp = princp_;
	width = width_;
	height = height_;
	K = (cv::Mat_<float>(3, 3) << focal, 0, princp.x, 0, focal, princp.y, 0, 0, 1);
	
}
void Sfm::unpack_intrinsics()
{	
	json root;	
	std::ifstream file(filename);	
	file >> root;
	
	json jsonintrinsics = root["intrinsics"];
	int intr_id;
	float focal, width, height;
	cv::Point2f princp;

	// Iterate over the elements of extrinsics	
	for (auto idata : jsonintrinsics.elements())
	{
		json json_intr_id = idata["key"];		
		intr_id = json_intr_id.as<int>();		
		json d = idata["value"]["ptr_wrapper"]["data"];
		json json_focal = d["focal_length"];	
		focal = json_focal.as<float>();
		json json_princp = d["principal_point"];
		princp.x = json_princp[0].as<float>();
		princp.y = json_princp[1].as<float>();
		json json_width = d["width"];
		width = json_width.as<float>();
		json json_height = d["height"];
		height = json_height.as<float>();
		intrinsics.push_back(Intrinsic(intr_id, focal, princp, width, height));
	}	
	cout << "Intrinsics unpacked." << " \r";
}

Intrinsic::~Intrinsic()
{
}

//-------------------------STRUCTURE---------------------
Structure::Structure(cv::Point3f voxel_point_, std::map<int, Point2f> observations_)
{
	
	
}
void Sfm::unpack_structure()
{
	json root;
	std::ifstream file(filename);
	file >> root;
	json d = root["structure"];

	cv::Point2f image_point;
	cv::Point3f voxel_point;
	
	std::map<int, Point2f> observations;
	int view_id;
	std::pair<int, cv::Point2f> imageobs;
	for (auto idata : d.elements())
	{
		//Structure (3d-point)
		json sdata = idata["value"];
		json jpoint = sdata["X"];
		voxel_point.x = jpoint[0].as<float>();
		voxel_point.y = jpoint[1].as<float>();
		voxel_point.z = jpoint[2].as<float>();
		json od = sdata["observations"];
		/*
		for (auto obs : od.elements())
		{		
		//Observation in image (2d)
		json jview_id = obs["key"];
		view_id = jview_id.as<int>();
		json json_point = obs["value"]["x"];
		image_point.x = json_point[0].as<float>();
		image_point.y = json_point[1].as<float>();
		//imageobs = std::make_pair(view_id, image_point);

		observations[view_id] = image_point;
		}
		*/
		structures.push_back(Structure(voxel_point, observations));
	}
	cout << "Structure unpacked." << " \r";
}
Structure::~Structure()
{

}