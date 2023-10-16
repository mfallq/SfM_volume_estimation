#pragma once

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
//VTK_MODULE_INIT(vtkInteractionStyle);
#include "SfmParse.h"
#include <vtkPolyData.h>


struct voxel {
	float xpos;
	float ypos;
	float zpos;
	float res;
	float value;
	bool occupied = true;
};

struct coord {
	int x;
	int y;
};
struct camera {
	cv::Mat Image;
	cv::Mat P;
	cv::Mat K;
	cv::Mat R;
	cv::Mat t;
	cv::Mat Silhouette;
};


struct startParams {
	float startX;
	float startY;
	float startZ;
	float voxelWidth;
	float voxelHeight;
	float voxelDepth;
};

class SpaceCarving
{
public:
	SpaceCarving(Sfm sfm_scene);
	void SpaceCarve();
	~SpaceCarving();
protected:
	camera SpaceCarving::CameraFromView(int i, cv::Mat& currrent_img);
	coord project(camera cam, voxel v);
	void exportModel(char *filename, vtkPolyData *polyData);
	void renderModel(float fArray[], startParams params);
	void SpaceCarving::carve(float fArray[], startParams params, camera cam);
	//Data
	cv::Mat K;
	int N_cameras;
	int IMG_WIDTH;
	int IMG_HEIGHT;
	//Specify voxel dim
	int VOXEL_DIM;
	int VOXEL_SIZE;
	int VOXEL_SLICE;
	int OUTSIDE;
	Sfm sfm_scene;
	
};

