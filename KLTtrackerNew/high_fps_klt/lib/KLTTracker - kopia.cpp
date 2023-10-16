#include "KLTTracker.h"
#include <iostream>

namespace vision{

trackers::KLTTracker::KLTTracker()
{
	medianThresh = 8; //default: 5	
	min_pt_d = 5; //default: 5 
	epipolarThresh = 0.7; //default: 0.7, max 3 
    max_iter = 3;
    pyr_lvl = 3;
    win_size = cv::Size(3, 3);
    target_npts = 1000;
    frame_num = 0;
	keyframe_num = 0;
}

trackers::KLTTracker::~KLTTracker()
{
}

MotionField trackers::KLTTracker::getMotionField()
{
	return mf;
}

void trackers::KLTTracker::process(SensorData& SF, double imageScaling, double NoFrames)
{	
	videoPath = SF.getVideoPath();
	cv::Mat greyMat, disp;
	cv::VideoCapture cap(videoPath);   //open the capture for video file
	if (!cap.isOpened()){  // check if we succeeded
		cout << "Cannot open video file, is the path correct?" << endl;
	}
	else
	{
		totalFrames  = cap.get(CV_CAP_PROP_FRAME_COUNT);
		fps = cap.get(CV_CAP_PROP_FPS);
		double timestamp = cap.get(CV_CAP_PROP_POS_MSEC);

		if (NoFrames == 0 || NoFrames > totalFrames)
		{
			NoFrames = totalFrames;
		}
		if (totalFrames < 2)
		{
			cout << "Not enough video frames loaded!" << endl;
		}	
		else
		{
			cap >> curr_im; //save captured image to frame variable
			// Resize if need be		
			cv::resize(curr_im, curr_im, cv::Size(), imageScaling, imageScaling, CV_INTER_AREA);

			//Transpose and flip 
			//cv::transpose(curr_im, curr_im);
			//cv::flip(curr_im, curr_im, 1);
			
			cv::cvtColor(curr_im, greyMat, CV_BGR2GRAY);

			current_frame = iqmatic::Frame(greyMat.clone(), curr_im.clone());
			current_frame.frame_num = frame_num;		
			frame_num++;
			//First frame is also a keyframe; generate more points to track:				
			keyPoints();
			keyframe_num = 0;
			// Generate ids etc for the first keypts and save them to the corresponding landmarks
			for (auto obspair : current_frame.getObservation_map()) {
				iqmatic::sObservation obs = obspair.second;					
				IDs[keyframe_num].push_back(obs->landmark_ID);							
			}

			ImagePyramid tmp_pyr;
			cv::buildOpticalFlowPyramid(greyMat, tmp_pyr, win_size, pyr_lvl, true);
			curr_pyr = tmp_pyr;

			int progress = 1;
			for (uint i = 1; i < NoFrames; i++)
			{/*
				cv::Mat disp;
				drawTracks(disp, 10, .8);
				cv::imshow("tracks", disp);
				char k = cv::waitKey(1);
				//cout << "Keyframe num: " << keyframe_num << "Frame_num: " << frame_num << endl;
				*/
				progress = int((100.0*(i) / (NoFrames - 1)));
				std::cout << "Tracking and selecting keyframes in videofile of "<< NoFrames << " frames: " << progress << " %\r";

				prev_im = curr_im;
				// Each frame needs to be in grayscale for the motionfield to be computed
				//curr_im = SF.VideoFrames[i];
				cap >> curr_im; //save captured image to frame variable

				// Resize if need be		
				cv::resize(curr_im, curr_im, cv::Size(), imageScaling, imageScaling, CV_INTER_AREA);

				//Transpose and flip 
				//cv::transpose(curr_im, curr_im);
				//cv::flip(curr_im, curr_im, 1);

				cv::cvtColor(curr_im, greyMat, CV_BGR2GRAY);

				current_frame = iqmatic::Frame(greyMat.clone(), curr_im.clone());
				current_frame.frame_num = frame_num;
				frame_num++;

				bool kf = newFrame();

				// Track the new frame using the found optical flow
				if (kf)
				{// If true we have found a keyframe, extract keypoints for this frame and update key_point coords etc.
					// Generate keyframe-observations				
					keyframe_num++;
					generateObservations();
					keyPoints();

					// Add IDs for new observations generated in this keyframe
					for (auto obspair : current_frame.getObservation_map()) {
						iqmatic::sObservation obs = obspair.second;
						//cv::Point2f pos(obs->position);								
						IDs[keyframe_num].push_back(obs->landmark_ID);						
					}

					// Find F and enforce epipolar constraint on points				
					geometricConstraint();	
					
				}
			}
			
		}	
	}
}
// Local function 
vector<std::pair<int,int>> getMatches(vector<int> prevID, vector<int> currID){
	vector<std::pair<int, int>> myMatches;
	std::pair<int, int> match;
	//int matches = 0;
	for (int i = 0; i < prevID.size(); i++)
	{// if element i in currID is found in prevID, save the position		
		if (std::count(currID.begin(), currID.end(), prevID[i]))
		{		
		match.first = i;
		match.second = find(currID.begin(), currID.end(), prevID[i]) - currID.begin();		
		myMatches.push_back(match);
		}
	}	
	return myMatches;
}
void trackers::KLTTracker::saveKeyFrames()
{	// Save data at the folder of the video
	string tempName, saveName, entry;
	
	double time = 0;
	int progress;
	uint NoFrames = key_frames.size();
	cout << "\nSaving data and converting to openMVG format" << endl;
	const char * path(videoPath.c_str());
	const char * textPath;
	//char path_buffer[_MAX_PATH];
	char drive[_MAX_DRIVE];
	char dir[_MAX_DIR];
	char fname[_MAX_FNAME];
	char ext[_MAX_EXT];	
	_splitpath_s(path, drive, dir, fname, ext); // C4996	
	/*
	printf("Path used: %s\n", path);
	printf("Path extracted with _splitpath_s:\n");
	printf("  Drive: %s\n", drive);
	printf("  Dir: %s\n", dir);
	printf("  Filename: %s\n", fname);
	printf("  Ext: %s\n", ext);
	*/
	saveName = drive + string(dir) + "images/"; //+ std::to_string(i); //+fname + ".png"
	FILE * pFile;
	FILE * textFile;
	int i = 0, img = 100;
	for (auto frame : key_frames)
	{
		time = (1000 * frame.frame_num / fps);
		progress = int((100.0*(i) / (NoFrames - 1)));
		std::cout << "Writing .feat files and .png images: " << progress << " %\r";
		// Save images	
		tempName = saveName;
		tempName.append(std::to_string(img));
		tempName.append(".png");		
		cv::imwrite(tempName, frame.getRgb());

		// Write *.feat file and save 2d points, in folder outDir/matches/
		// X Y scale orientation for each image		
		tempName = saveName;
		tempName.append("matches/");
		tempName.append(std::to_string(img));
		tempName.append(".feat");
		
		// Convert to const char *
		textPath = (tempName.c_str());
		
		pFile = fopen(textPath, "w");  //opens and writes to file
		if (pFile != NULL)
		{
			//fputs("x y 0 0", pFile);
			auto obs_map = frame.getObservation_map();
			for (int jx = 0; jx < IDs[i].size(); ++jx){
				iqmatic::sObservation obs = obs_map[IDs[i][jx]];
				cv::Point2f pos(obs->position);
				entry = std::to_string(pos.x) + " " + std::to_string(pos.y);				
				entry.append(" 0 0\n");
				textPath = entry.c_str();
				fputs(textPath, pFile);
			}
			fclose(pFile);
		}// close file *.feat
		i++;
		img++;
	}
	// Write matches.e.txt and matches.f.txt files to list ID correspondences, in folder outDir/matches/
	// 0 1 <--- depicts that there is some matches between the view 0 & 1
	// count <-- - the number of corresponding points between the views
	// idFeat idFeat <----map the id of the corresponding feature(row id inside the file *.feat)
	// *.e.txt & *.f.txt
	saveName = drive + string(dir) + "images/matches/";
	tempName = saveName;	
	tempName.append("matches.e.txt");
	textPath = tempName.c_str();
	
	pFile = fopen(textPath, "w");  //opens and writes to file matches.e.txt

	tempName = saveName;
	tempName.append("matches.f.txt");
	textPath = (tempName.c_str());
	
	textFile = fopen(textPath, "w");  //opens and writes to file matches.f.txt

	vector<std::pair<int, int>> matches;

	for (int i = 1; i < key_frames.size(); i++)
	{		
		if (pFile != NULL && textFile != NULL)
		{
			// Write view IDs of pair of views
			entry = std::to_string(i - 1) + " " + std::to_string(i) + "\n";
			textPath = entry.c_str();
			fputs(textPath, pFile);
			fputs(textPath, textFile);
			// Write number of points matching in the 2 views
			matches = getMatches(IDs[i - 1], IDs[i]);
			entry = std::to_string(matches.size()) + "\n";
			textPath = entry.c_str();
			fputs(textPath, pFile);
			fputs(textPath, textFile);
			// Write row number of corresponing points, in the 2 corresponding *.feat files			
			for (int j = 0; j < matches.size(); ++j){
				entry = std::to_string(matches[j].first) + " " + std::to_string(matches[j].second)+ "\n";
				textPath = entry.c_str();
				fputs(textPath, pFile);
				fputs(textPath, textFile);
			}			
		}
	}	
	fclose(pFile);
	fclose(textFile);	
}
void trackers::KLTTracker::geometricConstraint()
{
	cv::Mat F;
	cv::Point2f pos;
	vector<cv::Point2f> prev_points, points;
	vector<uint> to_remove;
	vector<uchar> mask;
	iqmatic::sObservation prev_obs, curr_obs;
	
	prev_points.clear();
	points.clear();
	//Grab two frames
	auto prevFrame = key_frames[keyframe_num - 1];
	auto currFrame = key_frames[keyframe_num];

	auto prev_obs_map = prevFrame.getObservation_map();
	auto curr_obs_map = currFrame.getObservation_map();

	// Build to arrays of corresponding points
	for (int jx = 0; jx < IDs[keyframe_num - 1].size(); ++jx){
		prev_obs = prev_obs_map[IDs[keyframe_num - 1][jx]];
		pos = prev_obs->position;
		prev_points.push_back(pos);

		curr_obs = curr_obs_map[IDs[keyframe_num][jx]];
		pos = curr_obs->position;
		points.push_back(pos);
	}

	int method = cv::FM_RANSAC; //  CV_FM_LMEDS; //
	double quality = 0.993; //0.99
	mask.clear();
	//Find F-matrix and save mask to remove points 
	F = findFundamentalMat(prev_points, points, method, epipolarThresh, quality, mask);

	//Now remove points not fullfilling geometric constraint (epipolar constraint)
	to_remove.clear();
	for (uint i = 0; i < mask.size(); ++i) {
		if (mask[i] == 0) {
			to_remove.push_back(i);
			continue;
		}
	}	
	for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it) {
		// Remove ids from keyframe
		IDs[keyframe_num - 1].erase(IDs[keyframe_num - 1].begin() + (*it));
		IDs[keyframe_num].erase(IDs[keyframe_num].begin() + (*it));
		key_points.erase(key_points.begin() + (*it));
		curr_pts.erase(curr_pts.begin() + (*it));
		mf.erase(mf.begin() + (*it));		
	}	
	
}


void trackers::KLTTracker::generateObservations()
{
	std::map<LandmarkID, iqmatic::sObservation> obs_map = key_frames[keyframe_num - 1].getObservation_map(); //prev_f->getObservation_map()

	for (int jx = 0; jx < IDs[keyframe_num - 1].size(); ++jx){
		int access_id = IDs[keyframe_num - 1].at(jx);	
		iqmatic::sObservation obs = obs_map[access_id];
		iqmatic::sLandmark lm = obs->landmark;
		cv::Point2f pos; 		
		pos.x = curr_pts[jx].x;
		pos.y = curr_pts[jx].y;
		current_frame.newObservation(lm, pos);
	}
}

void trackers::KLTTracker::displayKeyFrames()
{
	uint size = key_frames.size();
	cout << "Number of keyframes: " << size << endl;
	
	for (uint i = 0; i < size; i++)
	{
		const string name = "Key frame"; //+ to_string(ind);
		//namedWindow(name, WINDOW_AUTOSIZE);
		//klt.drawTracks(i, 10, .7);
		cv::imshow(name, key_frames.at(i).getRgb());
		char k = cv::waitKey(0);
		cout << "Keyframe number: " << i << endl;

	}
}
bool trackers::KLTTracker::newFrame()
{//Return true for keyframes; depending on median distance moved for each all tracks

	// If we already have data, save it in the previous state	
	prev_pyr = curr_pyr;
	// Set the current tracked points to the previous for the next loop
	prev_pts = curr_pts;	
	
	// Calculate the optical flow and save it in curr_pyr
	ImagePyramid tmp_pyr;
	cv::buildOpticalFlowPyramid(current_frame.getGray(), tmp_pyr, win_size, pyr_lvl, true);
	curr_pyr = tmp_pyr;

	// Compute motionfield frame-to-frame and track points	
	extendMotionField();
	
	// Keyframe?
	double medianDisp; //In pixels 
	medianDisp = medianDisplacement();
	//cout << "Median displaced distance: " << median << endl;
	if (medianDisp > medianThresh)
	{	// If this is achieved we have found a keyframe	
		return true;
	}
	else
	{// Keep tracking points from last frame with keypoints
		return false;
	}

}

double trackers::KLTTracker::medianDisplacement()
{
	Track currentTrack;
	cv::Point2f keypt, pt;
	vector<double> norm_vec;
	double median;
	size_t NoPoints = key_points.size();
	
	for (int i = 0; i < NoPoints; i++)
	{
		pt = curr_pts[i];
		keypt = key_points[i];
		norm_vec.push_back(cv::norm(pt - keypt));			
	}	

	sort(norm_vec.begin(), norm_vec.end());

	if (NoPoints % 2 == 0)
	{
		median = (norm_vec[NoPoints / 2 - 1] + norm_vec[NoPoints / 2]) / 2;
	}
	else
	{
		median = norm_vec[NoPoints / 2];
	}	
	return median;

}

void trackers::KLTTracker::keyPoints()
{ // Each keyFrame has key_points which should be tracked all the way to the next keyFrame, with known IDs of the points.
	
	// Start by saving those points we have tracked from last keyFrame	
	prev_pts = curr_pts;
	key_points = curr_pts;

	// Initiate and reset points
	PointList to_add, new_pts;
	std::vector<Track> new_tracks;		
	cv::goodFeaturesToTrack(current_frame.getGray(), new_pts, target_npts, 0.002, min_pt_d);
		
	// Add new points to the keyframe 
	for (cv::Point2f n_pt : new_pts) {
		bool add = true;
		// But only if its not already a known point, close to its last position
		for (cv::Point2f p_pt : prev_pts) { 
			if (cv::norm(n_pt - p_pt) < min_pt_d) {
				// Do not track points close to each other
				add = false;				
				break;
			}
		
		}
		if (add) {			
			// Add observations (2d points) and connect them to landmarks (3d points) to current frame			
			iqmatic::sLandmark lm = iqmatic::Landmark::create(); // Also creates a unique ID 
			current_frame.newObservation(lm, n_pt); 			
			
			// Also need to save each n_pt
			to_add.push_back(n_pt);
			Track nt;
			nt.push_back(n_pt);
			new_tracks.push_back(nt);
		}
	}	
	// Save which points to track from this key_frame	
	// Save the added points from this key_frame in key_frame-variables 
	key_points.insert(key_points.end(), to_add.begin(), to_add.end()); 	
	key_frames.push_back(current_frame);		
	// And also save the points into current_points 
	curr_pts.insert(curr_pts.end(), to_add.begin(), to_add.end());
	mf.insert(mf.end(), new_tracks.begin(), new_tracks.end());
	//cout << "KLT.keyframe number / no keypoints: " << frame_num-1 << " / " << key_points.size() << endl;
}


uint trackers::KLTTracker::extendMotionField()
{	// Track and retrack data
	PointList tmp_pts, to_backtrack, back_guess;
	std::vector<uchar> statusf, statusb;
	std::vector<float> errs, errsb;	
	curr_pts.clear();
	// Track points
	cv::calcOpticalFlowPyrLK(prev_pyr,
		curr_pyr,
		prev_pts,
		tmp_pts,
		statusf,
		errs,
		win_size,
		pyr_lvl,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, max_iter, 0.05));

	vector<uint> tobt_ind; // To backtrack
	vector<uint> to_remove, to_remove_after; 
	// Mark which elements to remove and to backtrack in tmp_pts
	for (uint i = 0; i < statusf.size(); ++i) {
		if (statusf[i] == 0) {
			to_remove.push_back(i);
			continue;
		}
		tobt_ind.push_back(i);
	}
	// Get points to backtrack
	for (auto i : tobt_ind) {
		to_backtrack.push_back(tmp_pts[i]);
		back_guess.push_back(prev_pts[i]);
	}
	
	// Remove from motionfield as well as ID
	for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it) {
		mf.erase(mf.begin() + (*it));		
		// Remove ids from keyframe
		IDs[keyframe_num].erase(IDs[keyframe_num].begin() + (*it));
		// Key points
		key_points.erase(key_points.begin() + (*it));		
	}	

	// backtrack
	cv::calcOpticalFlowPyrLK(curr_pyr,
		prev_pyr,
		to_backtrack,
		back_guess,
		statusb,
		errsb,
		win_size,
		pyr_lvl,
		cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.5),
		cv::OPTFLOW_USE_INITIAL_FLOW);

	uint success = 0;
	to_remove_after = to_remove;
	to_remove.clear();

	for (uint i = 0; i < statusb.size(); ++i) {
		// search for the original point corresponding to this backtracked one
		if (!statusb[i]) {
			to_remove.push_back(i);
			continue;
		}
		if (cv::norm(prev_pts[tobt_ind[i]] - back_guess[i]) > 2 ||
			cv::norm(prev_pts[tobt_ind[i]] - to_backtrack[i]) > 50) {
			// mark to remove
			to_remove.push_back(i);
			continue;
		}

		// extend track	
		curr_pts.push_back(to_backtrack[i]);
		mf[i].push_back(to_backtrack[i]);
		success++;
	}
	
	// remove lost tracks and their ID, as well as the corresponding keypoint for that track
	for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it) {
		mf.erase(mf.begin() + (*it));
		// Key points
		key_points.erase(key_points.begin() + (*it));		
		// ID removal
		IDs[keyframe_num].erase(IDs[keyframe_num].begin() + (*it));
		//point_id_vec.erase(point_id_vec.begin() + (*it));
	}
	to_remove.insert(to_remove.end(), to_remove_after.begin(), to_remove_after.end());
	for (auto it = to_remove.rbegin(); it != to_remove.rend(); ++it) {
		prev_pts.erase(prev_pts.begin() + (*it));
	}
	return success;
}

void trackers::KLTTracker::drawTracks(cv::Mat& out, uint max_length, float scaling_factor)
{
    uint counter = 0;
    cv::Mat tmp;
	cv::resize(current_frame.getGray(), tmp, cv::Size(), scaling_factor, scaling_factor); //current_frame.getGray()
    cv::cvtColor(tmp, out, CV_GRAY2BGR);
	
    for(Track t : mf) {
		
        cv::Point2f prev;
        uint i = 1;
        uint track_size = t.size();
		//cout << track_size << " <t and counter > "<< counter << endl;
        if(track_size > 1)
            counter++;
        for(cv::Point2f pt : t) {
            if(track_size - i < max_length && i > 2) {
                cv::line(out, prev * scaling_factor, pt * scaling_factor, cv::Scalar(128, 0, 0), 2);
                if(track_size - i == 0)
                    cv::circle(out, pt * scaling_factor, 3, cv::Scalar(0, 0, 128), -1);
            }
            prev = pt;
            ++i;
        }
		
    }
    std::stringstream txt;
    txt << "frame :" << frame_num << "  motion field: " << counter;
    cv::putText(out, txt.str(), cv::Point2f(30, 30), cv::FONT_HERSHEY_SIMPLEX, .5, cv::Scalar(0, 0, 128));
}


/*
void trackers::KLTTracker::drawSideBySide(cv::Mat& out, float scaling_factor)
{
    cv::Mat tmp;
    out = cv::Mat(std::ceil(prev_im.rows * scaling_factor), std::ceil(2 * prev_im.cols * scaling_factor), CV_8UC3);
    cv::Mat im_left(out,
                    cv::Rect(0, 0, std::ceil(prev_im.cols * scaling_factor), std::ceil(prev_im.rows * scaling_factor)));
    cv::Mat im_right(out,
                     cv::Rect(std::ceil(prev_im.cols * scaling_factor),
                              0,
                              std::ceil(prev_im.cols * scaling_factor),
                              std::ceil(prev_im.rows * scaling_factor)));
    cv::resize(prev_im, tmp, cv::Size(), scaling_factor, scaling_factor);
    cv::cvtColor(tmp, im_left, CV_GRAY2BGR);
    cv::resize(prev_im, tmp, cv::Size(), scaling_factor, scaling_factor);
    cv::cvtColor(tmp, im_right, CV_GRAY2BGR);
}
void trackers::KLTTracker::drawDiff(cv::Mat& out, float scaling_factor)
{
    cv::Mat tmp;
    cv::resize(cv::abs(prev_im - curr_im), tmp, cv::Size(), scaling_factor, scaling_factor);
    cv::cvtColor(tmp, out, CV_GRAY2BGR);
}
*/

}//vision