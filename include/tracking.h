#ifndef __TRACKING_H__
#define __TRACKING_H__

#include<opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mutex>
#include <queue>
#include <vector>
#include <thread>

// #include "frame.h"

class Tracking {
public:
    Tracking(bool setinitial = true, cv::Size winSize = cv::Size(21, 21), int maxLevel = 3, 
		cv::TermCriteria termcrit = cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 200, 0.1), 
		double minEigThreshold = 1e-3) {
		
		this->setInitial = setinitial;
		this->winSize = winSize;
		this->maxLevel = maxLevel;
		this->termcrit = termcrit;
		this->minEigThreshold = minEigThreshold;
		// optflow_thread = std::thread(&Tracking::trackingthread, this);
    }

    void trackingthread();
	void automaticInitPoints(cv::Mat gray);
	void feedFrameAndInitailflow(cv::Mat& im, std::vector<cv::Point2f>& p0, std::vector<cv::Point2f>& p1);
	void starttracking();
	void killTread();
	void frameTracking(std::vector<cv::Point2f>&, std::vector<cv::Point2f>&);
	void frameTracking(std::vector<cv::Point2f>&, std::vector<cv::Point2f>&, std::vector<uchar>&);
	void setLastFrame(cv::Mat& first);
	void drawMatch(cv::Mat&, std::vector<cv::Point2f>&, cv::Mat&, std::vector<cv::Point2f>&);
	void selectFeaturestoTrack(float, float, float, float, Eigen::Isometry3d& P1, Eigen::Isometry3d& P2, double* param, Eigen::Matrix3d& intri);

private:
	bool setInitial;
	int maxLevel, state = 1, times = 1;;
	double minEigThreshold;
	bool getFeed = false;
	std::thread optflow_thread; 


    cv::VideoCapture cap;
	cv::TermCriteria termcrit;
	// cv::Size subPixWinSize(10, 10); 
	cv::Size winSize;
 
	const int MAX_COUNT = 500;
	bool needToInit = false;
	bool nightMode = false;

	cv::Mat gray, prevGray, image, frame;
	std::vector<cv::Point2f> points[2];
	std::stringstream filename;

	// frames and plane points
	std::queue<std::vector<cv::Point2f>> plane_pts;
	std::queue<cv::Mat> frames; 
};

#endif