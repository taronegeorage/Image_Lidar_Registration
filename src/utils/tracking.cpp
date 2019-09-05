
#include "tracking.h"


void Tracking::trackingthread() {
	std::cout << "entered!\n";
	int ccc = 0;
	for(;;)
	{	
		// if(frames.size() > 1 && getFeed) {
		
		if(getFeed) {
			ccc++;
			std::cout << ccc << " track" << std::endl;	
			cv::namedWindow("LK Demo", 1);
            // frame = frames.front();
            // frames.pop();
            frame.copyTo(image);
            cvtColor(image, gray, cv::COLOR_BGR2GRAY);

            // std::vector<cv::Point2f> p0 = plane_pts.front();
            // plane_pts.pop();
            // std::vector<cv::Point2f> p1 = plane_pts.front();

            // if (nightMode)
		    // 	image = Scalar::all(0);
 
		    // if (needToInit)
		    // {
		    // 	// automatic initialization
		    // 	goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0);
		    // 	// goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
		    // 	cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1), termcrit);
		    // 	addRemovePt = false;
		    // }
		    // else if (!points[0].empty()) {
            std::vector<uchar> status;
            std::vector<float> err;
            if (prevGray.empty())
				gray.copyTo(prevGray);
            cv::calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize, maxLevel, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW, minEigThreshold);
            size_t i, k;
            for (i = k = 0; i < points[1].size(); i++)
			{
				if (!status[i])
					continue;
 
				points[1][k++] = points[1][i];
				circle(image, points[1][i], 3, cv::Scalar(0, 255, 0), -1, 8);
			}
			points[1].resize(k);
            // }

            // needToInit = false;
		    imshow("LK Demo", image);
            cv::waitKey(0);
		    // char c = (char)cv::waitKey(500);
		    // if (c == 27)
			//     break;
		    // switch (c)
		    // {
		    //     case 'r':
			//         needToInit = true;
			//         break;
		    //     case 'c':
			//         points[0].clear();
			//         points[1].clear();
			//         break;
		    //     case 'n':
			//         nightMode = !nightMode;
			//         break;
		    // }
 		    cv::swap(prevGray, gray);
			getFeed = false;
        }		
	}
}


void Tracking::feedFrameAndInitailflow(cv::Mat& im, std::vector<cv::Point2f>& p0, std::vector<cv::Point2f>& p1) {
    // std::lock_guard<std::mutex> lock(frame_mtx);
	frame = im;
	frame.copyTo(image);
    cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	// if(times == 1) {
	// 	points[0].clear(); points[0] = p0;
	// 	times++;
	// }
    points[0].clear(); points[0] = p0;
	points[1].clear(); points[1] = p1;
	getFeed = true;
}

void Tracking::starttracking() {
	optflow_thread = std::thread(&Tracking::trackingthread, this);
}

void Tracking::killTread() {
	state = 0;
	optflow_thread.join();
}

void Tracking::frameTracking(std::vector<cv::Point2f>& p1, std::vector<cv::Point2f>& p2) {
    std::vector<uchar> status;
    std::vector<float> err;
    // if (prevGray.empty())
	// 	gray.copyTo(prevGray);
	std::vector<cv::Point2f> tmp;
	tmp.insert(tmp.begin(), points[1].begin(), points[1].end());
	// points[1] = points[0];
	// cornerSubPix(gray, points[0], cv::Size(10, 10), cv::Size(-1, -1), termcrit);
	cv::calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize, maxLevel, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW, minEigThreshold);
	// std::cout << tmp.size() << " " << points[1].size() << std::endl;
	// for(int m = 0; m < tmp.size(); m++) {
	// 	std::cout << tmp[m].x << " " << tmp[m].y << " " << points[1][m].x << " " << points[1][m].y << std::endl;
	// }
	size_t i, k;
    for (i = k = 0; i < points[1].size(); i++) {
		if (!status[i])
			continue;
		points[1][k++] = points[1][i];
		circle(image, points[0][i], 3, cv::Scalar(255, 0, 0), -1, 8);
		circle(image, points[1][i], 3, cv::Scalar(0, 255, 0), -1, 8);
		circle(image, tmp[i], 3, cv::Scalar(0, 0, 255), -1, 8);
		cv::line(image, points[0][i], points[1][i], cv::Scalar(0, 0, 255));
		cv::line(image, points[0][i], tmp[i], cv::Scalar(0, 0, 255));
	}
	points[0].resize(k);
	points[1].resize(k);
	p1 = points[0];
	p2 = points[1];
	std::cout << "after tracking: " << k << " points" << std::endl;
	imshow("tracking result", image);
    cv::waitKey(0);
	// points[0] = points[1];
	cv::swap(prevGray, gray);
}                    

void Tracking::setLastFrame(cv::Mat& first) {
	// frame = first;                  
	cvtColor(first, prevGray, cv::COLOR_BGR2GRAY);
}
                                              
void Tracking::drawMatch(cv::Mat& im1, std::vector<cv::Point2f>& pt1, cv::Mat& im2, std::vector<cv::Point2f>& pt2) {
	std::vector<cv::DMatch> matches;
	for(int i = 0; i < pt1.size(); i++) {
		// if(i%10 == 0) {
			cv::DMatch match(i, i, 0);
			matches.push_back(match);
		// }
	}
	cv::Mat match_img;
	std::vector<cv::KeyPoint> kp1, kp2;
	cv::KeyPoint::convert(pt1, kp1, 1, 1, 0, -1); cv::KeyPoint::convert(pt2, kp2, 1, 1, 0, -1);
	cv::drawMatches(im1, kp1, im2, kp2, matches, match_img);
	cv::imshow("matches", match_img);
	cv::waitKey(0);
	
}

void Tracking::frameTracking(std::vector<cv::Point2f>& p1, std::vector<cv::Point2f>& p2, std::vector<uchar>& status) {
	frame.copyTo(image);
    cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    std::vector<float> err;
	std::vector<cv::Point2f> tmp;
	tmp.insert(tmp.begin(), points[1].begin(), points[1].end());
	// points[1] = points[0];
	// cornerSubPix(gray, points[0], cv::Size(10, 10), cv::Size(-1, -1), termcrit);
    cv::calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize, maxLevel, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW, minEigThreshold);

	size_t i, k;
    for (i = k = 0; i < points[1].size(); i++) {
		if (!status[i])
			continue;
		points[1][k++] = points[1][i];
	}
	points[0].resize(k);
	points[1].resize(k);
	p1 = points[0];
	p2 = points[1];
	// points[0] = points[1];
	cv::swap(prevGray, gray);
}

void Tracking::selectFeaturestoTrack(float x_min, float x_max, float y_min, float y_max, Eigen::Isometry3d& P1, Eigen::Isometry3d& P2, double* param, Eigen::Matrix3d& intri) {
	cv::goodFeaturesToTrack(prevGray, points[0], 500, 0.01, 5);
	cv::cornerSubPix(prevGray, points[0], cv::Size(10, 10), cv::Size(-1, -1), termcrit);
	points[1].resize(points[0].size());
	size_t num = 0;
	cv::Mat tmp = prevGray.clone();
	cv::Mat tmp2 = gray.clone();

	Eigen::Vector3d norm(param[0], param[1], param[2]);
	Eigen::Matrix3d func = P1.rotation().inverse();
	norm = func * norm;
	double dis = param[3] + param[0]*P1(0, 3) + param[1]*P1(1,3) + param[2]*P1(2,3);

	// std::cout << "original " << param[0] << " " << param[1] << " " << param[2] << " " << param[3] << std::endl;
	// std::cout << "new " << norm(0) << " " << norm(1) << " " << norm(2) << " " << dis << std::endl;
	for(int i = 0; i < points[0].size(); i++) {
		if(points[0][i].x < x_min+10 || points[0][i].x > x_max-10 || points[0][i].y < y_min+10 || points[0][i].y > y_max-10) 
			continue;
		points[0][num] = points[0][i];
		cv::circle(tmp, points[0][num], 3, cv::Scalar(0, 255, 0));

		Eigen::Vector3d uni_3dpts(points[0][num].x, points[0][num].y, 1);
		uni_3dpts = intri.inverse() * uni_3dpts;
		// double alpha = -1*dis/(points[0][num].x*norm(0)+points[0][num].y*norm(1)+norm(2));
		double alpha = -1*dis/(uni_3dpts(0)*norm(0)+uni_3dpts(1)*norm(1)+norm(2));
		Eigen::Vector4d pts(alpha*uni_3dpts(0), alpha*uni_3dpts(1), alpha, 1);
		pts = P1.inverse()*P2*pts;
		points[1][num].x = pts(0)*intri(0,0)/pts(2)+intri(0,2);
		points[1][num].y = pts(1)*intri(1,1)/pts(2)+intri(1,2);
		cv::circle(tmp2, points[1][num], 3, cv::Scalar(0, 255, 0));
		num++;
	}
	std::cout << "num" << num << std::endl;
	points[0].resize(num);
	points[1].resize(num);
	cv::cornerSubPix(gray, points[1], cv::Size(10, 10), cv::Size(-1, -1), termcrit);
	std::cout << points[0].size() << " " << points[1].size() << std::endl;
	
	cv::imshow(std::to_string(num)+"findcorner1", tmp);
	cv::imshow(std::to_string(num)+"findcorner2", tmp2);
	cv::waitKey();
}

