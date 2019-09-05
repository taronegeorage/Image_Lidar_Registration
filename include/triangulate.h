#ifndef __TRIANGULATE_H__
#define __TRIANGULATE_H__

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace cv;

class Triangulate {
public:
    Triangulate() {

    }

    void doTriangulate(const vector< KeyPoint >& keypoint_1, const vector< KeyPoint >& keypoint_2, const std::vector< DMatch >& matches,
        const Mat& R, const Mat& t, vector< Point3d >& points);
    Triangulate(const cv::Point2f &kp1, const cv::Point2f &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D) {
        cv::Mat A(4,4,CV_32F);
        A.row(0) = kp1.x*P1.row(2)-P1.row(0);
        A.row(1) = kp1.y*P1.row(2)-P1.row(1);
        A.row(2) = kp2.x*P2.row(2)-P2.row(0);
        A.row(3) = kp2.y*P2.row(2)-P2.row(1);
        
        cv::Mat u,w,vt;
        cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
        x3D = vt.row(3).t();
        x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
    }
};

#endif