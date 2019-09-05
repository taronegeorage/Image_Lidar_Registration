#ifndef __FRAME_H__
#define __FRAME_H__

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

using namespace Eigen;

struct  frame
{
    cv::Mat img;
    double timestamp;
    Eigen::Quaterniond orient;
    Eigen::Vector3d trans;
    Eigen::Isometry3d Pose = Eigen::Isometry3d::Identity();
    frame(cv::Mat im, double t, Eigen::Quaterniond q, Eigen::Vector3d p) {
        img = im;
        timestamp = t;
        orient = q;
        trans = p;
        Pose.rotate(q);
        Pose.pretranslate(trans);
    }

    frame(double t, Eigen::Quaterniond q, Eigen::Vector3d p) {
        timestamp = t;
        orient = q;
        trans = p;
        Pose.rotate(q);
        Pose.pretranslate(trans);
    }

    frame() {
        // do_slerp();
    }
    inline void do_slerp(double t_pre, Eigen::Quaterniond& q1, Eigen::Quaterniond& q2) {
        // Eigen::Quaterniond q_slerp;
        float cos_12 = q1.w() * q2.w() + q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z();
        if(cos_12 < 0.0f) {
            q1.w() = -1 * q1.w(); q1.x() = -1 * q1.x(); q1.y() = -1 * q1.y(); q1.z() = -1 * q1.z();
            cos_12 = q1.w() * q2.w() + q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z();
        }

        float k1, k2;
        if(cos_12 > 0.9995f) {
            k1 = 1.0f - t_pre;
            k2 = t_pre;
        } else {
            float sin_12 = sqrt(1.0f-cos_12*cos_12);
            float q1q2 = atan2(sin_12, cos_12);
            k1 = sin((1.0f-t_pre)*q1q2) / sin_12;
            k2 = sin(t_pre*q1q2) / sin_12;
        }

        orient.w()=k1*q1.w()+k2*q2.w();
	    orient.x()=k1*q1.x()+k2*q2.x();
	    orient.y()=k1*q1.y()+k2*q2.y();
	    orient.z()=k1*q1.z()+k2*q2.z();
    }
};


typedef struct frame Frame;

#endif