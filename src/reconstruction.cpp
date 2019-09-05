#include<iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <cmath>
#include <vector>
#include <sstream>
#include <unordered_map>
#include <thread>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "fisheyeimg.h"
#include "frame.h"
#include "tracking.h"
#include "points.h"


using namespace std;

cv::Mat MatrixtoCvMat(const Eigen::Isometry3d &matrix);
void Triangulate(const cv::Point2f &kp1, const cv::Point2f &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

int main(int argc, char **argv)
{
    const int camid = 3;
    Fisheye *f = new Fisheye();
    float fx, fy, cx, cy;
    f->getIntrinsics(fx, fy, cx, cy, camid);
    int w, h;
    f->getImagesize(w, h);
    Eigen::Matrix4d Tcl = f->getTcl(camid);
    Eigen::Matrix4d T_lidar_loam;
    T_lidar_loam << 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix4d Transform = Tcl * T_lidar_loam;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cld(new pcl::PointCloud<pcl::PointXYZ>);
	if(pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/cj/ros_ws2/src/RGB_TO_LIDAR/data/cornerMap_new.pcd", *cld) == -1) {
		PCL_ERROR("CANNOT READ FILE");
		return -1;
	}
	cout << "Loaded " << cld->width << "*" << cld->height << " data points." << endl;
    pcl::transformPointCloud(*cld, *cld, Transform);
    std::cout << Tcl.matrix() << std::endl;
    Eigen::Matrix3d R_cam; // camid = 3
    R_cam << 0, 0, -1, 0, 1, 0, 1, 0, 0;

    std::vector<Frame> poses;
    // read pose
    {
        //read laser pose file  test1_image_pose
        // ifstream read_pose("/home/cj/Downloads/outdoor_data/test6_image_pose.txt", ios::binary);
        ifstream read_pose("/home/cj/Downloads/outdoor_data/large_view2.txt");
        string line_pose; 
        std::stringstream filename;
        int idx = 0;
        double t;

        while(getline(read_pose, line_pose)) {
            stringstream ss(line_pose);
            string file;
            double time, p1, p2, p3, q_x, q_y, q_z, q_w;
            ss >> file >> p1 >> p2 >> p3 >> q_x >> q_y >> q_z >> q_w;
            Eigen::Quaterniond q_loam(q_w, q_x, q_y, q_z);
            Eigen::Vector3d p_laser(p1, p2, p3);
            Eigen::Matrix3d rot(Transform.topLeftCorner(3, 3).cast<double>());
            // Eigen::Matrix3d rot_loam_to_laser(T_lidar_loam.topLeftCorner(3, 3).cast<double>());
            Eigen::Matrix3d q_rot_loam = q_loam.toRotationMatrix();
            Eigen::Quaterniond q(R_cam*rot*q_rot_loam);
            Eigen::Vector3d p(rot*p_laser+Transform.topRightCorner(3, 1).cast<double>());
            filename.str("");
            filename << "/home/cj/Downloads/outdoor_data/image/" << camid << "_undistort/" << setw(6) << setfill('0')<< file <<".jpg";
            cout << "reading " << filename.str() << endl;
            cv::Mat img = cv::imread(filename.str());
            
            Frame f(img, t, q, p);
            poses.push_back(f);
        }
    }
    
    bool first = true;
    PixelPoint valid_pts[cld->points.size()];
    PixelPoint pre_valid_pts[cld->points.size()];
    PixelPoint *valid_pts_ptr = valid_pts;
    PixelPoint *pre_valid_pts_ptr = pre_valid_pts;

    for(int i = 0; i < 2; i++) {
        int point_cnt = 0;
        Eigen::Isometry3d func = poses[i].Pose.inverse();
        for(int j = 0; j < cld->points.size(); j++) {
            Eigen::Vector4d point_(cld->points[j].x, cld->points[j].y, cld->points[j].z, 1.0);
            double z = func.matrix().row(2) * point_;         
            double y = -func.matrix().row(1) * point_;
            double x = -func.matrix().row(0) * point_;
            (valid_pts_ptr+j)->visible = false;
            if(z <= 0)
                continue;
            int u = (int)(x*fx/z+cx+0.5);
            int v = (int)(y*fy/z+cy+0.5);
            if(u < 0 || v < 0 || u >= w || v >= h)
                continue;

            (valid_pts_ptr+j)->visible = true;
            (valid_pts_ptr+j)->lasesrpoint.x = u;
            (valid_pts_ptr+j)->lasesrpoint.y = v;

            point_cnt++;
        }

        if(first) {
            first = false;
            // exchange
            auto tmp = valid_pts_ptr;
            valid_pts_ptr = pre_valid_pts_ptr;
            pre_valid_pts_ptr = tmp;
            tmp = NULL;
            delete tmp;
            continue;
        }

        const cv::Mat img1 = poses[i-1].img;
        const cv::Mat img2 = poses[i].img;
        cv::Mat P1 = MatrixtoCvMat(poses[i-1].Pose);
        cv::Mat P2 = MatrixtoCvMat(poses[i].Pose);
        // reconstruction result
        pcl::PointCloud<pcl::PointXYZRGB> model;
        int count = 0;
        model.height = 1;
        model.width = cld->points.size();
        model.resize(model.height*model.width);
        model.is_dense = false;

        for(int id = 0; id < cld->points.size(); id++) {
            if((valid_pts_ptr+id)->visible && (pre_valid_pts_ptr+id)->visible) {
                cv::Point2f pt1 = (pre_valid_pts_ptr+id)->lasesrpoint;
                cv::Point2f pt2 = (valid_pts_ptr+id)->lasesrpoint;
                int sum1 = img1.data[(int)pt1.y*img1.step+(int)pt1.x*img1.channels()] + img1.data[(int)pt1.y*img1.step+(int)pt1.x*img1.channels()+1] + img1.data[(int)pt1.y*img1.step+(int)pt1.x*img1.channels()+2];
                int sum2 = img2.data[(int)pt1.y*img2.step+(int)pt1.x*img2.channels()] + img2.data[(int)pt1.y*img2.step+(int)pt1.x*img2.channels()+1] + img2.data[(int)pt1.y*img2.step+(int)pt1.x*img2.channels()+2];
                if(abs(sum1-sum2) > 15)
                    continue;
                cv::Mat p3d;
                Triangulate((pre_valid_pts_ptr+id)->lasesrpoint, (valid_pts_ptr+id)->lasesrpoint, 
                    P1, P2, p3d);
                model.points[count].x = p3d.data[0];
                model.points[count].y = p3d.data[1];
                model.points[count].z = p3d.data[2];
                model.points[count].b = img2.data[(int)pt1.y*img2.step+(int)pt1.x*img2.channels()];
                model.points[count].g = img2.data[(int)pt1.y*img2.step+(int)pt1.x*img2.channels()+1];
                model.points[count].r = img2.data[(int)pt1.y*img2.step+(int)pt1.x*img2.channels()+2];
                count++;
                // pts1.push_back((pre_valid_pts_ptr+id)->lasesrpoint);
                // pts2.push_back((valid_pts_ptr+id)->lasesrpoint);
            }
        }
        model.resize(count);
        pcl::io::savePCDFile("recon.pcd", model);

        // exchange
        auto tmp = valid_pts_ptr;
        valid_pts_ptr = pre_valid_pts_ptr;
        pre_valid_pts_ptr = tmp;
        tmp = NULL;
        delete tmp;
    }

    
    std::cout << "finish reconstruction" << std::endl;

    return 0;
}


cv::Mat MatrixtoCvMat(const Eigen::Isometry3d &m) {
    cv::Mat cvMat(3,3,CV_32F);
    for(int i=0;i<3;i++)
        for(int j=0; j<3; j++)
            cvMat.at<float>(i,j)=m(i,j);
    return cvMat.clone();
}

void Triangulate(const cv::Point2f &kp1, const cv::Point2f &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D) {
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