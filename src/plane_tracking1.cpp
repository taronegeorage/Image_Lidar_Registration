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

const bool show_traj = false;
const bool save_view = false;
void save_trajectory(pcl::PointCloud<pcl::PointXYZ>& cld, std::vector<Frame>& poses);
void save_pc_view(pcl::PointCloud<pcl::PointXYZ>& pc, Eigen::Isometry3d pose, int idx); 

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
    pcl::io::savePCDFile("trans.pcd", *cld);
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
    
    // Parameter for finding planes
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients() );
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices() );
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.01);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PCDWriter writer;
    double threshold = 0.8 * cld->points.size();
    int num_planes = 0;
    vector<pcl::PointCloud<pcl::PointXYZ> > planes;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>),
        cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cld, *tmp);

    while(tmp->points.size() > threshold) {
        seg.setInputCloud(tmp);
        seg.segment(*inliers, *coefficients);
        if(inliers->indices.size() == 0) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            exit(-1);
        }
        std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
        extract.setInputCloud(tmp);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        if((fabs(coefficients->values[0])>0.9 || fabs(coefficients->values[2])>0.9) && cloud_p->points.size()>200) {
            //remove outliers
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
            outrem.setInputCloud(cloud_p);
            outrem.setRadiusSearch(0.7);
            outrem.setMinNeighborsInRadius(5);
            outrem.filter(*cloud_p);

            cout << cloud_p->points.size() << endl;
            std::cerr << "plane " << num_planes << std::endl;
            num_planes++;
            planes.push_back(*cloud_p);
            std::stringstream ss;
            ss << "plane_" << num_planes << ".pcd";
            writer.write<pcl::PointXYZ> (ss.str(), *cloud_p);
        }
        extract.setNegative (true);
        extract.filter(*cloud_f);
        tmp.swap(cloud_f);
    }

    if(show_traj)
        save_trajectory(*cld, poses);
    
    if(save_view)
        save_pc_view(*cld, poses[1].Pose, 1);

    // double maxX = 25.0, maxY = 6.0, maxZ = 15.0;
    // // unordered_map<int, double> nearest;
    // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // kdtree.setInputCloud (cld);
    
    // optical flow tracking settings
    Tracking tracker = new Tracking(true, cv::Size(4, 4), 3);
    // start processing
    for(int idx = 0; idx < planes.size(); idx++) { //planes.size()
        pcl::PointCloud<pcl::PointXYZ> pc = planes[idx];
        int cam_cnt = 0;
        bool initialized = false;
        PixelPoint valid_pts[pc.points.size()];
        PixelPoint pre_valid_pts[pc.points.size()];

        PixelPoint *valid_pts_ptr = valid_pts;
        PixelPoint *pre_valid_pts_ptr = pre_valid_pts;
        // project planes onto cams
        for(int i = 0; i < poses.size(); i++) {
            int point_cnt = 0;
            std::vector<cv::Point2f> pts;
            cv::Mat img = poses[i].img.clone();
            Eigen::Isometry3d func = poses[i].Pose.inverse();
            // std::cout << "The transformation function is\n " << poses[i].Pose.matrix() << std::endl;
            for(int j = 0; j < pc.points.size(); j++) {
                Eigen::Vector4d point_(pc.points[j].x, pc.points[j].y, pc.points[j].z, 1.0);
                double z = func.matrix().row(2) * point_;         
                double y = -func.matrix().row(1) * point_;
                double x = -func.matrix().row(0) * point_;
                // Eigen::Vector3d p_uv = f->transformProject(point_, camid);
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

                cv::circle(img, (valid_pts_ptr+j)->lasesrpoint, 4, cv::Scalar(0,255,0));
                point_cnt++;
            }
            cout << "pose " << i+1 << " " << "plane " << idx+1 << " cnt " << point_cnt << endl;
            // stringstream pro_name;
            // pro_name << "pose " << i+1 << " " << "plane " << idx+1;
            // cv::imshow(pro_name.str(), img);
            // cv::waitKey(0);

            if(!initialized && point_cnt >= pc.points.size()*0.2) {
                std::cout << i+1 << " is the first frame" << std::endl;
                tracker.setLastFrame(poses[i].img);
                initialized = true;
                // exchange
                auto tmp = valid_pts_ptr;
                valid_pts_ptr = pre_valid_pts_ptr;
                pre_valid_pts_ptr = tmp;
                tmp = NULL;
                delete tmp;
                continue;
            }
            
            if(point_cnt >= pc.points.size()*0.2) {
                // cv::imshow("initial guess", img);
                // cv::waitKey(0);
                cam_cnt++;
                std::vector<cv::Point2f> pts1;
                std::vector<cv::Point2f> pts2;
                for(int id = 0; id < pc.points.size(); id++) {
                    if((valid_pts_ptr+id)->visible && (pre_valid_pts_ptr+id)->visible) {
                        pts1.push_back((pre_valid_pts_ptr+id)->lasesrpoint);
                        pts2.push_back((valid_pts_ptr+id)->lasesrpoint);
                    }
                    // if()
                }

                double percentage = 0;
                int iter = 0;
                
                cv::Mat Homo;
                
                while(percentage < 0.8) {
                    std::vector<cv::Point2f> pts1_tmp(pts1);
                    std::vector<cv::Point2f> pts2_tmp(pts2);
                    if(iter > 0) {
                        tracker.setLastFrame(poses[i-1].img);
                        cv::perspectiveTransform(pts1_tmp, pts2_tmp, Homo);
                    }

                    tracker.feedFrameAndInitailflow(poses[i].img, pts1_tmp, pts2_tmp);
                    cv::Mat cur_img = poses[i].img.clone();
                    // for(int n = 0; n < pts2_tmp.size(); n++)
                    //     cv::circle(cur_img, pts2_tmp[n], 4, cv::Scalar(0,0,255));
                    tracker.drawMatch(poses[i-1].img, pts1_tmp, cur_img, pts2_tmp);
                    tracker.frameTracking(pts1_tmp, pts2_tmp);
                
                    double threshold = 5.0;
                    cv::Mat mask;
                    std::cout << iter << " " << pts1_tmp.size() << " " << pts2_tmp.size() << std::endl;
                    Homo = cv::findHomography(pts1_tmp, pts2_tmp, mask, CV_RANSAC, threshold);
                    std::cout << "Homography between " << i << " and " << i+1 << std::endl;
                    std::cout << Homo << std::endl;
                    std::cout << "mask " << std::endl;
                    std::cout << mask.size() << std::endl;
                    int num_inlier = 0;
                    if(mask.size().height < 30)
                        break;
                    for(int flg = 0; flg < mask.size().height; flg++)
                        if((int)mask.at<uchar>(flg)==1) 
                            num_inlier++;
                    percentage = num_inlier*1.0 / mask.size().height;
                    std::cout << "percentage: " << percentage << std::endl;
                    // cv::perspectiveTransform(pts1_tmp, pts2_tmp, Homo);
                    iter++;

                    cv::Mat trans;// = poses[i-1].img.clone();
                    cv::warpPerspective(poses[i-1].img, trans, Homo, cv::Size(w, h));
                    cv::imshow("warp", trans);
                    {
                        // cv::Mat diff(h, w, CV_8UC1);
                        // cv::Mat curgray, trans_gray;
                        // cvtColor(poses[i].img, curgray, cv::COLOR_BGR2GRAY);
                        // cvtColor(trans, trans_gray, cv::COLOR_BGR2GRAY);
                        // for(int hh = 0; hh < h; hh++) {
                        //     uchar* data = diff.ptr<uchar>(hh);
                        //     for(int ww = 0; ww < w; ww++)
                        //         data[ww] = abs((int)curgray.at<uchar>(hh,ww) - (int)trans_gray.at<uchar>(hh,ww));
                        // }
                        // cv::imshow("diff", diff);
                        // cv::waitKey(0);
                    }
                    
                }
            }
            // exchange
            auto tmp = valid_pts_ptr;
            valid_pts_ptr = pre_valid_pts_ptr;
            pre_valid_pts_ptr = tmp;
            tmp = NULL;
            delete tmp;
        }

        // same plane projection constraints
    

        // H matrix constarin, optimize the plane and poses
        
    }
    
    std::cout << "finish processing" << std::endl;

    return 0;
}

void show_trajectory(pcl::PointCloud<pcl::PointXYZ>& cld, std::vector<Frame>& poses) {
    typedef pcl::PointXYZRGB PointT;
    pcl::PointCloud<PointT> traj;
    int traj_count = 0;
    traj.height = 1;
    traj.width = poses.size();
    traj.resize(traj.width+cld.points.size());
    traj.is_dense = false;

    for(int i = 0; i < poses.size(); i++) {
        Eigen::Vector3d p = poses[i].trans;
        traj.points[i].x = p(0);
        traj.points[i].y = p(1);
        traj.points[i].z = p(2);
        traj.points[i].r = traj.points[i].g = traj.points[i].b = 255;
    }
    traj_count = poses.size();
    for(auto& point : cld.points){
        traj.points[traj_count].x = point.x;
        traj.points[traj_count].y = point.y;
        traj.points[traj_count].z = point.z;
        traj.points[traj_count].r = traj.points[traj_count].g = traj.points[traj_count].b = 100;
        traj_count++;
    }
    pcl::io::savePCDFileBinary("traj.pcd", traj );
}

void save_pc_view(pcl::PointCloud<pcl::PointXYZ>& pc, Eigen::Isometry3d pose, int idx) {
    std::cout << "processing pose " << idx << std::endl;
    pcl::PointCloud<pcl::PointXYZ> cam_view;
    int count = 0;
    cam_view.height = 1;
    cam_view.width = pc.size();
    cam_view.resize(pc.points.size());
    cam_view.is_dense = false;

    Eigen::Isometry3d func = pose.inverse();
    for(int j = 0; j < pc.points.size(); j++) {
        Eigen::Vector4d point_(pc.points[j].x, pc.points[j].y, pc.points[j].z, 1.0);
        double z = func.matrix().row(2) * point_;         
        double y = func.matrix().row(1) * point_;
        double x = func.matrix().row(0) * point_;
        if(z < 0)
            continue;
        cam_view.points[j].x = x;
        cam_view.points[j].y = y;
        cam_view.points[j].z = z;
        count++;
    }
    stringstream ss;
    ss << "view_" << idx << ".pcd";
    cam_view.resize(count);        
    pcl::io::savePCDFileBinary(ss.str(), cam_view );
}