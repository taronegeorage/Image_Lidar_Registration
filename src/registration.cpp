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

template <class Container>
void split_str(const std::string& str, Container& cont, char delim=' ') {
    std::istringstream ss(str);
    std::string token;
    while(std::getline(ss, token, delim))
        cont.push_back(atof(token.c_str()));
}

int main(int argc, char **argv)
{
    Fisheye *f = new Fisheye();
    float fx, fy, cx, cy;
    f->getIntrinsics(fx, fy, cx, cy);
    int w, h;
    f->getImagesize(w, h);
    Eigen::Matrix4d Tcl = f->getTcl();
    Eigen::Matrix4f T_lidar_loam;
    T_lidar_loam << 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix4f Transform = Tcl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cld(new pcl::PointCloud<pcl::PointXYZ>);
	if(pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/cj/ros_ws2/src/RGB_TO_LIDAR/data/cornerMap.pcd", *cld) == -1) {
		PCL_ERROR("CANNOT READ FILE");
		return -1;
	}
	cout << "Loaded " << cld->width << "*" << cld->height << " data points." << endl;
    pcl::transformPointCloud(*cld, *cld, Tcl);
    std::cout << Tcl.matrix() << std::endl;
    Eigen::Matrix3d R_lidar_loam;
    R_lidar_loam << 0, 0, 1, 1, 0, 0, 0, 1, 0;

    std::vector<Frame> poses;
    std::stringstream filename;
    // registratoin
    {
        //read laser pose file  test1_image_pose
        ifstream read_pose("/home/cj/Downloads/outdoor_data/test2_image_pose.txt", ios::binary);
        ifstream read_img("/home/cj/Downloads/outdoor_data/image/time4.txt", ios::binary);
        string line_pose, line_img;
        int idx = 0;
        double t;

        while(getline(read_pose, line_pose)) {
            vector<double> line_pose_split;
            split_str<vector<double>>(line_pose, line_pose_split);
            while(getline(read_img, line_img)) {
                idx++;
                t = atof(line_img.c_str());
                if(fabs(line_pose_split[0]-t) < 0.08) {
                    break;
                }
            }
            filename.str("");
            filename << "/home/cj/Downloads/outdoor_data/image/4_distort/" << setw(6) << setfill('0')<< idx-1 <<".jpg";
            cv::Mat img = cv::imread(filename.str());
            cout << t << " " << filename.str() << endl;

            Eigen::Quaterniond q_laser(line_pose_split[7], line_pose_split[4], line_pose_split[5], line_pose_split[6]);
            Eigen::Vector3d p_laser(line_pose_split[1], line_pose_split[2], line_pose_split[3]);
            Eigen::Matrix3d rot(Tcl.topLeftCorner(3, 3).cast<double>());
            Eigen::Matrix3d q_rot_laser = q_laser.toRotationMatrix();
            Eigen::Quaterniond q(rot*q_rot_laser);
            Eigen::Vector3d p(rot*p_laser+Tcl.topRightCorner(3, 1).cast<double>());

            Frame f(img, t, q, p);
            poses.push_back(f);
            line_pose_split.clear();
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
        if((fabs(coefficients->values[1])>0.9 || fabs(coefficients->values[2])>0.9) && cloud_p->points.size()>200) {
            //remove outliers
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
            outrem.setInputCloud(cloud_p);
            outrem.setRadiusSearch(0.7);
            outrem.setMinNeighborsInRadius(5);
            outrem.filter(*cloud_p);

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

    // combine
    typedef pcl::PointXYZRGB PointT;
    pcl::PointCloud<PointT> newpc;
    newpc.resize(cld->width*cld->height*poses.size());
    int count = 0;
    
    if(show_traj) {
        typedef pcl::PointXYZRGB PointT;
        pcl::PointCloud<PointT> traj;
        int traj_count = 0;
        traj.height = 1;
        traj.width = poses.size();
        traj.resize(traj.width+cld->points.size());
        traj.is_dense = false;

        for(int i = 0; i < poses.size(); i++) {
            Eigen::Vector3d p = poses[i].trans;
            traj.points[i].x = p(0);
            traj.points[i].y = p(1);
            traj.points[i].z = p(2);
            traj.points[i].r = traj.points[i].g = traj.points[i].b = 255;
        }
        traj_count = poses.size();
        for(auto& point : cld->points){
            traj.points[traj_count].x = point.x;
            traj.points[traj_count].y = point.y;
            traj.points[traj_count].z = point.z;
            traj.points[traj_count].r = traj.points[traj_count].g = traj.points[traj_count].b = 100;
            traj_count++;
        }
        pcl::io::savePCDFileBinary("traj.pcd", traj );
    }
    
    
    //76
    // double maxX = 25.0, maxY = 6.0, maxZ = 15.0;
    // // unordered_map<int, double> nearest;
    // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    // kdtree.setInputCloud (cld);
    
    // optical flow tracking settings
    Tracking tracker = new Tracking();
    // tracker.starttracking();

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
        for(int i = 76; i < poses.size(); i++) {
            int point_cnt = 0;
            std::vector<cv::Point2f> pts;
            cv::Mat img = poses[i].img.clone();
            Eigen::Isometry3d func = poses[i].Pose.inverse();
            for(int j = 0; j < pc.points.size(); j++) {
                Eigen::Vector4d point_(pc.points[j].x, pc.points[j].y, pc.points[j].z, 1.0);
                double z = func.matrix().row(2) * point_;           
                double y = -func.matrix().row(1) * point_;
                double x = -func.matrix().row(0) * point_;
                (valid_pts_ptr+j)->visible = false;
                if(z <= 0)
                    continue;
                int u = (int)(x*fx/z+cx);
                int v = (int)(y*fy/z+cy);
                if(u < 0 || v < 0 || u >= w || v >= h)
                    continue;
                // cv::Point laserpoint;
                // laserpoint.x = u;
                // laserpoint.y = v;
                // pts.push_back(laserpoint);
                (valid_pts_ptr+j)->visible = true;
                (valid_pts_ptr+j)->lasesrpoint.x = u;
                (valid_pts_ptr+j)->lasesrpoint.y = v;
                // valid_pts[j].visible = true;
                // valid_pts[j].lasesrpoint.x = u;
                // valid_pts[j].lasesrpoint.y = v;
                cv::circle(img, (valid_pts_ptr+j)->lasesrpoint, 4, cv::Scalar(0,255,0));
                point_cnt++;
            }
            if(!initialized && point_cnt >= pc.points.size()*0.5) {
                std::cout << i+1 << " is the first" << std::endl;
                tracker.setFirstFrame(poses[i].img);
                initialized = true;
                // exchange
                auto tmp = valid_pts_ptr;
                valid_pts_ptr = pre_valid_pts_ptr;
                pre_valid_pts_ptr = tmp;
                tmp = NULL;
                delete tmp;
                continue;
            }
            if(point_cnt >= pc.points.size()*0.5) {
                stringstream name;
                cv::imshow("initial guess", img);
                cv::waitKey(0);
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
                tracker.feedFrameAndInitailflow(poses[i].img, pts1, pts2);
                std::cout << "Frame " << i+1 << std::endl;
                tracker.frameTracking();
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
    
    // tracker.killTread();
    std::cout << "finish processing" << std::endl;

    return 0;
}
