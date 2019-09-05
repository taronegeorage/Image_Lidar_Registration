#ifndef __FISHEYEIMAGE_H__
#define __FISHEYEIMAGE_H__

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

class Fisheye {
public:
    Fisheye();
    cv::Mat undistortimg(const cv::Mat&,const int camidx0);
    void getIntrinsics(float&, float&, float&, float&, int camid = 4);
    Eigen::Matrix3d getIntrinsicsMatrix(int);
    void getImagesize(int&, int&);
    inline Eigen::Matrix4d getTcl(int camid = 4) {
        if(camid == 1) return lidar_to_cam1;
        else if(camid == 2) return lidar_to_cam2;
        else if(camid == 3) return lidar_to_cam3;
        else return lidar_to_cam4;
    };
    inline Eigen::Vector3d transformProject(const Eigen::Vector4d& P_lidar, int camid = 4) {	    
        // Eigen::Vector3d z_P_uv;
        // if(camid == 1) z_P_uv = intri1 * lidar_to_cam1 * P_lidar;
        // else if(camid == 2) z_P_uv = intri2 * lidar_to_cam2 * P_lidar;
        // else if(camid == 3) z_P_uv = intri3 * lidar_to_cam3 * P_lidar;
        // else z_P_uv = intri4 * lidar_to_cam4 * P_lidar;
	    // return Eigen::Vector3d( int( z_P_uv[0]/z_P_uv[2] + 0.5 ) , int( z_P_uv[1]/z_P_uv[2] + 0.5 ), 1 );
    }
private:
    float getdistortcoeff0(int id);
    void getK0(int id,float (&intr)[3][3]);
    bool distortion(float u_p[2], float d_p[2], float fx, float fy, float cx, float cy, float w);
    void setIntrinsics();
    void setExtrinsics();

    Eigen::Matrix4d lidar_to_cam1; 
    Eigen::Matrix4d lidar_to_cam2;
    Eigen::Matrix4d lidar_to_cam3;
    Eigen::Matrix4d lidar_to_cam4;

    Eigen::Matrix<double, 3, 3> intri1;
    Eigen::Matrix<double, 3, 3> intri2;
    Eigen::Matrix<double, 3, 3> intri3;
    Eigen::Matrix<double, 3, 3> intri4;

    float cam_lidar_transform1[4][4]=
            {{-9.99955282e-01,7.78376820e-03,5.37091219e-03,5.69720922e-03},
            {-5.36678542e-03,5.50967092e-04,-9.99985447e-01,-1.12933050e-01},
            {-7.78661412e-03,-9.99969554e-01,-5.09168640e-04,-2.25455764e-01},
            {0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00}};
    float cam_lidar_transform2[4][4]=
            {{0.01192883,0.99992585,-0.00245026,-0.00857618},
            {-0.01652029,-0.00225301,-0.99986099,-0.1005358},
            {-0.99979237,0.01196765,0.01649219,-0.227512},
            {0.,0.,0.,1.}};
    float cam_lidar_transform3[4][4]=
            {{9.99967175e-01,6.05118632e-03,-5.38809830e-03,-6.76300340e-04},
            {-5.43088201e-03,7.07626705e-03,-9.99960215e-01,-1.50224504e-01},
            {-6.01281795e-03,9.99956654e-01,7.10889805e-03,-2.43711597e-01},
            {0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00}};
    float cam_lidar_transform4[4][4]=
            {{-1.44694624e-02,-9.99894963e-01,8.34671705e-04,-3.00805142e-03},
            {5.13704445e-03,-9.09086106e-04,-9.99986392e-01,-1.06576990e-01},
            {9.99882116e-01,-1.44649777e-02,5.14965886e-03,-2.40858246e-01},
            {0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00}};
    float distortion_coeffs1=0.9693829545338059;
    float intrinsics1[3][3] = {{540.1810323808173,0, 606.090026111307}, {0,540.2171662351367, 361.2552380993679},{0,0,1}};

    float distortion_coeffs2=0.9691586491475527;
    float intrinsics2[3][3] = {{545.1253024596331,0, 659.6122763365902}, {0,544.6953208304285, 387.65861867195775},{0,0,1}};
	
    float distortion_coeffs3=0.9738366638320757;
    float intrinsics3[3][3] = {{551.3448229223449,0, 663.0908939924225}, {0,551.0976537348477, 394.0275346174711},{0,0,1}};
	
    float distortion_coeffs4=0.97537410019784;
    float intrinsics4[3][3] = {{536.3822965039258,0, 675.7264038481511}, {0,536.0486148894282, 326.4673600234504},{0,0,1}};
	
	int img_w;
    int img_h;
    float fx1, fy1, cx1, cy1;
    float fx2, fy2, cx2, cy2;
    float fx3, fy3, cx3, cy3;
    float fx4, fy4, cx4, cy4;
};


#endif