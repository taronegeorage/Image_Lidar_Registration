#include<fstream>
#include <sstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "fisheyeimg.h"
using namespace std;


Fisheye::Fisheye() {
	img_w = 1280;
    img_h = 720;
    setIntrinsics();
	setExtrinsics();
}

void Fisheye::setIntrinsics() {
	fx4 = intrinsics4[0][0]; fy4 = intrinsics4[1][1]; cx4 = intrinsics4[0][2]; cy4 = intrinsics4[1][2];
	fx3 = intrinsics3[0][0]; fy3 = intrinsics3[1][1]; cx3 = intrinsics3[0][2]; cy3 = intrinsics3[1][2];
	fx2 = intrinsics2[0][0]; fy2 = intrinsics2[1][1]; cx2 = intrinsics2[0][2]; cy2 = intrinsics2[1][2];
	fx1 = intrinsics1[0][0]; fy1 = intrinsics1[1][1]; cx1 = intrinsics1[0][2]; cy1 = intrinsics1[1][2];
	intri1 << fx1, 0, cx1, 0, fy1, cy1, 0, 0, 1;
	intri2 << fx2, 0, cx2, 0, fy2, cy2, 0, 0, 1;
	intri3 << fx3, 0, cx3, 0, fy3, cy3, 0, 0, 1;
	intri4 << fx4, 0, cx4, 0, fy4, cy4, 0, 0, 1;
}

void Fisheye::setExtrinsics() {
	lidar_to_cam1 << -9.99955282e-01,7.78376820e-03,5.37091219e-03,5.69720922e-03, 
                    -5.36678542e-03,5.50967092e-04,-9.99985447e-01,-1.12933050e-01, 
                    -7.78661412e-03,-9.99969554e-01,-5.09168640e-04,-2.25455764e-01,
                    0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00;
	lidar_to_cam2 << 0.01192883,0.99992585,-0.00245026,-0.00857618,
            		-0.01652029,-0.00225301,-0.99986099,-0.1005358,
            		-0.99979237,0.01196765,0.01649219,-0.227512,
            		0.,0.,0.,1.;
	lidar_to_cam3 << 9.99967175e-01,6.05118632e-03,-5.38809830e-03,-6.76300340e-04,
            		-5.43088201e-03,7.07626705e-03,-9.99960215e-01,-1.50224504e-01,
            		-6.01281795e-03,9.99956654e-01,7.10889805e-03,-2.43711597e-01,
            		0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00;
	lidar_to_cam4 << -1.44694624e-02,-9.99894963e-01,8.34671705e-04,-3.00805142e-03,
            		5.13704445e-03,-9.09086106e-04,-9.99986392e-01,-1.06576990e-01,
            		9.99882116e-01,-1.44649777e-02,5.14965886e-03,-2.40858246e-01,
            		0.00000000e+00,0.00000000e+00,0.00000000e+00,1.00000000e+00;
}

float Fisheye::getdistortcoeff0(int id){
	switch(id){
		case 0:
			return distortion_coeffs1;
		case 1:
			return distortion_coeffs2;
		case 2:
			return distortion_coeffs3;
		case 3:
			return distortion_coeffs4;
		default:
			cout<<"  ERROR allow 0~3 , request "<<id<<endl;
			exit(1);
	}
}

void Fisheye::getK0(int id,float (&intr)[3][3]){
	switch(id){
		case 0:
			for(int i=0;i<9;i++){*(*(intr+i/3)+i%3)=*(*(intrinsics1+i/3)+i%3);}
			break;
		case 1:
			for(int i=0;i<9;i++){*(*(intr+i/3)+i%3)=*(*(intrinsics2+i/3)+i%3);}
			break;
		case 2:
			for(int i=0;i<9;i++){*(*(intr+i/3)+i%3)=*(*(intrinsics3+i/3)+i%3);}
			break;
		case 3:
			for(int i=0;i<9;i++){*(*(intr+i/3)+i%3)=*(*(intrinsics4+i/3)+i%3);}
			break;
		default:
			cout<<"  ERROR allow 0~3 , request "<<id<<endl;
			exit(1);
	}
} 

void Fisheye::getIntrinsics(float& fx, float& fy, float& cx, float& cy, int camid) {
	if(camid = 1) {
		fx = fx1; fy = fy1; cx = cx1; cy = cy1;
	}
	else if(camid = 2) {
		fx = fx2; fy = fy2; cx = cx2; cy = cy2;
	}
	else if(camid = 3) {
		fx = fx3; fy = fy3; cx = cx3; cy = cy3;
	}
	else {
		fx = fx4; fy = fy4; cx = cx4; cy = cy4;
	}
}

Eigen::Matrix3d Fisheye::getIntrinsicsMatrix(int camid) {
	if(camid==1) return intri1;
	else if(camid==2) return intri2;
	else if(camid == 3) return intri3;
	else return intri4;
}

void Fisheye::getImagesize(int& wid, int& height) {
	wid = img_w;
	height = img_h;
}


bool Fisheye::distortion(float u_p[2], float d_p[2], float fx, float fy, float cx, float cy, float w){
		
	u_p[0] = (u_p[0] - cx) / fx;
	u_p[1] = (u_p[1] - cy) / fy;
		
	double mul2tanwby2 = tan(w / 2.0) * 2.0;

	// Calculate distance from point to center.
	double r_u = sqrt(u_p[0]*u_p[0] + u_p[1]*u_p[1]);
	if (mul2tanwby2 == 0 || r_u == 0) {
		return false;
	}
	// Calculate undistorted radius of point.
	double kMaxValidAngle = 89.0;
	
	double r_d;
	r_d = atan(r_u * mul2tanwby2) / (w*r_u) ;
	if (fabs(r_d * w) <= kMaxValidAngle) {
		//1
	} else {
		return false;
	}

	d_p[0] = u_p[0] * r_d;
	d_p[1] = u_p[1] * r_d;
	
	d_p[0] = d_p[0]* fx + cx;
	d_p[1] = d_p[1]* fy + cy;
	
	return true;
}

cv::Mat Fisheye::undistortimg(const cv::Mat&img,const int camidx0){
		
	cv::Mat uimg = cv::Mat::zeros(cv::Size(img.cols,img.rows), CV_8UC3);
	float intrinsics[3][3];
    getK0(camidx0, intrinsics);
	float distortion_coeffs=getdistortcoeff0(camidx0);
	for (int i=0; i< uimg.rows; ++i) {
		for(int j=0; j< uimg.cols; ++j) {
			float u_p[] = {float(j),float(i)};//x,y
			float d_p[2];
			distortion(u_p, d_p,
			intrinsics[0][0],intrinsics[1][1],intrinsics[0][2],intrinsics[1][2],distortion_coeffs);

			int jj =floor( d_p[0]);
			int ii = floor(d_p[1]); 
			if (ii>=0 && ii <= img.rows-2 && jj>=0 && jj <= img.cols-2) {
				int jj1=jj+1;
				int ii1=ii+1;
				float ki=d_p[1]-ii;
				float kj=d_p[0]-jj;
				cv::Vec3b v00 = img.at<cv::Vec3b>(ii,jj);
				cv::Vec3b v10 = img.at<cv::Vec3b>(ii1,jj);
				cv::Vec3b v01 = img.at<cv::Vec3b>(ii,jj1);
				cv::Vec3b v11 = img.at<cv::Vec3b>(ii1,jj1);
				cv::Vec3b v;
				for(int i3=0;i3<3;i3++){
				    v[i3]=uchar(int((1-ki)*(1-kj)*v00[i3]+(ki)*(1-kj)*v10[i3]+(1-ki)*(kj)*v01[i3]+(ki)*(kj)*v11[i3]));
				}
				uimg.at<cv::Vec3b>(i,j) = v;
			}
		}
	}
	return uimg;
}