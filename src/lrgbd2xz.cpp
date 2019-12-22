#include "costmap_lrgbd_ros/lrgbd2xz.h"
#include<opencv2/core/eigen.hpp>
LRGBDCostMap::LRGBDCostMap(){
    cv::Mat T_camera2base_ = cv::Mat::eye(4,4,CV_32F);
    cv::Mat T_laser2base_ = cv::Mat::eye(4,4,CV_32F); 
}

LRGBDCostMap::~LRGBDCostMap(void){

}

void LRGBDCostMap::init(Eigen::Matrix3d camera_K, double resolution_size, double map_width, double map_height, bool display_costmap, int depthScale, cv::Mat T_camera2base, cv::Mat T_laser2base){
    resolution_size_ = resolution_size;
    map_width_ = map_width;
    map_height_ = map_height;
    display_costmap_ = display_costmap;
    depthScale_ = depthScale;
    
    cv::cv2eigen(T_camera2base,T_camera2base_);
    cv::cv2eigen(T_laser2base,T_laser2base_);
    map_image_w_ = int(map_width_/resolution_size_);
    map_image_h_ = int(map_height_/resolution_size_);
    
    std::cout << "costmap2d_ width is： " << map_image_w_ << "， and height is " << map_image_h_ << std::endl;

    cv::Mat costmap2(map_image_h_, map_image_w_, CV_8UC3, cv::Scalar(0,0,0));
    costmap2d_ = costmap2.clone(); 
    fx = camera_K(0,0);
    cx = camera_K(0,2);
    fy = camera_K(1,0);
    cy = camera_K(1,1); 
}

void LRGBDCostMap::laserToCostMap(void){
    
}

void LRGBDCostMap::depthCameraToCostMap(cv::Mat depth_image){ 
    for ( int v=0; v<depth_image.rows; v++ ){
        for ( int u=0; u<depth_image.cols; u++ ) {
            unsigned int d = depth_image.ptr<unsigned short> ( v )[u];  
            if ( d==0 ) continue;  
            Eigen::Matrix<double, 4, 1> point; 
            point[2] = double(d)/depthScale_;
            point[0] = (u-cx)*point[2]/fx;
            point[1] = (v-cy)*point[2]/fy;
            point[3] = 1;
            Eigen::Matrix<double, 4, 1> pointWorld;
            // pointWorld = T_camera2base_ * point;  
            pointWorld = point;
            if(pointWorld[0] > -(map_height_/2) && pointWorld[0] < map_height_/2 && pointWorld[2] > -(map_width_/2) && pointWorld[2] < map_width_/2  && pointWorld[1] > -0.85) {
            costmap2d_.data[ (uint)(pointWorld[0]/resolution_size_ + map_image_h_/2) * costmap2d_.step+(uint)(pointWorld[2]/resolution_size_  + map_image_w_/2)*costmap2d_.channels() ] = 255;
            costmap2d_.data[ (uint)(pointWorld[0]/resolution_size_ + map_image_h_/2) * costmap2d_.step+(uint)(pointWorld[2]/resolution_size_  + map_image_w_/2)*costmap2d_.channels()+1 ] = 255;
            costmap2d_.data[ (uint)(pointWorld[0]/resolution_size_ + map_image_h_/2) * costmap2d_.step+(uint)(pointWorld[2]/resolution_size_ + map_image_w_/2)*costmap2d_.channels()+2 ] = 255;
            // std::cout << pointWorld[0]/resolution_size_ << std::endl;

                // costmap2d_.data[ (uchar)(pointWorld[0]/resolution_size_ + map_image_h_/2) * costmap2d_.step+(uchar)(pointWorld[2]/resolution_size_ + map_image_w_/2)*depth_image.channels() ] = 255;
                // costmap2d_.data[ (uchar)(pointWorld[0]/resolution_size_ + map_image_h_/2) * costmap2d_.step+(uchar)(pointWorld[2]/resolution_size_ + map_image_w_/2)*depth_image.channels()+1 ] = 255;
                // costmap2d_.data[ (uchar)(pointWorld[0]/resolution_size_ + map_image_h_/2) * costmap2d_.step+(uchar)(pointWorld[2]/resolution_size_ + map_image_w_/2)*depth_image.channels()+2 ] = 255;
            }
        }
    }

    cv::namedWindow("1");
    cv::imshow("1",costmap2d_);
    cvWaitKey(1);

    for ( int v=0; v<costmap2d_.rows; v++ ){
        for ( int u=0; u<costmap2d_.cols; u++ )
        {
            costmap2d_.data[v*costmap2d_.step+u*costmap2d_.channels()] = 0;
            costmap2d_.data[v*costmap2d_.step+u*costmap2d_.channels() + 1] = 0;
            costmap2d_.data[v*costmap2d_.step+u*costmap2d_.channels() + 2] = 0;
        }   
    }
}

void LRGBDCostMap::obstacleMap(void){

}

void LRGBDCostMap::inflationHighResolution(void){
    
}