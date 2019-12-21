#include "costmap_lrgbd_ros/lrgbd2xz.h"

LRGBDCostMap::LRGBDCostMap(){
    cv::Mat T_camera2base_ = cv::Mat::eye(4,4,CV_32F);
    cv::Mat T_laser2base_ = cv::Mat::eye(4,4,CV_32F); 
}

LRGBDCostMap::~LRGBDCostMap(void){

}

void LRGBDCostMap::init(Eigen::Matrix3d camera_K, double resolution_size, double map_width, double map_height, bool display_costmap, int depthScale, cv::Mat T_camera2base, cv::Mat T_laser2base){
    camera_K_ << camera_K;
    resolution_size_ = resolution_size;
    map_width_ = map_width;
    map_height_ = map_height;
    display_costmap_ = display_costmap;
    depthScale_ = depthScale;
    T_camera2base_ = T_camera2base.clone();
    T_laser2base_ = T_laser2base.clone();
    cv::Mat costmap2d_(int(map_width_/resolution_size_), int(map_height_/resolution_size_), CV_8UC3, cv::Scalar(0,0,0)); 
}

void LRGBDCostMap::laserToCostMap(void){
    
}

void LRGBDCostMap::depthCameraToCostMap(cv::Mat depth_image){ 
    for ( int v=0; v<depth_image.rows; v++ ){
        for ( int u=0; u<depth_image.cols; u++ ) {
            unsigned int d = depth_image.ptr<unsigned short> ( v )[u];  
            if ( d==0 ) continue;  
            Eigen::Vector4d point; 
            point[2] = double(d)/depthScale_; 
            point[0] = (u-camera_K_(2))*point[2]/camera_K_(0);
            point[1] = (v-camera_K_(4))*point[2]/camera_K_(3);
            point[3] = 1;
            // Eigen::Vector4d pointWorld = T_camera2base_ * point;
            
    //         if(pointWorld[0] > -3.0 && pointWorld[0] < 3.0 && pointWorld[2] > -6.0 && pointWorld[2] < 6.0 && pointWorld[1] > 0.1) {
    //             // std::cout << "cout:"<< pointWorld[0] <<" "<< pointWorld[2] <<" "<< pointWorld[2] << std::endl;
    //             costmap2d_.data[ (uchar)(pointWorld[0]/resolution_size_ + 150) * costmap2d_.step+(uchar)(pointWorld[2]/resolution_size_ + 300)*depth_image.channels() ] = 255;
    //             costmap2d_.data[ (uchar)(pointWorld[0]/resolution_size_ + 150) * costmap2d_.step+(uchar)(pointWorld[2]/resolution_size_ + 300)*depth_image.channels()+1 ] = 255;
    //             costmap2d_.data[ (uchar)(pointWorld[0]/resolution_size_ + 150) * costmap2d_.step+(uchar)(pointWorld[2]/resolution_size_ + 300)*depth_image.channels()+2 ] = 255;
    //         }   
        }
    }

}

void LRGBDCostMap::obstacleMap(void){

}

void LRGBDCostMap::inflationHighResolution(void){
    
}