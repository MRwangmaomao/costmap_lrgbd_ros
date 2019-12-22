#include "costmap_lrgbd_ros/lrgbd2xz.h"


LRGBDCostMap::LRGBDCostMap(){
    cv::Mat T_camera2base_ = cv::Mat::eye(4,4,CV_32F);
    cv::Mat T_laser2base_ = cv::Mat::eye(4,4,CV_32F); 
}

LRGBDCostMap::~LRGBDCostMap(void){

}
 
void LRGBDCostMap::init(Eigen::Matrix3d camera_K, int image_height, int image_width, double resolution_size, double map_width, double map_height, bool display_costmap, int depthScale, cv::Mat T_camera2base, cv::Mat T_laser2base){
    resolution_size_ = resolution_size;
    // image_height_ = image_height;
    // image_width_ = image_width;
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
    fx_ = camera_K(0,0);
    cx_ = camera_K(0,2);
    fy_ = camera_K(1,0);
    cy_ = camera_K(1,1);

    band_width_ = (int)(((image_width_/2 - cx_) * (map_width_/2) / fx_) /resolution_size_);

    std::cout <<"band_width_: " <<  band_width_ << std::endl;
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
            point[0] = (u-cx_)*point[2]/fx_;
            point[1] = (v-cy_)*point[2]/fy_;
            point[3] = 1;
            Eigen::Matrix<double, 4, 1> pointWorld;
            // pointWorld = T_camera2base_ * point;  
            pointWorld = point;
            if(pointWorld[0] > -(map_height_/2) && pointWorld[0] < map_height_/2 && pointWorld[2] > -(map_width_/2) && pointWorld[2] < map_width_/2  && pointWorld[1] > -0.85) {
            costmap2d_.data[ (uint)(pointWorld[0]/resolution_size_ + map_image_h_/2) * costmap2d_.step+(uint)(pointWorld[2]/resolution_size_  + map_image_w_/2)*costmap2d_.channels() ] = 0;
            costmap2d_.data[ (uint)(pointWorld[0]/resolution_size_ + map_image_h_/2) * costmap2d_.step+(uint)(pointWorld[2]/resolution_size_  + map_image_w_/2)*costmap2d_.channels()+1 ] = 255;
            costmap2d_.data[ (uint)(pointWorld[0]/resolution_size_ + map_image_h_/2) * costmap2d_.step+(uint)(pointWorld[2]/resolution_size_ + map_image_w_/2)*costmap2d_.channels()+2 ] = 0;
            // std::cout << pointWorld[0]/resolution_size_ << std::endl;

                // costmap2d_.data[ (uchar)(pointWorld[0]/resolution_size_ + map_image_h_/2) * costmap2d_.step+(uchar)(pointWorld[2]/resolution_size_ + map_image_w_/2)*depth_image.channels() ] = 255;
                // costmap2d_.data[ (uchar)(pointWorld[0]/resolution_size_ + map_image_h_/2) * costmap2d_.step+(uchar)(pointWorld[2]/resolution_size_ + map_image_w_/2)*depth_image.channels()+1 ] = 255;
                // costmap2d_.data[ (uchar)(pointWorld[0]/resolution_size_ + map_image_h_/2) * costmap2d_.step+(uchar)(pointWorld[2]/resolution_size_ + map_image_w_/2)*depth_image.channels()+2 ] = 255;
            }
        }
    }
    drawFreeArea(); // 可通行区域
    
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
    drawCross();
    drawRGBDBounds();
    
}

void LRGBDCostMap::obstacleMap(void){

}

void LRGBDCostMap::inflationHighResolution(void){
    
}

void LRGBDCostMap::drawCross(void){
    int cross_line_size = 10;
    cv::line(costmap2d_, cvPoint(map_image_w_/2-cross_line_size/2,map_image_h_/2), cvPoint(map_image_w_/2+cross_line_size/2, map_image_h_/2), cv::Scalar(255,255,0), 1, 8, 0); 
	cv::line(costmap2d_, cvPoint(map_image_w_/2,map_image_h_/2-cross_line_size/2), cvPoint(map_image_w_/2, map_image_h_/2+cross_line_size/2), cv::Scalar(255,255,0), 1, 8, 0);
}

void LRGBDCostMap::drawRGBDBounds(void){ 
    cv::line(costmap2d_,cvPoint(map_image_w_/2, map_image_h_/2), cvPoint(map_image_w_-1, map_image_h_/2+band_width_), cv::Scalar(0,255,255),2,8,0); 
	cv::line(costmap2d_,cvPoint(map_image_w_/2, map_image_h_/2), cvPoint(map_image_w_-1, map_image_h_/2-band_width_), cv::Scalar(0,255,255),2,8,0);
}

void LRGBDCostMap::drawFreeArea(void){
    if(band_width_ < 0){
        band_width_ = -band_width_;
    }
    int min_h = map_image_h_/2-band_width_;
    int max_h = map_image_h_/2+band_width_;

    for(int i = min_h+5; i < max_h-5; i++){
        for(int j = map_image_w_/2; j < map_image_w_; j++){
            int point_x = j;
            int point_y;
            if(i != map_image_h_/2)
                point_y = (j-map_image_w_/2)*(i-map_image_h_/2)/(map_image_w_/2) + map_image_h_/2;
            else
                point_y = map_image_h_/2;
            int index = point_y * costmap2d_.step + point_x * costmap2d_.channels();
            if(costmap2d_.data[index] == 0 && costmap2d_.data[index+1] == 255 && costmap2d_.data[index+2] == 0){
                // std::cout << "meet obstcal;" << std::endl;
                break;
                
            }
                
            costmap2d_.data[index] = 255;
            costmap2d_.data[index+1] = 0;
            costmap2d_.data[index+2] =0;
        }
    }
}
