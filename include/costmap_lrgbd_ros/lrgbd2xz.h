#include <iostream>
#include <fstream>
 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


class LRGBDCostMap{

public:
    LRGBDCostMap();
    void init(Eigen::Matrix3d camera_K, double resolution_size, double map_width, double map_height, bool display_costmap, int depthScale, cv::Mat T_camera2base, cv::Mat T_laser2base);
    void laserToCostMap(void);
    void depthCameraToCostMap(cv::Mat depth_image);
    void obstacleMap(void);
    void inflationHighResolution(void);
    ~LRGBDCostMap(void);


private:
    cv::Mat costmap2d_;
    Eigen::Matrix3d camera_K_;
    double image_height_;
    double image_width_;
    double resolution_size_;
    double map_width_;
    double map_height_;
    bool display_costmap_;
    int depthScale_;
    cv::Mat T_camera2base_;
    cv::Mat T_laser2base_;
    cv::Mat T_base2world_;
};