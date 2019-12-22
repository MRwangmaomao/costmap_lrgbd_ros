#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>




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
    double fx;
    double fy;
    double cx;
    double cy;
    double image_height_;
    double image_width_;
    double resolution_size_;
    double map_width_;
    double map_height_;
    int map_image_w_;
    int map_image_h_;
    bool display_costmap_;
    int depthScale_;
    Eigen::Matrix<double, 4, 4> T_camera2base_;
    Eigen::Matrix<double, 4, 4> T_laser2base_;
    cv::Mat T_base2world_;
};