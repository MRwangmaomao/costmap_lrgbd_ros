#include <iostream>
#include <fstream>
 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>


class LRGBDCostMap{

public:
    LRGBDCostMap();
    void laserToCostMap(void);
    void depthCameraToCostMap(void);
    void obstacleMap(void);
    void inflationHighResolution(void);
    ~LRGBDCostMap(void);


private:
    Eigen::Matrix3d camera_K_;
    double image_height_;
    double image_width_;
    double map_width_;
    double map_height_;
    bool display_costmap_;
    Eigen::Matrix4d lidar2baselink_;
    Eigen::Matrix4d camera2baselink_;
    Eigen::Matrix4d baselink2orignal_;
};