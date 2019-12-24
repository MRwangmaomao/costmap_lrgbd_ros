#include <iostream>
#include <fstream>
#include<sstream>
#include <string>
#include <queue> 
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class DWAPlanning{

public:
    DWAPlanning();
    ~DWAPlanning(void);

    void init(std::string config_file_path); 
    void move(Eigen::Matrix4d robot_pose, const cv::Mat& config_map, double & go_v, double & turn_v);

private:
    void getWayPoint();
    void getRobotLocalization();
    void setWayPointInCostmap();
    void getAllWayPoints();
    bool isArriveWayPoint();
    bool isArriveDestination();
    bool dwa_control(const cv::Mat&); 
    double max_acc_x_;
    double max_acc_theta_;
    double max_speed_;
    double min_speed_;
    double foresee_;
    double sim_time_;
    double region_foresee_;
    double short_foresee_rate_;
    double sim_period_;
    bool aggressive_mode_;
    double go_v_;
    double turn_v_;
    std::string waypoints_file_path_;
    std::queue<Eigen::Vector3d> waypoints_queue_;
    Eigen::Vector3d robot_costmap_waypoint_;
    Eigen::Vector4d robot_pose_;

};
