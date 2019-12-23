
#include <ros/ros.h>
#include <ros/console.h>

#include <string>
#include <vector>
#include <queue>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h> 
#include <tf/transform_broadcaster.h> 
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud.h>
#include "costmap_lrgbd_ros/lrgbd2xz.h"

#define RAD2DEG(x) ((x)*180./M_PI) 

LRGBDCostMap lrgbd_tmap;

template<typename T>
T getOption(ros::NodeHandle& pnh,
                    const std::string& param_name, const T & default_val)
{
  T param_val;
  pnh.param<T>(param_name, param_val, default_val);
  return param_val;
}

 
void img_callback(const sensor_msgs::ImageConstPtr &depth)
{  
    try
    {
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(depth);
        lrgbd_tmap.depthCameraToCostMap(cv_image->image);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    } 
} 

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan){
    int count = scan->scan_time / scan->time_increment;
    // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        // ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "lrgbd_costmap");
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ROS_INFO_STREAM("\n-------------------------- start --------------------------\n");
    
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    std::string config_file = getOption<std::string>(pnh, "config_file", "");
    ROS_INFO_STREAM(config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    
    double fx = fsSettings["fx"];
    double fy = fsSettings["fy"];
    double cx = fsSettings["cx"];
    double cy = fsSettings["cy"]; 
    
    Eigen::Matrix3d camera_K; 
    camera_K << fx, 0, cx, fy, cy, 0, 0, 0, 1;
    // ROS_INFO_STREAM(camera_K(0,0) << " " << camera_K(0,2)<< " " << camera_K(1,0) << " " << camera_K(1,1));
    int image_height = fsSettings["image_height"];
    int image_width = fsSettings["image_width"]; 

    int depthScale = fsSettings["depthScale"];
    double resolution_size = fsSettings["resolution_size"];
    double map_width = fsSettings["map_width"];
    double map_height = fsSettings["map_height"];
    double robot_radius = fsSettings["robot_radius"];
    // bool display_costmap = fsSettings["display_costmap"];
    bool display_costmap = false;
    
    cv::Mat T_lidar2base, Tdepth2base;
    fsSettings["DEPTH_BASE"] >> Tdepth2base;
    fsSettings["LIDAR_BASE"] >> Tdepth2base; 
 
    lrgbd_tmap.init(camera_K, image_height, image_width, resolution_size, map_width, map_height, display_costmap, depthScale, Tdepth2base, Tdepth2base, robot_radius);
    
    std::string depth_topic = fsSettings["depth_topic"];
    std::string lidar_topic = fsSettings["lidar_topic"];
    std::string camera_info = fsSettings["camera_info"];
 
    ros::Subscriber sub_img = nh.subscribe(depth_topic, 100, img_callback);
    ros::Subscriber sub_laser = nh.subscribe(lidar_topic, 100, lidar_callback);
    
    

    ros::spin(); 
    ROS_INFO("shutting down!");
    ros::shutdown();
    return 0;
}
