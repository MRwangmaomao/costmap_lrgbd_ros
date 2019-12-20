
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
#include <sensor_msgs/image_encodings.h> 
#include <tf/transform_broadcaster.h> 
#include <geometry_msgs/Pose.h>

#include "costmap_lrgbd_ros/lrgbd2xz.h"

std::queue<cv::Mat> img_buf;

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
        ROS_INFO_STREAM("\nI get image");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
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
    double fx = getOption<double>(pnh, "fx", 0.0);
    double fy = getOption<double>(pnh, "fy", 0.0);
    double cx = getOption<double>(pnh, "cx", 0.0);
    double cy = getOption<double>(pnh, "cy", 0.0); 
    Eigen::Matrix3d camera_K;
    camera_K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    double resolution_size = getOption<double>(pnh, "resolution_size", 0.0);
    double map_width = getOption<double>(pnh, "map_width", 0.0);
    double map_height = getOption<double>(pnh, "map_height", 0.0);
    bool display_costmap = getOption<bool>(pnh, "display_costmap", true);

    std::string depth_topic = getOption<std::string>(pnh, "depth_topic", "");
    std::string lidar_topic = getOption<std::string>(pnh, "lidar_topic", "");
    std::string camera_info = getOption<std::string>(pnh, "camera_info_topic", ""); 
    ros::Subscriber sub_img = nh.subscribe(depth_topic, 100, img_callback);
    
    ros::spin(); 
    ROS_INFO("shutting down!");
    ros::shutdown();
    return 0;
}
