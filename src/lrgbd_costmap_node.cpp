
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
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include "costmap_lrgbd_ros/lrgbd2xz.h"
#include "costmap_lrgbd_ros/dwa_planning.h"

#define RAD2DEG(x) ((x)*180./M_PI) 

LRGBDCostMap lrgbd_tmap;
DWAPlanning dwa_planer;
Eigen::Matrix4d robot_pose;
ros::Publisher speed_pub;

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
        double go_v = 0.0, turn_v = 0.0;
        geometry_msgs::Twist pub_speed;
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(depth);
        lrgbd_tmap.depthCameraToCostMap(cv_image->image);
        dwa_planer.move(robot_pose, lrgbd_tmap.config_map_, go_v, turn_v);
        pub_speed.linear.x = go_v;
        pub_speed.angular.z = turn_v;
        speed_pub.publish(pub_speed);
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

void robot_pose_callback(const tf2_msgs::TFMessage::ConstPtr msg){ 
    robot_pose(3,0) = msg->transforms.at(0).transform.translation.x;
    robot_pose(3,1) = msg->transforms.at(0).transform.translation.y;
    robot_pose(3,2) = msg->transforms.at(0).transform.translation.z; 
    Eigen::Quaterniond robot_Q(msg->transforms.at(0).transform.rotation.w,msg->transforms.at(0).transform.rotation.x,
                            msg->transforms.at(0).transform.rotation.y,msg->transforms.at(0).transform.rotation.z);
    robot_pose.block(0,0,3,3) << robot_Q.toRotationMatrix();
}



int main(int argc, char **argv){
    ros::init(argc, argv, "lrgbd_costmap");
    ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::cout << "\n-------------------------- start --------------------------\n" << std::endl;
    
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    std::string config_file;
    std::string dwa_file;
    robot_pose << 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1.0;
    if(argc >=2){
        config_file = argv[1]; 
    }
    else{
        std::cout<<"args too small.\n";
        exit(0);
    } 
    
    ROS_INFO_STREAM("Setting file path is: " << config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
     
    double fx = fsSettings["fx"];
    double fy = fsSettings["fy"];
    double cx = fsSettings["cx"];
    double cy = fsSettings["cy"]; 
    
    Eigen::Matrix3d camera_K; 
    camera_K << fx, 0, cx, fy, cy, 0, 0, 0, 1; 
    int image_height = fsSettings["image_height"];
    int image_width = fsSettings["image_width"]; 
    fsSettings["dwa_file"] >> dwa_file;
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
    dwa_planer.init(dwa_file, depthScale, camera_K, map_width, map_height, resolution_size);

    std::string depth_topic = fsSettings["depth_topic"];
    std::string lidar_topic = fsSettings["lidar_topic"];
    std::string robot_pose_topic = fsSettings["robot_pose_topic"];
    std::string camera_info = fsSettings["camera_info"];
 
    ros::Subscriber sub_img = nh.subscribe(depth_topic, 100, img_callback);
    ros::Subscriber sub_laser = nh.subscribe(lidar_topic, 100, lidar_callback);
    ros::Subscriber sub_robot_pose = nh.subscribe(robot_pose_topic, 100, robot_pose_callback);
    speed_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::spin();
    ROS_INFO("shutting down!");
    ros::shutdown();
    return 0;
}
