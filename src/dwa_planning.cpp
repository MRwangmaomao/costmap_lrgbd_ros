#include "costmap_lrgbd_ros/dwa_planning.h"

DWAPlanning::DWAPlanning(){

}


DWAPlanning::~DWAPlanning(void){

}

void DWAPlanning::init(std::string config_file_path, int depthscale, Eigen::Matrix3d camera_k, double map_width, double map_height, double resolution_size){
    depthScale_ = depthscale;
    camera_k_ = camera_k;
    map_width_ =map_width;
    map_height_ =map_height;
    resolution_size_ = resolution_size;
    waypoint_id_ = 0;
    first_flag_ = true;
    cv::FileStorage fsSettings(config_file_path, cv::FileStorage::READ);
    max_acc_x_ = fsSettings["max_acc_x"];
    max_speed_ = fsSettings["max_speed"];
    min_speed_ = fsSettings["min_speed"];
    foresee_ = fsSettings["foresee"]; 
    sim_time_ = fsSettings["sim_time"];
    region_foresee_ = fsSettings["region_foresee"];
    short_foresee_rate_ = fsSettings["short_foresee_rate"];
    distance_threshold_ = fsSettings["distance_threshold"];
    sim_period_ = fsSettings["sim_period"];
    fsSettings["waypoints_file"] >> waypoints_file_path_; 
    std::ifstream waypoint_file(waypoints_file_path_);
    std::cout << "\nWaypoint Lists:\n";
    std::string temp;
    double waypoint_x, waypoint_y, waypoint_theta;
    double q_x, q_y, q_z, q_w;
    std::string temp_w;
    while(getline(waypoint_file,temp)) //按行读取字符串 
	{
        std::stringstream input(temp);
        input>>temp_w;
        input>>temp_w; 
        waypoint_x = atof(temp_w.c_str());
        input>>temp_w; 
        waypoint_y = atof(temp_w.c_str());
        input>>temp_w;
        input>>temp_w;
        q_w = atof(temp_w.c_str());
        input>>temp_w;
        q_x = atof(temp_w.c_str());
        input>>temp_w;
        q_y = atof(temp_w.c_str());
        input>>temp_w;
        q_z = atof(temp_w.c_str());
        
        waypoint_theta = (double)(atan2(2 * (q_x*q_y + q_w*q_z), 1 - 2*(q_y*q_y - q_z*q_z)));
		std::cout << "  " << waypoint_x << " " << waypoint_y << " " << waypoint_theta << std::endl;
        Eigen::Vector3d point_temp(waypoint_x, waypoint_y, waypoint_theta);
        waypoints_queue_.push(point_temp);
	} 
	waypoint_file.close();
    getWayPoint();

    std::cout << "\nConfigration the DWA parameters: " << std::endl << 
        "   max_acc_x: " << max_acc_x_ << std::endl << 
        "   max_speed: " << max_speed_ << std::endl << 
        "   min_speed: " << foresee_ << std::endl << 
        "   foresee: " << max_acc_x_ << std::endl << 
        "   sim_time: " << sim_time_ << std::endl << 
        "   region_foresee: " << region_foresee_ << std::endl << 
        "   short_foresee_rate: " << short_foresee_rate_ << std::endl << 
        "   sim_period: " << sim_period_ << std::endl << 
        "   waypoints_file_path" << waypoints_file_path_ << std::endl
        << std::endl;
}

void DWAPlanning::getWayPoint(){
    if(waypoints_queue_.size() > 0){
        robot_waypoint_ = waypoints_queue_.front();
        waypoints_queue_.pop();
        waypoint_id_++;
        std::cout << "\n\n ------------------------------\nget a new points: " 
        << robot_waypoint_(0) << " " << robot_waypoint_(1) << std::endl;
    }
    waypoints_queue_.size();
}

void DWAPlanning::setWayPointInCostmap(){
    Eigen::Matrix4d T_world_waypoint;
    T_world_waypoint  << 0, 0, 0, 0, 
                         0, 0, 0, 0,
                         0, 0, 0, 0,
                         0, 0, 0, 1;
    // 初始化欧拉角(Z-Y-X，即RPY, 先绕x轴roll,再绕y轴pitch,最后绕z轴yaw)  
    T_world_waypoint(3,0) = robot_waypoint_(0);
    T_world_waypoint(3,1) = robot_waypoint_(1);
    
    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(robot_waypoint_(2), Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
 
    Eigen::Matrix4d T_robot_waypoint =  robot_pose_ * T_world_waypoint.inverse();
    robot_wapoint_in_costmap_(0) = T_robot_waypoint(3,0);
    robot_wapoint_in_costmap_(1) = T_robot_waypoint(3,1);
    // ZYX顺序，即先绕x轴roll,再绕y轴pitch,最后绕z轴yaw, 0表示X轴,1表示Y轴,2表示Z轴
    rotation_matrix3 = T_robot_waypoint.block(0,0,3,3);
    Eigen::Vector3d euler_angles = rotation_matrix3.eulerAngles(2, 1, 0); 
    robot_wapoint_in_costmap_(0) = robot_wapoint_in_costmap_(0)/resolution_size_ + (map_height_/resolution_size_)/2;
    robot_wapoint_in_costmap_(1) = robot_wapoint_in_costmap_(1)/resolution_size_ + (map_width_/resolution_size_)/2;
    std::cout << robot_wapoint_in_costmap_(0) << "      " << robot_wapoint_in_costmap_(1) << std::endl;
}

// 根据距离误差，后面还需要添加角度误差
bool DWAPlanning::isArriveWayPoint(){
    std::cout << "robot position: " << robot_pose_(3,0) << "  " << robot_pose_(3,1) << 
                 "robot destination: " << robot_waypoint_(0) << " " << robot_waypoint_(1) << "      " << waypoint_id_ << std::endl;
    double distance_err = sqrt((robot_pose_(3,0) - robot_waypoint_(0))*(robot_pose_(3,0) - robot_waypoint_(0)) +
    (robot_pose_(3,1) - robot_waypoint_(1))*(robot_pose_(3,1) - robot_waypoint_(1)));
    // std::cout << "distance_err is : " << distance_err << std::endl;
    if(distance_err < distance_threshold_){      
        return true;
    }
    return false;
}

bool DWAPlanning::isArriveDestination(){
    if(waypoints_queue_.size() == 0 && isArriveWayPoint())
    {
        return true;
    } 
    return false;
}

bool DWAPlanning::dwa_control(const cv::Mat& config_map){
    cv::Mat dwa_image_map = config_map.clone();
    std::cout << robot_wapoint_in_costmap_(0) << " " <<  robot_wapoint_in_costmap_(1) << std::endl;
    int cross_line_size = 10;
    cv::line(dwa_image_map, cvPoint(robot_wapoint_in_costmap_(0)-cross_line_size/2,robot_wapoint_in_costmap_(1)), cvPoint(robot_wapoint_in_costmap_(0)+cross_line_size/2, robot_wapoint_in_costmap_(1)), cv::Scalar(255,165,0), 1, 8, 0); 
	cv::line(dwa_image_map, cvPoint(robot_wapoint_in_costmap_(0),robot_wapoint_in_costmap_(1)-cross_line_size/2), cvPoint(robot_wapoint_in_costmap_(0), robot_wapoint_in_costmap_(1)+cross_line_size/2), cv::Scalar(255,165,0), 1, 8, 0);

    cv::namedWindow("dwa");
    cv::imshow("dwa",dwa_image_map);
    cvWaitKey(1);

    return true;
}

void DWAPlanning::move(Eigen::Matrix4d robot_pose, const cv::Mat& config_map, double & go_v, double & turn_v){
    robot_pose_ = robot_pose;
    if(first_flag_ == true){
        first_flag_ = false;
        setWayPointInCostmap();
    }
    if(isArriveDestination() == true){
        go_v = 0.0;
        turn_v = 0.0;
        std::cout << "\n\nI have arrived !!!\n\n" << std::endl;
        return;
    }
    if(isArriveWayPoint() == true){
        getWayPoint();
        setWayPointInCostmap();
    }

    if(!dwa_control(config_map))
        std::cerr << "DWA Plan is failed!!\n";

    go_v = go_v_;
    turn_v = turn_v_;
    go_v_ = 0.0;
    turn_v_ = 0.0;
}

