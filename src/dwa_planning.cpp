#include "costmap_lrgbd_ros/dwa_planning.h"

DWAPlanning::DWAPlanning(){

}


DWAPlanning::~DWAPlanning(void){

}

void DWAPlanning::init(std::string config_file_path){
    cv::FileStorage fsSettings(config_file_path, cv::FileStorage::READ);
    max_acc_x_ = fsSettings["max_acc_x"];
    max_speed_ = fsSettings["max_speed"];
    min_speed_ = fsSettings["min_speed"];
    foresee_ = fsSettings["foresee"]; 
    sim_time_ = fsSettings["sim_time"];
    region_foresee_ = fsSettings["region_foresee"];
    short_foresee_rate_ = fsSettings["short_foresee_rate"];
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
        
        waypoint_theta = (double)(atan2(2 * (q_x*q_y + q_w*q_z), 1 - 2*(q_y*q_y - q_z*q_z)) * 57.3);
		std::cout << waypoint_x << " " << waypoint_y << " " << waypoint_theta << std::endl;
        Eigen::Vector3d point_temp(waypoint_x, waypoint_y, waypoint_theta);
        waypoints_queue_.push(point_temp);
	}

	waypoint_file.close();
    std::cout << "\nConfigration the DWA parameters: " << std::endl << 
        "max_acc_x: " << max_acc_x_ << std::endl << 
        "max_speed: " << max_speed_ << std::endl << 
        "min_speed: " << foresee_ << std::endl << 
        "foresee: " << max_acc_x_ << std::endl << 
        "sim_time: " << sim_time_ << std::endl << 
        "region_foresee: " << region_foresee_ << std::endl << 
        "short_foresee_rate: " << short_foresee_rate_ << std::endl << 
        "sim_period: " << sim_period_ << std::endl << 
        "waypoints_file_path" << waypoints_file_path_ << std::endl
        << std::endl;
}

void DWAPlanning::getWayPoint(){
    if(waypoints_queue_.size() > 0){
        robot_costmap_waypoint_ = waypoints_queue_.front();
        waypoints_queue_.pop();
    }
    waypoints_queue_.size();
}

void DWAPlanning::setWayPointInCostmap(){
    
}

bool DWAPlanning::isArriveWayPoint(){

}

bool DWAPlanning::isArriveDestination(){
    return false;
}

bool DWAPlanning::dwa_control(const cv::Mat& config_map){
    return true;
}

void DWAPlanning::move(Eigen::Matrix4d robot_pose, const cv::Mat& config_map, double & go_v, double & turn_v){
    getWayPoint();
    if(isArriveDestination()){
        go_v = 0.0;
        turn_v = 0.0;
        return;
    }
    if(isArriveWayPoint()){
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

