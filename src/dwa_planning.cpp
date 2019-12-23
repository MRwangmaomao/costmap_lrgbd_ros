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

}

void DWAPlanning::setWayPointInCostmap(){
    
}

bool DWAPlanning::isArriveWayPoint(){

}

bool DWAPlanning::isArriveDestination(){

}