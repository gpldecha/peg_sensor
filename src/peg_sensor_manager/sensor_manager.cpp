#include "peg_sensor_manager/sensor_manager.h"


namespace psm{


Sensor_manager::Sensor_manager(ros::NodeHandle& nh)
{
    service_server = nh.advertiseService("sensor_manager_cmd",&Sensor_manager::sensor_manager_callback,this);
}

bool Sensor_manager::select_model(const std::string& name){
    it = peg_model_map.find(name);
    if(it == peg_model_map.end()){
        return false;
    }else{
        return true;
    }
}

void Sensor_manager::add(const std::string name,Base_peg_sensor_model* peg_sensor_model){
    peg_model_map[name] = peg_sensor_model;
}


void Sensor_manager::update_peg(arma::colvec& Y,const arma::colvec3& pos, const arma::mat33& Rot){
    if(it != peg_model_map.end()){
        (it->second)->update(Y,pos,Rot);
    }
}


void Sensor_manager::compute_hY(arma::mat& Y,const arma::mat& points, const arma::mat33& Rot){
    //ROS_INFO_STREAM_THROTTLE(1.0,"compute_hY");
    if(it != peg_model_map.end()){
        (it->second)->update(Y,points,Rot);
    }else{
        ROS_ERROR("it == peg_model_map.end() [sensor_manager.cpp]");
    }
}

bool Sensor_manager::sensor_manager_callback(peg_sensor::String_cmd::Request& req, peg_sensor::String_cmd::Response& res){
    return true;
}


}
