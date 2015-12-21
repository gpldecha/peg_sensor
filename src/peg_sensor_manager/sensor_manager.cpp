#include "peg_sensor_manager/sensor_manager.h"


namespace psm{


Sensor_manager::Sensor_manager(ros::NodeHandle& nh, wobj::WrapObject& wrapped_objects, obj::Socket_one &socket_one, const std::string& model_path):
wrapped_objects(wrapped_objects),socket_one(socket_one)
{
    t_sensor       = psm::MODEL;
    service_server = nh.advertiseService("sensor_manager_cmd",&Sensor_manager::sensor_manager_callback,this);
    initialise(model_path);
}

void Sensor_manager::initialise(const std::string& model_path){
        sptr_cdist           = Sptr_cdist( new psm::Contact_distance_model(wrapped_objects,socket_one,model_path) );

        ptr_sensor_force_idd = Sptr_fii( new psm::Force_iid_model(psm::SIMPLE));

        //sptr_three_dist      = Sptr_three_dist( new psm::Three_pin_distance_model(wrapped_objects));
       // ptr_sensor_force_idd = Sptr_fii(new psm::Force_iid_model(SIMPLE));
}



void Sensor_manager::update_peg(arma::colvec& Y,const arma::colvec3& pos, const arma::mat33& Rot){   
    sptr_cdist->update(Y,pos,Rot);
}

void Sensor_manager::update_peg(arma::colvec &Y, const arma::colvec3& force, const arma::colvec3& torque){
    ptr_sensor_force_idd->update(Y,force,torque);
}


void Sensor_manager::update_particles(arma::mat& Y,const arma::mat& points, const arma::mat33& Rot){
    sptr_cdist->update(Y,points,Rot);
}


bool Sensor_manager::sensor_manager_callback(peg_sensor::String_cmd::Request& req, peg_sensor::String_cmd::Response& res){

    std::string cmd = req.req;
    if(cmd == "model"){
         t_sensor = MODEL;
         res.res  = "sensor type: MODEL!";
         return true;
    }else if(cmd == "ft"){
         t_sensor = FT;
         res.res  = "sensor type: FT!";
         return true;
    }else{
        res.res = "no such cmd: " + cmd + " !";
        return  false;
    }
}


}
