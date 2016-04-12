#include <ros/ros.h>

#include <optitrack_rviz/input.h>
#include <optitrack_rviz/listener.h>
#include <netft_rdt_driver/ft_listener.h>
#include <armadillo>


#include "optitrack_rviz/filter.h"

#include "peg_sensor/classifier/peg_classifier.h"
#include "peg_sensor/peg_world_wrapper/peg_world_wrapper.h"
#include "peg_sensor/peg_sensor_model/distance_model.h"
#include "optitrack_rviz/type_conversion.h"
#include "peg_sensor/peg_sensor_model/peg_model_visualise.h"



/**
*       ===  Force Torque sensor node ===
*
*      o Node subscribes to the force-torque sensor, filters the noise and removes the bias
*        and republished the filtered and bias free force-torque signal.
*
*      o Publishes also a classficiation result (contact/no contact, left contact, right contact)
*        based on the filtered-force torque sensor
*
**/

int main(int argc,char** argv){


    // -------------- Get node input paramters --------------

    std::map<std::string,std::string> input;
    input["-y_topic"]           = "";
    input["-ft_topic"]          = "";
    input["-fixed_frame"]       = "/world";
    input["-path_sensor_model"] = "";
    input["-peg_link_name"]     = "";
    input["-rate"]              = "100";
    input["-print"]             = "false";

    if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    double      rate_hz               = boost::lexical_cast<double>(input["-rate"]);
    std::string sensor_publish_topic  = input["-y_topic"];
    std::string sensor_listener_topic = input["-ft_topic"];
    std::string fixed_frame           = input["-fixed_frame"];
    std::string path_sensor_model     = input["-path_sensor_model"];
    std::string peg_link_name         = input["-peg_link_name"];

    // -------------- Initialise node --------------

    ros::init(argc, argv, "peg_sensor_classifier");
    ros::NodeHandle nh;

    Peg_world_wrapper peg_world_wrapper(nh,"peg_sensor_classifier",path_sensor_model,fixed_frame,peg_link_name);
    Peg_sensor_model* const peg_sensor_mode = peg_world_wrapper.peg_sensor_model.get();

    psm::Contact_distance_model contact_distance_model(*(peg_world_wrapper.peg_sensor_model.get()));

    psm::Sensor_manager sensor_manager(nh);
    sensor_manager.add("contact",&contact_distance_model);

    if(!sensor_manager.select_model("contact")){
        std::cout<< "FAILED to select_model" << std::endl;
        exit(0);
    }

    /// Peg Arrow visualiser
    Peg_model_visualise peg_model_visualise(nh,fixed_frame,"peg_model_arrows");


    ros::Rate rate(rate_hz);
    arma::colvec3 Y;

    opti_rviz::Listener tf_listener(fixed_frame,peg_link_name);
    tf::Vector3     peg_origin;
    tf::Matrix3x3   peg_orientation;
    arma::colvec3   pos;
    arma::mat33     Rot;

    arma::fcolvec3 model_pt_surf,closest_surf,model_pt_edge,closest_edge;


    while(nh.ok()){

        tf_listener.update(peg_origin,peg_orientation);
        opti_rviz::type_conv::tf2vec(peg_origin,pos);
        opti_rviz::type_conv::tf2mat(peg_orientation,Rot);

        peg_world_wrapper.update();
        sensor_manager.update_peg(Y,pos,Rot);

        opti_rviz::type_conv::tf2vec(peg_sensor_mode->get_model()[peg_sensor_mode->contact_info[SURFACE].index],model_pt_surf);
        closest_surf = peg_sensor_mode->contact_info[SURFACE].closest_point;

        opti_rviz::type_conv::tf2vec(peg_sensor_mode->get_model()[peg_sensor_mode->contact_info[EDGE].index],model_pt_edge);
        closest_edge  = peg_sensor_mode->contact_info[EDGE].closest_point;

        peg_model_visualise.visualise(closest_surf,model_pt_surf,closest_edge,model_pt_edge);

        //ROS_INFO_STREAM_THROTTLE(0.5,"Y: " << Y(0) << " " << Y(1) << " " << Y(2));

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
