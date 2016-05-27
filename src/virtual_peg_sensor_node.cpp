#include <ros/ros.h>

#include <optitrack_rviz/input.h>
#include <optitrack_rviz/listener.h>
#include <armadillo>

#include "optitrack_rviz/filter.h"

#include "peg_sensor/classifier/peg_classifier.h"
#include "peg_sensor/peg_world_wrapper/peg_world_wrapper.h"
#include "peg_sensor/peg_sensor_model/distance_model.h"
#include "optitrack_rviz/type_conversion.h"
#include "peg_sensor/peg_sensor_model/peg_model_visualise.h"
#include <std_msgs/Float64MultiArray.h>


/**
*       ===  Virtual Sensor node ===
*
*       Given a world model and peg model compute the distance to a set of features (edge, surface, socket)
*       and publish this as being what is sensed at the end-effector
*
**/

int main(int argc,char** argv){


    // -------------- Get node input paramters --------------

    std::map<std::string,std::string> input;
    input["-y_topic"]           = "";
    input["-fixed_frame"]       = "/world";
    input["-path_sensor_model"] = "";
    input["-peg_link_name"]     = "";
    input["-rate"]              = "100";
    input["-print"]             = "false";
    input["-socket_type"]       = "";

    if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    double      rate_hz               = boost::lexical_cast<double>(input["-rate"]);
    std::string sensor_publish_topic  = input["-y_topic"];
    std::string fixed_frame           = input["-fixed_frame"];
    std::string path_sensor_model     = input["-path_sensor_model"];
    std::string peg_link_name         = input["-peg_link_name"];
    std::string ssocket_type          = input["-socket_type"];

    SOCKET_TYPE socket_type;

    if(ssocket_type == "one"){
        socket_type = SOCKET_TYPE::ONE;
    }else if(ssocket_type == "two"){
        socket_type = SOCKET_TYPE::TWO;
    }else if(ssocket_type == "three"){
        socket_type = SOCKET_TYPE::THREE;
    }else{
        ROS_ERROR_STREAM("No such socket type defined: "  + ssocket_type);
        return 0;
    }






    // -------------- Initialise node --------------

    ros::init(argc, argv, "peg_sensor_classifier");
    ros::NodeHandle nh;

    Peg_world_wrapper peg_world_wrapper(nh,socket_type,true,"peg_sensor_classifier",path_sensor_model,fixed_frame,peg_link_name); // publish it
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
    arma::colvec Y;
    Y.resize(7);

    opti_rviz::Listener tf_listener(fixed_frame,peg_link_name);
    tf::Vector3     peg_origin;
    tf::Matrix3x3   peg_orientation;
    arma::colvec3   pos;
    arma::mat33     Rot;

    arma::fcolvec3 model_pt_surf,closest_surf,model_pt_edge,closest_edge;


    ros::Publisher pub_sensor = nh.advertise<std_msgs::Float64MultiArray>(sensor_publish_topic, 5);

    std_msgs::Float64MultiArray msg;
    msg.data.resize(13);

    arma::fcolvec3 b;

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

        b(0) = pos(0); b(1) = pos(1); b(2) = pos(2);

        peg_model_visualise.visualise(peg_sensor_mode->plat_dir,b);
        peg_model_visualise.visualise(closest_surf,model_pt_surf,closest_edge,model_pt_edge);

        {
            msg.data[psm::Contact_distance_model::C_SURF]           = Y(psm::Contact_distance_model::C_SURF);           // surface
            msg.data[psm::Contact_distance_model::C_EDGE_DIST]      = Y(psm::Contact_distance_model::C_EDGE_DIST);      // distance edge
            msg.data[psm::Contact_distance_model::C_EDGE_LEFT]      = Y(psm::Contact_distance_model::C_EDGE_LEFT);
            msg.data[psm::Contact_distance_model::C_EDGE_RIGHT]     = Y(psm::Contact_distance_model::C_EDGE_RIGHT);
            msg.data[psm::Contact_distance_model::C_EDGE_TOP]       = Y(psm::Contact_distance_model::C_EDGE_TOP);
            msg.data[psm::Contact_distance_model::C_EDGE_BOT]       = Y(psm::Contact_distance_model::C_EDGE_BOT);
            msg.data[psm::Contact_distance_model::C_RING]           = Y(psm::Contact_distance_model::C_RING);
            msg.data[psm::Contact_distance_model::C_S_HOLE]         = Y(psm::Contact_distance_model::C_S_HOLE);
            msg.data[psm::Contact_distance_model::C_SOCKET]         = Y(psm::Contact_distance_model::C_SOCKET);
            msg.data[psm::Contact_distance_model::C_EDGE_V1]        = Y(psm::Contact_distance_model::C_EDGE_V1);
            msg.data[psm::Contact_distance_model::C_EDGE_V2]        = Y(psm::Contact_distance_model::C_EDGE_V2);
            msg.data[psm::Contact_distance_model::C_EDGE_V3]        = Y(psm::Contact_distance_model::C_EDGE_V3);
            msg.data[psm::Contact_distance_model::C_RING_DIST]      = Y(psm::Contact_distance_model::C_RING_DIST);


        }

        pub_sensor.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
