#include "peg_sensor/peg_sensor_model/distance_model.h"
#include <optitrack_rviz/type_conversion.h>
#include <ros/ros.h>

namespace psm {

Contact_distance_model::Contact_distance_model(Peg_sensor_model &peg_sensor_model)
    :peg_sensor_model(peg_sensor_model){
}

void Contact_distance_model::update(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot){
    //  ROS_INFO("Contact_distance_model::update this should not be called");
    //  std::cout<< " start Contact_distance_model::update" << std::endl;
    Y.resize(13);

    tf::Matrix3x3 R_tmp;
    tf::Vector3   T_tmp;
    opti_rviz::type_conv::mat2tf(Rot,R_tmp);
    opti_rviz::type_conv::vec2tf(pos,T_tmp);

    peg_sensor_model.update_model(T_tmp,R_tmp);
    peg_sensor_model.get_distance_features();

    Y(C_SURF)        = peg_sensor_model.get_distance_surface();

    Y(C_EDGE_LEFT)   = static_cast<int>(peg_sensor_model.isInSock_Left_Edge);
    Y(C_EDGE_RIGHT)  = static_cast<int>(peg_sensor_model.isInSock_Right_Edge);
    Y(C_EDGE_BOT)    = static_cast<int>(peg_sensor_model.isInSock_Bot_Edge);
    Y(C_EDGE_TOP)    = static_cast<int>(peg_sensor_model.isInSock_Top_Edge);

    Y(C_EDGE_DIST)   = peg_sensor_model.get_distance_edge();
    Y(C_RING)        = peg_sensor_model.min_distance_ring;
    Y(C_S_HOLE)      = peg_sensor_model.min_distance_s_hole;
    Y(C_SOCKET)      = peg_sensor_model.is_inside_socket();


    Y(C_EDGE_V1)    = peg_sensor_model.edge_dir(0);
    Y(C_EDGE_V2)    = peg_sensor_model.edge_dir(1);
    Y(C_EDGE_V3)    = peg_sensor_model.edge_dir(2);

    Y(C_RING_DIST)  = peg_sensor_model.min_distance_ring;

    ROS_INFO_STREAM_THROTTLE(1.0,"RING: " << Y(C_RING_DIST));


    if(peg_sensor_model.is_inside_box()){
        Y(C_SURF) = -1;
    }

}

void Contact_distance_model::update(arma::mat& hY,const arma::mat& points,const arma::mat33& Rot){

    // This function should not be used anymore.
    /*
    assert(hY.n_rows == points.n_rows);
    Yone.resize(12);

    tf::Matrix3x3 R_tmp;
    tf::Vector3  T_tmp;
    opti_rviz::type_conv::mat2tf(Rot,R_tmp);

    for(std::size_t i = 0; i < points.n_rows;i++){
        opti_rviz::type_conv::vec2tf(points.row(i).st(),T_tmp);
        peg_sensor_model.update_model(T_tmp,R_tmp);
        peg_sensor_model.get_distance_features();

        Yone(C_SURF)            = peg_sensor_model.get_distance_surface();

        Yone(C_EDGE_LEFT)       = static_cast<int>(peg_sensor_model.isInSock_Left_Edge);
        Yone(C_EDGE_RIGHT)      = static_cast<int>(peg_sensor_model.isInSock_Right_Edge);
        Yone(C_EDGE_BOT)        = static_cast<int>(peg_sensor_model.isInSock_Bot_Edge);
        Yone(C_EDGE_TOP)        = static_cast<int>(peg_sensor_model.isInSock_Top_Edge);

        Yone(C_S_HOLE)          = peg_sensor_model.min_distance_s_hole;
        Yone(C_SOCKET)          = peg_sensor_model.is_inside_socket();


        Yone(C_EDGE_DIST)       = peg_sensor_model.get_distance_edge();
        Yone(C_SOCKET)          = peg_sensor_model.is_inside_socket();

        Yone(C_EDGE_V1)         = peg_sensor_model.edge_dir(0);
        Yone(C_EDGE_V2)         = peg_sensor_model.edge_dir(1);
        Yone(C_EDGE_V3)         = peg_sensor_model.edge_dir(2);

        if(peg_sensor_model.is_inside_box()){
            Yone(C_SURF) = -1;
        }

        hY.row(i) = Yone.st();
    }
*/
}

void Contact_distance_model::initialise_vision(ros::NodeHandle& node){}

void Contact_distance_model::visualise(){}


Fast_contact_distance_model::Fast_contact_distance_model(const std::string& path_to_peg_model,const std::string& fixed_frame){
    {
        arma::mat points;
        if(!points.load(path_to_peg_model)){
            std::cerr<< "Peg_sensor_vis::Peg_sensor_vis failed to load file: " + path_to_peg_model << std::endl;
        }else{

            model.resize(points.n_rows);
            model_TF.resize(points.n_rows);
            for(std::size_t r = 0; r < points.n_rows;r++){
                model_TF[r].setValue(points(r,0),points(r,1),points(r,2));
            }

        }
        R_tmp.setRPY(0,-M_PI/2,M_PI);
    }
}

void Fast_contact_distance_model::update(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot){





}

/**
 * @brief update    : compute the expected sensation given positions and known orientation
 * @param hY        : filled in expected sensed values (what is returned)
 * @param points    : particles, set of possible positions of the end-effector
 * @param Rot       : known orientation of the particle
 */
void Fast_contact_distance_model::update(arma::mat& hY, const arma::mat& points, const arma::mat33& Rot){

        for(std::size_t i = 0; i < points.n_rows;i++){


                // if in inside the box or not

                //  distance to edge

                //  distance to corner


        }

}

void Fast_contact_distance_model::initialise_vision(ros::NodeHandle& node){

}

void Fast_contact_distance_model::visualise(){

}





}
