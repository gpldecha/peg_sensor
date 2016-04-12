#include "peg_sensor/peg_sensor_model/distance_model.h"
#include <optitrack_rviz/type_conversion.h>

namespace psm {

Contact_distance_model::Contact_distance_model(Peg_sensor_model &peg_sensor_model)
    :peg_sensor_model(peg_sensor_model){
}

void Contact_distance_model::update(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot){
    //  ROS_INFO("Contact_distance_model::update this should not be called");
    //  std::cout<< " start Contact_distance_model::update" << std::endl;
    Y.resize(3);

    tf::Matrix3x3 R_tmp;
    tf::Vector3   T_tmp;
    opti_rviz::type_conv::mat2tf(Rot,R_tmp);
    opti_rviz::type_conv::vec2tf(pos,T_tmp);

    peg_sensor_model.update_model(T_tmp,R_tmp);
    peg_sensor_model.get_distance_features();

    Y(C_SURF)   = peg_sensor_model.get_distance_surface();
    Y(C_EDGE)   = peg_sensor_model.get_distance_edge();
    Y(C_SOCKET) = peg_sensor_model.is_inside_socket();
    if(peg_sensor_model.is_inside_box()){
        Y(C_SURF) = -1;
    }

}

void Contact_distance_model::update(arma::mat& hY,const arma::mat& points,const arma::mat33& Rot){
    assert(hY.n_rows == points.n_rows);
    Yone.resize(3);

    tf::Matrix3x3 R_tmp;
    tf::Vector3  T_tmp;
    opti_rviz::type_conv::mat2tf(Rot,R_tmp);

    for(std::size_t i = 0; i < points.n_rows;i++){
        opti_rviz::type_conv::vec2tf(points.row(i).st(),T_tmp);
        peg_sensor_model.update_model(T_tmp,R_tmp);
        peg_sensor_model.get_distance_features();

        Yone(C_SURF)   = peg_sensor_model.get_distance_surface();
        Yone(C_EDGE)   = peg_sensor_model.get_distance_edge();
        Yone(C_SOCKET) = peg_sensor_model.is_inside_socket();

        // only if point is inside a box and not inside the socket boxes
        if(peg_sensor_model.is_inside_box()){
            //    std::cout<< i << " is inside" << std::endl;
            Yone(C_SURF) = -1;
        }
        hY.row(i) = Yone.st();
    }

}


void Contact_distance_model::get_distance_single_point(arma::fcolvec3 &x){
    /*   distance_features.compute_surface_edge_vector(x);

    direction_surf = distance_features.point_surface - x;
    min_distance_surface = arma::norm(direction_surf);

    direction_edge = distance_features.point_edge - x;
    min_distance_edge = arma::norm(direction_edge);*/

}

void Contact_distance_model::get_distances(){

    /*  min_distance_surface = std::numeric_limits<float>::max();
    min_distance_edge    = std::numeric_limits<float>::max();
    std::size_t index_closet_point_s = -1;
    std::size_t index_closet_point_e = -1;

    isInSocket = true;

    for(std::size_t i = 0; i < 3;i++)
    {
        distance_features.compute_surface_edge_vector(model_points.row(i).st());


        direction_surf = distance_features.point_surface - model_points.row(i).st();
        current_distance_surface = arma::norm(direction_surf);

        direction_edge = distance_features.point_edge - model_points.row(i).st();
        current_distance_edge = arma::norm(direction_edge);

        if(current_distance_surface < min_distance_surface){
            min_distance_surface  =current_distance_surface;
            index_closet_point_s  = i;
            proj_points.row(0) = distance_features.point_surface.st();
        }

        if(current_distance_edge < min_distance_edge){
            min_distance_edge      =   current_distance_edge;
            index_closet_point_e   =   i;
            proj_points.row(1) = distance_features.point_edge.st();

        }

        isInSocket = isInSocket && is_inside_socket_box(model_points.row(i).st());


    }

    isInTable =   distance_features.bIsInside;

    direction_surf = proj_points.row(0).st() - model_points.row(index_closet_point_s).st();
    direction_edge = proj_points.row(1).st() - model_points.row(index_closet_point_e).st();

    if(b_visualise){


        opti_rviz::type_conv::vec2tf(model_points.row(index_closet_point_s).st(),dir_vectors[C_SURF].origin);
        opti_rviz::type_conv::vec2tf(direction_surf,dir_vectors[C_SURF].direction);

        opti_rviz::type_conv::vec2tf(model_points.row(index_closet_point_e).st(),dir_vectors[C_EDGE].origin);
        opti_rviz::type_conv::vec2tf(direction_edge,dir_vectors[C_EDGE].direction);
    }*/
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
