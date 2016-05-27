#include "peg_sensor/peg_sensor_model/peg_sensor_model.h"
#include <armadillo>
#include <limits>
#include <cmath>
#include <optitrack_rviz/type_conversion.h>
#include <optitrack_rviz/debug.h>

Peg_sensor_model::Peg_sensor_model(const std::string& path_to_peg_model,
                                   const std::string& fixed_frame,
                                   const std::string &target_frame,
                                   wobj::WrapObject& wrap_object)
    :wrapped_world(wrap_object),
      tf_listener(fixed_frame,target_frame),
      wall_box(             wrapped_world.get_wbox("link_wall")             ),
      socket_box(           wrapped_world.get_wbox("wbox_socket")           ),
      box_h1(               wrapped_world.get_wbox("box_hole_1")            ),
      box_h2(               wrapped_world.get_wbox("box_hole_2")            ),
      box_h3(               wrapped_world.get_wbox("box_hole_3")            ),
      socket_top_edge(      wrapped_world.get_wbox("socket_top_edge")       ),
      socket_bottom_edge(   wrapped_world.get_wbox("socket_bottom_edge")    ),
      socket_left_edge(     wrapped_world.get_wbox("socket_left_edge")      ),
      socket_right_edge(    wrapped_world.get_wbox("socket_right_edge")     ),
      wsocket(wrapped_world.wsocket)
{

    arma::mat points;
    if(!points.load(path_to_peg_model)){
        std::cerr<< "Peg_sensor_vis::Peg_sensor_vis failed to load file: " + path_to_peg_model << std::endl;
    }else{
        // Three points
        model.resize(points.n_rows);
        contact_info.resize(2);
        // Model of the position of the tree pegs in frame (0,0,0)
        model_TF.resize(points.n_rows);
        for(std::size_t r = 0; r < points.n_rows;r++){
            model_TF[r].setValue(points(r,0),points(r,1),points(r,2));
        }
    }
    contact_info[SURFACE] = Contact_points(SURFACE);
    contact_info[EDGE]    = Contact_points(EDGE);
    R_tmp.setRPY(0,-M_PI/2,M_PI);
}

void Peg_sensor_model::update(){
    // Gets position and orientation of the peg in Frame of World
    tf_listener.update(position,orientation);
    update_model(position,orientation);
    get_distance_features();
}

void Peg_sensor_model::get_distance_features(){

    min_distance_edge       = std::numeric_limits<float>::max();
    min_distance_surface    = std::numeric_limits<float>::max();
    min_distance_ring       = std::numeric_limits<float>::max();
    min_distance_s_hole     = std::numeric_limits<float>::max();

    isInTable               = false;
    isInSocket              = true;
    isIntSocketBOX          = false;

    isInSock_Top_Edge       = false;
    isInSock_Bot_Edge       = false;
    isInSock_Right_Edge     = false;
    isInSock_Left_Edge      = false;

    const wobj::distances& w_dist = wall_box.get_distances();
    const wobj::distances& s_dist = socket_box.get_distances();



    std::size_t i = 0;
    // iterate over the three pegs (in frame of reference of peg_link)
    for(; i < model.size();i++)
    {

        tmp_vec3f(0) = model[i][0];
        tmp_vec3f(1) = model[i][1];
        tmp_vec3f(2) = model[i][2];

         wall_box.distance_to_features(tmp_vec3f);
         socket_box.distance_to_features(tmp_vec3f);
         wsocket.distance_to_features(tmp_vec3f);
         box_h1.distance_to_features(tmp_vec3f);
         box_h2.distance_to_features(tmp_vec3f);
         box_h3.distance_to_features(tmp_vec3f);

         /// CLOSEST DISTANCE TO SURFACE BOX
         if(w_dist.min_s < s_dist.min_s)
         {
             if( w_dist.min_s < min_distance_surface)
             {
                 min_distance_surface                   = w_dist.min_s;
                 contact_info[SURFACE].index            = i;
                 contact_info[SURFACE].closest_point    = wall_box.get_surface_projection();
             }
         }else{
             if( s_dist.min_s < min_distance_surface)
             {
                 min_distance_surface                   = s_dist.min_s;
                 contact_info[SURFACE].index            = i;
                 contact_info[SURFACE].closest_point    = socket_box.get_surface_projection();
             }
         }


       // tf_debuf( box_wall.get_surface_projection(tmp_vec3f,0),"proj_p");
         /// CLOSEST DISTANCE TO EDGE BOX
        if(w_dist.min_e < s_dist.min_e)
        {
            if(w_dist.min_e < min_distance_edge){
                min_distance_edge                  = w_dist.min_e;
                contact_info[EDGE].index           = i;
                contact_info[EDGE].closest_point   = wall_box.get_edge_projection();
            }
        }else{
            if(s_dist.min_e < min_distance_edge){
                min_distance_edge                  = s_dist.min_e;
                contact_info[EDGE].index           = i;
                contact_info[EDGE].closest_point   = socket_box.get_edge_projection();
            }
        }

        // now check if in fact closet to Ring of socket
        if( wsocket.dist_edge < min_distance_edge){
            min_distance_edge = wsocket.dist_edge;
            contact_info[EDGE].index           = i;
            contact_info[EDGE].closest_point   = wsocket.get_edge_projection();
        }


        wsocket.dist_edge_ring;

        contact_info[SURFACE].distance         = min_distance_surface;
        contact_info[EDGE].distance            = min_distance_edge;

        isInSocket = isInSocket && (box_h1.is_inside() || box_h2.is_inside() || box_h3.is_inside());


        /// check if point is inside a socket edge bounding box
        if(socket_top_edge.is_inside()){
            isInSock_Top_Edge = true;
        }
        if(socket_bottom_edge.is_inside()){
            isInSock_Bot_Edge = true;
        }
        if(socket_right_edge.is_inside()){
            isInSock_Right_Edge = true;
        }
        if(socket_left_edge.is_inside()){
            isInSock_Left_Edge = true;
        }


         if( arma::norm(tmp_vec3f - wsocket.plate_edge_proj) < min_distance_ring)
         {
            min_distance_ring = arma::norm(tmp_vec3f - wsocket.plate.K);
         }

         if(wsocket.dist_edge_hole < min_distance_s_hole)
         {
             min_distance_s_hole = wsocket.dist_edge_hole;
         }

        if(socket_box.is_inside()){
            isIntSocketBOX=true;
        }
        if(wall_box.is_inside()){
            isInTable=true;
        }
    }


    socket_top_edge.distance_to_features(arma_position);
    socket_bottom_edge.distance_to_features(arma_position);
    socket_left_edge.distance_to_features(arma_position);
    socket_right_edge.distance_to_features(arma_position);

    wsocket.distance_to_features(arma_position);

    plat_dir = wsocket.plate.C - wsocket.get_surface_projection();



    opti_rviz::type_conv::tf2vec(model[contact_info[EDGE].index],tmp_vec3f);
    edge_dir = (contact_info[EDGE].closest_point -  tmp_vec3f);

    if(isInSocket){
        isInTable      = false;
        isIntSocketBOX = false;
    }

}

double Peg_sensor_model::get_distance_surface() const{
    return min_distance_surface;
}

double Peg_sensor_model::get_distance_edge() const{
    return min_distance_edge;
}

bool Peg_sensor_model::is_inside_box() const{
    return isInTable || isIntSocketBOX;
}

bool Peg_sensor_model::is_inside_socket() const{
    return isInSocket;
}

wobj::WrapObject& Peg_sensor_model::get_wrapped_objects(){
    return wrapped_world;
}

void Peg_sensor_model::update_model(const tf::Vector3& T,const tf::Matrix3x3& R){

    opti_rviz::type_conv::tf2vec(T,arma_position);

    for(std::size_t i = 0; i < model.size();i++){
        // transform the peg model to the frame of reference of peg_link
        model[i]  = (R * R_tmp * model_TF[i]) + T;
    }
}

const std::vector<tf::Vector3> &Peg_sensor_model::get_model(){
    return model;
}

const arma::fcolvec3 &Peg_sensor_model::get_closet_point(contact_type c_type){
   return contact_info[c_type].closest_point;
}


