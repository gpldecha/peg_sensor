#include "peg_sensor/peg_sensor_model/box_sensor_model.h"
#include <optitrack_rviz/type_conversion.h>

namespace psm{

Box_sensor_likelihood::Box_sensor_likelihood(Peg_sensor_model& peg_sensor_model)
    :peg_sensor_model(peg_sensor_model){

   wobj::WrapObject& wrapped_objects = peg_sensor_model.get_wrapped_objects();

   socket_top_edge      = &wrapped_objects.get_wbox("socket_top_edge");
   socket_bottom_edge   = &wrapped_objects.get_wbox("socket_bottom_edge");
   socket_left_edge     = &wrapped_objects.get_wbox("socket_left_edge");
   socket_right_edge    = &wrapped_objects.get_wbox("socket_right_edge");

}

void Box_sensor_likelihood::update(double* L, const arma::colvec& Y,const arma::colvec3& F, const arma::mat& points,const arma::mat33& Rot)
{


    tf::Matrix3x3   R_tmp;
    tf::Vector3     T_tmp;
    opti_rviz::type_conv::mat2tf(Rot,R_tmp);

    for(std::size_t p = 0; p < points.n_rows;p++){

        opti_rviz::type_conv::vec2tf(points.row(p).st(),T_tmp);
        peg_sensor_model.update_model(T_tmp,R_tmp);

        const std::vector<tf::Vector3>&  model = peg_sensor_model.get_model();



        std::size_t i = 0;
        // iterate over the three pegs (in frame of reference of peg_link)
        for(; i < model.size();i++)
        {

            tmp_vec3f(0) = model[i][0];
            tmp_vec3f(1) = model[i][1];
            tmp_vec3f(2) = model[i][2];



            // check if is in a box



        }





    }



}



}
