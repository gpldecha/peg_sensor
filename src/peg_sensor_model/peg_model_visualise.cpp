#include "peg_sensor/peg_sensor_model/peg_model_visualise.h"
#include <optitrack_rviz/type_conversion.h>

Peg_model_visualise::Peg_model_visualise(ros::NodeHandle& nh,const std::string& frame_id, const std::string& topic_name)
    :vis_vectors(nh,topic_name)

{
    arrows.resize(2);



    arrows[0] = opti_rviz::Arrow(tmp_orign,tmp_direction,"edge");
    arrows[0].set_rgba(0, 0, 1, 1);
    arrows[0].set_scale(0.005, 0.005, 0.005);

    arrows[1] = opti_rviz::Arrow(tmp_orign,tmp_direction,"surface");
    arrows[1].set_rgba(1, 0, 0, 1);
    arrows[1].set_scale(0.005, 0.005, 0.005);

    vis_vectors.initialise(frame_id,arrows);

}


void Peg_model_visualise::visualise(const arma::fcolvec3&surf, const arma::fcolvec3 &model_suf, const arma::fcolvec3 edge, const arma::fcolvec3 &model_edge){

    opti_rviz::type_conv::vec2tf(model_edge,tmp_orign);
    opti_rviz::type_conv::vec2tf(edge - model_edge,tmp_direction);
    arrows[0].set_pos_dir(tmp_orign,tmp_direction);

    opti_rviz::type_conv::vec2tf(model_suf,tmp_orign);
    opti_rviz::type_conv::vec2tf(surf - model_suf,tmp_direction);
    arrows[1].set_pos_dir(tmp_orign,tmp_direction);
    vis_vectors.update(arrows);
    vis_vectors.publish();
}
