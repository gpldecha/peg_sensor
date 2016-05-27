#ifndef PEG_MODEL_VISUALISE_H_
#define PEG_MODEL_VISUALISE_H_

#include <ros/ros.h>
#include <visualise/vis_vector.h>
#include <armadillo>

#include "peg_sensor_model.h"

class Peg_model_visualise{

public:

    Peg_model_visualise(ros::NodeHandle& nh, const std::string &frame_id, const std::string& topic_name);

    void visualise(const arma::fcolvec3& surf,const arma::fcolvec3& model_suf,const arma::fcolvec3 edge,const arma::fcolvec3& model_edge);

    void visualise(const arma::fcolvec3& direction, const arma::fcolvec3& pos);

private:

    opti_rviz::Vis_vectors              vis_vectors;
    std::vector<opti_rviz::Arrow>       arrows;
    tf::Vector3                         tmp_orign,tmp_direction;

};


#endif
