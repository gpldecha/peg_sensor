#ifndef PEG_SENSOR_DISTANCE_MODEL_H_
#define PEG_SENSOR_DISTANCE_MODEL_H_

#include <ros/ros.h>
#include <visualise/vis_points.h>

#include "wrapobject.h"
#include "peg_sensor/peg_sensor_model/base_peg_sensor_model.h"


#include <armadillo>


namespace psm{

class Peg_distance_model : public Base_peg_sensor_model{

public:

    Peg_distance_model(wobj::WrapObject &wrap_object, const std::string &model_path);

    virtual void update(arma::colvec& Y,const arma::colvec3& pos,const arma::mat33& Rot) = 0;

    virtual void update(arma::mat& Y,const arma::mat& points,const arma::mat33& Rot) = 0;

    void print(const arma::colvec& Y) const;

    virtual void initialise_vision(ros::NodeHandle& node);

    virtual void visualise();

protected:

    void update_model(const arma::colvec3& X, const arma::mat33& R);

protected:

    wobj::WrapObject&   wrap_object;
    arma::mat           model_TF;
    arma::colvec3       plug_point;
    std::size_t         num_model_points;
    arma::fmat          model_points;
    std::shared_ptr<opti_rviz::Vis_points>      ptr_vis_points;
    bool                                        b_visualise;

};

}


#endif
