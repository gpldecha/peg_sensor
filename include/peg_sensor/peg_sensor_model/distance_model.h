#ifndef DISTANCE_MODEL_H_
#define DISTANCE_MODEL_H_


#include "wrapobject.h"
#include <visualise/vis_vector.h>
#include <objects/socket_one.h>
#include "peg_sensor/peg_sensor_model/peg_sensor_model.h"
#include "peg_sensor/peg_sensor_model/base_peg_sensor_model.h"

namespace psm{

class Contact_distance_model : public Base_peg_sensor_model {

public:

    typedef enum {C_SURF=0,C_EDGE=1,C_SOCKET=2} contact_types;

public:

    Contact_distance_model(Peg_sensor_model& peg_sensor_model);

    virtual void update(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot);

    /**
     * @brief update    : compute the expected sensation given positions and known orientation
     * @param hY        : filled in expected sensed values (what is returned)
     * @param points    : particles, set of possible positions of the end-effector
     * @param Rot       : known orientation of the particle
     */
    virtual void update(arma::mat& hY, const arma::mat& points, const arma::mat33& Rot);

    virtual void initialise_vision(ros::NodeHandle& node);

    virtual void visualise();

protected:

    void get_distances();

    void get_distance_single_point(arma::fcolvec3 &x);

    inline double gaussian_function(const double x, const double var=1.0){
        return exp(-(1.0 / (2.0 * var)) * (x * x));
    }

private:

    double                              min_half_one_div_var;
    double                              dist_variance;

protected:

    Peg_sensor_model                    &peg_sensor_model;

    arma::fcolvec3                      tmp;
    arma::colvec                        Yone;
};


class Fast_contact_distance_model : public Base_peg_sensor_model{

public:

    Fast_contact_distance_model(const std::string& path_to_peg_model,const std::string& fixed_frame);

    virtual void update(arma::colvec &Y,const arma::colvec3& pos,const arma::mat33& Rot);

    /**
     * @brief update    : compute the expected sensation given positions and known orientation
     * @param hY        : filled in expected sensed values (what is returned)
     * @param points    : particles, set of possible positions of the end-effector
     * @param Rot       : known orientation of the particle
     */
    virtual void update(arma::mat& hY, const arma::mat& points, const arma::mat33& Rot);

    virtual void initialise_vision(ros::NodeHandle& node);

    virtual void visualise();


private:

    std::vector<tf::Vector3>        model,model_TF;
    std::vector<Contact_points>     contact_info;
    tf::Vector3                     position;
    tf::Matrix3x3                   orientation, R_tmp;
    arma::fcolvec3                  tmp_vec3f;
    tf::Vector3                     tmp_Vec3;
};

}

#endif
