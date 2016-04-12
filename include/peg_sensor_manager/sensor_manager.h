#ifndef PEG_SENSOR_MANAGER_H_
#define PEG_SENSOR_MANAGER_H_

///
/// \brief The Sensor_manager class
/// Managers a set of senor functions which populates a multivariate sensor vector Y


#include <ros/ros.h>
#include <map>
#include <peg_sensor/peg_sensor_model/base_peg_sensor_model.h>
#include <std_msgs/Float32MultiArray.h>
#include <peg_sensor/String_cmd.h>


#include <armadillo>
#include <memory>

#include "wrapobject.h"

namespace psm{

class Sensor_manager{

public:

    Sensor_manager(ros::NodeHandle& nh);

    void add(const std::string name, Base_peg_sensor_model* peg_sensor_model);

    bool select_model(const std::string& name);


    /**
     * @brief update_peg        : computes actual sensation Y from the peg end-effector.
     *                            This function would tipically be used in the peg_sensor node
     *                            which publishes the sensations Y felt by the peg. In our case
     *                            Y is a probability distribution over a set of discrete features.
     */
    void update_peg(arma::colvec& Y,const arma::colvec3& pos, const arma::mat33& Ro);


    /**
     * @brief update_particles  : computes expected sensation hY for a set of hypothetical
     *                            positions of the end-effector. This function would tipically
     *                            be used on the particle filters side to compute hypothetical
     *                            sensations, hY, which are then used to compute the likelihood
     *                            of each particle.
     */
    void compute_hY(arma::mat& Y,const arma::mat& points, const arma::mat33& Rot);

private:

    bool sensor_manager_callback(peg_sensor::String_cmd::Request& req, peg_sensor::String_cmd::Response& res);

private:

    std::map<std::string,Base_peg_sensor_model*> peg_model_map;
    std::map<std::string,Base_peg_sensor_model*>::iterator it;

    ros::ServiceServer      service_server;

};

}



#endif
