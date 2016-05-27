#ifndef PEG_BOX_SENSOR_MODEL_H_
#define PEG_BOX_SENSOR_MODEL_H_


#include "wrapobject.h"
#include <visualise/vis_vector.h>
#include <objects/socket_one.h>
#include "peg_sensor/peg_sensor_model/peg_sensor_model.h"
#include "peg_sensor/peg_sensor_model/base_peg_sensor_model.h"


/**
 *
 */

namespace psm{

class Box_sensor_likelihood{

public:


    Box_sensor_likelihood(Peg_sensor_model& peg_sensor_model);

    /**
     * @brief update
     * @param L           : likelihood
     * @param Y           : current measurment [SURF,EDGE,SOCKET]
     * @param F           : current sensed force
     * @param points      : current origin of a hypothetical location of the peg FR
     * @param Rot         : current orienation of a hypothetical location of the peg FR
     */
    virtual void update(double* L, const arma::colvec& Y,const arma::colvec3& F, const arma::mat& points,const arma::mat33& Rot);

protected:

    Peg_sensor_model                    &peg_sensor_model;

    wobj::WBox*     socket_top_edge;
    wobj::WBox*     socket_bottom_edge;
    wobj::WBox*     socket_left_edge;
    wobj::WBox*     socket_right_edge;
    arma::fcolvec3  tmp_vec3f;

};

}


#endif
