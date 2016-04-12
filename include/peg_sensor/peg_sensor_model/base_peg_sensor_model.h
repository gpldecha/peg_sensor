#ifndef BASE_PEG_SENSOR_MODEL_H_
#define BASE_PEG_SENSOR_MODEL_H_

#include <armadillo>

class Base_peg_sensor_model{

public:

    virtual void update(arma::colvec& Y,const arma::colvec3& pos,const arma::mat33& Rot) = 0;

    virtual void update(arma::mat& Y,const arma::mat& points,const arma::mat33& Rot) = 0;

private:

};


#endif
