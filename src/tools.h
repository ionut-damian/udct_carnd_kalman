#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"

class Tools
{
public:
    /**
    * A helper method to calculate RMSE.
    */
    static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

    /**
    * A helper method to calculate Jacobians.
    */
    static Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

    /**
    * A helper method to compute the radar measurement vector from the (predicted) state vector
    */
    static Eigen::MatrixXd ComputeRadarMeasVector(const Eigen::VectorXd& x_state);

    /**
    * A helper method to compute the state vector from the radar measurement vector
    */
    static Eigen::MatrixXd ComputeRadarStateVector(const Eigen::VectorXd& z);
};

#endif /* TOOLS_H_ */
