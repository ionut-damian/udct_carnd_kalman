#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter
{
public:

    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    /**
     * Constructor
     */
    KalmanFilter();

    /**
     * Destructor
     */
    virtual ~KalmanFilter();

    /**
    * Updates the process covariance matrix using the provided noise values and time increment
    * @param dt time increment
    * @param sigma_ax noise value for x-axis
    * @param sigma_ay noise value for y-axis
    */
    void setQ(double dt, double sigma_ax, double sigma_ay);

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     */
    void Predict();

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement error at k+1, i.e. z - h(x) or z - Hx
     * @param z_pred The predicted state (at k+1) converted to a measurement vector
     * @param H Measurement matrix
     * @param R Measurement covariance matrix
     */
    void Update(const Eigen::VectorXd &y, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);

};

#endif /* KALMAN_FILTER_H_ */
