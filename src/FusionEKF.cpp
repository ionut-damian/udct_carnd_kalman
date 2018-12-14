#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

/*
 * Constructor.
 */
FusionEKF::FusionEKF()
{
    is_initialized_ = false;

    previous_timestamp_ = 0;

    /*
    * Initializing matrices
    */
    //Measurement matrix - laser
    H_laser_ = Eigen::MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
        0, 1, 0, 0;
    //radar measurement matrix is computed every frame

    //measurement covariance matrix - laser
    R_laser_ = Eigen::MatrixXd(2, 2);
    R_laser_ << 0.0225, 0,
        0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ = Eigen::MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

    //state covariance matrix P
    ekf_.P_ = Eigen::MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1.0e+5, 0,
        0, 0, 0, 1.0e+5;

    //the initial transition matrix F_
    ekf_.F_ = Eigen::MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

    //set initial state vector
    ekf_.x_ = Eigen::VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    //set the acceleration noise components
    noise_ax_ = 9;
    noise_ay_ = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_)
    {
        // first measurement
        std::cout << "*** INIT ***" << std::endl;

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            //Convert radar from polar to cartesian coordinates and initialize state.
            ekf_.x_ = Tools::ComputeRadarStateVector(measurement_pack.raw_measurements_);

            //correct initial P to match radar covariance
            ekf_.P_(0, 0) = R_radar_(0, 0);
            ekf_.P_(1, 1) = R_radar_(0, 0); //use variance of ro since it is the highest
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            //set the state with the initial location and zero velocity
            ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
            
            //correct initial P to match lidar covariance
            ekf_.P_(0, 0) = R_laser_(0, 0);
            ekf_.P_(1, 1) = R_laser_(1, 1);
        }

        // done initializing, no need to predict or update
        is_initialized_ = true;
        previous_timestamp_ = measurement_pack.timestamp_;
        return;
    }

    /*****************************************************************************
     *  Prediction (the same for LIDAR and RADAR)
     ****************************************************************************/

     //compute the time elapsed between the current and previous measurements
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    //1. Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    //2. Set the process covariance matrix Q
    ekf_.setQ(dt, noise_ax_, noise_ay_);

    //3. Call the Kalman Filter predict() function
    ekf_.Predict();

    /*****************************************************************************
     *  Update (different for LIDAR and RADAR)
     ****************************************************************************/

    switch (measurement_pack.sensor_type_)
    {
        case MeasurementPackage::LASER:
        {
            Eigen::VectorXd y = measurement_pack.raw_measurements_ - H_laser_ * ekf_.x_;
            ekf_.Update(y, H_laser_, R_laser_);
            break;
        }
        case MeasurementPackage::RADAR:
        {
            Eigen::VectorXd z_pred = Tools::ComputeRadarMeasVector(ekf_.x_);
            Eigen::VectorXd y = measurement_pack.raw_measurements_ - z_pred;

            //fix angle to be within [-pi, pi]
            while (y(1) > M_PI)
            {
                y(1) -= 2 * M_PI;
            }
            while (y(1) < -M_PI)
            {
                y(1) += 2 * M_PI;
            }

            ekf_.Update(y, Tools::CalculateJacobian(ekf_.x_), R_radar_);
            break;
        }
    }

    std::cout << "x_ = " << ekf_.x_ << std::endl;
    std::cout << "P_ = " << ekf_.P_ << std::endl;
}
