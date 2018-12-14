#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() 
{
    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const Eigen::VectorXd &y, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R)
{
    Eigen::MatrixXd Ht = H.transpose();
    Eigen::MatrixXd S = H * P_ * Ht + R;
    Eigen::MatrixXd Si = S.inverse();
    Eigen::MatrixXd PHt = P_ * Ht;
    Eigen::MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H) * P_;
}

void KalmanFilter::setQ(double dt, double sigma_ax, double sigma_ay)
{
	Q_ = Eigen::MatrixXd(4, 4);
	Q_ << (pow(dt, 4) / 4) * sigma_ax, 0, (pow(dt, 3) / 2) * sigma_ax, 0,
		0, (pow(dt, 4) / 4) * sigma_ax, 0, (pow(dt, 3) / 2) * sigma_ax,
		(pow(dt, 3) / 2) * sigma_ax, 0, (pow(dt, 2) / 1) * sigma_ax, 0,
		0, (pow(dt, 3) / 2) * sigma_ax, 0, (pow(dt, 2) / 1) * sigma_ax;
}
