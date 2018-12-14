#include <iostream>
#include "tools.h"

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                              const std::vector<Eigen::VectorXd> &ground_truth) {

	Eigen::VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	if (estimations.size() <= 0)
		return rmse;

	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size())
		return rmse;

	//accumulate squared residuals
	Eigen::VectorXd sum(4);
	sum << 0, 0, 0, 0;
	for (int i = 0; i < estimations.size(); ++i) {
		sum += (Eigen::VectorXd)((estimations.at(i) - ground_truth.at(i)).array() * (estimations.at(i) - ground_truth.at(i)).array());
	}

	//calculate the mean
	sum /= estimations.size();

	//calculate the squared root
	rmse = sum.array().sqrt();

	//return the result
	return rmse;
}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd& x_state) {

	Eigen::MatrixXd Hj(3, 4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	if (px == 0 && py == 0)
	{
		std::cout << "Error computing jacobian. Some state vector elements are 0.";
		return Hj;
	}

	double px2py2 = pow(px, 2) + pow(py, 2);
	Hj << px / sqrt(px2py2), py / sqrt(px2py2), 0, 0,
		-py / px2py2, px / px2py2, 0, 0,
		(py*(vx*py - vy * px)) / pow(px2py2, 3.0 / 2.0), (px*(vy*px - vx * py)) / pow(px2py2, 3.0 / 2.0), px / sqrt(px2py2), py / sqrt(px2py2);

	return Hj;
}

Eigen::MatrixXd Tools::ComputeRadarMeasVector(const Eigen::VectorXd& x_state)
{
	Eigen::VectorXd meas(3);
	meas << 0, 0, 0;

	if (x_state.size() != 4 || x_state(0) == 0)
	{
		std::cout << "Error computing radar measurement vector.";
		return meas;
	}

	meas(0) = sqrt(pow(x_state(0), 2) + pow(x_state(1), 2));
	meas(1) = atan2(x_state(1), x_state(0));
	meas(2) = (x_state(0)*x_state(2) + x_state(1)*x_state(3)) / meas(0);

	return meas;
}

Eigen::MatrixXd Tools::ComputeRadarStateVector(const Eigen::VectorXd& z)
{
    Eigen::VectorXd x(4);

    if (z.size() != 3)
    {
        std::cout << "Error computing radar state vector.";
        return x;
    }

    double ro = z[0];
    double theta = z[1];
    x << ro * cos(theta), ro * sin(theta), 0, 0;

    return x;
}
