#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  // ... your code here
  if(estimations.size() == 0) {
    std::cout << "Error, estimations size is 0" << std::endl;
    return rmse;
  }

  if(estimations.size() != ground_truth.size()) {
    std::cout << "Error, estimations and ground_truth are not the same size" << std::endl;
    return rmse;
  }


  VectorXd residual(4);
  //accumulate squared residuals
  for(int i = 0; i < estimations.size(); i++){
    residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse /= estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj(3,4);

	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float px2 = pow(px, 2);
	float py2 = pow(py, 2);
  float px2_py2 = px2 + py2;
	float sqrt_px2_py2 = sqrt(px2_py2);
  float pow_15_px2_py2 = pow(px2_py2, 1.5);

	if (px2_py2 == 0) {
    std::cout << "Error in CalculateJacobian: Can't divide by zero." << std::endl;
    return Hj;
	}

	float Hj_00 = px / sqrt_px2_py2;
	float Hj_01 = py / sqrt_px2_py2;
	float Hj_02 = 0;
	float Hj_03 = 0;

	float Hj_10 = -py / px2_py2;
	float Hj_11 = px / px2_py2;
	float Hj_12 = 0;
	float Hj_13 = 0;

	float Hj_20 = (py * (vx * py - vy * px)) / pow_15_px2_py2;
	float Hj_21 = (px * (vy * px - vx * py)) / pow_15_px2_py2;
	float Hj_22 = px / sqrt_px2_py2;
	float Hj_23 = py / sqrt_px2_py2;

	Hj << Hj_00, Hj_01, Hj_02, Hj_03,
	      Hj_10, Hj_11, Hj_12, Hj_13,
	      Hj_20, Hj_21, Hj_22, Hj_23;

  return Hj;
}

VectorXd Tools::PolarToCartesian(const VectorXd& x_state) {
  float rho = x_state[0];
  float phi = x_state[1];
  float rho_dot = x_state[2];

  float px = rho * cos(phi);
  float py = rho * sin(phi);
  float vx = rho * cos(phi);
  float vy = rho * sin(phi);

  VectorXd cartesian(4);
  cartesian << px,
               py,
               vx,
               vy;

  return cartesian;
}

VectorXd Tools::CartesianToPolar(const VectorXd& x_state) {
  float px = x_state[0];
  float py = x_state[1];
  float vx = x_state[2];
  float vy = x_state[3];

  float px_2 = px * px;
  float py_2 = py * py;

  float rho = sqrt(px_2 + py_2);
  float phi = atan(py / px);
  float rho_dot = (px * vx + py * vy) / rho;

  VectorXd polar(3);
  polar << rho,
           phi,
           rho_dot;

  return polar;
}
