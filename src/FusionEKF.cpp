#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;
  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0,    0,    0,
			       0, 1,    0,    0,
			       0, 0, 1000,    0,
			       0, 0,    0, 1000;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      VectorXd cartesian = Tools::PolarToCartesian(measurement_pack.raw_measurements_);
      ekf_.x_ = cartesian;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

    // remember the timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  // Skip 0's
  if(measurement_pack.raw_measurements_[0] == 0 && measurement_pack.raw_measurements_[1] == 0) {
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  long current_timestamp = measurement_pack.timestamp_;
  float dt = (current_timestamp - previous_timestamp_) / 1000000.0;
  int noise_ax = 15;
  int noise_ay = 9;

  ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1,  0, dt,  0,
             0,  1,  0, dt,
             0,  0,  1,  0,
             0,  0,  0,  1;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << (noise_ax * (dt_4 / 4)), 0, (noise_ax * (dt_3 / 2)), 0,
             0, (noise_ay * (dt_4 / 4)), 0, (noise_ay * (dt_3 / 2)),
             (noise_ax * (dt_3 / 2)), 0, (noise_ax * dt_2), 0,
             0, (noise_ay * (dt_3 / 2)), 0, (noise_ay * dt_2);

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = MatrixXd(2, 4);
    ekf_.H_ << 1, 0, 0, 0,
               0, 1, 0, 0;

    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // remember the timestamp
  previous_timestamp_ = measurement_pack.timestamp_;

  // print the output
  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
}
