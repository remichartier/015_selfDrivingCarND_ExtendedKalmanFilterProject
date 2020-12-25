#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>       /* cos */

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
* History
* v0 : init H_laser_ matrix 
*/


/**
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

  /* No Need to Tune Parameters
  *  • The R matrix values and Q noise values are provided for you. 
  *  There is no need to tune these parameters for this project.
  */
  
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  // initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
  H_laser_ << 1,0,0,0
    		  0,1,0,0;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      /* For a row containing radar data, the columns are: sensor_type, 
      * rho_measured, phi_measured, rhodot_measured, timestamp, 
      * x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, 
      * yaw_groundtruth, yawrate_groundtruth.
      */
	  float rho_measured 	= measurement_pack.raw_measurements_[0];
      float phi_measured 	= measurement_pack.raw_measurements_[1];
      float rhodot_measured = measurement_pack.raw_measurements_[2];
      
      
      ekf_.x_[0] = rho_measured * cos(phi_measured); // conversion to px = cos(phi) * rho
      ekf_.x_[1] = -rho_measured * sin(phi_measured); // converstion to py = -sin(phi) * rho
      ekf_.x_[2] = rhodot_measured * cos(phi_measured); // conversion to vx = cos(phi) * rhodot_measured
      ekf_.x_[3] = -rhodot_measured * sin(phi_measured); // conversion to vy = -sin(phi) * rhodot_measured
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      /* For a row containing lidar data, the columns are: sensor_type, x_measured, 
      * y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, 
      * vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.
      */
	  ekf_.x_[0] = measurement_pack.raw_measurements_[0]; // px
      ekf_.x_[1] = measurement_pack.raw_measurements_[0]; // py
      ekf_.x_[2] = 0; // vx
      ekf_.x_[3] = 0; // vy
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates

  } else {
    // TODO: Laser updates

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
