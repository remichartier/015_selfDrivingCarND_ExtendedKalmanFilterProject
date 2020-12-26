#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <cmath>       /* cos/sin/pi */

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * History
 * v00 : init H_laser_ matrix, initial x state, previous_timestamp
 *		Matrix F and Q. Call Update() functions for Radar or Laser
 *		Remaining to check if "initialize variables and matrices 
 *		(x, F, H_laser, H_jacobian, P, etc.)" complete ?
 * v01 : Initialization Hj_, P_ matrix, first "working" version
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

  /**
   * No Need to Tune Parameters
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
  H_laser_ << 1,0,0,0,
    		  0,1,0,0;
  
  /**
   * For P : State Covariance Matrix :
   * No info on state deviations --> I initialize to very large deviations
   */
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;
  

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  /////////////////////////////////
  //cout << "--> in ProcessMeasurement()" << endl;
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
      /////////////////////////////////
  	  //cout << "if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {" << endl;
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      /**
       * For a row containing radar data, the columns are: sensor_type, 
       * rho_measured, phi_measured, rhodot_measured, timestamp, 
       * x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth, 
       * yaw_groundtruth, yawrate_groundtruth.
       */
	  float rho_measured 	= measurement_pack.raw_measurements_(0);
      float phi_measured 	= measurement_pack.raw_measurements_(1);
      /**
       * below is commented due to comment below indicating useless to get vx/vy
       * from rhodot_measured
       */
      // float rhodot_measured = measurement_pack.raw_measurements_[2];      
      
      //ekf_.x_(0) = rho_measured * cos(phi_measured); // conversion to px = cos(phi) * rho
      //ekf_.x_(1) = -rho_measured * sin(phi_measured); // converstion to py = -sin(phi) * rho
      ekf_.x_(0) = rho_measured * cos(-phi_measured); // conversion to px = cos(phi) * rho
      ekf_.x_(1) = rho_measured * sin(-phi_measured); // converstion to py = -sin(phi) * rho

      /**
       *Although radar gives velocity data in the form of the range rate \dot{\rho} ρ, a radar 
       * measurement does not contain enough information to determine the state variable velocities vx
       * and vy. You can, however, use the radar measurements \rhoρ and \phiϕ to initialize 
       * the state variable locations px and py.
       */
      // ekf_.x_[2] = rhodot_measured * cos(phi_measured); // conversion to vx = cos(phi) * rhodot_measured
      // ekf_.x_[3] = -rhodot_measured * sin(phi_measured); // conversion to vy = -sin(phi) * rhodot_measured
      ekf_.x_(2) = 0;
      ekf_.x_(3) = 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /////////////////////////////////
  	  //cout << "if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {" << endl;
      // TODO: Initialize state.
      /**
       * For a row containing lidar data, the columns are: sensor_type, x_measured, 
       * y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, 
       * vy_groundtruth, yaw_groundtruth, yawrate_groundtruth.
       */
	  ekf_.x_(0) = measurement_pack.raw_measurements_(0); // px
      ekf_.x_(1) = measurement_pack.raw_measurements_(1); // py
      ekf_.x_(2) = 0; // vx
      ekf_.x_(3) = 0; // vy
    }

    // previous timestamp
  	previous_timestamp_ = measurement_pack.timestamp_;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    /////////////////////////////////
  	//cout << "<-- out ProcessMeasurement(first init)" << endl;
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
  /////////////////////////////////
  //cout << "Calculation F_ and Q_" << endl;
  // compute the time elapsed between the current and previous measurements
  // big number in the timestamp variable is # of microseconds elapsed since Jan 1 1970.
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  // TODO: Update the state transition matrix F according to the new elapsed time.
  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // TODO: Update the process noise covariance matrix.
  //  * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
  // set the process covariance matrix Q
  float noise_ax=9, noise_ay=9;
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  /////////////////////////////////
  //cout << "Call ekf_.Predict()" << endl;
  ekf_.Predict();
  /////////////////////////////////
  //cout << "Exit ekf_.Predict()" << endl;

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    /////////////////////////////////
    //cout << "Update : if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {" << endl;
    // Calculate Jacobian Matrix first for Radar
    Tools tools;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    
    // KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
    //                    MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
    ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, Hj_, R_radar_, ekf_.Q_);
    
    // TODO: Radar updates
    // void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /////////////////////////////////
    //cout << "Call  : ekf_.UpdateEKF(measurement_pack.raw_measurements_);" << endl;
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    /////////////////////////////////
    //cout << "Exit  : ekf_.UpdateEKF(measurement_pack.raw_measurements_);" << endl;
  } else {
    // TODO: Laser updates
    
    // KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
    //                    MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
    ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, H_laser_, R_laser_, ekf_.Q_);    
    
    // Update the state and covariance matrices.

    // New measurement in measurement_pack.raw_measurements
    // which is a VectorXd(2) for px, py if representing 2D vector z
    // --> pass it to KalmanFilter::Update()
	// void KalmanFilter::Update(const VectorXd &z) {
    /////////////////////////////////
    //cout << "Call  : ekf_.Update(measurement_pack.raw_measurements_);" << endl;
    ekf_.Update(measurement_pack.raw_measurements_);
    /////////////////////////////////
    //cout << "Exit  : ekf_.Update(measurement_pack.raw_measurements_);" << endl;
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
  /////////////////////////////////
  //cout << "<-- out ProcessMeasurement()" << endl;
  
}
