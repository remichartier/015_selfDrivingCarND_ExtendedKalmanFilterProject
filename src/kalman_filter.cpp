#include "kalman_filter.h"
#include <iostream> // for std::cout / std::endl
#include <cmath>       // cos/sin/pi=M_PI

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/* 
 * History 
 * v01 : implement Predict(), Update(), UpdateEKF(), fix build errors
 */


/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_*x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_*Ht*Si;
  MatrixXd I = MatrixXd::Identity(4, 4);

  // new state
  x_ = x_ + (K*y);
  P_ = (I - K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  // VectorXd y = z - H_ * x_;
  // For Radar : y=z−h(x′).y=z−Hx′ becomes y=z−h(x′).
  // Calculation hxprime, assuming H_ is Hj.
  
  
  VectorXd hxprime = VectorXd(3);
  float px(x_(0)), py(x_(1)), vx(x_(2)), vy(x_(3));
  hxprime(0) = sqrt((px*px) + (py*py));
  // check division by zero
  if (fabs(px) < 0.0001) {
    cout << "UpdateEKF() - Error - px : Division by Zero" << endl;
    hxprime(0) = 0 ;
  }
  else {
    hxprime(1) = atan2(py,px);
  }
  
  /**
   * Tips/tricks : 
   * The resulting angle phi in the y vector should be adjusted so that it is between -pi   and pi.   * The Kalman filter is expecting small angle values between the range -pi and pi. 
   * HINT: when working in radians, you can add 2\pi2π or subtract 2\pi2π until the angle is within
   * the desired range.
   * I will do phi = ((phi + pi) % (2*pi)) - pi
   */
  
  while( (hxprime(1) < -M_PI) || (hxprime(1)> M_PI)){
  	if(hxprime(1) < -M_PI){
      hxprime(1) += 2*M_PI;
    }
    else{
      hxprime(1) -= 2*M_PI;
    }
  }
  
  // check division by zero
  if (fabs(hxprime(0)) < 0.0001) {
    cout << "UpdateEKF() - Error - hxprime(2) : Division by Zero" << endl;
    hxprime(2) = 0 ;
  }
  else {
    hxprime(2) = ((px*vx) + (py*vy)) / hxprime(0);
  }
  
  // Now can apply normal EKF equations
  /////////////////////////////////
  cout << "Apply EKF equations" << endl;
  VectorXd y = z - hxprime;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_*Ht*Si;
  MatrixXd I = MatrixXd::Identity(4, 4);

  // new state
  /////////////////////////////////
  cout << "Calculate new state" << endl;
  x_ = x_ + (K*y);
  P_ = (I - K*H_)*P_;
}
