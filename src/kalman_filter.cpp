#include <iostream>
#include <math.h>
#include "kalman_filter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  TODO [Done]:
    * predict the state
  */
  
  // Debug
  // cout << "KalmanFilter::Predict Start" << endl;
  
  x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
  
  // Debug
  // cout << "KalmanFilter::Predict End" << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO [Done]:
    * update the state by using Kalman Filter equations
  */
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  UpdateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO [Done]:
    * update the state by using Extended Kalman Filter equations
  */

  // convert current state from cartesian to polar
  float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

  double rho = sqrt(px*px + py*py);
  double phi = atan2(py, px);
  double rho_dot = (px*vx + py*vy) / rho;

  VectorXd h_x(3);
  h_x << rho, phi, rho_dot;

  VectorXd z_pred = h_x;
  VectorXd y = z - z_pred;

  if(y(1) < -M_PI || y(1) > M_PI)
  {
    cout << "Error!!! y(1) out of range: " << y(1) << endl;

    while(y(1) < -M_PI)
      y(1) += 2*M_PI;

    while(y(1) > M_PI)
      y(1) -= 2*M_PI;

    cout << "Corrected!!! y(1): " << y(1) << endl;
  }
  
  UpdateCommon(y);
}

void KalmanFilter::UpdateCommon(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}