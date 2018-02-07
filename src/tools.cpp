#include <iostream>
#include "tools.h"

// small num for divide by zero detection
#define SMALL_NUM 0.0001

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO [Done]:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
	rmse << 0,0,0,0;

  // Debug:
  // cout << "Tools::CalculateRMSE Starting to calculate rmse" << endl; 
  
  // check the validity of the inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() == 0
	  || estimations.size() != ground_truth.size())
	{
    cout << "ERROR!!! Input to CalculateRMSE is malformed!!! Estimations size: " 
      << estimations.size() << ", Ground_truth: " << ground_truth.size() << endl;
    return rmse;
	}
	
	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i) {
		VectorXd diff = estimations[i] - ground_truth[i];
		diff = (diff.array() * diff.array());
		rmse += diff;
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
  TODO [Done]:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	Hj << 0,0,0,
        0,0,0,
        0,0,0,
        0,0,0;
        
  // pre-compute terms that can be reused
  float p_mag_2 = px*px + py*py;
  float p_mag = sqrt(p_mag_2);
  float p_mag_3_2 = p_mag * p_mag_2;
  
  //check for potential division by zero
  if(fabs(p_mag_2) < SMALL_NUM){
		cout << "Error!!! CalculateJacobian - Division by Zero" << endl;
		return Hj;
	}
  
	//compute the Jacobian matrix
  
	float d_rho_d_px = px/p_mag;
	float d_rho_d_py = py/p_mag;
	float d_phi_d_px = -(py/p_mag_2);
	float d_phi_d_py = px/p_mag_2;
	float d_rho_dot_d_px = py*(vx*py - vy*px)/p_mag_3_2;
	float d_rho_dot_d_py = px*(vy*px - vx*py)/p_mag_3_2;
	
	Hj << d_rho_d_px, d_rho_d_py, 0, 0,
	        d_phi_d_px, d_phi_d_py, 0, 0,
	        d_rho_dot_d_px, d_rho_dot_d_py, d_rho_d_px, d_rho_d_py;

	return Hj;
}
