#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
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
        
	//compute the Jacobian matrix
  float p_mag_2 = px*px + py*py;
  
  //check division by zero
	if(p_mag_2 == 0)
	{
	    cout << "Deviding by zero!!!" << endl;
	    return Hj;
  }
  
	float d_rho_d_px = px/sqrt(p_mag_2);
	float d_rho_d_py = py/sqrt(p_mag_2);
	float d_phi_d_px = -py/p_mag_2;
	float d_phi_d_py = px/p_mag_2;
	float d_rho_dot_d_px = py*(vx*py - vy*px)/pow(p_mag_2, 1.5);
	float d_rho_dot_d_py = px*(vy*px - vx*py)/pow(p_mag_2, 1.5);
	
	Hj << d_rho_d_px, d_rho_d_py, 0, 0,
	        d_phi_d_px, d_phi_d_py, 0, 0,
	        d_rho_dot_d_px, d_rho_dot_d_py, d_rho_d_px, d_rho_d_py;

	return Hj;
}
