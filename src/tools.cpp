#include <iostream>
#include "tools.h"

using namespace std;
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
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  
  //Check for input validity, estimations size must be equal to ground truth size, also estimations can't be empty
  if (estimations.size() != ground_truth.size() || estimations.size() ==0){
    cout << "Invalid input: estimations and ground truth must be equal length, estimations can't be empty!" << endl;
    return rmse;
  }
  
  // accumulate squared risduals
  for (unsigned int i = 0; i< estimations.size(); i++){
  	VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array()*residual.array(); //coefficient wise multip
    rmse += residual;    
  }
  //calculate the mean
  rmse = rmse/estimations.size();
  //Calculate the sqrt
  rmse = rmse.array().sqrt();
  
  return rmse;
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  // recover state parameters 
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // pre-compute some parameters to avoid repitition
  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = (c1*c2);
  //Check for division by zero
  if(fabs(c1) < 0.0001){
    cout << "CalculateJacobian () - Error - Division by Zero" << endl;
    return Hj;
  }
  //compute the jacpbian matrix
  Hj << (px/c2), (py/c2), 0, 0,
       -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  return Hj;
  
}
