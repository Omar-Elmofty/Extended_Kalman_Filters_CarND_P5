#include "kalman_filter.h"
#include <math.h>
#include <iostream>

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
  TODO:
    * predict the state
  */
  x_=F_*x_;
  P_=F_*P_*F_.transpose()+Q_;
  
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y=z-H_*x_;
  MatrixXd S=H_*P_*H_.transpose()+R_;
  MatrixXd K=P_*H_.transpose()*S.inverse();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  
  x_=x_+K*y;
  P_=(I-K*H_)*P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  
  VectorXd h(3);
  
  h << sqrt(pow(x_[0],2)+pow(x_[1],2)),
      atan2(x_[1],x_[0]),
      (x_[0]*x_[2]+x_[1]*x_[3])/sqrt(pow(x_[0],2)+pow(x_[1],2));
  
  //Avoid division by zero
  if (abs(x_[0]) <0.0001){
    h[1] =0;
  }
  if ((abs(x_[0]) <0.0001) && (abs(x_[1])<0.0001)){
    h[2] =0;
  }
	
  //correct for phi higher than pi or less than -pi  
  while ((h[1]-z[1])>M_PI || (h[1]-z[1])<-M_PI){
  	if ((h[1]-z[1])>M_PI){
    h[1]=h[1]-2*M_PI;
    }
    else{
      h[1]=h[1]+2*M_PI;
    }
  }
  VectorXd y = z-h;
  MatrixXd S=H_*P_*H_.transpose()+R_;
  MatrixXd K=P_*H_.transpose()*S.inverse();
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  
  x_=x_+K*y;
  P_=(I-K*H_)*P_;
  
  
}
