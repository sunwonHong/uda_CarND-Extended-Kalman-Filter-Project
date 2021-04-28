#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  x_ = F_ * x_ ;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd new_y = z - H_ * x_;
  
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K =  P_ * H_.transpose() * S.inverse();
  MatrixXd Ident = MatrixXd::Identity(4, 4);
  x_ = x_ + (K * new_y);
  P_ = (Ident - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd temp;
  temp = VectorXd(3);
  VectorXd new_y;
  
  float x_px = x_(0);
  float x_py = x_(1);
  float x_vx = x_(2);
  float x_vy = x_(3);

  float x_rho = sqrt(x_px*x_px + x_py*x_py);
  float x_pxy = atan2(x_py, x_px);
  float x_rho_2 = (x_px*x_vx + x_py*x_vy) / x_rho;
  
  temp << x_rho, x_pxy, x_rho_2;
  new_y = z - temp;

  while(new_y(1) > M_PI){
    if(new_y(1) > M_PI){
      new_y(1) = new_y(1) - 2 * M_PI;
    }
  }
  while(new_y(1) < -M_PI){
    if(new_y(1) < -M_PI){
      new_y(1) = new_y(1) + 2 * M_PI;
    }
  }
  
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K =  P_ * H_.transpose() * S.inverse();
  x_ = x_ + (K * new_y);
  MatrixXd I = MatrixXd::Identity(4, 4);
  P_ = (I - K * H_) * P_;
}
