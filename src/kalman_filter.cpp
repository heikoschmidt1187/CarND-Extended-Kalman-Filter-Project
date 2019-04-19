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
  x = x_in;
  P = P_in;
  F = F_in;
  H = H_in;
  R = R_in;
  Q = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
   x = F * x;
   P = F * P * F.transpose() + Q;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   VectorXd y = z - H * x;

   UpdateY(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   // extract current values from vector for calculation
   double px = x(0);
   double py = x(1);
   double vx = x(2);
   double vy = x(3);

   // avoid multiple calculation by pre-calculating terms
   double rho = sqrt(px * px + py * py);
   double phi = atan2(py, px);
   double rho_dot = (px * vx + py * vy) / rho;

   // buld h
   VectorXd h = VectorXd(3);
   h << rho, phi, rho_dot;

   // calculate y for further processing
   VectorXd y = z - h;

   // normalize y by shifting values
   while((y(1) > M_PI) || (y(1) < -M_PI)) {
     if(y(1) > M_PI) {
       y(1) -= M_PI;
     } else {
       y(1) += M_PI;
     }
   }

   UpdateY(y);
}

void KalmanFilter::UpdateY(const Eigen::VectorXd& y) {

   MatrixXd Ht = H.transpose();
   MatrixXd S = H * P * Ht + R;
   MatrixXd K = P * Ht * S.inverse();

   // new state
   x = x + (K * y);
   MatrixXd I = MatrixXd::Identity(x.size(), x.size());
   P = (I - K * H) * P;
}
