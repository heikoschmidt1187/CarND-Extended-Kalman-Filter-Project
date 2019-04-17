#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf;

 private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized;

  // previous timestamp
  long long previous_timestamp;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser;
  Eigen::MatrixXd R_radar;
  Eigen::MatrixXd H_laser;
  Eigen::MatrixXd Hj;
};

#endif // FusionEKF_H_
