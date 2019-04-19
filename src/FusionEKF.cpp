#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized = false;

  previous_timestamp = 0;

  // initializing matrices
  R_laser = MatrixXd(2, 2);
  R_radar = MatrixXd(3, 3);
  H_laser = MatrixXd(2, 4);
  Hj = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser << 0.0225, 0.,
              0., 0.0225;

  //measurement covariance matrix - radar
  R_radar << 0.09, 0., 0.,
             0., 0.0009, 0.,
              0., 0., 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
   H_laser << 1., 0., 0., 0.,
              0., 1., 0., 0.;

    // create covariance matrix
    ekf.P = MatrixXd(4, 4);
    ekf.P <<  1., 0., 0., 0.,
              0., 1., 0., 0.,
              0., 0., 1000., 0.,
              0., 0., 0., 1000.;


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized) {
    /**
     * TODO: Initialize the state ekf.x with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
     //std::cout << "Initializing Kalman" << std::endl;

    // first measurement
    cout << "EKF: " << endl;
    ekf.x = VectorXd(4);
    ekf.x << 1., 1., 1., 1.;



    if (measurement_pack.sensor_type == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.

      // read values
      float rho = measurement_pack.raw_measurements[0];
      float phi = measurement_pack.raw_measurements[1];
      float rho_dot = measurement_pack.raw_measurements[2];

      // convert polar to cartesian and take care not to create zero division
      // according to tips and tricks section, vx/vy not applicable w/ radar
      ekf.x <<  std::min(rho * cos(phi), 0.0001),   // px
                std::min(rho * sin(phi), 0.0001),   // py
                0., // vx
                0.; // vy
    }
    else if (measurement_pack.sensor_type == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf.x <<  measurement_pack.raw_measurements[0],
                measurement_pack.raw_measurements[1],
                0.,
                0.;
    }

    // save first timestamp
    previous_timestamp = measurement_pack.timestamp;

    // done initializing, no need to predict or update
    is_initialized = true;
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

   // calc delta timestamp in secs
   double dt = (measurement_pack.timestamp - previous_timestamp) / 1000000.0;
   previous_timestamp = measurement_pack.timestamp;

   // calculate F
   ekf.F = MatrixXd(4, 4);
   ekf.F << 1., 0., dt, 0.,
            0., 1., 0., dt,
            0., 0., 1., 0.,
            0., 0., 0., 1.;


  double noise_ax = 9.;
  double noise_ay = 9.;

  // calculate Q matrix - pre-calculate terms to avoid multiple calculation
  double dt_2 = std::pow(dt, 2);
  double dt_3 = dt * dt_2;
  double dt_4 = dt * dt_3;
  double dt_4_div_4 = dt_4 / 4.;
  double dt_3_div_2 = dt_3 / 2.;

  ekf.Q = MatrixXd(4, 4);
  ekf.Q <<  dt_4_div_4 * noise_ax,  0.,  dt_3_div_2 * noise_ax,  0.,
            0.,  dt_4_div_4 * noise_ay,  0.,  dt_3_div_2 * noise_ay,
            dt_3_div_2 * noise_ax,  0.,  dt_2 * noise_ax,  0.,
            0.,  dt_3_div_2 * noise_ay,  0.,  dt_2 * noise_ay;

  // call predict function
  ekf.Predict();

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type == MeasurementPackage::RADAR) {

    // TODO: Radar updates

    // need to calculate Jacobian matrix
    ekf.H = tools.CalculateJacobian(ekf.x);
    ekf.R = R_radar;
    ekf.UpdateEKF(measurement_pack.raw_measurements);

  } else {
    // TODO: Laser updates

    // directly use H
    ekf.H = H_laser;
    ekf.R = R_laser;
    ekf.Update(measurement_pack.raw_measurements);
  }

  // print the output
  cout << "x = " << ekf.x << endl;
  cout << "P = " << ekf.P << endl;
}
