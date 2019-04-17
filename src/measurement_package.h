#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
 public:
  enum SensorType{
    LASER,
    RADAR
  } sensor_type;

  long long timestamp;

  Eigen::VectorXd raw_measurements;
};

#endif // MEASUREMENT_PACKAGE_H_
