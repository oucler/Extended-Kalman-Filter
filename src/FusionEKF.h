#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
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
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;    // laser measurement noise
  Eigen::MatrixXd R_radar_;    // radar measurement noise
  Eigen::MatrixXd H_laser_;    // measurement function for laser
  Eigen::MatrixXd H_jacobian;         // measurement function for radar

  float noise_ax;
  float noise_ay;

  float rho; // range: radial distance from origin
  float phi; // bearing: angle between rho and x axis
  float rho_dot; // radial velocity: change of rho

  float dt; // delta time between previous and current time
  float dt_2;
  float dt_3;
  float dt_4;
};

#endif /* FusionEKF_H_ */
