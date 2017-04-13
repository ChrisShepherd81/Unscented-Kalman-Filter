#ifndef UKF_H
#define UKF_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include "tools.h"
#include "data/TestDataFileHandler.hpp"
#include "Eigen/Dense"
#include "SigmaPoints.hpp"

#define PRINT 1

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
using std::vector;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage &meas_package);

  const VectorXd GetX() const
  {
    return x_;
  }

  const MatrixXd GetP() const
  {
    return P_;
  }

  const double GetNIS() const
  {
    return NIS_;
  }

private:
  const size_t n_x_ = 5; ///* State dimension
  static constexpr size_t n_aug_ = 7; ///* Augmented state dimension
  static constexpr double lambda_ = 3.0 - n_aug_; ///* Sigma point spreading parameter

  ///* Noise standard deviation of sensors
  const double std_laspx_  = 0.15; //Laser noise standard deviation position1 in m
  const double std_laspy_  = 0.15; //Laser noise standard deviation position2 in m
  const double std_radr_   = 0.3;  //Radar noise standard deviation radius in m
  const double std_radphi_ = 0.03; //Radar noise standard deviation angle in rad
  const double std_radrd_  = 0.3;  //Radar noise standard deviation radius change in m/s

  bool use_laser_; ///* if this is false, laser measurements will be ignored (except for init)
  bool use_radar_; ///* if this is false, radar measurements will be ignored (except for init)

  ///* Weights of sigma points
  VectorXd weights_;

  double NIS_;  ///* the current normalized innovation squared value

  SigmaPoints Xsig_pred_;

  Tools tools_;
  long previous_timestamp_ = 0;
  MatrixXd Zsig_;
  VectorXd z_pred_;
  MatrixXd S_;

  MatrixXd H_lidar_;
  MatrixXd R_lidar_;
  MatrixXd I_;

  double getDeltaTime(long timestamp);
  void initalize(const MeasurementPackage &measurement);
  void predictMeanAndCovariance();
  VectorXd hFuncRadar(const VectorXd & x);
  void predictRadarMeasurement(size_t n_z);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void updateLidar(const VectorXd &measurement);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void updateRadar(const VectorXd &measurement);

  void initalizeWeights();
  void initalizeMatrices();

};

#endif /* UKF_H */
