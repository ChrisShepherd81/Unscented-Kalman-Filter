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
#include "SensorConfig.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
using std::vector;

class UKF {
public:
  enum UseSensor { Radar=1, Lidar=2, Both=3 };

  UKF(double std_a, double std_yawdd, UseSensor sensors);
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage &meas_package);

  //Getters
  const VectorXd GetX() const { return x_; }
  const double GetNIS() const { return NIS_; }

private:

  const size_t n_x_ = 5; ///* State dimension
  static constexpr size_t n_aug_ = 7; ///* Augmented state dimension
  static constexpr double lambda_ = 3.0 - n_aug_; ///* Sigma point spreading parameter

  SensorConfig lidar_sensor_;
  SensorConfig radar_sensor_;

  UseSensor usedSensors_;

  ///* Weights of sigma points
  VectorXd weights_;

  double NIS_;  ///* the current normalized innovation squared value

  SigmaPoints Xsig_pred_;

  Tools tools_;
  long previous_timestamp_ = 0;
  bool is_initialized_;

  VectorXd x_; ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  MatrixXd P_; ///* state covariance matrix

  MatrixXd Zsig_;
  VectorXd z_pred_;
  MatrixXd I_;

  double getDeltaTime(long timestamp);
  VectorXd hFuncRadar(const VectorXd & x);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void prediction(double delta_t);
  void predictRadarMeasurement(size_t n_z);
  void predictMeanAndCovariance();

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

  //Initialization methods
  void initalize(const MeasurementPackage &measurement);
  void initalizeWeights();
  void initalizeMatrices();

};

#endif /* UKF_H */
