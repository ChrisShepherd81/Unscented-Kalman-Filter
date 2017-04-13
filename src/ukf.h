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
  int n_x_; ///* State dimension
  int n_aug_; ///* Augmented state dimension

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  //MatrixXd Xsig_pred_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  SigmaPoints Xsig_pred_;

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

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(const VectorXd &measurement);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(const VectorXd &measurement);

  const VectorXd GetX() const
  {
    return x_;
  }

  const MatrixXd GetP() const
  {
    return P_;
  }

private:
  Tools tools_;
  long previous_timestamp_ = 0;
  MatrixXd Zsig_;
  MatrixXd H_lidar_;
  MatrixXd R_lidar_;
  MatrixXd S_;
  MatrixXd I_;
  VectorXd z_pred_;

  double getDeltaTime(long timestamp);
  void initalize(const MeasurementPackage &measurement);
  //MatrixXd generateSigmaPoints();
  //void predictSigmaPoints(MatrixXd &Xsig_aug, double dt);
  void predictMeanAndCovariance();
  //VectorXd predictSigmaPointColumn(VectorXd& row, double dt);
  VectorXd hFuncRadar(const VectorXd & x);
  void predictRadarMeasurement(size_t n_z);
};

#endif /* UKF_H */
