#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

///////////////////////////////////////////////////////////////////////////////////////
UKF::UKF() {
  use_laser_ = true;
  use_radar_ = true;
  x_ = VectorXd::Zero(5);
  P_ = MatrixXd::Identity(5, 5);
  std_a_ = 30;
  std_yawdd_ = 30;
  std_laspx_ = 0.15;
  std_laspy_ = 0.15;
  std_radr_ = 0.3;
  std_radphi_ = 0.03;
  std_radrd_ = 0.3;
  n_x_ = 5;
  n_aug_ = 7;

  //define spreading parameter
  lambda_ = 3 - n_aug_;

  Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

  //set weights
  weights_ = VectorXd(2*n_aug_ +1);
  double w1 = lambda_/(lambda_+n_aug_);
  double w2 = 1.0/(2*(lambda_+n_aug_));
  weights_ += VectorXd::Constant(2*n_aug_+1, w2);
  weights_(0) = w1;

  double NIS_radar_;
  double NIS_laser_;
  is_initialized_ = false;
}
///////////////////////////////////////////////////////////////////////////////////////
UKF::~UKF() {}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::initalize(const MeasurementPackage &measurement)
{
  //Initialize time
  this->previous_timestamp_ = measurement.timestamp_;

  //Initalize state vector
  if (measurement.sensor_type == MeasurementPackage::RADAR)
  {
    //Convert radar from polar to cartesian coordinates and initialize state.
    VectorXd x = tools_.MapRadarPolarToCartesianPosition(measurement.values);
    x_(0) = x(0);
    x_(1) = x(1);
    //TODO: estimate yaw and velocity
  }
  else if (measurement.sensor_type == MeasurementPackage::LASER)
  {
    x_ << measurement.values(0), measurement.values(1), 0, 0, 0;
  }

  is_initialized_ = true;
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::ProcessMeasurement(const MeasurementPackage &measurement)
{
  /*****************************************************************************
  *  Initialization
  ****************************************************************************/
  if (!is_initialized_)
  {
    this->initalize(measurement);
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  double dt = this->getDeltaTime(measurement.timestamp_);
  //ekf.Q = tools.CalculateProcessCovarianceMatrix(dt);
  this->Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  //Use the sensor type to perform the update step
  if (measurement.sensor_type == MeasurementPackage::RADAR && use_radar_)
  {
    // Radar updates
  //      ekf.R = this->R_radar_;
  //      ekf.H = tools.CalculateJacobian(ekf.GetX());
    this->UpdateLidar(measurement.values);
  }
  else if(use_laser_)
  {
    // Laser updates
  //      ekf.R = this->R_laser_;
  //      ekf.H = this->H_laser_;
    this->UpdateRadar(measurement.values);
  }
  else
    return;

#if PRINT
  // print the output
  cout << "x_ = \n" << GetX() << endl;
  cout << "P_ = \n" << GetP() << endl;
#endif
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t)
{

  MatrixXd sigmaPoints = generateSigmaPoints();
  this->predictSigmaPoints(sigmaPoints, delta_t);
  this->predictMeanAndCovariance();
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const VectorXd &measurement) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const VectorXd &measurement) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
///////////////////////////////////////////////////////////////////////////////////////
MatrixXd UKF::generateSigmaPoints()
{
  //create augmented mean vector
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  x_aug.head(x_.size()) = x_;

  //create augmented covariance matrix
  MatrixXd Q(2,2);
  Q << std_a_*std_a_, 0,
       0, std_yawdd_*std_yawdd_;
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.topLeftCorner(P_.rows(), P_.cols()) = P_;
  P_aug.bottomRightCorner(Q.rows(), Q.cols()) = Q;

  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();

  MatrixXd B = (A *std::sqrt(lambda_ + n_aug_));
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.block(0,1,B.rows(),B.cols()) = B;
  Xsig_aug.block(0,1+B.cols(),B.rows(),B.cols()) = -B;
  Xsig_aug = Xsig_aug.colwise() + x_aug;

#if PRINT
  cout << "Xsig_aug = \n" << Xsig_aug << endl;
#endif

  return Xsig_aug;
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::predictSigmaPoints(MatrixXd &Xsig_aug, double dt)
{
  //predict sigma points
  for(size_t i=0; i < Xsig_aug.cols(); ++i)
  {
    VectorXd col = Xsig_aug.col(i);
    Xsig_pred_.col(i) += predictSigmaPointColumn(col, dt);
  }
#if PRINT
  cout << "Xsig_pred_ = \n" << Xsig_pred_ << endl;
#endif
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::predictMeanAndCovariance()
{
  //predict state mean
  for(size_t  i=0; i < Xsig_pred_.rows(); ++i)
    x_(i) = Xsig_pred_.row(i)*weights_;

  //predict state covariance matrix
  for(size_t j=0; j < Xsig_pred_.cols(); ++j)
  {
    P_ += weights_(j)*(Xsig_pred_.col(j)-x_)*(Xsig_pred_.col(j)-x_).transpose();
  }
}
///////////////////////////////////////////////////////////////////////////////////////
double UKF::getDeltaTime(long timestamp)
{
  //calculate time delta and convert from mu secs to secs.
  double dt = (timestamp - this->previous_timestamp_)/1.0e6;
  this->previous_timestamp_ = timestamp;
  return dt;
}
///////////////////////////////////////////////////////////////////////////////////////
VectorXd UKF::predictSigmaPointColumn(VectorXd& col, double dt)
{
  double v      = col(2);
  double psi    = col(3);
  double psid   = col(4);
  double va     = col(5);
  double vpsidd = col(6);
  double dt2    = dt*dt;
  double sinPsi = std::sin(psi);
  double cosPsi = std::cos(psi);

  VectorXd vk = VectorXd::Zero(n_x_);

  vk << 0.5*dt2*cosPsi*va,
        0.5*dt2*sinPsi*va,
        dt*va,
        0.5*dt2*vpsidd,
        dt*vpsidd;

  VectorXd xTemp = VectorXd::Zero(n_x_);

  if(psid != 0)
  {
    xTemp(0) = (v/psid)*(std::sin(psi+(psid*dt))-sinPsi);
    xTemp(1) = (v/psid)*(-std::cos(psi+(psid*dt))+cosPsi);
    xTemp(3) = psid*dt;
  }
  else
  {
    xTemp(0) = v*cosPsi*dt;
    xTemp(1) = v*sinPsi*dt;
  }

  VectorXd result = VectorXd::Zero(5);
  result = col.head(5) + vk + xTemp;
  return result;
}
///////////////////////////////////////////////////////////////////////////////////////
