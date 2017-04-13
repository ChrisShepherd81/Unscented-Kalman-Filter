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
  use_laser_ = false;
  use_radar_ = true;
  x_ = VectorXd::Zero(5);
  P_ = MatrixXd::Identity(5, 5);
  std_a_ = 0.2;
  std_yawdd_ = 0.2;
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
  weights_ = VectorXd::Zero(2*n_aug_ +1);
  double w1 = lambda_/(lambda_+n_aug_);
  double w2 = 1.0/(2*(lambda_+n_aug_));
  weights_ += VectorXd::Constant(2*n_aug_+1, w2);
  weights_(0) = w1;

  NIS_radar_ = 0;
  NIS_laser_ = 0;
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

  if ((measurement.sensor_type == MeasurementPackage::RADAR) && !use_radar_)
    return;
  if ((measurement.sensor_type == MeasurementPackage::LASER) && !use_laser_)
    return;

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  double dt = this->getDeltaTime(measurement.timestamp_);
  this->Prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  //Use the sensor type to perform the update step
  if ((measurement.sensor_type == MeasurementPackage::RADAR) && use_radar_)
  {
    // Radar updates
    std::cout << "Upadte with radar data\n";
    this->UpdateRadar(measurement.values);
  }
  else if((measurement.sensor_type == MeasurementPackage::LASER) && use_laser_)
  {
    // Laser updates
    this->UpdateLidar(measurement.values);
  }
  else
    return;

#if PRINT
  // print the output
  cout << "x_ = \n" << GetX() << endl;
  cout << "P_ = \n" << GetP() << endl;
#endif
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::Prediction(double delta_t)
{
  MatrixXd sigmaPoints = generateSigmaPoints();
  this->predictSigmaPoints(sigmaPoints, delta_t);
  this->predictMeanAndCovariance();
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::UpdateLidar(const VectorXd &measurement) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::UpdateRadar(const VectorXd &z)
{
  size_t n_z = z.size();

  predictRadarMeasurement(n_z);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  //calculate cross correlation matrix
  for(size_t j=0; j < Zsig_.cols(); ++j)
  {
    Tc += weights_(j)*(Xsig_pred_.col(j)-x_)*(Zsig_.col(j)-z_pred_).transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc*S_.inverse();

  //update state mean and covariance matrix
  x_ = x_ + K*(z-z_pred_);
  P_ = P_ - (K*S_*K.transpose());
}
///////////////////////////////////////////////////////////////////////////////////////
MatrixXd UKF::generateSigmaPoints()
{
  //create augmented mean vector
  VectorXd x_aug = VectorXd::Zero(n_aug_);
  x_aug.head(x_.size()) = x_;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug.topLeftCorner(P_.rows(), P_.cols()) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

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
  Xsig_pred_.fill(0.0);
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
  P_.fill(0);
  for(size_t j=0; j < Xsig_pred_.cols(); ++j)
  {
    // state difference
    VectorXd x_diff = Xsig_pred_.col(j) - x_;
    //angle normalization TODO fmod()
    while (x_diff(3) >  M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3) < -M_PI) x_diff(3)+=2.*M_PI;
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

  if(std::fabs(psid) > 1e-4)
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

  return col.head(5) + vk + xTemp;
}
///////////////////////////////////////////////////////////////////////////////////////
VectorXd UKF::hFuncRadar(const VectorXd & x)
{
  VectorXd zTemp = VectorXd::Zero(3);

  double px = x(0);
  double py = x(1);

  double phi = std::atan2(py, px);
  zTemp(1) = phi;

  double px2py2 = (px*px)+(py*py);
  if(std::fabs(px2py2) > 1e-4)
  {
    double v = x(2);
    double phi = x(3);
    double sqrtpx2py2 = std::sqrt(px2py2);

    zTemp(0) = sqrtpx2py2;
    zTemp(2) = ((px*v*std::cos(phi))+(py*v*std::sin(phi)))/sqrtpx2py2;
  }

  return zTemp;
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::predictRadarMeasurement(size_t n_z)
{
  //create matrix for sigma points in measurement space
  Zsig_ = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  z_pred_ = VectorXd::Zero(n_z);

  //measurement covariance matrix S
  S_ = MatrixXd::Zero(n_z,n_z);

  //transform sigma points into measurement space
  for(size_t i=0; i < Xsig_pred_.cols(); ++i)
  {
    Zsig_.col(i) = hFuncRadar(Xsig_pred_.col(i));
  }

  //calculate mean predicted measurement
  for(size_t j=0; j < Zsig_.rows(); ++j)
  {
    z_pred_.row(j) = Zsig_.row(j)*weights_;
  }

  //calculate measurement covariance matrix S
  for(size_t j=0; j < Zsig_.cols(); ++j)
  {
    S_ += weights_(j)*(Zsig_.col(j)-z_pred_)*(Zsig_.col(j)-z_pred_).transpose();
  }

  MatrixXd R = MatrixXd(n_z, n_z);
  R <<  std_radr_*std_radr_, 0, 0,
        0, std_radphi_*std_radphi_, 0,
        0, 0, std_radrd_*std_radrd_;
  S_ += R;

#if PRINT
  cout << "Zsig_ = \n" << Zsig_ << endl;
  cout << "S_ = \n" << S_ << endl;
  cout << "z_pred_ = \n" << z_pred_ << endl;
#endif
}
///////////////////////////////////////////////////////////////////////////////////////
