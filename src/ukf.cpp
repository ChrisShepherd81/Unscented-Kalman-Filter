#include "ukf.h"
///////////////////////////////////////////////////////////////////////////////////////
//size_t n_aug, size_t n_x, double std_a, double std_yawdd, double lambda
UKF::UKF() : Xsig_pred_(n_aug_, n_x_, 0.7, 0.5)
{
  use_laser_ = false;
  use_radar_ = true;

  NIS_ = 0;
  is_initialized_ = false;
}
///////////////////////////////////////////////////////////////////////////////////////
UKF::~UKF() {}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::initalize(const MeasurementPackage &measurement)
{
  //Initialize time
  this->previous_timestamp_ = measurement.timestamp_;

  //InitInitialize weights
  this->initalizeWeights();

  //Initialize matrices and state vector
  this->initalizeMatrices();

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
void UKF::initalizeWeights()
{
  //set weights
  weights_ = VectorXd::Zero(2*n_aug_ +1);
  double w1 = lambda_/(lambda_+n_aug_);
  double w2 = 1.0/(2*(lambda_+n_aug_));
  weights_ += VectorXd::Constant(2*n_aug_+1, w2);
  weights_(0) = w1;
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::initalizeMatrices()
{
  x_ = VectorXd::Zero(n_x_);
  P_ = MatrixXd::Identity(n_x_, n_x_);

  //observation model mapping matrix lidar
  H_lidar_ = MatrixXd::Zero(2, n_x_);
  H_lidar_ << 1, 0, 0, 0, 0,
             0, 1, 0, 0, 0;

  //measurement noise matrix - lidar
  R_lidar_ = MatrixXd::Zero(2, 2);
  R_lidar_ << std_laspx_*std_laspx_, 0,
             0, std_laspy_*std_laspy_;

  I_ = MatrixXd::Identity(n_x_, n_x_);
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
  this->prediction(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  //Use the sensor type to perform the update step
  if ((measurement.sensor_type == MeasurementPackage::RADAR) && use_radar_)
  {
    // Radar updates
    this->updateRadar(measurement.values);
  }
  else if((measurement.sensor_type == MeasurementPackage::LASER) && use_laser_)
  {
    // Laser updates
    this->updateLidar(measurement.values);
  }
  else
    return;

#if PRINT
  cout << "x_ = \n" << GetX() << endl;
  cout << "P_ = \n" << GetP() << endl;
#endif
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::prediction(double delta_t)
{
  this->Xsig_pred_.predictSigmaPoints(x_, P_, delta_t);
  this->predictMeanAndCovariance();
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::updateRadar(const VectorXd &z)
{
  size_t n_z = z.size();

  predictRadarMeasurement(n_z);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  //calculate cross correlation matrix
  for(size_t j=0; j < Zsig_.cols(); ++j)
  {
    Tc += weights_(j)*(Xsig_pred_.Get().col(j)-x_)*(Zsig_.col(j)-z_pred_).transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc*S_.inverse();

  //update state mean and covariance matrix
  x_ = x_ + K*(z-z_pred_);
  P_ = P_ - (K*S_*K.transpose());
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::updateLidar(const VectorXd &measurement) {

  VectorXd z_pred = H_lidar_ * x_;
  VectorXd y = measurement - z_pred;
  MatrixXd Ht = H_lidar_.transpose();
  MatrixXd S = H_lidar_ * P_ * Ht + R_lidar_;
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * S.inverse();

  //update state mean and covariance matrix
  x_ = x_ + (K * y);
  P_ = (I_ - K * H_lidar_) * P_;
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::predictMeanAndCovariance()
{
  MatrixXd Xsig_pred = Xsig_pred_.Get();
  //predict state mean
  for(size_t  i=0; i < Xsig_pred.rows(); ++i)
    x_(i) = Xsig_pred.row(i)*weights_;

  //predict state covariance matrix
  P_.fill(0);
  for(size_t j=0; j < Xsig_pred.cols(); ++j)
  {
    // state difference
    VectorXd x_diff = Xsig_pred.col(j) - x_;
    x_diff(3) = tools_.NormalizeAngle(x_diff(3));

    P_ += weights_(j)*x_diff*x_diff.transpose();
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
  MatrixXd Xsig_pred = Xsig_pred_.Get();

  //create matrix for sigma points in measurement space
  Zsig_ = MatrixXd::Zero(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  z_pred_ = VectorXd::Zero(n_z);

  //measurement covariance matrix S
  S_ = MatrixXd::Zero(n_z,n_z);

  //transform sigma points into measurement space
  for(size_t i=0; i < Xsig_pred.cols(); ++i)
  {
    Zsig_.col(i) = hFuncRadar(Xsig_pred.col(i));
  }

  //calculate mean predicted measurement
  for(size_t j=0; j < Zsig_.rows(); ++j)
  {
    z_pred_.row(j) = Zsig_.row(j)*weights_;
  }

  //calculate measurement covariance matrix S
  for(size_t j=0; j < Zsig_.cols(); ++j)
  {
    VectorXd z_diff = Zsig_.col(j)-z_pred_;
    z_diff(1) = tools_.NormalizeAngle(z_diff(1));

    S_ += weights_(j)*z_diff*z_diff.transpose();
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
