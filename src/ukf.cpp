#include "ukf.h"
///////////////////////////////////////////////////////////////////////////////////////
//size_t n_aug, size_t n_x, double std_a, double std_yawdd, double lambda
UKF::UKF(double std_a, double std_yawdd, UseSensor sensors) :
  Xsig_pred_(n_aug_, n_x_, std_a, std_yawdd, lambda_)
{
  usedSensors_ = sensors;
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
  I_ = MatrixXd::Identity(n_x_, n_x_);

  //measurement noise matrix  and observation model mapping matrix - lidar
  double std_laspx_  = 0.15; //Laser noise standard deviation position1 in m
  double std_laspy_  = 0.15; //Laser noise standard deviation position2 in m
  SensorConfig lidar({std_laspx_, std_laspy_}, n_x_);
  lidar_sensor_ = lidar;

  //measurement noise matrix - radar
  double std_radr_   = 0.3;  //Radar noise standard deviation radius in m
  double std_radphi_ = 0.03; //Radar noise standard deviation angle in rad
  double std_radrd_  = 0.3;  //Radar noise standard deviation radius change in m/s
  SensorConfig radar({std_radr_, std_radphi_, std_radrd_}, n_x_);
  radar_sensor_ = radar;
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

  if ((measurement.sensor_type == MeasurementPackage::RADAR) && !(usedSensors_ & UseSensor::Radar))
    return;
  if ((measurement.sensor_type == MeasurementPackage::LASER) && !(usedSensors_ & UseSensor::Lidar))
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
  if ((measurement.sensor_type == MeasurementPackage::RADAR) && (usedSensors_ & UseSensor::Radar))
  {
    // Radar updates
    this->updateRadar(measurement.values);
  }
  else if((measurement.sensor_type == MeasurementPackage::LASER) && (usedSensors_ & UseSensor::Lidar))
  {
    // Laser updates
    this->updateLidar(measurement.values);
  }
  else
    return;

#if PRINT
  cout << "x_ = \n" << x_ << endl;
  cout << "P_ = \n" << P_ << endl;
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

  //updates Zsig_ and z_pred_
  predictRadarMeasurement(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd::Zero(n_z,n_z);

  //calculate measurement covariance matrix S
  for(size_t j=0; j < Zsig_.cols(); ++j)
  {
    VectorXd z_diff = tools_.SubtractAndNormalize(Zsig_.col(j), z_pred_, 1);
    S += weights_(j)*z_diff*z_diff.transpose();
  }
  S += radar_sensor_.GetR();

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  //calculate cross correlation matrix
  for(size_t j=0; j < Zsig_.cols(); ++j)
  {
    Tc += weights_(j)*tools_.SubtractAndNormalize(Xsig_pred_.Get().col(j),x_,3)
        *tools_.SubtractAndNormalize(Zsig_.col(j),z_pred_,1).transpose();
  }

  //calculate Kalman gain K;
  MatrixXd K = Tc*S.inverse();

  //update state mean and covariance matrix
  VectorXd z_diff = tools_.SubtractAndNormalize(z, z_pred_, 1);
  x_ = x_ + K*z_diff;
  P_ = P_ - (K*S*K.transpose());

  //Calculate NIS
  this->NIS_ = z_diff.transpose()*S.inverse()*z_diff;
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::updateLidar(const VectorXd &z) {

  VectorXd z_pred = lidar_sensor_.GetH() * x_;
  VectorXd z_diff = z - z_pred;
  MatrixXd Ht = lidar_sensor_.GetH().transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = lidar_sensor_.GetH() * PHt + lidar_sensor_.GetR();
  MatrixXd K = PHt * S.inverse();

  //update state mean and covariance matrix
  x_ = x_ + (K * z_diff);
  P_ = (I_ - K * lidar_sensor_.GetH()) * P_;

  //Calculate NIS
  this->NIS_ = z_diff.transpose()*S.inverse()*z_diff;
}
///////////////////////////////////////////////////////////////////////////////////////
void UKF::predictMeanAndCovariance()
{
  MatrixXd Xsig_pred = Xsig_pred_.Get();

  //predict state mean
  x_ = Xsig_pred*weights_;

  //predict state covariance matrix
  P_.fill(0);
  for(size_t j=0; j < Xsig_pred.cols(); ++j)
  {
    // state difference
    VectorXd x_diff = tools_.SubtractAndNormalize(Xsig_pred.col(j), x_, 3);

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
  //TODO: Better implemented in own class for radar sensors.
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

  //transform sigma points into measurement space
  for(size_t i=0; i < Xsig_pred.cols(); ++i)
  {
    Zsig_.col(i) = hFuncRadar(Xsig_pred.col(i));
  }

  //mean predicted measurement
  z_pred_ = VectorXd::Zero(n_z);

  //calculate mean predicted measurement
  z_pred_ = Zsig_*weights_;

#if PRINT
  cout << "Zsig_ = \n" << Zsig_ << endl;
  cout << "z_pred_ = \n" << z_pred_ << endl;
#endif
}
///////////////////////////////////////////////////////////////////////////////////////
