#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Tools::Tools() {}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Tools::~Tools() {}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth, const size_t gtVectorLenght)
{
  VectorXd rmse = VectorXd::Zero(gtVectorLenght);

  if(estimations.size() != ground_truth.size() || estimations.size() <= 0 )
  {
    std::cout << "Error in CalculateRMSE()" << std::endl;
    return rmse;
  }

  //accumulate squared residuals
  for(size_t i=0; i < estimations.size(); ++i)
  {
    VectorXd temp = transformEstimationVector(estimations[i]) - ground_truth[i];
    temp = temp.array()*temp.array();
    rmse += temp;
  }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd Tools::transformEstimationVector(const Eigen::VectorXd &est)
{
  Eigen::VectorXd transformed = Eigen::VectorXd::Zero(4);

  //Copy px and py values
  transformed(0) = est(0);
  transformed(1) = est(1);

  double v = est(2);
  double yaw = est(3);

  //Calculate vx and vy
  transformed(2) = v * std::cos(yaw);
  transformed(3) = v * std::sin(yaw);

  return transformed;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
VectorXd Tools::MapRadarPolarToCartesianPosition(const VectorXd& x_radar)
{
  VectorXd result = VectorXd::Zero(4);
  double rho = x_radar(0);
  double phi = x_radar(1);

  result(0) = rho*std::cos(phi);
  result(1) = rho*std::sin(phi);

  return result;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
