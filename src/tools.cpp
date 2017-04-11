#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
  VectorXd rmse = VectorXd::Zero(4);

  if(estimations.size() != ground_truth.size() || estimations.size() <= 0 )
  {
    std::cout << "Error in CalculateRMSE()" << std::endl;
    return rmse;
  }

  //accumulate squared residuals
  for(size_t i=0; i < estimations.size(); ++i)
  {
    VectorXd temp = estimations[i] - ground_truth[i];
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
