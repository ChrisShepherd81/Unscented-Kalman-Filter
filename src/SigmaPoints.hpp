/*
 * SigmaPoints.hpp
 *
 *  Created on: 13.04.2017
 *      Author: christian@inf-schaefer.de
 */

#ifndef SRC_SIGMAPOINTS_HPP_
#define SRC_SIGMAPOINTS_HPP_

#include <iostream>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define PRINT 1

class SigmaPoints
{
 public:
  SigmaPoints(size_t n_aug, size_t n_x, double std_a, double std_yawdd, double lambda);
  virtual ~SigmaPoints();

  void predictSigmaPoints(const VectorXd &x, const MatrixXd &P, double dt);
  MatrixXd& Get()
  {
    return Xsig_pred_;
  }

 private:
  MatrixXd Xsig_pred_;
  size_t n_aug_;
  size_t n_x_;
  double std_a_; ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_yawdd_; ///* Process noise standard deviation yaw acceleration in rad/s^2
  double lambda_;

  MatrixXd generateSigmaPoints(const VectorXd &x, const MatrixXd &P);
  VectorXd predictSigmaPointColumn(VectorXd& col, double dt);

};

#endif /* SRC_SIGMAPOINTS_HPP_ */
