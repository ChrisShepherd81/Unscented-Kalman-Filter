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

class SigmaPoints {

  MatrixXd Xsig_pred_;
  size_t n_aug_;
  size_t n_x_;
  double std_a_; ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_yawdd_; ///* Process noise standard deviation yaw acceleration in rad/s^2
  double lambda_;

  MatrixXd generateSigmaPoints(const VectorXd &x, const MatrixXd &P);
  VectorXd predictSigmaPointColumn(VectorXd& col, double dt);
 public:
  SigmaPoints(size_t n_aug, size_t n_x, double std_a, double std_yawdd)
   : n_aug_(n_aug), n_x_(n_x), std_a_(std_a), std_yawdd_(std_yawdd)
 {
    Xsig_pred_ = MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);
    lambda_ = 3.0 - n_aug_;
 }
  virtual ~SigmaPoints();
  void predictSigmaPoints(const VectorXd &x, const MatrixXd &P, double dt);
  MatrixXd& Get()
  {
    return Xsig_pred_;
  }
};

#endif /* SRC_SIGMAPOINTS_HPP_ */
