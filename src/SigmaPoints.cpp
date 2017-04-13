/*
 * SigmaPoints.cpp
 *
 *  Created on: 13.04.2017
 *      Author: christian@inf-schaefer.de
 */

#include "SigmaPoints.hpp"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SigmaPoints::~SigmaPoints() {
  // TODO Auto-generated destructor stub
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MatrixXd SigmaPoints::generateSigmaPoints(const VectorXd &x, const MatrixXd &P)
{
  //create augmented mean vector
    VectorXd x_aug = VectorXd::Zero(n_aug_);
    x_aug.head(n_x_) = x;

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);
    P_aug.topLeftCorner(P.rows(), P.cols()) = P;
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
    std::cout << "Xsig_aug = \n" << Xsig_aug << std::endl;
  #endif

    return Xsig_aug;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SigmaPoints::predictSigmaPoints(const VectorXd &x, const MatrixXd &P, double dt)
{
  MatrixXd Xsig_aug = generateSigmaPoints(x, P);
  //predict sigma points
  Xsig_pred_.fill(0.0);
  for(size_t i=0; i < Xsig_aug.cols(); ++i)
  {
    VectorXd col = Xsig_aug.col(i);
    Xsig_pred_.col(i) += predictSigmaPointColumn(col, dt);
  }
#if PRINT
  std::cout << "Xsig_pred_ = \n" << Xsig_pred_ << std::endl;
#endif
}
///////////////////////////////////////////////////////////////////////////////////////
VectorXd SigmaPoints::predictSigmaPointColumn(VectorXd& col, double dt)
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
