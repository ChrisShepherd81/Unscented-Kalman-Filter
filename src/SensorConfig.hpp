/*
 * SensorConfig.hpp
 *
 *  Created on: 19.04.2017
 *      Author: christian@inf-schaefer.de
 */

#ifndef SRC_SENSORCONFIG_HPP_
#define SRC_SENSORCONFIG_HPP_

#include <initializer_list>
#include <stdexcept>

#include "Eigen/Dense"

class SensorConfig {
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
  bool isLinear_;

 public:
  SensorConfig();
  SensorConfig(std::initializer_list<double> std_errors, size_t stateLength, bool isLinear=true);

  Eigen::MatrixXd& GetH();
  Eigen::MatrixXd& GetR();
};

#endif /* SRC_SENSORCONFIG_HPP_ */
