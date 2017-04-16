#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include <cmath>
#include "Eigen/Dense"

using Eigen::VectorXd;

class Tools {
public:
  Tools();
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const std::vector<VectorXd> &estimations,
                                const std::vector<VectorXd> &ground_truth,
                                const size_t stateVectorLenght);

  VectorXd transformEstimationVector(const Eigen::VectorXd &est);

  VectorXd MapRadarPolarToCartesianPosition(const VectorXd& x_radar);

  VectorXd SubtractAndNormalize(VectorXd a, VectorXd b, size_t indexAngle);
  double NormalizeAngle(double angle);

};

#endif /* TOOLS_H_ */
