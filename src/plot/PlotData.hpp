/*
 * PlotData.hpp
 *
 *  Created on: 04.04.2017
 *      Author: christian@inf-schaefer.de
 */

#ifndef SRC_PLOT_PLOTDATA_HPP_
#define SRC_PLOT_PLOTDATA_HPP_

#include <vector>
#include "../Eigen/Dense"

class PlotData {

	std::string title_;
	std::vector<double> x_;
	std::vector<double> y_;

public:
	PlotData(std::string title);
	void addPoint(const Eigen::VectorXd &point);
	void addPoint(double x, double y);
	auto getAllX() const -> std::vector<double>;
	auto getAllY() const -> std::vector<double>;
	bool empty() const;
	std::string getTitle() const;
	virtual ~PlotData() {};
};

#endif /* SRC_PLOT_PLOTDATA_HPP_ */
