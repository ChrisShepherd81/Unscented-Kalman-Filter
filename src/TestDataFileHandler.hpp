/*
 * TestDataFileHandler.hpp
 *
 *  Created on: 04.04.2017
 *      Author: christian@inf-schaefer.de
 */
#ifndef SRC_TESTDATAFILEREADER_HPP_
#define SRC_TESTDATAFILEREADER_HPP_
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "Data.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::string;
using std::cerr;
using std::ifstream;
using std::ofstream;
using std::endl;
using std::vector;
using std::istringstream;

class TestDataFileHandler
{
  string in_name;
  ifstream in_file;

  string out_name;
  ofstream out_file;
  
 public:
  TestDataFileHandler(string in_file_name, string out_file_name);
  ~TestDataFileHandler();

  bool check_files();
  
  void read_file(vector<MeasurementPackage> &measurement_pack_list,
                 vector<GroundTruthPackage> &gt_pack_list);

  void write_to_file(const VectorXd& v, size_t valsToWrite=0);
  void write_to_file(double val);
  void write_to_file(string val);
};

#endif /* SRC_TESTDATAFILEREADER_HPP_ */
