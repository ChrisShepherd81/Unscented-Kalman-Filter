/*
 * TestDataFileReader.cpp
 *
 *  Created on: 04.04.2017
 *      Author: christian@inf-schaefer.de
 */

#include "TestDataFileHandler.hpp"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TestDataFileHandler::TestDataFileHandler(string in_file_name, string out_file_name) :
  in_name(in_file_name),
  in_file(in_file_name.c_str(), ifstream::in),
  out_name(out_file_name),
  out_file(out_file_name.c_str(), ofstream::out)
{
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
TestDataFileHandler::~TestDataFileHandler()
{
  // close files
  if (out_file.is_open()) {
    out_file.close();
  }

  if (in_file.is_open()) {
    in_file.close();
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool TestDataFileHandler::check_files() {
  if (!in_file.is_open()) {
    cerr << "Cannot open input file: " << in_name << endl;
    return false;
  }

  if (!out_file.is_open()) {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
    return false;
  }

  return true;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TestDataFileHandler::read_file(vector<MeasurementPackage> &measurement_pack_list,
                               vector<GroundTruthPackage> &gt_pack_list)
{
  string line;
  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file, line))
  {
    string sensor_type;
    MeasurementPackage meas_package;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    if (sensor_type.compare("L") == 0)
    {
      // LASER MEASUREMENT
      // read measurements at this timestamp
      meas_package.sensor_type = MeasurementPackage::LASER;
      meas_package.values = VectorXd(2);
      float x;
      float y;
      iss >> x;
      iss >> y;
      meas_package.values << x, y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }
    else if (sensor_type.compare("R") == 0)
    {
      // RADAR MEASUREMENT
      // read measurements at this timestamp
      meas_package.sensor_type = MeasurementPackage::RADAR;
      meas_package.values = VectorXd(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.values << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

    // read ground truth data to compare later
    float x_gt;
    float y_gt;
    float vx_gt;
    float vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    gt_package.values = VectorXd(4);
    gt_package.values << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TestDataFileHandler::writeFileHeader()
{
  // column names for output file
  out_file << "px" << "\t";
  out_file << "py" << "\t";
  out_file << "v" << "\t";
  out_file << "yaw_angle" << "\t";
  out_file << "yaw_rate" << "\t";
  out_file << "px_measured" << "\t";
  out_file << "py_measured" << "\t";
  out_file << "px_true" << "\t";
  out_file << "py_true" << "\t";
  out_file << "vx_true" << "\t";
  out_file << "vy_true" << "\t";
  out_file << "NIS" << "\n";
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TestDataFileHandler::write_to_file(const VectorXd& v, size_t valToWrite)
{
  size_t vals = valToWrite == 0 ? v.size() : valToWrite;
  for(size_t i=0; i < vals; ++i)
  {
    out_file << v(i) << "\t";
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TestDataFileHandler::write_to_file(double val)
{
  out_file << val << "\t";
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TestDataFileHandler::write_to_file(string val)
{
  out_file << val;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

