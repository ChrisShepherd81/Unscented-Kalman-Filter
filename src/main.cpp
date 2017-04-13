#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "data/TestDataFileHandler.hpp"

#define GNU_PLOT 1

#if GNU_PLOT
#include "plot/gnuplot_i.hpp"
#include "plot/PlotData.hpp"
#endif

using Eigen::VectorXd;

using std::string;
using std::cout;
using std::endl;

const size_t GroundTruthVectorLength = 4;

void check_arguments(int argc, char* argv[]);

int main(int argc, char* argv[])
{
  check_arguments(argc, argv);
  string in_file_name_ = argv[1];
  string out_file_name_ = argv[2];

  TestDataFileHandler fileHandler(in_file_name_, out_file_name_);

  if(!fileHandler.check_files())
    exit(EXIT_FAILURE);

  //Read in the measurements and ground truth from file
  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;
  fileHandler.read_file(measurement_pack_list, gt_pack_list);

  // Create a UKF instance
  UKF ukf;

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

#if GNU_PLOT
  PlotData plot_estimations("Estimations");
  PlotData plot_laser("Laser Measurements");
  PlotData plot_radar("Radar Measurements");
  PlotData plot_ground("Ground truth");
  PlotData plot_NIS_radar("NIS Radar");
  PlotData plot_NIS_lidar("NIS Lidar");
  size_t nis_count_lidar = 0;
  size_t nis_count_radar = 0;
#endif

  fileHandler.writeFileHeader();
  //Call the EKF-based fusion
  size_t N = measurement_pack_list.size();
  for (size_t k = 0; k < N; ++k) {
    // start filtering from the second frame (the speed is unknown in the first frame)
    ukf.ProcessMeasurement(measurement_pack_list[k]);

    // output the estimation
    VectorXd estimation = ukf.GetX();
    fileHandler.write_to_file(estimation);
#if GNU_PLOT
    plot_estimations.addPoint(estimation);
#endif

    // output the measurements
    if (measurement_pack_list[k].sensor_type == MeasurementPackage::LASER)
    {
      // output the measurements
      fileHandler.write_to_file(measurement_pack_list[k].values, 2);
#if GNU_PLOT
      plot_laser.addPoint(measurement_pack_list[k].values);
      plot_NIS_lidar.addPoint(nis_count_lidar++, ukf.GetNIS());
#endif
    }
    else if (measurement_pack_list[k].sensor_type == MeasurementPackage::RADAR)
    {
      // output the measurements in the cartesian coordinates
      float ro = measurement_pack_list[k].values(0);
      float phi = measurement_pack_list[k].values(1);
      double x = ro * cos(phi);
      double y = ro * sin(phi);

      fileHandler.write_to_file(x);
      fileHandler.write_to_file(y);

#if GNU_PLOT
      plot_radar.addPoint(x, y);
      plot_NIS_radar.addPoint(nis_count_radar++, ukf.GetNIS());
#endif
    }

    // output the ground truth packages
    fileHandler.write_to_file(gt_pack_list[k].values);
    fileHandler.write_to_file(ukf.GetNIS());
    fileHandler.write_to_file("\n");

#if GNU_PLOT
    plot_ground.addPoint(gt_pack_list[k].values);
#endif

    estimations.push_back(estimation);
    ground_truth.push_back(gt_pack_list[k].values);
  }

  // compute the accuracy (RMSE)
  Tools tools;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth, GroundTruthVectorLength) << endl;

#if GNU_PLOT
  Gnuplot gp;
  gp.set_legend("top left");

  gp.set_style("points pt 5 ps .5 lc rgb 'red'");
  gp.plot_xy(plot_ground.getAllX(), plot_ground.getAllY(), plot_ground.getTitle());

  gp.set_style("points pt 2 ps 0.5 lc rgb 'green'");
  gp.plot_xy(plot_radar.getAllX(), plot_radar.getAllY(), plot_radar.getTitle());

  gp.set_style("points pt 3 ps 0.5 lc rgb 'blue'");
  gp.plot_xy(plot_laser.getAllX(), plot_laser.getAllY(), plot_laser.getTitle());

  gp.set_style("points pt 1 ps 1 lc rgb 'black'");
  gp.plot_xy(plot_estimations.getAllX(), plot_estimations.getAllY(), plot_estimations.getTitle());


  Gnuplot gp2;
  gp2.set_legend("top left");
  gp2.set_style("lines lc rgb 'blue'");
  gp2.plot_xy(plot_NIS_lidar.getAllX(), plot_NIS_lidar.getAllY(), plot_NIS_lidar.getTitle());
  gp2.set_style("lines lc rgb 'green'");
  gp2.plot_xy(plot_NIS_radar.getAllX(), plot_NIS_radar.getAllY(), plot_NIS_radar.getTitle());

  std::cout << "Press 'Enter' to continue...";
  std::cin.ignore();

  gp.remove_tmpfiles();
  gp2.remove_tmpfiles();
#endif
  std::cout << "Program exit.";
  return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void check_arguments(int argc, char* argv[]) {
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1) {
    cerr << usage_instructions << endl;
  } else if (argc == 2) {
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  } else if (argc == 3) {
    has_valid_args = true;
  } else if (argc > 3) {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
