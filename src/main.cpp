#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "ukf.h"
#include "data/TestDataFileHandler.hpp"

#define TIME_MEASURE 0

#if TIME_MEASURE
  #include <ctime>
  #include <ratio>
  #include <chrono>

  using Hclock_t = std::chrono::high_resolution_clock;
  using std::chrono::duration;
#endif

#define GNU_PLOT 1

#if GNU_PLOT
  #include "plot/gnuplot_i.hpp"
  #include "plot/PlotData.hpp"
#endif

using Eigen::VectorXd;

using std::string;
using std::cout;
using std::endl;

void check_arguments(int argc, char* argv[]);

int main(int argc, char* argv[])
{
  check_arguments(argc, argv);
  string in_file_name_ = argv[1];
  string out_file_name_ = argv[2];

  double std_a = 0.9;
  double std_yawdd = 0.6;
  if(argc == 5)
  {
    std_a = std::stod(argv[3]);
    std_yawdd = std::stod(argv[4]);
  }

  TestDataFileHandler fileHandler(in_file_name_, out_file_name_);

  if(!fileHandler.check_files())
    exit(EXIT_FAILURE);

  //Read in the measurements and ground truth from file
  vector<MeasurementPackage> measurement_pack_list;
  vector<GroundTruthPackage> gt_pack_list;
  fileHandler.read_file(measurement_pack_list, gt_pack_list);

  // Create a UKF instance
  UKF ukf(std_a, std_yawdd, UKF::UseSensor::Both);

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

#if TIME_MEASURE
    double processTime = 0.0;
#endif
  for (size_t k = 0; k < N; ++k)
  {
#if TIME_MEASURE
    Hclock_t::time_point startTime = Hclock_t::now();
#endif
    /*
     * Process sensor fusion with UKF
     */
    ukf.ProcessMeasurement(measurement_pack_list[k]);
#if TIME_MEASURE
    Hclock_t::time_point endTime = Hclock_t::now();
    duration<double> time_span = std::chrono::duration_cast<duration<double>>(endTime - startTime);
    processTime += time_span.count();
#endif

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

#if TIME_MEASURE
    std::cout << "Time for UKF: " << processTime << " secs.\n";
    std::cout << "Avg processing time for measurement " << processTime/N << " secs.\n";
#endif

  // compute the accuracy (RMSE)
  Tools tools;
  //Target sample1 : [0.09, 0.09, 0.65, 0.65]
  //Target sample2 : [0.20, 0.20, 0.55, 0.55]
  const size_t GroundTruthVectorLength = 4;
  cout << "Accuracy - RMSE:" << endl << tools.CalculateRMSE(estimations, ground_truth, GroundTruthVectorLength) << endl;

#if GNU_PLOT
  Gnuplot posGraph;
  posGraph.set_legend("top left");

  posGraph.set_style("points pt 5 ps .5 lc rgb 'red'");
  posGraph.plot_xy(plot_ground.getAllX(), plot_ground.getAllY(), plot_ground.getTitle());

  posGraph.set_style("points pt 2 ps 0.5 lc rgb 'green'");
  posGraph.plot_xy(plot_radar.getAllX(), plot_radar.getAllY(), plot_radar.getTitle());

  posGraph.set_style("points pt 3 ps 0.5 lc rgb 'blue'");
  posGraph.plot_xy(plot_laser.getAllX(), plot_laser.getAllY(), plot_laser.getTitle());

  posGraph.set_style("points pt 1 ps 1 lc rgb 'black'");
  posGraph.plot_xy(plot_estimations.getAllX(), plot_estimations.getAllY(), plot_estimations.getTitle());

  Gnuplot nisGraph;
  double x_max = (double)std::max(nis_count_radar, nis_count_lidar);
  nisGraph.set_legend("top left");
  nisGraph.set_xrange(0.0, x_max);

  nisGraph.set_style("lines lc rgb 'blue'");
  nisGraph.plot_xy(plot_NIS_lidar.getAllX(), plot_NIS_lidar.getAllY(), plot_NIS_lidar.getTitle());
  nisGraph.plot_xy(std::vector<double>({0, x_max}), std::vector<double>({5.991, 5.991}), "\u03c7\u00b2 0.95 Lidar");
  std::cout << "Values over 0.95 " << count_if(plot_NIS_lidar.getAllY().begin(), plot_NIS_lidar.getAllY().end(), [](double a){return a > 5.991;});

  nisGraph.set_style("lines lc rgb 'green'");
  nisGraph.plot_xy(std::vector<double>({0, x_max}), std::vector<double>({7.815, 7.815}), "\u03c7\u00b2 0.95 Radar");
  nisGraph.plot_xy(plot_NIS_radar.getAllX(), plot_NIS_radar.getAllY(), plot_NIS_radar.getTitle());^
  std::cout << "Values over 0.95 " << (double)count_if(plot_NIS_radar.getAllY().begin(), plot_NIS_radar.getAllY().end(), [](double a){return a > 7.815;})/plot_NIS_radar.getAllY().size();

  std::cout << "Press 'Enter' to continue...";
  std::cin.ignore();

  posGraph.remove_tmpfiles();
  nisGraph.remove_tmpfiles();
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
  } else if (argc <= 5) {
    has_valid_args = true;
  } else {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args) {
    exit(EXIT_FAILURE);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
