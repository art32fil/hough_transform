#pragma once

#include "hough.h"
#ifdef DEBUG
  #include "grid_scan_matcher.h"
#else
  #include "../../core/grid_scan_matcher.h"
#endif
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include "extra_util_functions.h"

#include <memory>

class HoughScanMatcher : public GridScanMatcher {
public:
  HoughScanMatcher(std::shared_ptr<ScanCostEstimator> estimator):
                                               GridScanMatcher(estimator) {
    int i = 1;
    char* c = "~/";
    glutInit(&i,&c);
    //window_scan = create_window(300,300,"scan");
    //window_local_map = create_window(300,300,"local_map");
  }

  virtual double process_scan (const RobotState &init_pose,
                               const TransformedLaserScan &scan,
                               const GridMap &map,
                               RobotState &pose_delta) override;
  static int count;
private: //fields
  double min_sensity = 0.8;
  int window_scan;
  int window_local_map;

  //int max_iter = 10;
  //TransformedLaserScan prev_scan;
  std::shared_ptr<HoughTransform::Array_cov> prev_spectr;
  std::shared_ptr<HoughTransform> prev_HT;
  RobotState prev_pose;
};
