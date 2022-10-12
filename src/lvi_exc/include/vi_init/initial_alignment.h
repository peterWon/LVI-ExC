#pragma once
#include <iostream>
#include <map>
#include <deque>
#include "vi_init/integration_base.h"
#include <eigen3/Eigen/Dense>


using namespace Eigen;
using namespace std;

class ImageFrame{
public:
  ImageFrame(){};
  ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<
    double, 7, 1>>>>& _points, double _t): t{_t}, is_key_frame{false}{
    points = _points;
  };
  map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>> > > points;
  double t;
  Matrix3d R;
  Vector3d T;
  IntegrationBase *pre_integration;
  bool is_key_frame;
};

bool VisualIMUAlignment(const std::deque<ImageFrame> &all_image_frame, 
                    Vector3d* Bgs, Vector3d &g, Vector3d &T_I_C, VectorXd &x,
                    bool fix_scale = false);

bool VisualIMUAlignmentWithScale(const std::deque<ImageFrame> &all_image_frame, 
                    Vector3d* Bgs, Vector3d &g, Vector3d &T_I_C, VectorXd &x,
                    double scale_guess);