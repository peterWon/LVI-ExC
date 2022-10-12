//
// Created by hannes on 2017-03-20.
//

#ifndef KONTIKI_PINHOLE_CAMERA_H
#define KONTIKI_PINHOLE_CAMERA_H

#include <iostream>

#include <Eigen/Dense>

#include "camera.h"
#include <entity/paramstore/dynamic_pstore.h>

namespace kontiki {
namespace sensors {

namespace internal {

struct PinholeMeta : public CameraMeta {
  size_t NumParameters() const override {
    return CameraMeta::NumParameters();
  }

  Eigen::Matrix3d camera_matrix; // FIXME: This should be a set of parameters
  double m_k1 = 0.;
  double m_k2 = 0.;
  double m_p1 = 0.;
  double m_p2 = 0.;
  double m_k3 = 0.;

  bool do_distortion = false;
  double m_inv_K11 = 1.;
  double m_inv_K13 = 0.;
  double m_inv_K22 = 1.; 
  double m_inv_K23 = 0.;
  double m_fx;
  double m_fy;
  double m_cx;
  double m_cy;
};

template<typename T, typename MetaType>
class PinholeView : public CameraView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Vector2 = Eigen::Matrix<T, 2, 1>;
  using Result = std::unique_ptr<CameraEvaluation<T>>;
 public:
  using CameraMatrix = Eigen::Matrix<T, 3, 3>;
  
  // Inherit constructors
  using CameraView<T, MetaType>::CameraView;

  CameraMatrix camera_matrix() const {
    return this->meta_.camera_matrix.template cast<T>();
  }

  void set_camera_matrix(const CameraMatrix& K) {
    this->meta_.camera_matrix = K.template cast<double>();
    // Inverse camera projection matrix parameters
    Eigen::Matrix3d K_inv = this->meta_.camera_matrix.inverse();
    this->meta_.m_fx = this->meta_.camera_matrix(0,0);
    this->meta_.m_fy = this->meta_.camera_matrix(1,1);
    this->meta_.m_cx = this->meta_.camera_matrix(0,2);
    this->meta_.m_cy = this->meta_.camera_matrix(1,2);
    this->meta_.m_inv_K11 = 1.0 / this->meta_.m_fx ;
    this->meta_.m_inv_K13 = -this->meta_.m_cx / this->meta_.m_fx;
    this->meta_.m_inv_K22 = 1.0 / this->meta_.m_fy;
    this->meta_.m_inv_K23 = -this->meta_.m_cy / this->meta_.m_fy;
  }

  void set_distortion_params(double k1, double k2, double p1, double p2, double k3) {
    this->meta_.m_k1 = k1;
    this->meta_.m_k2 = k2;
    this->meta_.m_p1 = p1;
    this->meta_.m_p2 = p2;
    this->meta_.m_k3 = k3;
    bool do_distortion = std::abs(k1)>1e-5 || std::abs(k2)>1e-5 || std::abs(p1)>1e-5 || std::abs(p1)>1e-5;
    this->meta_.do_distortion = do_distortion;
  }

  std::vector<double> distortion_params(){
    std::vector<double> res = {};
    res.push_back( this->meta_.m_k1);
    res.push_back( this->meta_.m_k2);
    res.push_back( this->meta_.m_p1);
    res.push_back( this->meta_.m_p2);
    return res;
  }

  bool do_distortion() const{
    return this->meta_.do_distortion;
    //TODO(wz): if perform distortion, the optimization process will produce a nan error.
  }

  Result EvaluateProjection(const Vector3 &X, const Vector3 &dX, bool derive) const override {
    auto result = std::make_unique<CameraEvaluation<T>>(derive);
    Vector3 p = camera_matrix() * X;
    // result->y = p.head(2)/p(2);
    spaceToPlane(X, result->y);

    if (derive) {
      const T z_eps = T(1e-32);
      Vector3 dp = camera_matrix() * dX;
      T denominator = (p(2)*p(2)) + z_eps;
      result->dy(0) = ((dp(0) * p(2)) - (p(0)*dp(2))) / denominator;
      result->dy(1) = ((dp(1) * p(2)) - (p(1)*dp(2))) / denominator;
    }

    return result;
  }

  Vector3 Unproject(const Vector2 &y) const override {
    Vector3 xh(y(0), y(1), T(1));
    CameraMatrix K = camera_matrix();
    
    if(this->do_distortion()){
      Vector3 res;
      liftProjective(y, res);
      return res;
    }else{
      return camera_matrix().inverse() * xh;
    }
  }

  void liftSphere(const Vector2& p, Vector3& P) const{
    liftProjective(p, P);
    P.normalize();
  }

  void liftProjective(const Vector2& p, Vector3& P) const{
    T mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
    T rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    //double lambda;

    // Lift points to normalised plane
    mx_d = T(this->meta_.m_inv_K11) * p(0) + T(this->meta_.m_inv_K13);
    my_d = T(this->meta_.m_inv_K22) * p(1) + T(this->meta_.m_inv_K23);
    

    Vector2 y_ori;
    y_ori << mx_d, my_d;

    if(!this->do_distortion()){
      mx_u = mx_d;
      my_u = my_d;
    }else{
      // Apply inverse distortion model
      // proposed by Heikkila
      /* T k1 = T(this->meta_.m_k1);
      T k2 = T(this->meta_.m_k2);
      T p1 = T(this->meta_.m_p1);
      T p2 = T(this->meta_.m_p2);

      mx2_d = mx_d*mx_d;
      my2_d = my_d*my_d;
      mxy_d = mx_d*my_d;
      rho2_d = mx2_d+my2_d;
      rho4_d = rho2_d*rho2_d;
      radDist_d = k1*rho2_d+k2*rho4_d;
      Dx_d = mx_d*radDist_d + p2*(rho2_d+T(2)*mx2_d) + T(2)*p1*mxy_d;
      Dy_d = my_d*radDist_d + p1*(rho2_d+T(2)*my2_d) + T(2)*p2*mxy_d;
      inv_denom_d = T(1)/(T(1)+T(4)*k1*rho2_d+T(6)*k2*rho4_d+T(8)*p1*my_d+T(8)*p2*mx_d);

      mx_u = mx_d - inv_denom_d*Dx_d;
      my_u = my_d - inv_denom_d*Dy_d; */
      
      // Recursive distortion model
      int n = 8;
      Vector2 d_u;
      distortion(Vector2(mx_d, my_d), d_u);
      // Approximate value
      mx_u = mx_d - d_u(0);
      my_u = my_d - d_u(1);

      for (int i = 1; i < n; ++i){
        distortion(Vector2(mx_u, my_u), d_u);
        mx_u = mx_d - d_u(0);
        my_u = my_d - d_u(1);
        
        // Vector2 y_tmp;
        // y_tmp << mx_u, my_u;
        // Vector2 e(y_ori - y_tmp);
        // if(e.dot(e) < 1e-15)
        //   break;
      }
    }

    // Obtain a projective ray
    P << mx_u, my_u, T(1);
  }

    /**
   * \brief Apply distortion to input point (from the normalised plane)
   *
   * \param p_u undistorted coordinates of point on the normalised plane
   * \return to obtain the distorted point: p_d = p_u + d_u
   */
  void distortion(const Vector2& p_u, Vector2& d_u) const {
    T k1 = T(this->meta_.m_k1);
    T k2 = T(this->meta_.m_k2);
    T p1 = T(this->meta_.m_p1);
    T p2 = T(this->meta_.m_p2);
    T k3 = T(this->meta_.m_k3);

    T mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u + k3 * rho2_u * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + T(2.0) * p1 * mxy_u + p2 * (rho2_u + T(2.0) * mx2_u),
           p_u(1) * rad_dist_u + T(2.0) * p2 * mxy_u + p1 * (rho2_u + T(2.0) * my2_u);
  }

  void spaceToPlane(const Vector3& P, Vector2& p) const{
    const T eps = T(1e-32);
    Vector2 p_u, p_d;

    // Project points to the normalised plane
    p_u << P(0) /  (eps + P(2)), P(1) / (eps+P(2));
    // p_u << P(0) / P(2), P(1) / P(2);

    if (!this->do_distortion()){
      p_d = p_u;
    }else{
      // Apply distortion
      Vector2 d_u;
      distortion(p_u, d_u);
      p_d = p_u + d_u;
    }
     
    // Apply generalised projection matrix
    CameraMatrix K = camera_matrix();
    p << T(K(0, 0)) * p_d(0) + T(K(0, 2)),
         T(K(1, 1)) * p_d(1) + T(K(1, 2));
  }

};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class PinholeEntity : public CameraEntity<ViewTemplate, MetaType, StoreType> {
  using Base = CameraEntity<ViewTemplate, MetaType, StoreType>;
 public:
  using CameraMatrix = Eigen::Matrix3d;

  PinholeEntity(size_t rows, size_t cols, double readout, const CameraMatrix &camera_matrix) :
    Base(rows, cols, readout) {
    this->set_camera_matrix(camera_matrix);
  }

  PinholeEntity(size_t rows, size_t cols, double readout, 
      double k1, double k2, double p1, double p2, double k3,
      double fx, double fy, double cx, double cy) :
    Base(rows, cols, readout) {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    this->set_camera_matrix(K);
    this->set_distortion_params(k1, k2, p1, p2, k3);
  }

  PinholeEntity(size_t rows, size_t cols, double readout) :
      PinholeEntity(rows, cols, readout, Eigen::Matrix3d::Identity()) {
  }

  void AddToProblem(ceres::Problem &problem,
                    time_init_t times,
                    MetaType &meta,
                    std::vector<entity::ParameterInfo<double>> &parameters) const override {
    //Pinhole Camera直接使用SensorEntity的AddToProblem函数，仅优化与IMU的外参和时间偏置，K不做优化
    meta = this->meta_;
    Base::AddToProblem(problem, times, meta, parameters);
  }
};

} // namespace internal



class PinholeCamera : public internal::PinholeEntity<internal::PinholeView,
                                                     internal::PinholeMeta,
                                                     entity::DynamicParameterStore<double>> {
 public:
  using internal::PinholeEntity< internal::PinholeView,
                                 internal::PinholeMeta,
                                 entity::DynamicParameterStore<double>>::PinholeEntity;

  static constexpr const char *CLASS_ID = "PinholeCamera";
};

} // namespace sensors
} // namespace kontiki
#endif //KONTIKI_PINHOLE_CAMERA_H
