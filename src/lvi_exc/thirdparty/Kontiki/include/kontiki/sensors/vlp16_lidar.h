#ifndef VLP16_LIDAR_H
#define VLP16_LIDAR_H


#include <iostream>
#include <Eigen/Dense>

#include "lidar.h"
#include <entity/paramstore/dynamic_pstore.h>

namespace kontiki {
namespace sensors {

namespace internal {

struct VLP16Meta : public LiDARMeta {
  size_t NumParameters() const override {
    return LiDARMeta::NumParameters();
  }
};

template<typename T, typename MetaType>
class VLP16View : public LiDARView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Result = std::unique_ptr<LiDAREvaluation<T>>;
 public:

  // Inherit constructors
  using LiDARView<T, MetaType>::LiDARView;
};

template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class VLP16Entity : public LiDAREntity<ViewTemplate, MetaType, StoreType> {
  using Base = LiDAREntity<ViewTemplate, MetaType, StoreType>;
 public:

  VLP16Entity() : Base() {}

  void AddToProblem(ceres::Problem &problem,
                    time_init_t times,
                    MetaType &meta,
                    std::vector<entity::ParameterInfo<double>> &parameters) const override {
    meta = this->meta_;
    Base::AddToProblem(problem, times, meta, parameters);
  } 
};

} // namespace internal


class VLP16LiDAR : public internal::VLP16Entity<internal::VLP16View,
                                                internal::VLP16Meta,
                                                entity::DynamicParameterStore<double> > {
 public:
  using internal::VLP16Entity< internal::VLP16View,
                               internal::VLP16Meta,
                               entity::DynamicParameterStore<double>>::VLP16Entity;

  static constexpr const char *CLASS_ID = "VLP16LiDAR";
};



} // namespace sensors
} // namespace kontiki

#endif // VLP16_LIDAR_H
