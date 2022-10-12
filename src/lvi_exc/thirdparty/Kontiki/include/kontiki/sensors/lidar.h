#ifndef LIDAR_H
#define LIDAR_H

#include "sensors.h"

namespace kontiki {
namespace sensors {

namespace internal {

struct LiDARMeta : public SensorMeta {

};


template<typename T>
struct LiDAREvaluation {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
};


template<typename T, typename MetaType>
class LiDARView : public SensorView<T, MetaType> {
 protected:
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Result = std::unique_ptr<LiDAREvaluation<T>>;
 public:
  using SensorView<T, MetaType>::SensorView;
};


// Base class for LiDAR entities
template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class LiDAREntity : public SensorEntity<ViewTemplate, MetaType, StoreType> {
 public:
  LiDAREntity() {}

};



} // namespace detail
} // namespace sensors



namespace type {
template<typename _Entity, typename T>
using LiDAR = typename entity::type::base::ForView<_Entity, sensors::internal::LiDARView, T>;
} // namespace type

} // namespace kontiki






#endif // LIDAR_H
