#ifndef KONTIKIV2_IMU_H
#define KONTIKIV2_IMU_H

#include <Eigen/Dense>

#include "sensors.h"
#include <entity/entity.h>
#include <kontiki/trajectories/trajectory.h>
#include <kontiki/types.h>
#include <kontiki/constants.h>
#include <entity/paramstore/empty_pstore.h>
#include <entity/paramstore/dynamic_pstore.h>

namespace kontiki {
namespace sensors {

namespace internal {

struct ImuMeta : public SensorMeta {
  size_t NumParameters() const override {
    return SensorMeta::NumParameters() + 2;
  }
};

static const double STANDARD_GRAVITY = -9.79;

// Base Imu view using CRTP to access the correct Gyroscope()/Accelerometer() methods
// All IMU views must inherit from this one directly, and not through subclasses.
template<typename T, typename MetaType, typename Derived>
class ImuView : public SensorView<T, MetaType> {
  using Vector3 = Eigen::Matrix<T, 3, 1>;
  using Matrix3 = Eigen::Matrix<T, 3, 3>;
  using Flags = trajectories::EvaluationFlags;
 protected:
    const size_t PARAM_GRAVITY_ROLL = 3;
    const size_t PARAM_GRAVITY_PITCH = 4;
 public:
  // Import constructor
  using SensorView<T, MetaType>::SensorView;

  T& gravity_orientation_roll() const {
      auto ptr = this->pstore_->ParameterData(PARAM_GRAVITY_ROLL);
      return *ptr;
  }

  void set_gravity_orientation_roll(T gr) {
      auto ptr = this->pstore_->ParameterData(PARAM_GRAVITY_ROLL);
      *ptr = gr;
  }

  T& gravity_orientation_pitch() const {
      auto ptr = this->pstore_->ParameterData(PARAM_GRAVITY_PITCH);
      return *ptr;
  }

  void set_gravity_orientation_pitch(T gp) {
      auto ptr = this->pstore_->ParameterData(PARAM_GRAVITY_PITCH);
      *ptr = gp;
  }

  Vector3 refined_gravity() const {
      T cosRoll = ceres::cos(this->gravity_orientation_roll());
      T sinRoll = ceres::sin(this->gravity_orientation_roll());
      T cosPitch = ceres::cos(this->gravity_orientation_pitch());
      T sinPitch = ceres::sin(this->gravity_orientation_pitch());

      return Vector3(-sinPitch * cosRoll * T(STANDARD_GRAVITY),
                      sinRoll * T(STANDARD_GRAVITY),
                     -cosRoll * cosPitch * T(STANDARD_GRAVITY));
  }

  // Accelerometer measurement (exploits CRTP)
  template<typename TrajectoryModel>
  Vector3 Accelerometer(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    return static_cast<const Derived*>(this)->template Accelerometer<TrajectoryModel>(trajectory, t);
  }

  // Gyroscope measurement (exploits CRTP)
  template<typename TrajectoryModel>
  Vector3 Gyroscope(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    return static_cast<const Derived*>(this)->template Gyroscope<TrajectoryModel>(trajectory, t);
  }

 protected:
  // Standard gyroscope function
  template<typename TrajectoryModel>
  Vector3 StandardGyroscope(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    auto result = trajectory.Evaluate(t + this->time_offset(), Flags::EvalOrientation | Flags::EvalAngularVelocity);
    // Rotate from world to body coordinate frame
    return result->orientation.conjugate()*result->angular_velocity;
  }

  // Standard gyroscope function
  template<typename TrajectoryModel>
  Vector3 StandardAccelerometer(const type::Trajectory<TrajectoryModel, T> &trajectory, T t) const {
    auto result = trajectory.Evaluate(t + this->time_offset(), Flags::EvalOrientation | Flags::EvalAcceleration);
    return result->orientation.conjugate() * (result->acceleration + this->refined_gravity());
   // return result->orientation.conjugate() * (result->acceleration + this->relative_orientation() * Constants<T>::Gravity);


  }

};

//template<typename T, typename MetaType, typename Derived>
//const Eigen::Matrix<T, 3, 1> ImuView<T, MetaType, Derived>::GRAVITY = Eigen::Matrix<T, 3, 1>(T(0), T(0), T(-STANDARD_GRAVITY));


template<template<typename...> typename ViewTemplate, typename MetaType, typename StoreType>
class ImuEntity : public SensorEntity<ViewTemplate, MetaType, StoreType> {
  using Vector3 = Eigen::Vector3d;
  using Base = SensorEntity<ViewTemplate, MetaType, StoreType>;
 public:
   ImuEntity(double g_r, double g_p) :
       Base() {
       // Define parameters
       this->pstore_->AddParameter(1);

       this->pstore_->AddParameter(1);

       this->set_gravity_orientation_roll(g_r);

       this->set_gravity_orientation_pitch(g_p);
   }

   ImuEntity() :
       ImuEntity(0.01, 0.01) {}

   void AddToProblem(ceres::Problem &problem,
                     time_init_t times,
                     MetaType &meta,
                     std::vector<entity::ParameterInfo<double>> &parameters) const override {
     Base::AddToProblem(problem, times, meta, parameters);

     auto p_gravity_roll = this->pstore_->Parameter(this->PARAM_GRAVITY_ROLL);
     problem.AddParameterBlock(p_gravity_roll.data, p_gravity_roll.size, p_gravity_roll.parameterization);
     parameters.push_back(p_gravity_roll);

     auto p_gravity_pitch = this->pstore_->Parameter(this->PARAM_GRAVITY_PITCH);
     problem.AddParameterBlock(p_gravity_pitch.data, p_gravity_pitch.size, p_gravity_pitch.parameterization);
     parameters.push_back(p_gravity_pitch);
   }


};

} // namespace detail
} // namespace sensors

// Type specifier to get an Imu instance
namespace type {
template<typename E, typename T>
using Imu = typename sensors::internal::ImuView<T,
                                                typename E::Meta,
                                                typename E::template View<T, typename E::Meta>>;
}

} // namespace kontiki

#endif //KONTIKIV2_IMU_H
