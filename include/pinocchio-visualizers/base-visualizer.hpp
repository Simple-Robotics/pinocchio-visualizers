#pragma once

#include "namespace.hpp"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

#include <optional>

namespace pinocchio_visualizers {
namespace pin = pinocchio;

namespace {
using pin::GeometryData;
using pin::GeometryModel;
}  // namespace

/// @brief A base class for defining visualizers for Pinocchio in C++. This
/// provides basic building blocks (a base constructor, data members, getters
/// for the models).
/// @details The base API assumes that the visualizer is not the owner of the
/// underlying Pinocchio multibody, visual or collision models. Their lifetimes
/// should be managed by the application context itself.
/// @remark C++ port of the BaseVisualizer abstract class in Pinocchio's Python
/// bindings.
template <typename _Scalar>
class BaseVisualizer {
 public:
  typedef _Scalar Scalar;
  typedef pin::ModelTpl<Scalar> Model;
  typedef pin::DataTpl<Scalar> Data;
  enum { Options = Eigen::ColMajor };

  typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1, Options> VectorXs;
  typedef Eigen::Matrix<Scalar, 3, 1, Options> Vector3s;
  typedef Eigen::Matrix<Scalar, 4, 4, Options> Matrix4s;
  typedef pin::SE3Tpl<Scalar, Options> SE3;
  typedef Eigen::Ref<const VectorXs> ConstVectorRef;

  BaseVisualizer(const Model& model, const pin::GeometryModel& viz_model,
                 const pin::GeometryModel* collision_model = nullptr);

  /// @brief Initialize the viewer.
  virtual void initViewer() {}

  /// @brief Load the Pinocchio model.
  virtual void loadViewerModel() = 0;

  /// @brief Re-build data objects. Required if the models were modified.
  void rebuildData();

  /// @brief Display configuration @p q (if an actual value is given) or update
  /// the Pinocchio frames.
  virtual void display(const std::optional<ConstVectorRef>& q = std::nullopt);

  /// @brief Play an entire trajectory, waiting for time @p dt between each
  /// keyframe.
  virtual void play(const std::vector<ConstVectorRef>& qs, Scalar dt);

  /// @brief Override this in child class when the scene has to be redrawn.
  /// Useful for @ref play().
  virtual bool forceRedraw() { return true; }

  /// @brief Set the active camera target.
  virtual void setCameraTarget(const Eigen::Ref<const Vector3s>& /*target*/) {}

  /// @brief Set the active camera position.
  virtual void setCameraPosition(
      const Eigen::Ref<const Vector3s>& /*position*/) {}

  /// @brief Set the active camera 6D pose.
  virtual void setCameraPose(const Eigen::Ref<const Matrix4s>& /*pose*/) {}

  /// @copybrief setCameraPose()
  inline void setCameraPose(const SE3& pose) {
    this->setCameraPose(pose.toHomogeneousMatrix());
  }

  /// @brief Set camera zoom level; what this means depends on the
  /// implementation (FOV zoom or moving forwards).
  virtual void setCameraZoom(Scalar /*value*/) {}

  /// @brief Enable/disable controlling the camera from keyboard and mouse.
  virtual void enableCameraControl(bool) {}

  /// @brief Delete all objects from the scene.
  virtual void clean() {}

  virtual ~BaseVisualizer() = default;

  const Model& model() const { return *m_model; }
  const GeometryModel& visualModel() const { return *m_visualModel; }
  const GeometryModel* collisionModel() const { return m_collisionModel; }

 protected:
  Model const* m_model;
  GeometryModel const* m_visualModel;
  GeometryModel const* m_collisionModel;

  virtual void displayImpl() = 0;

 public:
  Data data;
  GeometryData visualData;
  GeometryData collisionData;
};

}  // namespace pinocchio_visualizers

#include "base-visualizer.hxx"
