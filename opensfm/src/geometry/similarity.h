#pragma once
#include <geometry/pose.h>

namespace geometry {
class Similarity {
 public:
  Similarity() = default;
  Similarity(const Vec3d& R, const Vec3d& t, double s) : Rt_(R, t), scale_(s) {}

  Vec3d Translation() const { return Rt_.TranslationWorldToCamera(); }
  void SetTranslation(const Vec3d& t) { Rt_.SetWorldToCamTranslation(t); }

  Vec3d Rotation() const { return Rt_.RotationWorldToCameraMin(); }
  Mat3d RotationMatrix() const { return Rt_.RotationWorldToCamera(); }
  void SetRotation(const Vec3d& r) { Rt_.SetWorldToCamRotation(r); }

  double Scale() const { return scale_; }
  void SetScale(double s) { scale_ = s; }

  Vec3d Transform(const Vec3d& point) const {
    return scale_ * RotationMatrix() * point + Translation();
  }

 private:
  geometry::Pose Rt_;
  double scale_{1.0};  // [s * R, t] world to cam
};

}  // namespace geometry
