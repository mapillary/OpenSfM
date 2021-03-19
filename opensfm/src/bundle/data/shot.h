#pragma once

#include <bundle/data/camera.h>
#include <bundle/data/data.h>
#include <bundle/data/pose.h>

namespace bundle {

struct Shot : public DataContainer {
  Shot(const std::string &id, Camera *camera, const geometry::Pose &pose)
      : DataContainer(id), pose_(id, pose, geometry::Pose(), geometry::Pose()) {
    RegisterData("camera", camera);
    RegisterData("pose", &pose_);
  }
  Shot(const std::string &id, const geometry::Pose &pose)
      : DataContainer(id), pose_(id, pose, geometry::Pose(), geometry::Pose()) {
    RegisterData("pose", &pose_);
  }

  Pose *GetPose() { return static_cast<Pose *>(GetData("pose")); }
  const Pose *GetPose() const {
    return static_cast<const Pose *>(GetData("pose"));
  }
  Camera *GetCamera() { return static_cast<Camera *>(GetData("camera")); }
  const Camera *GetCamera() const {
    return static_cast<const Camera *>(GetData("camera"));
  }

 private:
  Pose pose_;
};
}  // namespace bundle
