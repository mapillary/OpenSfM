#pragma once

#include <bundle/data/camera.h>
#include <bundle/data/data.h>
#include <bundle/data/pose.h>

namespace bundle {

using RigCamera = Pose;
using RigInstance = Pose;

struct RigShot : public DataContainer {
  RigShot(const std::string &id, bundle::Camera *camera, RigCamera *rig_camera,
          RigInstance *rig_instance)
      : DataContainer(id) {
    RegisterData("camera", camera);
    RegisterData("rig_camera", rig_camera);
    RegisterData("rig_instance", rig_instance);
  }

  bundle::Camera *GetCamera() {
    return static_cast<bundle::Camera *>(GetData("camera"));
  }
  RigCamera *GetRigCamera() {
    return static_cast<RigCamera *>(GetData("rig_camera"));
  }
  RigInstance *GetRigInstance() {
    return static_cast<RigInstance *>(GetData("rig_instance"));
  }
};
}  // namespace bundle
