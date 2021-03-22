#pragma once

#include <geometry/camera_distortions_functions.h>
#include <geometry/camera_projections_functions.h>
#include <geometry/transformations_functions.h>

namespace geometry {
enum class ProjectionType {
  PERSPECTIVE,
  BROWN,
  FISHEYE,
  FISHEYE_OPENCV,
  FISHEYE62,
  SPHERICAL,
  DUAL,
  RADIAL,
  SIMPLE_RADIAL,
  NONE,
};

template <class FUNC>
struct ForwardWrapper : public FUNC {
  template <class T>
  static void Apply(const T* in, const T* parameters, T* out) {
    FUNC::Forward(in, parameters, out);
  }
};

template <class FUNC>
struct BackwardWrapper : public FUNC {
  template <class T>
  static void Apply(const T* in, const T* parameters, T* out) {
    FUNC::Backward(in, parameters, out);
  }
};

struct ProjectPoseDerivatives {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* projected,
                    T* jacobian) {
    ComposeForwardDerivatives<T, true, TYPE, PoseFunctor>(point, parameters,
                                                          projected, jacobian);
  }
};

struct ProjectPosePointDerivatives {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* projected,
                    T* jacobian) {
    ComposeForwardDerivatives<T, false, TYPE, PoseFunctor>(point, parameters,
                                                           projected, jacobian);
  }
};

struct ProjectRigPoseDerivatives {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* projected,
                    T* jacobian) {
    ComposeForwardDerivatives<T, true, TYPE, PoseFunctor, PoseFunctor>(
        point, parameters, projected, jacobian);
  }
};

struct PoseNormalizedDerivatives {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* projected,
                    T* jacobian) {
    ComposeForwardDerivatives<T, true, Normalize, PoseFunctor>(
        point, parameters, projected, jacobian);
  }
};

struct ProjectFunction {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* projected) {
    TYPE::Forward(point, parameters, projected);
  }
};

struct ProjectDerivativesFunction {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* projected,
                    T* jacobian) {
    TYPE::template ForwardDerivatives<T, true>(point, parameters, projected,
                                               jacobian);
  }
};

struct BearingFunction {
  template <class TYPE, class T>
  static void Apply(const T* point, const T* parameters, T* bearing) {
    TYPE::Backward(point, parameters, bearing);
  }
};

/* This struct helps define most cameras models as they tend to follow the
 * pattern PROJ - > DISTO -> AFFINE. However, its is not mandatory for any
 * camera model to follow it. You can add any new camera models as long as it
 * implements the Forward and Backward functions. */

/* Here's some trait that defines where to look for parameters that follows the
 * generic scheme */
template <class T>
struct FunctorTraits {
  static constexpr int Size = T::ParamSize;
};
template <>
struct FunctorTraits<SphericalProjection> {
  static constexpr int Size = 0;
};

template <class PROJ, class DISTO, class AFF>
struct SizeTraits {
  static constexpr int ConstexprMax(int a, int b) { return (a < b) ? b : a; }
  static constexpr int Size =
      ConstexprMax(1, FunctorTraits<PROJ>::Size + FunctorTraits<AFF>::Size +
                          FunctorTraits<DISTO>::Size);
};

/* Finally, here's the generic camera that implements the PROJ - > DISTO ->
 * AFFINE pattern. */
template <class PROJ, class DISTO, class AFF>
struct ProjectGeneric : Functor<3, SizeTraits<PROJ, DISTO, AFF>::Size, 2> {
  using ProjectionType = PROJ;
  using DistoType = DISTO;
  using AffineType = AFF;

  static constexpr int Size = SizeTraits<PROJ, DISTO, AFF>::Size;

  template <class T>
  static void Forward(const T* point, const T* parameters, T* projected) {
    ComposeFunctions<T, ForwardWrapper<AFF>, ForwardWrapper<DISTO>,
                     ForwardWrapper<PROJ>>(point, parameters, projected);
  }

  template <class T, bool DERIV_PARAMS>
  static void ForwardDerivatives(const T* point, const T* parameters,
                                 T* projected, T* jacobian) {
    ComposeForwardDerivatives<T, DERIV_PARAMS, AFF, DISTO, PROJ>(
        point, parameters, projected, jacobian);
  }

  template <class T>
  static void Backward(const T* point, const T* parameters, T* bearing) {
    T parameters_backward[Size];
    ConstructReversedParams(parameters, parameters_backward);
    ComposeFunctions<T, BackwardWrapper<PROJ>, BackwardWrapper<DISTO>,
                     BackwardWrapper<AFF>>(point, parameters_backward, bearing);
  }

 private:
  template <class T>
  static void ConstructReversedParams(const T* parameters_forward,
                                      T* parameters_backward) {
    int count = 0;
    int index = Size - FunctorTraits<PROJ>::Size;
    for (int i = 0; i < FunctorTraits<PROJ>::Size; ++i) {
      parameters_backward[index + i] = parameters_forward[count++];
    }
    index -= FunctorTraits<DISTO>::Size;
    for (int i = 0; i < FunctorTraits<DISTO>::Size; ++i) {
      parameters_backward[index + i] = parameters_forward[count++];
    }
    index -= FunctorTraits<AFF>::Size;
    for (int i = 0; i < FunctorTraits<AFF>::Size; ++i) {
      parameters_backward[index + i] = parameters_forward[count++];
    }
  }
};

using PerspectiveCamera =
    ProjectGeneric<PerspectiveProjection, Disto24, UniformScale>;
using RadialCamera = ProjectGeneric<PerspectiveProjection, Disto24, Affine>;
using SimpleRadialCamera =
    ProjectGeneric<PerspectiveProjection, Disto2, Affine>;
using BrownCamera = ProjectGeneric<PerspectiveProjection, DistoBrown, Affine>;
using FisheyeCamera = ProjectGeneric<FisheyeProjection, Disto24, UniformScale>;
using FisheyeOpencvCamera =
    ProjectGeneric<FisheyeProjection, Disto2468, Affine>;
using Fisheye62Camera = ProjectGeneric<FisheyeProjection, Disto62, Affine>;
using DualCamera = ProjectGeneric<DualProjection, Disto24, UniformScale>;
using SphericalCamera = ProjectGeneric<SphericalProjection, Identity, Identity>;

/* This is where the pseudo-strategy pattern takes place. If you want to add
 * your own new camera model, just add a new enum value, the corresponding
 * case below and the implementation (see above). */
template <class FUNC, class... IN>
void Dispatch(const ProjectionType& type, IN&&... args) {
  switch (type) {
    case ProjectionType::PERSPECTIVE:
      FUNC::template Apply<PerspectiveCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::BROWN:
      FUNC::template Apply<BrownCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::FISHEYE:
      FUNC::template Apply<FisheyeCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::FISHEYE_OPENCV:
      FUNC::template Apply<FisheyeOpencvCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::FISHEYE62:
      FUNC::template Apply<Fisheye62Camera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::RADIAL:
      FUNC::template Apply<RadialCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::SIMPLE_RADIAL:
      FUNC::template Apply<SimpleRadialCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::DUAL:
      FUNC::template Apply<DualCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::SPHERICAL:
      FUNC::template Apply<SphericalCamera>(std::forward<IN>(args)...);
      break;
    case ProjectionType::NONE:
    default:
      throw std::runtime_error("Invalid ProjectionType");
  }
};
}  // namespace geometry
