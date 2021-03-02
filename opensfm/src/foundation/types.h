#pragma once

#include <Eigen/Eigen>

template <class T>
using MatX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
using MatXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using MatXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

template <class T>
using Mat2 = Eigen::Matrix<T, 2, 2>;
using Mat2f = Eigen::Matrix<float, 2, 2>;
using Mat2d = Eigen::Matrix<double, 2, 2>;

template <class T>
using Mat3 = Eigen::Matrix<T, 3, 3>;
using Mat3f = Eigen::Matrix<float, 3, 3>;
using Mat3d = Eigen::Matrix<double, 3, 3>;

template <class T>
using Mat4 = Eigen::Matrix<T, 4, 4>;
using Mat4f = Eigen::Matrix<float, 4, 4>;
using Mat4d = Eigen::Matrix<double, 4, 4>;

template <class T>
using Mat34 = Eigen::Matrix<T, 3, 4>;
using Mat34f = Eigen::Matrix<float, 3, 4>;
using Mat34d = Eigen::Matrix<double, 3, 4>;

template <class T>
using MatX2 = Eigen::Matrix<T, Eigen::Dynamic, 2>;
using MatX2f = Eigen::Matrix<float, Eigen::Dynamic, 2>;
using MatX2d = Eigen::Matrix<double, Eigen::Dynamic, 2>;
using MatX2i = Eigen::Matrix<int, Eigen::Dynamic, 2>;

template <class T>
using MatX3 = Eigen::Matrix<T, Eigen::Dynamic, 3>;
using MatX3f = Eigen::Matrix<float, Eigen::Dynamic, 3>;
using MatX3d = Eigen::Matrix<double, Eigen::Dynamic, 3>;
using MatX3i = Eigen::Matrix<int, Eigen::Dynamic, 3>;

template <class T>
using MatX4 = Eigen::Matrix<T, Eigen::Dynamic, 4>;
using MatX4f = Eigen::Matrix<float, Eigen::Dynamic, 4>;
using MatX4d = Eigen::Matrix<double, Eigen::Dynamic, 4>;
using MatX4i = Eigen::Matrix<int, Eigen::Dynamic, 4>;

template <class T>
using VecX = Eigen::Matrix<T, Eigen::Dynamic, 1>;
using VecXf = Eigen::Matrix<float, Eigen::Dynamic, 1>;
using VecXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using VecXi = Eigen::Matrix<int, Eigen::Dynamic, 1>;

template <class T, int N>
using VecN = Eigen::Matrix<T, N, 1>;
template <int N>
using VecNf = Eigen::Matrix<float, N, 1>;
template <int N>
using VecNd = Eigen::Matrix<double, N, 1>;
template <int N>
using VecNi = Eigen::Matrix<int, N, 1>;

template <class T>
using Vec1 = Eigen::Matrix<T, 1, 1>;
using Vec1f = Eigen::Matrix<float, 1, 1>;
using Vec1d = Eigen::Matrix<double, 1, 1>;

template <class T>
using Vec2 = Eigen::Matrix<T, 2, 1>;
using Vec2f = Eigen::Matrix<float, 2, 1>;
using Vec2d = Eigen::Matrix<double, 2, 1>;
using Vec2i = Eigen::Matrix<int, 2, 1>;

template <class T>
using Vec3 = Eigen::Matrix<T, 3, 1>;
using Vec3f = Eigen::Matrix<float, 3, 1>;
using Vec3d = Eigen::Matrix<double, 3, 1>;
using Vec3i = Eigen::Matrix<int, 3, 1>;

template <class T>
using Vec4 = Eigen::Matrix<T, 4, 1>;
using Vec4f = Eigen::Matrix<float, 4, 1>;
using Vec4d = Eigen::Matrix<double, 4, 1>;
using Vec4i = Eigen::Matrix<int, 4, 1>;
