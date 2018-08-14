// GSLAM - A general SLAM framework and benchmark
// Copyright 2018 PILAB Inc. All rights reserved.
// https://github.com/zdzhaoyong/GSLAM
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: 18392360048@163.com (Boni Hu)
//
// This is the GSLAM main API header

#ifndef GSLAM_CORE_POINT_H_
#define GSLAM_CORE_POINT_H_

#include <Point.h>
#include <iostream>

#ifdef HAS_TOON
#include <TooN/TooN.h>
#endif

namespace pi {

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

template <class Precision> struct Point2_ {
  Point2_() : x(0), y(0) {}

  Point2_(Precision x_, Precision y_) : x(x_), y(y_) {}

  template <typename Scalar> operator Point2_<Scalar>() {
    return Point2_<Scalar>(x, y);
  }

#ifdef EIGEN_MATRIX_H
  template <typename Scalar> operator Eigen::Matrix<Scalar, 2, 1>() {
    return Eigen::Matrix<Scalar, 2, 1>(x, y);
  }

  template <typename Scalar>
  explicit Point2_(const Eigen::Matrix<Scalar, 2, 1> &eig)
      : x(eig[0]), y(eig[1]) {}
#endif

  inline Precision &operator[](int index) const {
    return ((reinterpret_cast<Precision *>)this)[index];
  }

  friend Point2_ operator+(const Point2_ &a, const Point2_ &b) {
    return Point2_(a.x + b.x, a.y + b.y);
  }

  friend Point2_ operator-(const Point2_ &a, const Point2_ &b) {
    return Point2_(a.x - b.x, a.y - b.y);
  }

  friend Point2_ operator-(const Point2_ &a) { return Point2_(-a.x, -a.y); }

  friend Precision operator*(const Point2_ &a, const Point2_ &b) {
    return (a.x * b.x + a.y * b.y);
  }

  friend Point2_ operator*(const Precision &a, const Point2_ &b) {
    return Point2_(a * b.x, a * b.y);
  }

  friend Point2_ operator*(const Point2_ &b, const Precision &a) {
    return Point2_(a * b.x, a * b.y);
  }

  friend Point2_ operator/(const Point2_ &a, const Precision &b) {
    return (1. / b) * a;
  }

  inline Precision norm() const { return sqrt(x * x + y * y); }

  inline Point2_<Precision> normalize() const {
    if (x * x + y * y != 0)
      return (*this) * (1. / norm());
    else
      return Point2_<Precision>(0, 0);
  }

  friend inline std::ostream &operator<<(std::ostream &os, const Point2_ &p) {
    os << p.x << " " << p.y;
    return os;
  }

  friend inline std::istream &operator>>(std::istream &is, Point2_ &p) {
    is >> p.x >> p.y;
    return is;
  }

  Precision x, y;
};

typedef Point2_<double> Point2d;
typedef Point2_<float> Point2f;
typedef Point2_<int> Point2i;

template <class Precision> struct Point3_ {
  Point3_() : x(0), y(0), z(0) {}

  Point3_(Precision x_, Precision y_, Precision z_) : x(x_), y(y_), z(z_) {}

  inline Precision &operator[](int index) const {
    return ((reinterpret_cast<Precision *>)this)[index];
  }

  inline Precision norm() const { return sqrt(x * x + y * y + z * z); }

  inline Precision dot(const Point3_ &a) { return x * a.x + y * a.y + z * a.z; }

  inline Point3_<Precision> cross(const Point3_ &a) {
    return Point3_<Precision>(y * a.z - z * a.y, z * a.x - x * a.z,
                              x * a.y - y * a.x);
  }

  inline Point3_<Precision> normalize() const {
    if (x * x + y * y + z * z != 0)
      return (*this) * (1. / norm());
    else
      return Point3_<Precision>(0, 0, 0);
  }

  friend inline std::ostream &operator<<(std::ostream &os, const Point3_ &p) {
    os << p.x << " " << p.y << " " << p.z;
    return os;
  }

  friend inline std::istream &operator>>(std::istream &is, Point3_ &p) {
    is >> p.x >> p.y >> p.z;
    return is;
  }

  friend Point3_ operator+(const Point3_ &a, const Point3_ &b) {
    return Point3_(a.x + b.x, a.y + b.y, a.z + b.z);
  }

  friend Point3_ operator-(const Point3_ &a, const Point3_ &b) {
    return Point3_(a.x - b.x, a.y - b.y, a.z - b.z);
  }

  friend Point3_ operator-(const Point3_ &a) {
    return Point3_(-a.x, -a.y, -a.z);
  }

  friend Precision operator*(const Point3_ &a, const Point3_ &b) {
    return (a.x * b.x + a.y * b.y + a.z * b.z);
  }

  friend Point3_<Precision> operator^(const Point3_ &a, const Point3_ &b) {
    return Point3_(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                   a.x * b.y - a.y * b.x);
  }

  friend Point3_ operator*(const Precision &a, const Point3_ &b) {
    return Point3_(a * b.x, a * b.y, a * b.z);
  }

  friend Point3_ operator*(const Point3_ &b, const Precision &a) {
    return Point3_(a * b.x, a * b.y, a * b.z);
  }

  friend Point3_ operator/(const Point3_ &a, const Precision &b) {
    return (1. / b) * a;
  }

  friend inline bool operator<(const Point3_ &a, const Point3_ b) {
    return a.x < b.x;
  }

  template <typename Scalar> operator Point3_<Scalar>() {
    return Point3_<Scalar>(x, y, z);
  }

#ifdef EIGEN_MATRIX_H
  template <typename Scalar> operator Eigen::Matrix<Scalar, 3, 1>() {
    return Eigen::Matrix<Scalar, 3, 1>(x, y, z);
  }

  template <typename Scalar>
  explicit Point3_(const Eigen::Matrix<Scalar, 3, 1> &eig)
      : x(eig[0]), y(eig[1]), z(eig[2]) {}
#endif

#ifdef HAS_TOON
  operator TooN::Vector<3, Precision> &() {
    return *((TooN::Vector<3, Precision> *)this);
  }
#endif

  Precision x, y, z;
};

typedef Point3_<unsigned char> Point3ub;
typedef Point3_<float> Point3f;
typedef Point3_<double> Point3d;
typedef Point3_<int> Point3i;
}  // namespace pi

namespace GSLAM {
using pi::Point2_;
using pi::Point3_;

typedef Point2_<int> Point2i;
typedef Point2_<float> Point2f;
typedef Point2_<double> Point2d;
typedef Point3_<unsigned char> Point3ub;
typedef Point3_<float> Point3f;
typedef Point3_<double> Point3d;
typedef Point3_<int> Point3i;
}  // namespace GSLAM

#endif  // GSLAM_CORE_POINT_H_
