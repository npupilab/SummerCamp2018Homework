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
// Author:xulei   email:xlnwpu@gmail.com
//
// This is the GSLAM main API header
#ifndef GSLAM_ARRAY_H  // NOLINT
#define GSLAM_ARRAY_H
#include <iostream>
namespace pi {

template <class Type, int Size>
struct Array_ {
  Array_() {}
  explicit Array_(Type def) {
    for (int i = 0; i < Size; i++) data[i] = def;
  }
  Type data[Size];

  inline friend std::ostream& operator<<(std::ostream& os,
                                         const Array_<Type, Size>& p) {
    for (int i = 0; i < Size; i++) os << p.data[i] << " ";
    return os;
  }

  inline friend std::istream& operator>>(std::istream& os,
                                         const Array_<Type, Size>& p) {
    for (int i = 0; i < Size; i++) os >> p.data[i];
    return os;
  }

  const int size() { return Size; }
};

template <int Size>
struct Array_<float, Size> {
#ifdef EIGEN_MATRIX_H
  operator Eigen::Matrix<float, Size, 1>&() {
    return *(Eigen::Matrix<float, Size, 1>*)this;
  }

  operator Eigen::Matrix<float, Size, 1>() const {
    return *(Eigen::Matrix<float, Size, 1>*)this;
  }
#endif

  Array_() {}
  explicit Array_(float def) {
    for (int i = 0; i < Size; i++) data[i] = def;
  }
  float data[Size];

  inline friend std::ostream& operator<<(std::ostream& os,
                                         const Array_<float, Size>& p) {
    for (int i = 0; i < Size; i++) os << p.data[i] << " ";
    return os;
  }

  inline friend std::istream& operator>>(std::istream& os,
                                         const Array_<float, Size>& p) {
    for (int i = 0; i < Size; i++) os >> p.data[i];
    return os;
  }

  const int size() { return Size; }
};

template <int Size>
struct Array_<double, Size> {
  Array_() {}
  explicit Array_(double def) {
    for (int i = 0; i < Size; i++) data[i] = def;
  }
  double data[Size];

  inline friend std::ostream& operator<<(std::ostream& os,
                                         const Array_<double, Size>& p) {
    for (int i = 0; i < Size; i++) os << p.data[i] << " ";
    return os;
  }

  inline friend std::istream& operator>>(std::istream& os,
                                         const Array_<double, Size>& p) {
    for (int i = 0; i < Size; i++) os >> p.data[i];
    return os;
  }

  const int size() { return Size; }

#ifdef EIGEN_MATRIX_H
  operator Eigen::Matrix<double, Size, 1>&() {
    return *(Eigen::Matrix<double, Size, 1>*)this;
  }

  operator Eigen::Matrix<double, Size, 1>() const {
    return *(Eigen::Matrix<double, Size, 1>*)this;
  }
#endif
};
template <int Size = 2>
struct Byte {
  unsigned char data[Size];
};

}  // end namespace pi

#endif  // GSLAM_ARRAY_H   //NOLINT
