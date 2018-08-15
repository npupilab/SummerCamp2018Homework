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
// Author: 1529901761@qq.com (tongpinmo)
//
// This is the GSLAM main API header

#ifndef GSLAM_CORE_RANDOM_H_
#define GSLAM_CORE_RANDOM_H_

#include <cstdlib>

namespace GSLAM {

class Random {
 public:
  Random();

  /**
   * Returns a random int in the range [min..max]
   * @param min
   * @param max
   * @return random int in [min..max]
   */
  static int RandomInt(int min, int max);

  /**
   * Returns a random number in the range [0..1]
   * @return random T number in [0..1]
   */
  template <class T>
  static T RandomValue() {
    return (T)rand() / (T)RAND_MAX;
  }

  /**
   * Returns a random number in the range [min..max]
   * @param min
   * @param max
   * @return random T number in [min..max]
   */
  template <class T>
  static T RandomValue(T min, T max) {
    return Random::RandomValue<T>() * (max - min) + min;
  }

  /**
   * Returns a random number from a gaussian distribution
   * @param mean
   * @param sigma standard deviation
   */
  template <class T>
  static T RandomGaussianValue(T mean = 0., T sigma = 1.) {
    // Box-Muller transformation
    T x1, x2, w, y1;

    do {
      x1 = (T)2. * RandomValue<T>() - (T)1.;
      x2 = (T)2. * RandomValue<T>() - (T)1.;
      w = x1 * x1 + x2 * x2;
    } while (w >= (T)1. || w == (T)0.);

    w = sqrt(((T)-2.0 * log(w)) / w);
    y1 = x1 * w;

    return (mean + y1 * sigma);
  }
};

inline Random::Random() {}

inline int Random::RandomInt(int min, int max) {
  int d = max - min + 1;
  return static_cast<int>((static_cast<double>(
                               rand()) /  // NOLINT FIXME: convert to rand_r()
                           (static_cast<double>(RAND_MAX) + 1.0)) *
                          d) +
         min;
}

}  // namespace GSLAM
#endif  // GSLAM_CORE_RANDOM_H_
