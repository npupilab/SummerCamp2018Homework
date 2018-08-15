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
// Author: 420985011@qq.com (Cris Cheng)
//
// This is the GSLAM main API header

#ifndef CPP_STYLE___________VECPARAMENT_H_
#define CPP_STYLE___________VECPARAMENT_H_

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

template <typename VarType>
class VecParament {
 public:
  explicit VecParament(std::string str = "") { fromString(str); }
  explicit VecParament(const std::vector<VarType>& vars) : data(vars) {}

  VecParament(int n, const VarType& defV) : data(n, defV) {}

  size_t size() { return data.size(); }

  VarType& operator[](size_t idx) { return data[idx]; }

  bool fromString(std::string str) {
    data.clear();
    if (str.empty()) return false;
    std::string::size_type start, stop;
    start = str.find('[');
    stop = str.find(']');
    if (start == std::string::npos || stop == std::string::npos ||
        start >= stop) {
    } else {
      str = str.substr(start + 1, stop - 1);
    }
    bool hasDot = (str.find(',') != std::string::npos);
    char alige;
    if (hasDot)
      alige = ',';
    else
      alige = ' ';

    std::string str_num;
    while (str.size()) {
      std::string::size_type n = str.find(alige);
      if (n != std::string::npos) {
        str_num = str.substr(0, n);
        str = str.substr(n + 1);
      } else {
        str_num = str;
        str = "";
      }
      if (str_num == "" || str_num == " ") continue;

      std::istringstream iss(str_num);
      VarType x;
      iss >> x;
      data.push_back(x);
    }
    return true;
  }

  std::string toString() {
    std::ostringstream ost;
    ost << "[";
    for (size_t i = 0; i < data.size() - 1; i++) ost << data.at(i) << " ";
    ost << data.at(size() - 1) << "]";
    return ost.str();
  }

  friend inline std::istream& operator>>(std::istream& is, VecParament& p) {
    std::string line;
    std::getline(is, line);
    p.fromString(line);
    return is;
  }

  friend inline std::ostream& operator<<(std::ostream& os, VecParament& p) {
    os << p.toString();
    return os;
  }

  std::vector<VarType> data;
};

#endif  // CPP_STYLE___________VECPARAMENT_H_
