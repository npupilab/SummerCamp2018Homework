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

#ifndef GSLAM_FILERESOURCE_H  // NOLINT
#define GSLAM_FILERESOURCE_H

#include <GSLAM/core/SPtr.h>
#include <GSLAM/core/Svar.h>
#include <cstdio>
#include <fstream>
#include <istream>
#include <string>
#include <utility>
#include <vector>

#define GSLAM_REGISTER_RESOURCE(R)                                  \
  class ResourceRegister##R {                                       \
   public:                                                          \
    ResourceRegister##R() {                                         \
      GSLAM::FileResource::Register(resource_names, resource_index, \
                                    resource_data);                 \
    }                                                               \
  } ResourceRegister##R##inst;
typedef unsigned char u_char;
namespace GSLAM {

class FileResource {
 public:
  typedef std::vector<std::pair<std::string, std::string> > FileLUT;

  static std::string toHex(u_char* buf, int size) {
    std::string str;
    str.resize(5 * size, '0');
    for (int i = 0; i < size; i++) {
      u_char c = buf[i];
      std::snprintf(&str.at(5 * i), "0x%02x,", c);
    }
    return str;
  }

  static bool exportResourceFile(const FileLUT& lut, const std::string& to) {
    std::vector<std::string> names;
    std::vector<int> idxes;
    std::vector<u_char> data;

    // load
    for (int i = 0; i < lut.size(); i++) {
      std::pair<std::string, std::string> corr = lut[i];
      FILE* in = fopen(corr.first.c_str(), "r");
      if (!in) {
        std::cerr << "Failed to load file " << corr.first << std::endl;
        continue;
      }
      while (true) {
        int ch = fgetc(in);
        if (ch == EOF) break;
        data.push_back(ch);
      }
      fclose(in);
      int finishIdx = data.size();
      names.push_back(corr.second);
      idxes.push_back(finishIdx);
    }
    if (data.empty()) return false;

    // export
    std::ofstream fileOut(to, std::ios::out);
    if (!fileOut.is_open()) return false;
    fileOut << "#include \"GSLAM/core/FileResource.h\"\n\n\n\n";

    fileOut << "static const std::string resource_names[]={\n";
    for (int i = 0; i < names.size(); i++) {
      fileOut << "\"" << names[i] << "\",\n";
    }
    fileOut << "\"\"};\n\n";

    fileOut << "static const int resource_index[]={";
    for (int i = 0; i < idxes.size(); i++) {
      fileOut << idxes[i];
      if (i + 1 != names.size())
        fileOut << ",\n";
      else
        fileOut << "};\n\n";
    }

    fileOut << "static const unsigned char resource_data[]= {\n";
    for (int i = 0; i < data.size(); i += 16) {
      if (i + 16 <= data.size())
        fileOut << toHex(&data[i], 16) << "\n";
      else
        fileOut << toHex(&data[i], data.size() - i) << "\n";
    }
    fileOut << "};\n\n";
    fileOut << "GSLAM_REGISTER_RESOURCE(" << GSLAM::Svar::getBaseName(to)
            << ")";
    fileOut.close();
    return true;
  }

  struct FileBuffer {
    explicit FileBuffer(const u_char* b = NULL, int sz = 0)
        : buf(b), size(sz) {}
    const u_char* buf;
    int size;
  };

  static void Register(const std::string resource_names[],
                       const int resource_index[],
                       const unsigned char resource_data[]) {
    for (int i = 0;; i++) {
      std::string name = resource_names[i];
      if (name.empty()) break;
      int start = i == 0 ? 0 : resource_index[i - 1];
      int end = resource_index[i];
      GSLAM::SvarWithType<FileBuffer>::instance().insert(
          name, {&resource_data[start], end - start});
    }
  }
  static char* ccchar(const char* cch) {
    char* pcch = reinterpret_cast<char*> cch;
    return pcch;
  }

  static std::pair<char*, int> getResource(const std::string& resource) {
    FileBuffer buf = GSLAM::SvarWithType<FileBuffer>::instance()[resource];
    if (!buf.buf || buf.size <= 0) std::pair<char*, int>(NULL, 0);
    return std::pair<char*, int>(ccchar((const char*)buf.buf), buf.size);
  }

  static bool saveResource2File(const std::string& resource,
                                const std::string& file) {
    FileBuffer buf = GSLAM::SvarWithType<FileBuffer>::instance()[resource];
    if (!buf.buf || buf.size <= 0) return false;
    std::ofstream ofs(file, std::ios::out | std::ios::binary);
    if (!ofs.is_open()) return false;
    ofs.write((const char*)buf.buf, buf.size);
    return ofs.good();
  }
};
}  // namespace GSLAM
#endif  // GSLAM_FILERESOURCE_H  //NOLINT
