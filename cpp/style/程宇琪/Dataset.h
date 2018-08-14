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

#ifndef CPP_STYLE___________DATASET_H_
#define CPP_STYLE___________DATASET_H_

#include <string>
#include <vector>
#include "GSLAM.h"
#include "SharedLibrary.h"
#include "Svar.h"

namespace GSLAM {

#define REGISTER_DATASET(D, E)                                                 \
  extern "C" SPtr<GSLAM::Dataset> createDataset##E() {                         \
    return SPtr<GSLAM::Dataset>(new D());                                      \
  }                                                                            \
  class D##E##_Register {                                                      \
   public:                                                                     \
    D##E##_Register() {                                                        \
      GSLAM::DatasetFactory::instance()._ext2creator.insert(#E,                \
                                                            createDataset##E); \
    }                                                                          \
  } D##E##_instance;

/// create
class Dataset;
typedef SPtr<Dataset> DatasetPtr;
typedef SPtr<Dataset> (*funcCreateDataset)();

// A dataset configuration file : DatasetName.DatasetType --eg. Sequence1.kitti
// desk.tumrgbd
class Dataset : public GSLAM::GObject {
 public:
  typedef std::vector<std::string> StrVec;
  Dataset() : _name("Untitled") {}
  virtual ~Dataset() {}

  std::string name() const {
    if (_impl)
      return _impl->type();
    else
      return _name;
  }
  virtual std::string type() const {
    if (_impl)
      return _impl->type();
    else
      return "Dataset";
  }
  virtual bool isOpened() {
    if (_impl)
      return _impl->isOpened();
    else
      return false;
  }
  virtual FramePtr grabFrame() {
    if (_impl)
      return _impl->grabFrame();
    else
      return FramePtr();
  }

  virtual bool open(const std::string& dataset);

 protected:
  std::string _name;
  DatasetPtr _impl;
};

class DatasetFactory {
 public:
  typedef SPtr<Dataset> DatasetPtr;

  static DatasetFactory& instance() {
    static SPtr<DatasetFactory> inst(new DatasetFactory());
    return *inst;
  }

  static DatasetPtr create(std::string dataset);

  SvarWithType<funcCreateDataset> _ext2creator;
};

inline bool Dataset::open(const std::string& dataset) {
  DatasetPtr impl = DatasetFactory::create(dataset);
  if (impl) {
    _impl = impl;
    return _impl->open(dataset);
  }
  return false;
}

inline DatasetPtr DatasetFactory::create(std::string dataset) {
  std::string extension;
  // The input path could be dataset configuration file or just a folder path
  size_t dotPosition = dataset.find_last_of('.');
  if (dotPosition != std::string::npos) {  // call by extension
    extension = dataset.substr(dotPosition + 1);
  }
  if (extension.empty()) return DatasetPtr();

  if (!instance()._ext2creator.exist(extension)) {
    SharedLibraryPtr plugin = Registry::get("libgslamDB_" + extension);
    if (!plugin.get()) return DatasetPtr();
    funcCreateDataset createFunc =
        (funcCreateDataset)plugin->getSymbol("createDataset" + extension);
    if (createFunc) return createFunc();
  }

  if (!instance()._ext2creator.exist(extension)) return DatasetPtr();
  funcCreateDataset createFunc =
      instance()._ext2creator.get_var(extension, NULL);
  if (!createFunc) return DatasetPtr();

  return createFunc();
}
}  // namespace GSLAM

#endif  // CPP_STYLE___________DATASET_H_
