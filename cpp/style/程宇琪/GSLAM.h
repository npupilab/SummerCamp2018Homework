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

#ifndef GSLAM_CORE_GSLAM_H_
#define GSLAM_CORE_GSLAM_H_

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "Camera.h"
#include "GImage.h"
#include "KeyPoint.h"
#include "Mutex.h"
#include "SIM3.h"
#include "SharedLibrary.h"
#include "Svar.h"

#define GSLAM_VERSION_MAJOR 2
#define GSLAM_VERSION_MINOR 4
#define GSLAM_VERSION_PATCH 3
#define GSLAM_COMMAND_STRHELPER(COMMAND) #COMMAND
#define GSLAM_COMMAND_STR(COMMAND) GSLAM_COMMAND_STRHELPER(COMMAND)
#define GSLAM_VERSION                                            \
  (GSLAM_COMMAND_STR(GSLAM_VERSION_MAJOR) "." GSLAM_COMMAND_STR( \
      GSLAM_VERSION_MINOR) "." GSLAM_COMMAND_STR(GSLAM_VERSION_PATCH))

#define USE_GSLAM_PLUGIN_(SLAMCLASS)        \
  extern "C" {                              \
  GSLAM::SLAMPtr createSLAMInstance() {     \
    return GSLAM::SLAMPtr(new SLAMCLASS()); \
  }                                         \
  }
#define USE_GSLAM_PLUGIN(SLAMCLASS) USE_GSLAM_PLUGIN_(SLAMCLASS)
namespace GSLAM {
class GObject;
class MapFrame;
class MapPoint;
class Map;
class SLAM;

typedef size_t NodeId;
typedef size_t WordId;
typedef float WordValue;
typedef std::map<WordId, float> BowVector;
typedef std::map<WordId, std::vector<unsigned int> > FeatureVector;
typedef pi::SO3d SO3;
typedef pi::SE3d SE3;
typedef pi::SIM3d SIM3;
typedef pi::Point2i Point2i;
typedef pi::Point2d Point2d;
typedef pi::Point3d Point3d;
typedef pi::Point3f Point3f;
typedef pi::Point2f Point2f;
typedef pi::Point3ub Point3ub;
typedef pi::Point3d Point3Type;
typedef Point3ub ColorType;
typedef size_t PointID;
typedef size_t FrameID;
typedef SPtr<GObject> GObjectPtr;
typedef SPtr<MapFrame> FramePtr;
typedef SPtr<MapPoint> PointPtr;
typedef SPtr<Map> MapPtr;
typedef SPtr<SLAM> SLAMPtr;
typedef std::vector<FramePtr> FrameArray;
typedef std::vector<PointPtr> PointArray;
typedef GSLAM::Point3d
    CameraAnchor;  // for both Pinhole projection and Sphere projection
typedef GSLAM::Point2d IdepthEstimation;  // [idepth,invSqSigma]^T
typedef GSLAM::SLAMPtr (*funcCreateSLAMInstance)();

enum ImageChannelFlags {
  IMAGE_UNDEFINED = 0,
  // For colorful or depth camera
  IMAGE_RGBA =
      1 << 0,  // when channals()==3 means RGB, Alpha means the image mask
  IMAGE_BGRA = 1 << 1,
  IMAGE_GRAY = 1 << 2,
  IMAGE_DEPTH = 1 << 3,   // the depth image cam be obtained from lidar or depth
                          // camera such as Kinect
  IMAGE_IDEPTH = 1 << 4,  // this is usually obtained with

  IMAGE_RGBD = IMAGE_BGRA | IMAGE_DEPTH,

  // For multispectral camera
  IMAGE_GRE = 1 << 5,  // Green
  IMAGE_NIR = 1 << 6,  // Near
  IMAGE_RED = 1 << 7,  // Red
  IMAGE_REG = 1 << 8,  // Red Edge

  IMAGE_LIDAR = 1 << 9,
  IMAGE_SONAR = 1 << 10,
  IMAGE_SAR = 1 << 11,

  IMAGE_THUMBNAIL = 1 << 12  // The thumbnail should be in format RGBA
};

class GObject {
 public:
  virtual ~GObject() {}
  virtual std::string type() const { return "GObject"; }
  virtual void call(const std::string& command, void* arg = NULL) {}
  virtual void draw() {}
  virtual bool toByteArray(std::vector<uchar>& array) {  // NOLINT
    return false;
  }
  virtual bool fromByteArray(std::vector<uchar>& array) {  // NOLINT
    return false;
  }
};

class GObjectHandle {
 public:
  virtual ~GObjectHandle() {}
  virtual void handle(const SPtr<GObject>& obj) {}
  void handle(GObject* obj) { handle(SPtr<GObject>(obj)); }
};

/**
 * @brief The MapPoint class
 */
class MapPoint : public GObject {
 public:
  explicit MapPoint(const PointID& id, \
                    const Point3Type& position = Point3Type(0, 0, 0));
  virtual ~MapPoint() {}
  virtual std::string type() const { return "InvalidPoint"; }
  const PointID id() { return _id; }

  Point3Type getPose() const;
  void setPose(const Point3Type& pt);

  virtual Point3Type getNormal() const { return Point3Type(0, 0, 0); }
  virtual bool setNormal(const Point3Type& nVec) { return false; }

  virtual ColorType getColor() const { return ColorType(255, 255, 255); }
  virtual bool setColor(const ColorType& color) const { return false; }

  virtual bool setDescriptor(const GImage& des) { return false; }
  virtual GImage getDescriptor() const { return GImage(); }

  virtual bool isPoseRelative() const {
    return false;
  }  // Default use world coordinate
  virtual FrameID refKeyframeID() const {
    return 0;
  }  // If using relative pose, which keyframe referencing
  virtual FramePtr refKeyframe() const { return FramePtr(); }

  virtual int observationNum() const { return -1; }
  virtual bool getObservations(
      std::map<FrameID, size_t>& obs) const {  // NOLINT
    return false;
  }
  virtual bool addObservation(GSLAM::FrameID frId, size_t featId) {
    return false;
  }
  virtual bool eraseObservation(GSLAM::FrameID frId) { return false; }
  virtual bool clearObservation() { return false; }

 protected:
  mutable MutexRW _mutexPt;

 private:
  const PointID _id;
  Point3Type _pt;
};

// MapFrame <-> MapFrame  : MapFrame connections for Pose Graph
class FrameConnection : public GObject {
 public:
  virtual std::string type() const { return "FrameConnection"; }
  virtual int matchesNum() { return 0; }

  virtual bool getMatches(
      std::vector<std::pair<int, int> >& matches) {  // NOLINT
    return false;
  }
  virtual bool getChild2Parent(GSLAM::SIM3& sim3) { return false; }  // NOLINT
  virtual bool getChild2Parent(GSLAM::SE3& se3) { return false; }    // NOLINT
  virtual bool getInformation(double* info) { return false; }

  virtual bool setMatches(
      std::vector<std::pair<int, int> >& matches) {  // NOLINT
    return false;
  }
  virtual bool setChild2Parent(GSLAM::SIM3& sim3) { return false; }  // NOLINT
  virtual bool setChild2Parent(GSLAM::SE3& se3) { return false; }    // NOLINT
  virtual bool setInformation(double* info) { return false; }
};

class MapFrame : public GObject {
 public:
  explicit MapFrame(const FrameID& id = 0, const double& timestamp = 0);
  virtual ~MapFrame() {}
  virtual std::string type() const { return "InvalidFrame"; }

  // Basic things, ID, Timestamp, Image, CameraModel, IMU, GPS informations
  const PointID id() const { return _id; }
  const double& timestamp() const { return _timestamp; }

  // Frame transform from local to world, this is essential
  void setPose(const SE3& pose);
  void setPose(const SIM3& pose);
  SE3 getPose() const;
  bool getPose(SIM3& pose) const;  // NOLINT
  SIM3 getPoseScale() const;

  // When the frame contains one or more images captured from cameras
  virtual int cameraNum() const { return 0; }  // Camera number
  virtual SE3 getCameraPose(int idx = 0) const {
    return SE3();
  }  // The transform from camera to local
  virtual int imageChannels(int idx = 0) const {
    return IMAGE_BGRA;
  }  // Default is a colorful camera
  virtual Camera getCamera(int idx = 0) {
    return Camera();
  }  // The camera model
  virtual GImage getImage(int idx = 0, int channalMask = IMAGE_UNDEFINED) {
    return GImage();
  }  // Just return the image if only one channel is available
  virtual bool setImage(const GImage& img, int idx = 0,
                        int channalMask = IMAGE_UNDEFINED) {
    return false;
  }
  virtual bool setCamera(const Camera& camera, int idx = 0) { return false; }

  // When the frame contains IMUs or GPSs
  virtual int getIMUNum() const { return 0; }
  virtual SE3 getIMUPose(int idx = 0) const { return SE3(); }
  virtual bool getAcceleration(Point3d& acc, int idx = 0) const {  // NOLINT
    return false;
  }  // m/s^2
  virtual bool getAngularVelocity(Point3d& angularV,  // NOLINT
                                  int idx = 0) const {
    return false;
  }                                                            // rad/s
  virtual bool getMagnetic(Point3d& mag, int idx = 0) const {  // NOLINT
    return false;
  }  // gauss
  virtual bool getAccelerationNoise(Point3d& accN,  // NOLINT
                                    int idx = 0) const {
    return false;
  }
  virtual bool getAngularVNoise(Point3d& angularVN,  // NOLINT
                                int idx = 0) const {
    return false;
  }
  virtual bool getPitchYawRoll(Point3d& pyr, int idx = 0) const {  // NOLINT
    return false;
  }                                                                 // in rad
  virtual bool getPYRSigma(Point3d& pyrSigma, int idx = 0) const {  // NOLINT
    return false;
  }  // in rad

  virtual int getGPSNum() const { return 0; }
  virtual SE3 getGPSPose(int idx = 0) const { return SE3(); }
  virtual bool getGPSLLA(Point3d& LonLatAlt, int idx = 0) const {  // NOLINT
    return false;
  }  // WGS84 [longtitude latitude altitude]
  virtual bool getGPSLLASigma(Point3d& llaSigma, int idx = 0) const {  // NOLINT
    return false;
  }                                                           // meter
  virtual bool getGPSECEF(Point3d& xyz, int idx = 0) const {  // NOLINT
    return false;
  }                                                                    // meter
  virtual bool getHeight2Ground(Point2d& height, int idx = 0) const {  // NOLINT
    return false;
  }  // height against ground

  // Tracking things for feature based methods
  virtual int keyPointNum() const { return 0; }
  virtual bool setKeyPoints(
      const std::vector<GSLAM::KeyPoint>& keypoints,
      const GSLAM::GImage& descriptors = GSLAM::GImage()) {
    return false;
  }
  virtual bool getKeyPoint(int idx, Point2f& pt) const {  // NOLINT
    return false;
  }
  virtual bool getKeyPoint(int idx, KeyPoint& pt) const {  // NOLINT
    return false;
  }
  virtual bool getKeyPoints(std::vector<Point2f>& keypoints) const {  // NOLINT
    return false;
  }
  virtual bool getKeyPoints(std::vector<KeyPoint>& keypoints) const {  // NOLINT
    return false;
  }
  virtual bool getKeyPointColor(int idx, ColorType& color) {  // NOLINT
    return false;
  }
  virtual bool getKeyPointIDepthInfo(int idx, Point2d& idepth) {  // NOLINT
    return false;
  }
  virtual PointID getKeyPointObserve(int idx) { return 0; }
  virtual GImage getDescriptor(int idx = -1) const {
    return GImage();
  }  // idx<0: return all descriptors
  virtual bool getBoWVector(BowVector& bowvec) const {  // NOLINT
    return false;
  }
  virtual bool getFeatureVector(FeatureVector& featvec) const {  // NOLINT
    return false;
  }
  virtual std::vector<size_t> getFeaturesInArea(const float& x, const float& y,
                                                const float& r,
                                                bool precisely = true) const {
    return std::vector<size_t>();
  }

  // MapPoint <-> KeyPoint  : MapPoint Observation usually comes along
  // Mappoint::*obs*
  virtual int observationNum() const { return 0; }
  virtual bool getObservations(
      std::map<GSLAM::PointID, size_t>& obs) const {  // NOLINT
    return false;
  }
  virtual bool addObservation(const GSLAM::PointPtr& pt, size_t featId,
                              bool add2Point = false) {
    return false;
  }
  virtual bool eraseObservation(const GSLAM::PointPtr& pt,
                                bool erasePoint = false) {
    return false;
  }
  virtual bool clearObservations() { return false; }

  virtual SPtr<FrameConnection> getParent(GSLAM::FrameID parentId) const {
    return SPtr<FrameConnection>();
  }
  virtual SPtr<FrameConnection> getChild(GSLAM::FrameID childId) const {
    return SPtr<FrameConnection>();
  }
  virtual bool getParents(std::map<GSLAM::FrameID, SPtr<FrameConnection> >&
                              parents) const {  // NOLINT
    return false;
  }
  virtual bool getChildren(std::map<GSLAM::FrameID, SPtr<FrameConnection> >&
                               children) const {  // NOLINT
    return false;
  }
  virtual bool addParent(GSLAM::FrameID parentId,
                         const SPtr<FrameConnection>& parent) {
    return false;
  }
  virtual bool addChildren(GSLAM::FrameID childId,
                           const SPtr<FrameConnection>& child) {
    return false;
  }
  virtual bool eraseParent(GSLAM::FrameID parentId) { return false; }
  virtual bool eraseChild(GSLAM::FrameID childId) { return false; }
  virtual bool clearParents() { return false; }
  virtual bool clearChildren() { return false; }

  // Extra utils
  virtual double getMedianDepth() { return getPoseScale().get_scale(); }

  static std::string channelTypeString(const int channels);
  std::string channelString(int idx = 0) const {
    return channelTypeString(imageChannels(idx));
  }

 public:
  const FrameID _id;
  double _timestamp;

 protected:
  mutable MutexRW _mutexPose;

 private:
  SIM3 _c2w;  // worldPt=c2w*cameraPt;
};

struct LoopCandidate {
  LoopCandidate(const FrameID& frameId_, const double& score_)
      : frameId(frameId_), score(score_) {}

  FrameID frameId;
  double score;
  friend bool operator<(const LoopCandidate& l, const LoopCandidate& r) {
    return l.score < r.score;
  }
};
typedef std::vector<LoopCandidate> LoopCandidates;

// The LoopDetector is used to detect loops with Poses or Descriptors
// information
class LoopDetector : public GObject {
 public:
  virtual std::string type() const { return "LoopDetector"; }
  virtual bool insertMapFrame(const FramePtr& frame) { return false; }
  virtual bool eraseMapFrame(const FrameID& frame) { return false; }
  virtual bool obtainCandidates(const FramePtr& frame,
                                LoopCandidates& candidates) {  // NOLINT
    return false;
  }
};
typedef SPtr<LoopDetector> LoopDetectorPtr;

class Map : public GObject {
 public:
  Map();
  virtual ~Map() {}
  virtual std::string type() const { return "InvalidMap"; }

  /// MapFrame & MapPoint interface
  virtual bool insertMapPoint(const PointPtr& point) { return false; }
  virtual bool insertMapFrame(const FramePtr& frame) { return false; }
  virtual bool eraseMapPoint(const PointID& pointId) { return false; }
  virtual bool eraseMapFrame(const FrameID& frameId) { return false; }
  virtual void clear() {}

  virtual std::size_t frameNum() const { return 0; }
  virtual std::size_t pointNum() const { return 0; }

  virtual FramePtr getFrame(const FrameID& id) const { return FramePtr(); }
  virtual PointPtr getPoint(const PointID& id) const { return PointPtr(); }
  virtual bool getFrames(FrameArray& frames) const { return false; }  // NOLINT
  virtual bool getPoints(PointArray& points) const { return false; }  // NOLINT

  virtual bool setLoopDetector(const LoopDetectorPtr& loopdetector) {
    return false;
  }
  virtual LoopDetectorPtr getLoopDetector() const { return LoopDetectorPtr(); }
  virtual bool obtainCandidates(const FramePtr& frame,
                                LoopCandidates& candidates) {  // NOLINT
    return false;
  }

  /// Save or load the map from/to the file
  virtual bool save(std::string path) const { return false; }
  virtual bool load(std::string path) { return false; }

  /// 0 is reserved for INVALID
  PointID getPid() { return _ptId++; }  // obtain an unique point id
  FrameID getFid() { return _frId++; }  // obtain an unique frame id

 protected:
  PointID _ptId;
  FrameID _frId;
};

typedef SPtr<Map> MapPtr;

class SLAM : public GObject {
 public:
  SLAM() {}
  virtual ~SLAM() {}
  virtual std::string type() const { return "InvalidSLAM"; }
  virtual bool valid() const { return false; }
  virtual bool isDrawable() const { return false; }

  bool setMap(const MapPtr& map);
  MapPtr getMap() const;

  virtual bool setSvar(Svar& var);  // NOLINT
  virtual bool setCallback(GObjectHandle* cbk) {
    _handle = cbk;
    return true;
  }
  virtual bool track(FramePtr& frame) { return false; }  // NOLINT
  virtual bool finalize() { return false; }

  static SLAMPtr create(const std::string& slamPlugin);

 protected:
  MapPtr _curMap;
  mutable MutexRW _mutexMap;
  GObjectHandle* _handle;
};

inline MapPoint::MapPoint(const PointID& id, const Point3Type& position)
    : _id(id), _pt(position) {}

inline Point3Type MapPoint::getPose() const {
  ReadMutex lock(_mutexPt);
  return _pt;
}

inline void MapPoint::setPose(const Point3Type& pt) {
  WriteMutex lock(_mutexPt);
  _pt = pt;
}

inline MapFrame::MapFrame(const FrameID& id, const double& timestamp)
    : _id(id), _timestamp(timestamp) {}

inline SE3 MapFrame::getPose() const {
  ReadMutex lock(_mutexPose);
  return _c2w.get_se3();
}

inline SIM3 MapFrame::getPoseScale() const {
  ReadMutex lock(_mutexPose);
  return _c2w;
}

inline bool MapFrame::getPose(SIM3& pose) const {
  pose = getPoseScale();
  return true;
}

inline void MapFrame::setPose(const SE3& pose) {
  WriteMutex lock(_mutexPose);
  _c2w.get_se3() = pose;
}

inline void MapFrame::setPose(const SIM3& pose) {
  WriteMutex lock(_mutexPose);
  _c2w = pose;
}

inline std::string MapFrame::channelTypeString(const int channels) {
  static std::string _type[32] = {
      "RGBA", "BGRA", "GRAY",  "DEPTH", "IDEPTH", "GRE",      "NIR",
      "RED",  "REG",  "LIDAR", "SONAR", "SAR",    "THUMBNAIL"};
  std::string type = "";
  if (channels == 0) {
    return "IMAGE_UNDEFINED";
  } else {
    for (int i = 0; i < sizeof(channels) * 8; i++) {
      int j = channels & (0x00000001 << i);
      if (j != 0) {
        type += (type.empty() ? "" : "|") + _type[i];
      }
    }
    return type;
  }
}

inline Map::Map() : _ptId(1), _frId(1) {}

inline bool SLAM::setMap(const MapPtr& map) {
  ReadMutex lock(_mutexMap);
  _curMap = map;
  return true;
}

inline MapPtr SLAM::getMap() const {
  ReadMutex lock(_mutexMap);
  return _curMap;
}

inline bool SLAM::setSvar(Svar& var) {
  for (auto it : var.get_data()) svar.insert(it.first, it.second);
  return true;
}

inline SLAMPtr SLAM::create(const std::string& slamPlugin) {
  SPtr<SharedLibrary> plugin = Registry::get(slamPlugin);
  if (!plugin) return SLAMPtr();
  funcCreateSLAMInstance createFunc =
      (funcCreateSLAMInstance)plugin->getSymbol("createSLAMInstance");
  if (!createFunc)
    return SLAMPtr();
  else
    return createFunc();
}

}  // end of namespace GSLAM

#include "Optimizer.h"

#endif  // GSLAM_CORE_GSLAM_H_
