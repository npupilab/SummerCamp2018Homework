# Exercise of slam/Camera

现有一相机通过OpenCV标定得到了fx,fy,cx,cy,k1,k2,p1,p2,k3：

1. 请编写函数实现相机投影函数Project和反投影函数UnProject;
2. 请问现将图像缩放到原来的scale倍，相机的内参如何变化？请实现applyScale函数给出结果;


编辑并提交: src/Camera.h


```

class CameraOpenCV {
 public:
  typedef GSLAM::Point2d Point2d;
  typedef GSLAM::Point3d Point3d;
  CameraOpenCV(double Fx, double Fy, double Cx, double Cy, double K1, double K2,
               double P1, double P2, double K3);

  virtual std::string CameraType() const { return "OpenCV"; }
  virtual bool isValid() const;
  virtual Point2d Project(const Point3d& p3d) const;
  virtual Point3d UnProject(const Point2d& p2d) const;
  virtual bool applyScale(double scale = 0.5);
  double fx, fy, cx, cy, k1, k2, p1, p2, k3;
};

```
