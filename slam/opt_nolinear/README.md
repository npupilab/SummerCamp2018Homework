# Exercise of slam/opt_nolinear

请使用LM算法，继承GSLAM::Optimizer并实现optimizePnP函数，使程序编译通过并获得S。


## 接口说明

```
// Update pose with 3D-2D corrospondences

  virtual bool optimizePnP(
      const std::vector<std::pair<GSLAM::Point3d, CameraAnchor> >& matches,    // 3D points and projected point
      GSLAM::SE3& pose, KeyFrameEstimzationDOF dof = UPDATE_KF_SE3,            // Pose input and output
      double* information = NULL);

```


### 注意事项

1. 脱离内置PICMake编译时，请pull更新GSLAM版本（为了保持与作业内置GSLAM一致）
2. 实现类请使用USE_OPTIMIZER_PLUGIN注册，具体可参考赵勇实现

