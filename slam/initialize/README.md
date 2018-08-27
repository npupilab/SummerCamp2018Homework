# Exercise of slam/initialize

请参考ORBSLAM或DIYSLAM等实现，完成Initializer的一个实现并编译测试通过，其中实现建议只依赖GSLAM与Eigen3，实在有困难的可以依赖OpenCV.

*注意需要使用REGISTER_INITIALIZER注册实现，其中第一个参数为自己定义的实现类名，第二个参数是注册名请使用default*

## 函数接口说明
```
virtual bool initialize(const std::vector<std::pair<GSLAM::Point2f,GSLAM::Point2f> >& matches, // 特征点匹配
                            const GSLAM::Camera& camera, // 相机内参
                            GSLAM::SE3& t21,             // 输出参数，相机一到二的变换
                            std::vector<std::pair<int,GSLAM::Point3d> >& mpts)// 输出参数，成功三角化的点及对应在matches中的Index
```
