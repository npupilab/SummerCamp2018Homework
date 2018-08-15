# Exercise of slam/match

请继承[FeatureDetector](Matcher/src/FeatureDetector.cpp)和[Matcher](Matcher/src/Matcher.cpp)并实现特征点提取及匹配，其中Matcher可只实现match4initialize. 

*注意：可参考赵勇实现，将实现源文件放入作业文件夹即可，实现类需要使用REGISTER_FEATUREDETECTOR和REGISTER_MATCHER进行注册，如注册名不是Default,请在作业文件夹添加Default.cfg文件进行配置*


## 1. 编译运行Matcher工程

```
# Compile
cd Matcher
mkdir build
cd build
cmake .. -DUSER_NAME=赵勇
make

# Run
./Matcher conf=../Default.cfg -Name 赵勇 -ShowImage 1
```


## 2. 依赖及代码量要求

实现代码仅可依赖OpenCV2.4.9及CMakeLists.txt中已经列出的依赖，不可修改源码中的CMakeLists.txt文件，若要引入第三方特征提取实现，请将作业文件夹大小控制在1MB以内。
