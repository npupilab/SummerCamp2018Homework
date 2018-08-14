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
