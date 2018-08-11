# Exercise of tool/cmake

请编写CMakeLists.txt,　编译src文件夹的源码,其中包含３个target:

1. StaticLibDemo: static库
2. SharedLibDemo: shared库
3. AppDemo      : 可执行，依赖前两个库

支持make install安装库到$(CMAKE_INSTALL_PREFIX)/lib,安装可执行到$(CMAKE_INSTALL_PREFIX)/bin

*只需要提交CMakeLists.txt到作业文件夹，源文件请不要修改或复制*
