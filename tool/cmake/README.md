# Exercise of tool/cmake

请编写CMakeLists.txt,　编译src文件夹的源码,其中包含4个target:

1. StaticLibDemo: static库（libStaticLibDemo.a）

2. SharedLibDemo: shared库（libSharedLibDemo.so）

3. AppDemo: main.cpp编译的可执行，依赖前两个库（libStaticLibDemo.a和libSharedLibDemo.so）
	
4. HelloDemo: test_for_find_package编译的可执行，依赖lib文件夹下Hello库（动态库与静态库均可）

要求：
* 支持make install，安装库到\$(CMAKE_INSTALL_PREFIX)/lib，安装可执行到\$(CMAKE_INSTALL_PREFIX)/bin
* 通过FIND_PACKAGE来添加Hello库，且要求写FindHello.cmake文件，并在其中给出
<NAME>_FOUND
<NAME>_INCLUDE_DIRS or <NAME>_INCLUDES
<NAME>_LIBRARIES or <NAME>_LIBRARIES or <NAME>_LIBS
变量值。
不可暴力地在CMakeList.txt中直接FIND_LIBRARIES。

<font color='red'>
*只需要提交CMakeLists.txt与FindHello.cmake到作业文件夹*
*源文件(src)和库文件(lib与include)目录及内容不要修改或复制*　
</font>

