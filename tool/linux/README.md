# Exercise of tool/linux

本文件（./README.md)分别列出了部分SummerCamp中的课程作业和参与人员列表，请编辑当前目录下的Shell脚本Statistics.sh，实现函数'evaluate()'，从而自动统计作业完成情况，生成如SampleOutput中的统计表格，输出到文件。

*本例子主要考察基础Bash编程能力，需要综合循环，判断，文件读写，字符串匹配查找等知识，在实际Linux批处理中应用较为广泛。*

### 输入：
1. README.md文件，用于识别课程列表及学员列表
2. 课程文件夹如tool/linux，每个文件夹用于提交对应课程作业，文件夹名称应与参与人员列表中的Name对应，作业文件夹中包含README.md文件，标志着作业完成质量：

```
完成质量
- S : Perfect
- A : Nice
- B : OK
- C : Poor
- D : None，作业未完成或未打分
```
### 编辑：
1. 文件如'zhaoyong/Statistics.sh'，最好不要引入其他文件

### 输出：
1. 运行脚本：
```
cd zhaoyong&&./Staticstics.sh ..
```

2. 生成文件'README.md'到Statistics.sh所在路径
```
Here_Path=$(dirname $(readlink -f $0))
```

### 作业提交：
1. 将完成的Shell脚本及生成的README.md文件打包放入文件夹中，文件夹以名字命名，提交到 https://github.com/zdzhaoyong/SummerCamp2018Homework

## 1. Homework List

| Topic | Description | 
| :---: | :---------: | 
| [tool/linux]| 了解常用命令，练习Bash编程 | 
| [tool/git]| git工具介绍和协作开发实践 | 
| [python/hellopython]     |介绍编译原理，引入CMake，并练习使用cmake进行编译，介绍PICMake，练习使用PICMake进行编译，Linux下编写简单包含三方库本地库的工程，并分别用Makefile，qmake，CMake编译|Lab|王伟|
|[cpp/helloc]     |C语言入门回顾，这里主要介绍与C语言重叠的部分，包含宏定义，变量，函数，循环体，指针，函数指针，数组，结构体，联合体等部分，编写简单函数库，并使用Makefile进行编译|

[tool/linux]: ./tool/linux/README.md
[tool/git]: ./tool/git/README.md
[python/hellopython]: ./python/hellopython/README.md
[cpp/helloc]: ./cpp/helloc/README.md

## 2. Attendance

| Name     | Phone Number |       Email     |  GitHub      |
| :---:    | :---------:  |   :---------:   | :---------:   |
| 张三      | 15339027461  | zd5945@126.com  | https://github.com/zdzhaoyong|
| 李四      | 15339027461  | zd5945@126.com  | https://github.com/zdzhaoyong|
| 王五      | 15339027461  | zd5945@126.com  ||
| 赵六      | 15339027461  | zd5945@126.com  ||
| Steven   | 15339027461  | zd5945@126.com  ||


## 3. Sample output

| Topic | 张三 | 李四 | 王五 | 赵六 | Steven |
| :---: | :--: | :--: | :--: | :--: | :--: | 
| [tool/linux]| S | A | B | C | D | 
| [tool/git]|  S | A | B | C |  D |  
| [python/hellopython] | S | A | B | C |  D |
| [cpp/helloc]     | S | A | B | C |  D |

## 4. Your output




