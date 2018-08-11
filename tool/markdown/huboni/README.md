# Markdown For Typora

[TOC]
>[TOC]用来生成目录，但有些软件可能不支持，比如github

### 概述：
**Markdown**是一种十分轻量级的***标记语言*** Markdown具有一系列衍生版本可以转换为许多个格式，比如Latex，PDF等。也可以选择多种书写风格，比如github。
### 标题
>用#来开始标题，一个代表一级标题，以此类推。
> # 一级
> ## 二级
> ### 三级
> ...

### 引用块
> 引用块格式如上所示
>  >块中块
>  >

### 列表
> 列表分为有序和无序两种

#### 有序列表
1. item1
2. item2
3. item3
#### 无序列表
+ item
+ item 
  + item11
+ item
  + item2
  + item3
### 代码块
#### c++代码块
``` c++
#include<iostream>
#include<stdlib.h>

using namespace std;
```
#### python代码块
```python
class hello(object)
	def print_hello(self):
        print("huboni")
a = hello()
a.print_hello()
```
### 数学公式
#### 行内公式
$ \sum_{i=1}^{100} i^2$
#### 行间公式
$$
E=mc^2
$$

$$
\mathbf{V}_1 \times \mathbf{V}_2 =  \begin{vmatrix} 
\mathbf{i} & \mathbf{j} & \mathbf{k} \\
\frac{\partial X}{\partial u} &  \frac{\partial Y}{\partial u} & 0 \\
\frac{\partial X}{\partial v} &  \frac{\partial Y}{\partial v} & 0 \\
\end{vmatrix}
$$
#### 表格

| title1 | title2 | title3 |
| :------: | :------: | :------: |
| hubni  | cindy  | mike   |
| 100    | 1000   | 1000   |
| 100    | 1      | 1      |
> 可以通过：位置来控制对齐方式

#### 关系图
``` sequence
huboni->cindy: hello
mike->phyby:hi
phyby->huboni:how are you
```
#### 流程图
``` flow 
st=>start: 开始


```

*建议用process on*
### 添加网址链接
[百度] :  https://www.baidu.com/
#### URLS
<18302360048@163.com>
### 脚注
文本[^脚注].
[脚注]:这是脚注的注释.
### 文本格式
#### 文本加粗
*huboni*
_huboni_
#### 文本斜体
**huboni**
#### 加粗&斜体
***huboni***
#### 下划线
<u>下划线</u>
###  插入图片
![](/home/hu/Pictures/thumb.jpg)
### 一些高级功能
+ 添加选项
- [ ] c++
- [ ] python
- [x] markdown
+ 添加表情
  :car:

+ 标记错误
  ~~error~~

+ 添加按钮
  `tab`

  