---
haha
---



# Markdown For Typora

## 概述

__Markdown__ 是由[Daring Fireball](https://daringfireball.net/)创建的，原来的教程在[这](https://daringfireball.net/projects/markdown/syntas)。然而，它的语法在各个解释器或编辑器都不一样。**Typora**使用了[GitHub Flavored Markdown][GFM]。

请注意，在Markdown源HTML片段将被认可而不是分析或渲染。另外，有可能保存后在Markdown重新显示为源代码格式。

*概述*

[TOC]

## 区块组件

### 段落和换行

段落只是一个或多个连续的文本行。在标记源代码中，段落由多条空白行分隔。在Typora，你只需要按下`return`来创建一个新的段落。

按`Shift` +`return`创建单行中断。但是，大多数降价分析器会忽略单行中断，为了让其他的标记解析器识别出你的断线，你可以在行的末尾留下两个空白，或者插入`<BR/>`。

### 标头

标头在行开始时使用1-6散列字符，对应于标题级别1-6。例如：

```markdown
# This is an H1

## This is an H2

###### This is an H6
```

# This is an H1

## This is an H2

###### This is an H6

在typora，输入“#其次是标题的内容，按`回车键`将创建一个标题。

### 块引用

标记使用电子邮件样式>字符用于块引用。它们被呈现为：

```markdown
> This is a blockquote with two paragraphs. This is first paragraph.
>
> This is second pragraph.Vestibulum enim wisi, viverra nec, fringilla in, laoreet vitae, risus.



> This is another blockquote with one paragraph. There is three empty line to seperate two blockquot
```

> This is a blockquote with two paragraphs. This is first paragraph.
>
> This is second pragraph.Vestibulum enim wisi, viverra nec, fringilla in, laoreet vitae, risus.



> This is another blockquote with one paragraph. There is three empty line to seperate two blockquot

在typora，只要输入“>”后面引用内容块引用生成。Typora将为你插入适当的“>”或“断线”。通过添加附加的“>”允许块内报价。

### 列表

输入`* 列表项1`将创建未排序列表，*符号可以用+或-替换。

输入`1. 列表项1`将创建有序列表，它们的降价源代码如下：

```markdown
## un-ordered list
*   Red
*   Green
*   Blue

## ordered list
1.  Red
2. 	Green
3.	Blue
```

## un-ordered list
*   Red
*   Green
*   Blue

## ordered list
1. Red
2. Green
3. Blue

### 任务列表

任务清单是标记为[ ]或[×]的项目表（不完整或不完整的）。例如：

```markdown
- [ ] a task list item
- [ ] list syntax required
- [ ] normal **formatting**, @mentions, #1234 refs
- [ ] incomplete
- [x] completed
```

- [ ] a task list item
- [ ] list syntax required
- [ ] normal **formatting**, @mentions, #1234 refs
- [ ] incomplete
- [x] completed

您可以通过单击项目前的复选框来更改完整/不完整状态。

### （栅栏）代码块

Typora仅支持Github Flavored Markdown中的栅栏。 不支持markdown中的原始代码块。

使用栅栏很简单：输入“`”并按`回车键`。 在```之后添加一个可选的语言标识符，我们将通过语法高亮显示它：

```markdown
Here's an example:

​```
function test() {
  console.log("notice the blank line before this function?");
}
​```

syntax highlighting:
​```ruby
require 'redcarpet'
markdown = Redcarpet.new("Hello World!")
puts markdown.to_html
​```
```

Here's an example:

```
function test() {
  console.log("notice the blank line before this function?");
}
```

syntax highlighting:
```ruby
require 'redcarpet'
markdown = Redcarpet.new("Hello World!")
puts markdown.to_html
```
### 数学块

您可以使用__MathJax__渲染_LaTeX_数学表达式。

输入`$$`，然后按'返回'键将触发一个接受Tex / LaTex源的输入字段。 以下是一个例子：
$$
\mathbf{V}_1 \times \mathbf{V}_2 = \begin{vmatrix}
\mathbf{i} & \mathbf{j} & \mathbf{k} \\
\frac{\partial X}{\partial u} & \frac{\partial Y}{\partial u} & 0 \\
\frac{\partial X}{\partial v} &  \frac{\partial Y}{\partial v} & 0 \\
\end{vmatrix}
$$
在markdown源文件中，math块是由'$$'标记包装的_LaTeX_表达式：

```markdown
$$
\mathbf{V}_1 \times \mathbf{V}_2 =  \begin{vmatrix} 
\mathbf{i} & \mathbf{j} & \mathbf{k} \\
\frac{\partial X}{\partial u} &  \frac{\partial Y}{\partial u} & 0 \\
\frac{\partial X}{\partial v} &  \frac{\partial Y}{\partial v} & 0 \\
\end{vmatrix}
$$
```

### 表格

输入`| 第一个标题| 第二个标题| `并按`回车键`将创建一个包含两列的表。

创建表后，焦点在该表上将弹出一个表格工具栏，您可以在其中调整表格，对齐或删除表格。 您还可以使用上下文菜单来复制和添加/删除列/行。

可以跳过以下描述，因为表格的降价源代码是由typora自动生成的。

在markdown源代码中，它们看起来像：

```markdown
| First Header  | Second Header |
| ------------- | ------------- |
| Content Cell  | Content Cell  |
| Content Cell  | Content Cell  |
```

| First Header | Second Header |
| ------------ | ------------- |
| Content Cell | Content Cell  |
| Content Cell | Content Cell  |

您还可以包括内联Markdown，例如链接，粗体，斜体或删除线。

最后，通过在标题行中包含冒号：您可以将文本定义为左对齐，右对齐或居中对齐：

```markdown
| Left-Aligned  | Center Aligned  | Right Aligned |
| :------------ |:---------------:| -----:|
| col 3 is      | some wordy text | $1600 |
| col 2 is      | centered        |   $12 |
| zebra stripes | are neat        |    $1 |
```

| Left-Aligned  | Center Aligned  | Right Aligned |
| :------------ | :-------------: | ------------: |
| col 3 is      | some wordy text |         $1600 |
| col 2 is      |    centered     |           $12 |
| zebra stripes |    are neat     |            $1 |

最左侧的冒号表示左对齐的列; 最右侧的冒号表示右对齐的列; 两侧的冒号表示中心对齐的列。

### 脚注

``` markdown
You can create footnotes like this[^footnote].

[^footnote]: Here is the *text* of the **footnote**.
```
会生成：

你可以像这样[^footnote]产生脚注

[^footnote]: here is the *text* of the **footnote**.

鼠标在'脚注'上标中查看脚注的内容。

### 水平规则

在空白行输入`***`或`---`并按`回车`键将绘制一条水平线。

-----

### YAML Front Matter

Typora现在支持YAML Front Matter。 输入---在文章的顶部，然后按Enter将引入一个。 或者从菜单中插入一个元数据块。

### TOC

输入`[toc]`然后按`Return`键将创建一个“目录”部分，从一个人的书写中提取所有标题，其内容将自动更新。

### 图表（序列，流程图和Mermaid）

从首选项面板启用此功能后，Typora支持，序列，流程图和Mermaid。

详细信息请参阅此文档。

### 跨度元素

您可以在输入后立即解析并呈现Span元素。 在这些span元素的中间移动光标会将这些元素扩展为markdown源。 以下将解释这些span元素的语法。

### 链接

Markdown支持两种类型的链接：内联和引用。

在这两种样式中，链接文本由[方括号]分隔。

要创建内联链接，请在链接文本的结束方括号后立即使用一组常规括号。 在括号内，将URL指向要指向的链接，以及链接的可选标题，用引号括起来。 例如：

```markdown
This is [an example](http://example.com/ "Title") inline link.

[This link](http://example.net/) has no title attribute.
```

会产生：

This is [an example](http://example.com/"Title") inline link. (`<p>This is <a href="http://example.com/" title="Title">`)

[This link](http://example.net/) has no title attribute. (`<p><a href="http://example.net/">This link</a> has no`)

### 内联

__您可以将href设置为标题__，这将创建一个书签，允许您在单击后跳转到该部分。 例如：

命令（在Windows上：Ctrl）+单击此[链接](#block-elements)将跳转到`标题块元素`。 要查看如何编写，请移动光标或单击该链接并按`⌘`键将元素展开为markdown源。

### 参考链接

参考样式链接使用第二组方括号，在其中放置您选择的标签以标识链接：

```markdown
This is [an example][id] reference-style link.

Then, anywhere in the document, you define your link label like this, on a line by itself:

[id]: http://example.com/  "Optional Title Here"
```

在typora中，它们将呈现为：

这是一个[示例](haha)参考样式链接。

[id]: http://example.com/  "Optional Title Here"

隐式链接名称快捷方式允许您省略链接的名称，在这种情况下，链接文本本身将用作名称。 只需使用一组空的方括号 - 例如，将“Google”字词链接到google.com网站，您只需编写：

```markdown
[Google][]
And then define the link:

[Google]: http://google.com/
```

[Google][]
And then define the link:

[Google]: http://google.com/

在typora中单击链接将其展开以进行编辑，命令+单击将在Web浏览器中打开超链接。

### URLs

Typora允许您将URL作为链接插入，由`<`bracket`>`包装。

`<i@typora.io`>成为i@typora.io。

Typora将aslo自动链接标准URL。 例如：www.google.com。

### 图片

图像看起来与链接相似，但需要额外的！ 在链接开始之前的char。 图像语法如下所示：

```markdown
![Alt text](/path/to/img.jpg)

![Alt text](/path/to/img.jpg "Optional title")
```

您可以使用拖放操作从图像文件或浏览器插入图像。 并通过单击图像修改markdown源代码。 如果图像在拖放时与当前编辑文档位于同一目录或子目录中，则将使用相对路径。

有关图像的更多提示，请阅读http://support.typora.io//Images/

### 重点

Markdown将星号（`*`）和下划线（`_`）视为重点指标。 用一个`*`或`_`包装的文本将用HTML `<em>`标签包装。 例如：

```markdown
*single asterisks*

_single underscores_
```

输出：

*单个星号*

*单下划线*

GFM将忽略单词中的下划线，这通常用在代码和名称中，如下所示：

> wow_great_stuff
>
>do_this_and_do_that_and_another_thing.

要在其它方式用作强调分隔符的位置生成文字星号或下划线，可以反斜杠转义：

```markdown
\*this text is surrounded by literal asterisks\*
```

\*this text is surrounded by literal asterisks\*

###　强调

double *或_'s将包含HTML`<strong>`标记，例如：

```markdown
**double asterisks**

__double underscores__
```

output:

**double asterisks**

__double underscores__

Typora建议使用`**`符号。

### 代码

要指示代码范围，请使用反引号（`）进行包装。 与预格式化的代码块不同，代码跨度表示正常段落中的代码。 例如：

```markdown
Use the `printf()` function.
```

会产生：

使用`printf（）`函数。

### 删除线

StrikethroughGFM添加了创建删除线文本的语法，标准Markdown中缺少该文本。

`~~Mistaken text.~~`变成了~~Mistaken text.~~

### 下划线

下划线由原始HTML提供支持。

`<u>Underline</u>` becomes </u>underline</u>.

### 表情:smile:	

用户可以通过按`ESC`键触发表情符号的自动完成建议，或者在首选项面板上启用后自动触发表情符号。 此外，还支持从菜单栏中的`编辑` - >`表情符号和符号`直接输入UTF8表情符号字符。

### HTML

Typora无法呈现html片段。 但是typora可以解析和渲染非常有限的HTML片段，作为Markdown的扩展，包括：

- 下划线：`<u>underline</u>`
- 图片: `<img src="http://www.w3.org/html/logo/img/mark-word-icon.png" width="200px" />` (And `width`, `height` attribute in HTML tag, and `width`, `height`, `zoom` style in `style` attribute will be applied.)
- 评论:`<!-- This is some comments -->`<!-- This is some comments -->
- 超链接: `<a href="http://typora.io" target="_blank">link</a>`.

它们的大多数属性，样式或类都将被忽略。 对于其他标记，typora会将它们呈现为原始HTML代码段。

但这些HTML将在打印或导出时导出。

### 内联函数

要使用此功能，请首先在“首选项面板” - >“标记”选项卡中启用它。 然后使用`$`来包装TeX命令，例如：$ \ lim_ {x \ to \ infty} \ exp（-x）= 0 $将呈现为LaTeX命令。

要触发内联数学的内联预览：输入“$”，然后按`ESC`键，然后输入TeX命令，将显示预览工具提示，如下所示：

<img src="http://typora.io/img/inline-math.gif" style="zoom:50%;" />

### 标

要使用此功能，请首先在“首选项面板” - >“标记”选项卡中启用它。 然后使用〜来包含下标内容，例如：`H~2~O`，`X~long \ text~ /`

### Superscript

Superscript要使用此功能，首先，请在“首选项面板” - >“Markdown”选项卡中启用它。 然后使用^来包装上标内容，例如：X ^ 2 ^。

### 高亮

要使用此功能，请首先在“首选项面板” - >“标记”选项卡中启用它。 然后使用`==`来包装上标内容，例如：`== highlight ==`。

GFM https://help.github.com/articles/github-flavored-markdown/ 