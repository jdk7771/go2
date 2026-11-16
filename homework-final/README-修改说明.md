# GE6001 作业：Introduction 修改稿

本作业依据 `/home/jiang/Downloads/Homework-final.docx` 完成。题目要求修改论文 Introduction，并保留修改痕迹。

## 应提交的文件

- **Homework-final-marked.pdf**：固定显示全部修改痕迹，红色删除线表示删去的内容，蓝色下划线表示新增内容。适合直接提交和查看。
- **Homework-final-tracked.docx**：包含真正的 Word 修订记录，可以逐条接受或拒绝修改。用 Word 的“审阅 → 显示以供审阅 → 所有标记”查看；不要在提交前接受全部修订，否则会失去题目要求的修改痕迹。

`Homework-final-clean.pdf` 与 `Homework-final-clean.docx` 是修改后的通读版本，供核对内容；单独提交干净稿不满足“保留修改痕迹”的要求。

## 修改重点

1. **语言与术语**：修正拼写、主谓一致、冠词和搭配，例如 `venders`、`hardwares`、`an easy work`、`Different with` 等；规范 SDN、VXLAN 和云平台名称的写法。
2. **结构与衔接**：按“云计算背景 → 连接诊断难点 → 相关工作 → VTrace → 三项贡献”组织内容。将 VTrace 的介绍放在相关工作之后，减少背景重复和过长句子。
3. **学术表述**：将 `unacceptable`、`never an easy work` 等夸张或口语表达改为具体、克制的说明。
4. **证据边界**：删除未经论证的“the first”优先权断言；不再将配置转发设备的方案直接称为“noninvasive”。将“negligible effect”“a short time”等缺少测量支持的效果，改为设计目标。原始材料未提供实验数据，因此没有编造时延、开销或规模数字。
5. **引用与归属**：保留原有全部引用组 `[4]`、`[6, 7, 9, 35]`、`[3, 11, 12, 37]`、`[13, 33]`、`[8, 27]`、`[1, 15]`。题目未提供参考文献表，不能确认各编号对应的文献，也未补造参考文献。若后续加入完整论文，需与原参考文献表核对。

## LaTeX 源码

`Homework-final.tex` 默认输出带修改痕迹的版本，然后另起一页输出干净稿。可上传 Overleaf，使用 pdfLaTeX 编译；本地安装 TeX Live 后也可运行：

```sh
pdflatex Homework-final.tex
pdflatex Homework-final.tex
```

- 只输出带痕迹版本：将 `\cleansectiontrue` 改为 `\cleansectionfalse`。
- 只输出干净稿：将 `\markedsectiontrue` 改为 `\markedsectionfalse`。

当前环境没有 LaTeX 编译器，所附 PDF 由同一份修订内容的 Word 排版稿经 LibreOffice 导出；LaTeX 源码未在本机编译验证，分页和字体可能与附带 PDF 不同。

## 可复核材料

`source/build_homework.py` 使用 Python 标准库从原题生成 Word 原生修订、固定标记稿、干净稿和 LaTeX 源码。`source/revised_introduction.txt` 提供最终纯文本。重新生成文件时运行：

```sh
python3 source/build_homework.py /path/to/Homework-final.docx
```

生成脚本检查全部增删是否能还原原文与定稿，以及引用编号是否完整保留。姓名、学号等个人信息未提供，未代填。
