#import "@preview/i-figured:0.2.4"
#import "@preview/codly:1.0.0" : *

// 假段落的实现，用于解决typst 标题后第一段首行不缩进的问题
#let empty-par = par[#box()] // 假段落，附着于 heading 之后可以实现首行缩进
#let fake-par = context empty-par + v(-measure(empty-par + empty-par).height)

#let report_monci(
  title: none,
  num_columns: 2, 
  content_header: none,
  abstract: none,
  font_size_global: 11pt,
  doc
) = {

// 页面设置
set page(
  paper: "a4", // 页面大小
  numbering: "1 / 1", // 页码
    header: [ // 页眉
      #set align(right)
      #text(
        content_header,
        1em
      )
      #line(length: 100%)
    ],

)

set heading(
  numbering: "1.1.1)", // 标题样式
)

// 设置公式显示格式
show math.equation: it => [
#show math.equation: i-figured.show-equation.with(
    level: 0,
    zero-fill: true,
    leading-zero: true,
    numbering: "(1.1)",
    prefix: "eqt:",
    only-labeled: false,  // numbering all block equations implicitly
    unnumbered-label: "-",
)
#if(it.has("label")){
  it
  fake-par
}else{
  it
}
]

// 显示图表显示格式
show figure: it => [
  #set text(size: 9pt)
  #show figure: i-figured.show-figure.with(
     numbering: "1",
     level: 0,
    //  extra-prefixes: (simp:"fig:"),
     leading-zero: true,
     fallback-prefix: "fig:",
  )
  #it
  #fake-par
]

show: codly-init.with()
codly(
  languages:(
    python:(
      name: "Python",
      color: rgb("#1573df"),
    ),
  ),
)
codly(zebra-fill: none)

// 编程语言源代码文本显示时，与后续文本间强制分段（typst默认不分段，无段首缩进）
show raw: it => [
  #it
  #fake-par
]

//------------------------------------------------------------------------

set align(center)

text(
  title, //正文大标题
  1.8em
)


par(justify: true, first-line-indent: 2em)[
  // 两端对齐，段前缩进2字符
  #show heading: it => {
    it
    fake-par
  }


#set align(left)

#outline(indent: 1em, title: [目  录],depth: 3)

#colbreak()

#set text(size: font_size_global)
#columns(num_columns, doc)
]
}