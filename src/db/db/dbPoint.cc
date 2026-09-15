
/*

  KLayout Layout Viewer
  Copyright (C) 2006-2026 Matthias Koefferlein

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/


#include "dbPoint.h"
#include "dbVector.h"

// ----------------------------------------------------------------
//  Implementation of the custom extractors
// [[ZH-BEGIN]]
// 功能：本文件把“**从文本/脚本里读出一个点**”接入 tl::Extractor 体系。
//
// 【Extractor 是什么】
//   tl::Extractor 是库内置的**文本解析器**（见 tl/tlExtractor.h），
//   负责把字符串里的一段内容读成 C++ 值。它是“脚本参数 / 文本格式 / GDS 属性”
//   等场景的**统一入口**。
//   对点而言，接受的文本形式是 **"x,y"**（逗号分隔；
//   注意 edge 用的是分号："(x1,y1;x2,y2)"，两者不同）。
//
// 【为什么需要两个函数（test 与 非 test）】
//   · _test_extractor_impl  —— **试探性**读取：成功返回 true，失败返回 false。
//     用于“看看下一个 token 是不是一个点”。
//   · _extractor_impl       —— **强制性**读取：失败则报错（ex.error）。
//     用于“这里必须是一个点”。
//   这一对是库内的固定配套：凡是可解析的类型都提供这两个版本。
//
// 【★ 上游遗留的一个已知缺陷（有待者自行判断，本注释不改代码）】
//   看下面 _test_extractor_impl 里的 TODO：
//     “this is not really a \"test\": we should move back if we did not receive a \",\"”
//   意思是：它读到一个可解析的 x 之后，会**强制要求**后面跟逗号；
//   若没有逗号就会报错，而“试探”本应能**回退**到读取前的状态。
//   因此这个 test 函数在“长得像数字但不是点”的输入上不够健壮。
//   ★ 实用影响：解析外部文本时，不要假设 test_* 一定能安全回退。
//
// 下面两个模板按坐标类型 C 泛化，最后由本文件末尾按 Coord / DCoord 特化。
// [[ZH-END]]

namespace {

  // [[ZH]] 功能：**试探性**读一个点（"x,y"）；成功则写入 p 并返回 true，否则返回 false。
  // [[ZH]] 参数：ex 文本解析器；p 输出参数（构造出的点）。
  // [[ZH]] 修改：p（仅在成功时）。
  // [[ZH]] 坑：见上面文件头的 TODO 说明 —— 读到 x 后会强制 expect(",")，
  // [[ZH]]     缺少逗号时会直接报错，而不是干净地返回 false。
  template <class C>
  bool _test_extractor_impl (tl::Extractor &ex, db::point<C> &p)
  {
    //  TODO: this is not really a "test": we should move back if we did not
    //  receive a ",".
    C x = 0;
    if (ex.try_read (x)) {
      ex.expect (",");
      C y = 0;
      ex.read (y);
      p = db::point<C> (x, y);
      return true;
    } else {
      return false;
    }
  }

  // [[ZH]] 功能：**强制性**读一个点；读不到就报错（而不是返回 false）。
  // [[ZH]] 参数：ex 文本解析器；p 输出参数。
  // [[ZH]] 何时用：确定此处必须是点时（例如解析格式规定的字段）。
  // [[ZH]] 提示：报错信息用的是 tr() 包裹的可翻译字串，因此支持多语言界面。
  template <class C>
  void _extractor_impl (tl::Extractor &ex, db::point<C> &p)
  {
    if (! _test_extractor_impl (ex, p)) {
      ex.error (tl::to_string (tr ("Expected a point specification")));
    }
  }

}

namespace db
{

// [[ZH-BEGIN]]
// 功能：显式实例化（explicit instantiation）—— 让模板的实现编译进本 TU。
//
// ★ 为什么需要它（这是全库很常见的一种模式）：
//   模板的定义通常写在头文件里（否则其他 TU 用不了）；
//   而本库为了缩短编译时间/缩小目标文件，把很多模板的**定义放在 .cc**，
//   再用这里的 `template class X<Coord>;` 明确告知编译器
//   “请在本单元生成这两个特化的代码”。
//   因此这类 .cc 文件往往只有少量“真正的逻辑”（如上面的 extractor），
//   大部分行数都是实例化语句。
//
// ★ 实用影响：看到 `template class foo<Coord>;` 时，
//   不要期待这里有实现细节 —— 实现要去对应的 .h 找。
// [[ZH-END]]
//  instantiations
template class point<Coord>;
template class point<DCoord>;

}

namespace tl 
{
  
// [[ZH-BEGIN]]
// 功能：为 tl::Extractor 注册“如何解析一个点”，分**强制**与**试探**两组。
//
// ★ 为什么要放在 namespace tl 里：
//   Extractor 的扩展点是 tl 命名空间下的模板函数
//   extractor_impl / test_extractor_impl。库对每个可解析类型做一次特化，
//   就把它接入了统一的解析体系（因此在脚本里能直接写点坐标、
//   在文本格式里也能自动解析）。
//   这套“在哪都放同一对特化”的做法，是库内可解析类型的统一约定。
//
// 本文件提供四个特化：
//   extractor_impl<Point> / extractor_impl<DPoint>           强制版
//   test_extractor_impl<Point> / test_extractor_impl<DPoint>  试探版
// 每个都只是转到上面匿名命名空间里的实现。
// [[ZH-END]]
template <> 
void 
extractor_impl (tl::Extractor &ex, db::Point &p)
{
  _extractor_impl (ex, p);
}

template <> 
void 
extractor_impl (tl::Extractor &ex, db::DPoint &p)
{
  _extractor_impl (ex, p);
}

template <> 
bool 
test_extractor_impl (tl::Extractor &ex, db::Point &p)
{
  return _test_extractor_impl (ex, p);
}

template <> 
bool 
test_extractor_impl (tl::Extractor &ex, db::DPoint &p)
{
  return _test_extractor_impl (ex, p);
}

} // namespace tl

