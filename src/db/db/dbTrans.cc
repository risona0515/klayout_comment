
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


#include "dbTrans.h"
#include "tlInternational.h"

// ----------------------------------------------------------------
//  Implementation of the custom extractors
// [[ZH-BEGIN]]
// 功能：本文件为**所有变换类型**注册文本解析（tl::Extractor 体系）。
//
// 【本文件的特点：八个类型、每个两份、共 16 个特化】
//   强制版 extractor_impl<T> 与试探版 test_extractor_impl<T>，T 包括：
//     FTrans / Trans / DTrans / Disp / DDisp / CplxTrans / ICplxTrans /
//     DCplxTrans / VCplxTrans
//   它们都只是转给上面匿名命名空间里的模板实现（与 dbPoint.cc 同构）。
//
// 【★ 变换的文本语法很“宽松”，值得理解】
//   看下面 simple_trans 的 _test_extractor_impl：它在一个**循环**里反复尝试：
//       · 能读到一个 fixpoint 变换（如 “r90”、“m0”） → 记为旋转/镜像部分
//       · 能读到一个 vector（如 “100,200”）           → 记为位移部分
//       两者都读不到时就停下，并把已读到的部分组合成结果。
//   因此以下写法都能被接受（顺序不限）：
//       "r90"          "100,200"          "r90 100,200"
//   ★ 实用影响：在脚本/文本格式里描述变换时，可以只写你关心的那部分
//     （只写旋转、或只写位移）——“缺的部分”默认就是单位/零。
//   代价：解析逻辑比“固定格式”复杂，且**顺序不敏感**意味着写错顺序不会报错。
//
// 【与其它 extractor 文件的一致约定】
//   出错信息统一为 “Expected a transformation specification”（translation 可翻译）。
// [[ZH-END]]

namespace {

  // [[ZH-BEGIN]]
  // 功能：解析一个 simple_trans —— **循环累积**“旋转/镜像”与“位移”两部分。
  //
  // 参数：ex 解析器；t 输出参数。
  // 返回：读到至少一部分 → true；什么都没读到 → false（此时不改 t）。
  //
  // 修改：t（仅在读到内容时）。
  // 实现要点：
  //   · any 标志表示“到目前为止是否读到了任何东西”——
  //     它既决定“是否返回 true”，也决定“是否要写 t”。
  //     ★ 这样做的好处：完全没读到内容时**不修改** t，
  //       使试探能够干净地失败（调用方可回退）。
  //   · f（fixpoint）与 p（vector）交替尝试，因此**顺序无关**。
  //   · 构造时用 f.rot() 而不是整个 f —— 位移由 p 单独提供。
  // [[ZH-END]]
  template <class C>
  bool _test_extractor_impl (tl::Extractor &ex, db::simple_trans<C> &t)
  {
    bool any = false;
    db::FTrans f;
    db::vector<C> p;
    while (true) {
      if (ex.try_read (f)) {
        any = true;
      } else if (ex.try_read (p)) {
        any = true;
      } else {
        if (any) {
          t = db::simple_trans<C> (f.rot (), p);
        }
        return any;
      }
    }
  }

  // [[ZH]] 功能：**强制**解析一个 simple_trans；读不到就报错。
  // [[ZH]] 参数：ex 解析器；t 输出参数。
  // [[ZH]] 何时用：格式规定此处必须是变换时。
  template <class C>
  void _extractor_impl (tl::Extractor &ex, db::simple_trans<C> &t)
  {
    if (! _test_extractor_impl (ex, t)) {
      ex.error (tl::to_string (tr ("Expected a transformation specification")));
    }
  }

  // [[ZH]] 功能：解析一个 disp_trans（只读位移 vector）。
  // [[ZH]] 返回：读到至少一个 vector → true；否则 false（不改 t）。
  // [[ZH]] 与 simple_trans 版的差异：这里**没有**旋转部分（disp_trans 不支持旋转），
  // [[ZH]]       因此循环里只尝试读 vector，解析更快也更明确。
  template <class C>
  bool _test_extractor_impl (tl::Extractor &ex, db::disp_trans<C> &t)
  {
    bool any = false;
    db::vector<C> p;
    while (true) {
      if (ex.try_read (p)) {
        any = true;
      } else {
        if (any) {
          t = db::disp_trans<C> (p);
        }
        return any;
      }
    }
  }

  template <class C>
  void _extractor_impl (tl::Extractor &ex, db::disp_trans<C> &t)
  {
    if (! _test_extractor_impl (ex, t)) {
      ex.error (tl::to_string (tr ("Expected a transformation specification")));
    }
  }

  template <class I, class F, class R>
  bool _test_extractor_impl (tl::Extractor &ex, db::complex_trans<I, F, R> &t)
  {
    t = db::complex_trans<I, F, R> ();
    bool any = false;
    while (true) {
      db::vector<F> p;
      if (ex.test ("*")) {
        double f = 1.0;
        ex.read (f);
        t.mag (f);
        any = true;
      } else if (ex.try_read (p)) {
        t.disp (p);
        any = true;
      } else if (ex.test ("m")) {
        double a = 0.0;
        ex.read (a);
        t.mirror (true);
        t.angle (a * 2.0);
        any = true;
      } else if (ex.test ("r")) {
        double a = 0.0;
        ex.read (a);
        t.mirror (false);
        t.angle (a);
        any = true;
      } else {
        break;
      }
    }
    return any;
  }

  template <class I, class F, class R>
  void _extractor_impl (tl::Extractor &ex, db::complex_trans<I, F, R> &t)
  {
    if (! _test_extractor_impl (ex, t)) {
      ex.error (tl::to_string (tr ("Expected transformation specification")));
    }
  }

  template <class C1, class C2>
  bool _test_extractor_impl (tl::Extractor &ex, db::combined_trans<C1, C2> &t)
  {
    bool any = false;
    C1 t1;
    C2 t2;
    while (true) {
      if (ex.try_read (t1)) {
        any = true;
      } else if (ex.try_read (t2)) {
        any = true;
      } else {
        if (any) {
          t = db::combined_trans<C1, C2> (t1, t2);
        }
        return any;
      }
    }
  }

  template <class C1, class C2>
  void _extractor_impl (tl::Extractor &ex, db::combined_trans<C1, C2> &t)
  {
    if (! _test_extractor_impl (ex, t)) {
      ex.error (tl::to_string (tr ("Expected transformation/magnification specification")));
    }
  }

}

namespace tl
{

// [[ZH-BEGIN]]
// 功能：为各个变换类型注册“强制解析 + 试探解析”两种特化（与 dbPoint.cc 完全同构）。
//
// ★ 为什么要按类型分别特化，而不能写一个泛化的：
//   每种变换能接受的文本**不同**，因此必须各自指明用哪个 _test_extractor_impl：
//     · disp_trans      不接受旋转（只能读位移）
//     · fixpoint_trans  只读旋转/镜像码
//     · simple_trans    两者都可，且顺序无关
//     · complex_trans   还可读角度与缩放
//   这就是本文件看起来“重复”的原因 —— 每个特化都只是两三行转发。
//
// ★ 从报错文案里就能读出各类型的**文本语法**（很有用，等于自带文档）：
//     · fixpoint/rotation 类：
//         \"Expected rotation/mirror code (r0,r90,r180,r270,m0,m45,m90,m135)\"
//       → 即旋转码 r0/r90/r180/r270，镜像码 m0/m45/m90/m135。
//       ★ 注意 m0/m45/m90/m135 是**镜像后再旋转**的组合码，
//         因此 m0 与 r0 不同（m0 含镜像，r0 不含）。
//     · complex 类另有 “transformation/magnification specification” 的提示，
//       说明它还能读缩放。
//     · 其余统一是 “Expected transformation specification”。
//
// ★ 与库内其它可解析类型一致的两条约定：
//   1) 强制版失败时**报错**（而非返回 false）；
//   2) 试探版失败时**不修改**输出参数 —— 属性（见上面的 any 标志）。
//
// 提示：下面每个特化只做转发，不做额外处理，因此不逐个注释；
//   想知道某类型支持什么文本，看它转发的实现（匿名命名空间内）即可。
// [[ZH-END]]
template <>
void DB_PUBLIC 
extractor_impl (tl::Extractor &ex, db::FTrans &t)
{
  if (! test_extractor_impl (ex, t)) {
    ex.error (tl::to_string (tr ("Expected rotation/mirror code (r0,r90,r180,r270,m0,m45,m90,m135)")));
  }
}

template <>
void DB_PUBLIC 
extractor_impl (tl::Extractor &ex, db::Trans &t)
{
  _extractor_impl (ex, t);
}

template <>
void DB_PUBLIC 
extractor_impl (tl::Extractor &ex, db::DTrans &t)
{
  _extractor_impl (ex, t);
}

template <>
void DB_PUBLIC 
extractor_impl (tl::Extractor &ex, db::Disp &t)
{
  _extractor_impl (ex, t);
}

template <>
void DB_PUBLIC 
extractor_impl (tl::Extractor &ex, db::DDisp &t)
{
  _extractor_impl (ex, t);
}

template <>
void DB_PUBLIC 
extractor_impl (tl::Extractor &ex, db::CplxTrans &t)
{
  _extractor_impl (ex, t);
}

template <>
DB_PUBLIC void 
extractor_impl (tl::Extractor &ex, db::ICplxTrans &t)
{
  _extractor_impl (ex, t);
}

template <>
DB_PUBLIC void 
extractor_impl (tl::Extractor &ex, db::DCplxTrans &t)
{
  _extractor_impl (ex, t);
}

template <>
DB_PUBLIC void
extractor_impl (tl::Extractor &ex, db::VCplxTrans &t)
{
  _extractor_impl (ex, t);
}


template <>
DB_PUBLIC bool 
test_extractor_impl (tl::Extractor &ex, db::FTrans &t)
{
  if (ex.test ("r0")) {
    t = db::FTrans (db::FTrans::r0);
    return true;
  } else if (ex.test ("r90")) {
    t = db::FTrans (db::FTrans::r90);
    return true;
  } else if (ex.test ("r180")) {
    t = db::FTrans (db::FTrans::r180);
    return true;
  } else if (ex.test ("r270")) {
    t = db::FTrans (db::FTrans::r270);
    return true;
  } else if (ex.test ("m0")) {
    t = db::FTrans (db::FTrans::m0);
    return true;
  } else if (ex.test ("m45")) {
    t = db::FTrans (db::FTrans::m45);
    return true;
  } else if (ex.test ("m90")) {
    t = db::FTrans (db::FTrans::m90);
    return true;
  } else if (ex.test ("m135")) {
    t = db::FTrans (db::FTrans::m135);
    return true;
  } else {
    return false;
  }
}

template <>
DB_PUBLIC bool 
test_extractor_impl (tl::Extractor &ex, db::Trans &t)
{
  return _test_extractor_impl (ex, t);
}

template <>
DB_PUBLIC bool 
test_extractor_impl (tl::Extractor &ex, db::DTrans &t)
{
  return _test_extractor_impl (ex, t);
}

template <>
DB_PUBLIC bool 
test_extractor_impl (tl::Extractor &ex, db::Disp &t)
{
  return _test_extractor_impl (ex, t);
}

template <>
DB_PUBLIC bool 
test_extractor_impl (tl::Extractor &ex, db::DDisp &t)
{
  return _test_extractor_impl (ex, t);
}

template <>
DB_PUBLIC bool 
test_extractor_impl (tl::Extractor &ex, db::CplxTrans &t)
{
  return _test_extractor_impl (ex, t);
}

template <>
DB_PUBLIC bool 
test_extractor_impl (tl::Extractor &ex, db::ICplxTrans &t)
{
  return _test_extractor_impl (ex, t);
}

template <>
DB_PUBLIC bool 
test_extractor_impl (tl::Extractor &ex, db::DCplxTrans &t)
{
  return _test_extractor_impl (ex, t);
}

template <>
DB_PUBLIC bool
test_extractor_impl (tl::Extractor &ex, db::VCplxTrans &t)
{
  return _test_extractor_impl (ex, t);
}

} // namespace tl

