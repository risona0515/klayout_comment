
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


#include "dbEdgePair.h"

namespace tl
{

// [[ZH-BEGIN]]
// 功能：为 tl::Extractor 注册“如何解析一个**边对**”，与 dbPoint.cc / dbEdge.cc 同构。
//
// 【★ 边的文本语法（这是本文件最值得记住的内容）】
//   一个边对由两条边 + 一个分隔符构成，分隔符决定它的**对称性**：
//       "e1/e2"   → 用斜杠 → **有向 (directed)**：e1 与 e2 不可交换
//       "e1|e2"   → 用竖线 → **对称 (symmetric)**：e1 与 e2 可交换
//   这正好对应 db::edge_pair 的 symmetric 标志（见 dbEdgePair.h 的说明）。
//   ★ 因此从字符串就能读出语义：竖线意味着“这两条边不分先后”。
//     在做 DRC 结果去重/比较时，这个区别直接影响到“同一条违规会不会被算两次”。
//
// 【★ 一个写法上的亮点：规范的“**保存-回退**”模式】
//   看 _test_extractor_impl 里：
//       tl::Extractor ex_saved = ex;      // 先快照
//       ... 尝试解析 ...
//       if (中间任何一步失败) { ex = ex_saved; return false; }   // 完全回退
//   ★ 这才是“试探性解析”应该有的样子：失败时**完整恢复**解析器位置，
//     不留下任何副作用。
//   ★ 对比：dbPoint.cc 的同名函数就缺点这个（上游自己标了 TODO，
//     说它的 test “并不是真的 test”）。因此本文件可以作为
//     “怎么写才对”的参照，而 dbPoint.cc 是“怎么写得不够好”的例子。
//
// 提供的特化（强制版 + 试探版，Coord 与 DCoord 各一份）：
//   extractor_impl<EdgePair> / extractor_impl<DEdgePair>
//   test_extractor_impl<EdgePair> / test_extractor_impl<DEdgePair>
//
// 注意：出错信息复用的是 “Expected an edge specification”——
//   即边对沿用了边的报错文案，没有单独区分。
// [[ZH-END]]
template<> void extractor_impl (tl::Extractor &ex, db::EdgePair &e)
{
  if (! test_extractor_impl (ex, e)) {
    ex.error (tl::to_string (tr ("Expected an edge specification")));
  }
}

template<> void extractor_impl (tl::Extractor &ex, db::DEdgePair &e)
{
  if (! test_extractor_impl (ex, e)) {
    ex.error (tl::to_string (tr ("Expected an edge specification")));
  }
}

template<class C> bool _test_extractor_impl (tl::Extractor &ex, db::edge_pair<C> &e)
{
  typedef db::edge<C> edge_type;
  tl::Extractor ex_saved = ex;

  edge_type e1, e2;

  if (ex.try_read (e1)) {

    bool symmetric = false;
    if (ex.test ("|")) {
      symmetric = true;
    } else if (! ex.test ("/")) {
      ex = ex_saved;
      return false;
    }

    if (! ex.try_read (e2)) {
      ex = ex_saved;
      return false;
    }

    e = db::edge_pair<C> (e1, e2, symmetric);
    return true;

  } else {
    return false;
  }
}

template<> bool test_extractor_impl (tl::Extractor &ex, db::EdgePair &e)
{
  return _test_extractor_impl (ex, e);
}

template<> bool test_extractor_impl (tl::Extractor &ex, db::DEdgePair &e)
{
  return _test_extractor_impl (ex, e);
}

} // namespace tl

