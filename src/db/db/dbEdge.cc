
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


#include "dbEdge.h"
#include "tlLongInt.h"

namespace db
{

// [[ZH-BEGIN]]
// 功能：求两个整数的**最大公约数**（欧几里得算法）。
//
// 【为什么 dbEdge.cc 里需要一个 gcd】
//   因为 div_exact（本文件下面那个函数，见 dbEdge.h 的声明）要**精确**计算
//   表达式 (a*b + d/2) / d 这类“乘以再除以”的值。
//   若 b 与 d 有公因子，先约分可以避免中间乘积溢出 ——
//   这是“用整数得到精确有理数结果”的经典手法。
//
// 参数：a、b 两个非负整数（本用途下）。
// 返回：gcd(a, b)。
// 实现：标准的辗转相除（取模 + 交换），无需递归。
// [[ZH-END]]
/**
 *  @brief Computes the gcd of two numbers
 */
template <class C>
inline C gcd (C a, C b)
{
  while (b != 0) {
    a %= b;
    std::swap (a, b);
  }
  return a;
}

// [[ZH-BEGIN]]
// 功能：定义一个**比 area_type 更宽一倍**的中间乘积类型 a2_type。
//
// ★ 为什么还需要它（area_type 已经是最宽的了）：
//   div_exact 内部要算 (a * b)，其中 a 是**坐标**、b 是**面积类型**。
//   两者的乘积再乘一次就超出了 area_type 本身的范围 → 必须再加宽一倍。
//   这正是“为什么 dbTypes.h 里那些类型宽度选择是硬需求”的具体体现。
//
// 【两种实现路径（编译器能力探测）】
//   · 有 __int128（GCC/Clang）→ 直接用内建的 128 位整数（最快）。
//   · 否则（如 MSVC）→ 回退到 tl::long_int<4, uint32_t, uint64_t>，
//     即用库自己实现的“多字长整数”（4 个 32 位字 = 128 位）。
//   ★ 这是可移植性处理的一个好例子：功能相同，但按编译器能力选最快的实现。
// [[ZH-END]]
#if defined(__SIZEOF_INT128__)
typedef __int128 a2_type;
#else
//  fallback to long_int in case the __int128 isn't defined
typedef tl::long_int<4, uint32_t, uint64_t> a2_type;
#endif

// [[ZH-BEGIN]]
// 功能：计算 (a * b) / d 并**精确取整**；0.5 的情形向零方向舍（舍入方向固定）。
//
// 参数（见 dbEdge.h 的声明处也有说明）：
//   a —— 被除数因子（坐标，可为负）；
//   b, d —— 面积的分子/分母，均为正（b 是叉积/面积，d 是分母）。
// 返回：db::Coord —— 已回到坐标类型的结果。
//
// ★★ 为什么不能直接写 `a * b / d`（本函数存在的全部理由）：
//   1) **溢出**：a*b 会超出 area_type（即使它是 int64）——
//      所以下面全部用 a2_type（128 位）做中间运算。
//   2) **舍入方向必须确定**：C++ 整数除法是**向零截断**，而几何需要的是
//      “四舍五入且 0.5 固定朝一个方向”。因此先**加上 d/2** 再除：
//          正数：+ (d-1)/2   （等价于四舍五入，0.5 向上）
//          负数：+ d/2       （因为负数除法向零截，这样恰好得到对称行为）
//      ★ 正负用不同偏移，是为了让结果**关于符号对称** ——
//        否则同一个交点从正方向/反方向计算会得到差 1 的坐标，
//        进而破坏扫描线排序的稳定性（参见 dbEdgeProcessor 的说明）。
//
// 【谁在用】db::edge::intersect_point / crossed_by_point 求交点坐标，
//   以及 EdgeProcessor 的求交 —— 它们都依赖本函数给出**可复现**的整数结果。
// [[ZH-END]]
db::Coord div_exact (db::Coord a, db::coord_traits<db::Coord>::area_type b, db::coord_traits<db::Coord>::area_type d)
{
  if (a < 0) {
    return -db::Coord ((a2_type (-a) * a2_type (b) + a2_type (d / 2)) / a2_type (d));
  } else {
    return db::Coord ((a2_type (a) * a2_type (b) + a2_type ((d - 1) / 2)) / a2_type (d));
  }
}

}

namespace tl
{

// [[ZH-BEGIN]]
// 功能：为 tl::Extractor 注册“如何解析一条边”，与 dbPoint.cc 的做法同构。
//
// 【边的文本格式】注意与点不同：边用 **括号 + 分号**，形如
//       "(x1,y1;x2,y2)"
//   而 point 是 "x,y"（逗号分隔、无括号）。
//   ★ 这些格式会出现在脚本、文本图形格式、错误信息里，改动会破坏兼容性。
//
// 【实现要点】
//   _test_extractor_impl 里的解析顺序：
//     test("(")   → 先要求左括号（这是它区分“是边”与“不是边”的。
//                   优点：失败时可以干净地返回 false，不消费输入）；
//              → 读点1 → expect(";") → 读点2 → expect(")");
//     ★ 两个点都复用点的解析器（ex.read(p1)）——
//       所以点的解析规则改了，边的解析会自动跟着变。
//
// 下面提供的特化（强制版 + 试探版，Coord 与 DCoord 各一份）：
//   extractor_impl<Edge> / extractor_impl<DEdge>
//   test_extractor_impl<Edge> / test_extractor_impl<DEdge>
// [[ZH-END]]
template<> void extractor_impl (tl::Extractor &ex, db::Edge &e)
{
  if (! test_extractor_impl (ex, e)) {
    ex.error (tl::to_string (tr ("Expected an edge specification")));
  }
}

template<> void extractor_impl (tl::Extractor &ex, db::DEdge &e)
{
  if (! test_extractor_impl (ex, e)) {
    ex.error (tl::to_string (tr ("Expected an edge specification")));
  }
}

template<class C> bool _test_extractor_impl (tl::Extractor &ex, db::edge<C> &e)
{
  typedef db::point<C> point_type;

  if (ex.test ("(")) {

    point_type p1, p2;
    ex.read (p1);
    ex.expect (";");
    ex.read (p2);

    e = db::edge<C> (p1, p2);

    ex.expect (")");

    return true;

  } else {
    return false;
  }
}

template<> bool test_extractor_impl (tl::Extractor &ex, db::Edge &e)
{
  return _test_extractor_impl (ex, e);
}

template<> bool test_extractor_impl (tl::Extractor &ex, db::DEdge &e)
{
  return _test_extractor_impl (ex, e);
}

} // namespace tl

