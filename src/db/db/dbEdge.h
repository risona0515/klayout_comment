
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



#ifndef HDR_dbEdge
#define HDR_dbEdge

#include "dbCommon.h"

#include "dbTypes.h"
#include "dbPoint.h"
#include "dbVector.h"
#include "dbTrans.h"
#include "dbObjectTag.h"
#include "dbBox.h"

#include <string>

namespace db {

// [[ZH-BEGIN]]
// 功能：本文件定义核心几何基元之一 —— 「有向边」(directed edge)。
//       一条边由两个端点 p1、p2 构成，方向从 p1 指向 p2。
//
//       ★★★ 全库最重要的约定：边的方向是有语义的 ★★★
//       多边形轮廓上的边被赋予固定走向，使得：
//           「边的右侧」 == 多边形内部
//           「边的左侧」 == 多边形外部
//       于是「边翻个方向」就等价于「内外互换」。
//       所有需要判断内部/外部的算法（布尔运算、DRC、LVS、面积计算）都建立在此约定上。
//       这就是下面 transform()/transformed() 遇到镜像变换时必须交换 p1/p2 的原因，
//       也是 EdgeProcessor 能够只靠「左右」信息重现多边形的原因。
//
// 参数：模板参数 C 是坐标类型 ——
//       C = db::Coord  (int32_t) → typedef 为 db::Edge，整数坐标，数据库存储/布尔运算用
//       C = db::DCoord (double)  → typedef 为 db::DEdge，浮点坐标，几何计算用
//       两者接口完全一致，仅精度与数值类型不同。
//
// 坑：1) 库中不区分「线段」与「其所在的直线」，同一对象在不同方法里含义不同：
//         按线段理解：contains / contains_excl / intersect / intersect_point / crossed_by
//         按直线理解：cut_point / clipped_line
//       阅读时必须看清方法名，别想当然。
//     2) 「长度」有多个版本，别混用（已逐一核对实现）：
//         length()         欧氏长度 sqrt(dx^2+dy^2)，四舍五入回 distance_type
//         double_length()  欧氏长度，保持 double 精度（不做舍入）
//         sq_length()      欧氏长度平方，比较长短时免开方
//         ortho_length()   曼哈顿长度 |dx|+|dy|
//         dx_abs()/dy_abs() 单轴分量长度（实现上对 int32 防溢出）
//     3) 「模糊」比较（less/equal/contains/coincident）带 ±1 DBU 容差
//        (coord_traits::prec_distance())，不是精确相等。
// [[ZH-END]]
/**
 *  @brief A helper function for dividing integers with exact rounding
 *  This function computes (a*b/d) where rounding is exact in the sense of:
 *    a*b/d == N+0.5 => div_exact(a*b/d) = N
 *  b and d needs to be positive.
 *  a can be positive or negative.
 *  The implementation uses the gcd to reduce the ratios. This way we can
 *  represent the numbers with the area type.
 */
// [[ZH]] 功能：计算 (a*b/d) 并「精确」取整，用整数面积类型避免溢出。
// [[ZH]]       精确的含义：恰为 N+0.5 时向下取整成 N（普通四舍五入会得 N+1）。
// [[ZH]] 参数：a 可正可负；b、d 必须为正（当作长度/分母用）。
// [[ZH]] 坑：这是全库求交点坐标的基础工具 —— 交点 x = x1 + dx*vxa/(vxa+vxb)
// [[ZH]]     这类表达式必须用它，否则会因 .5 舍入方向不同而算出两个不同交点，
// [[ZH]]     导致 EdgeProcessor 的 scanline 排序失去稳定性。见 dbEdgeProcessor.cc 中的调用。
db::Coord DB_PUBLIC div_exact (db::Coord a, db::coord_traits<db::Coord>::area_type b, db::coord_traits<db::Coord>::area_type d);

/**
 *  @brief An overload of div_exact for double types
 */
// [[ZH]] 功能：div_exact 的浮点版本，直接算 double 表达式后再 rounded() 回整数网格。
// [[ZH]] 坑：与整数版行为一致（.5 向下），但依赖 coord_traits<DCoord>::rounded() 的实现。
inline db::DCoord div_exact (db::DCoord a, db::coord_traits<db::DCoord>::area_type b, db::coord_traits<db::DCoord>::area_type d)
{
  return db::coord_traits<db::DCoord>::rounded (double (a) * double (b) / double (d));
}

template <class C> class generic_repository;
class ArrayRepository;

// [[ZH]] 功能：有向边模板类。真正的实现体，db::Edge / db::DEdge 都是它的实例化别名。
// [[ZH]] 参数：C = 坐标类型（db::Coord 或 db::DCoord），决定所有坐标运算的数值类型。
// [[ZH]] 坑：它只有两个成员 m_p1、m_p2（见文件末尾 private 区），没有任何缓存/标志位，
// [[ZH]]     因此 sizeof(edge) 很小、拷贝廉价 —— 库中大量按值传递 edge 正是基于这一点。
template <class C>
class DB_PUBLIC_TEMPLATE edge
{
public:
  // [[ZH]] 以下是一组「类型别名」，作用是让容器/模板能在不知道 C 具体是什么的情况下
  // [[ZH]] 取到配套类型（例如 BoxRepository 需要 box_type 才能实例化）。
  // [[ZH]] 注意：这些别名是「绑定在类型上」的，不是运行时变量，因此没有存储开销。
  // [[ZH]] coord_type      → 坐标类型本身（db::Coord 或 db::DCoord）
  // [[ZH]] box_type        → 本边类型对应的包围盒类型 db::box<C>
  // [[ZH]] point_type      → 本边类型对应的点类型 db::point<C>
  // [[ZH]] vector_type     → 本边类型对应的向量类型 db::vector<C>
  // [[ZH]] coord_traits    → 该坐标类型的数值运算策略（取整、防溢出、带符号乘等）
  // [[ZH]] distance_type   → 距离类型：Coord→int64_t，DCoord→double（用更宽类型防溢出）
  // [[ZH]] area_type       → 面积/叉积类型：Coord→int64_t，DCoord→double
  // [[ZH]] tag             → 供 db::object 体系识别「这是个 edge」，实现通用存取
  typedef C coord_type;
  typedef db::box<C> box_type;
  typedef db::point<C> point_type;
  typedef db::vector<C> vector_type;
  typedef db::coord_traits<C> coord_traits;
  typedef typename coord_traits::distance_type distance_type; 
  typedef typename coord_traits::area_type area_type; 
  typedef db::object_tag< edge<C> > tag;

  /**
   *  @brief The default constructor.
   *  
   *  The default constructor creates a degenerated edge
   *  with two points (0, 0).
   */
  edge ()
    : m_p1 (0, 0), m_p2 (0, 0)
  {
    //  .. nothing else ..
  }

  /**
   *  @brief The standard constructor taking four coordinates.
   *
   *  Creates an edge from (x1,y1) to (x2,y2).
   *
   *  @param x1 The first point's x coordinate.
   *  @param y1 The first point's y coordinate.
   *  @param x2 The second point's x coordinate.
   *  @param y2 The second point's y coordinate.
   */
  template <class D>
  edge (D x1, D y1, D x2, D y2)
    : m_p1 (x1, y1), m_p2 (x2, y2)
  {
    //  .. nothing else ..
  }

  /**
   *  @brief The standard constructor taking two point objects.
   *
   *  Creates an edge from p1 to p2.
   *
   *  @param p1 The first point.
   *  @param p2 The first point.
   */
  template <class D>
  edge (const point<D> &p1, const point<D> &p2)
    : m_p1 (p1), m_p2 (p2)
  {
    //  .. nothing else ..
  }

  /**
   *  @brief The standard constructor taking two point objects.
   *
   *  Creates an edge from p1 to p2.
   *
   *  @param p The first point.
   *  @param v The distance to the end point.
   */
  template <class D>
  edge (const point<D> &p, const vector<D> &v)
    : m_p1 (p), m_p2 (p + v)
  {
    //  .. nothing else ..
  }

  /**
   *  @brief The copy constructor
   */
  template <class D>
  explicit edge (const edge<D> &e)
    : m_p1 (e.p1 ()), m_p2 (e.p2 ())
  {
    //  .. nothing else ..
  }

  /**
   *  @brief The (dummy) translation operator
   */
  void translate (const edge<C> &d, db::generic_repository<C> &, db::ArrayRepository &)
  {
    *this = d;
  }

  /**
   *  @brief The (dummy) translation operator
   */
  template <class T>
  void translate (const edge<C> &d, const T &t, db::generic_repository<C> &, db::ArrayRepository &)
  {
    *this = d;
    transform (t);
  }

  /**
   *  @brief A less operator to establish a sorting order.
   */
  bool operator< (const edge<C> &b) const
  {
    return m_p1 < b.m_p1 || (m_p1 == b.m_p1 && m_p2 < b.m_p2);
  }

  /** 
   *  @brief Equality test
   */
  bool operator== (const edge<C> &b) const
  {
    return m_p1 == b.m_p1 && m_p2 == b.m_p2;
  }

  /** 
   *  @brief Inequality test
   */
  bool operator!= (const edge<C> &b) const
  {
    return !operator== (b);
  }

  /**
   *  @brief A fuzzy less operator to establish a sorting order.
   */
  bool less (const edge<C> &b) const
  {
    return m_p1.less (b.m_p1) || (m_p1.equal (b.m_p1) && m_p2.less (b.m_p2));
  }

  /**
   *  @brief Fuzzy equality test
   */
  bool equal (const edge<C> &b) const
  {
    return m_p1.equal (b.m_p1) && m_p2.equal (b.m_p2);
  }

  /**
   *  @brief Fuzzy inequality test
   */
  bool not_equal (const edge<C> &b) const
  {
    return !equal (b);
  }

  /**
   *  @brief A method binding of operator* (mainly for automation purposes)
   */
  edge<db::DCoord> scaled (double s) const
  {
    return edge<db::DCoord> (db::point<db::DCoord> (p1 ()) * s, db::point<db::DCoord> (p2 ()) * s);
  }

  /**
   *  @brief Returns the moved edge
   *
   *  Moves the edge by the given offset and returns the 
   *  moved edge. The edge is not modified.
   *
   *  @param p The distance to move the edge.
   * 
   *  @return The moved edge.
   */
  edge<C> moved (const vector<C> &p) const
  {
    edge<C> b (*this);
    b.move (p);
    return b;
  }

  /**
   *  @brief Returns the enlarged edge
   *
   *  Enlarges the edge by the given offset and returns the 
   *  moved edge. The edge is not modified. Enlargement means
   *  that the first point is shifted by -p, the second by p.
   *
   *  @param p The distance to move the edge.
   * 
   *  @return The moved edge.
   */
  edge<C> enlarged (const vector<C> &p) const
  {
    edge<C> b (*this);
    b.enlarge (p);
    return b;
  }

  /**
   *  @brief Extends the edge
   *
   *  The extension is applied parallel to the edge at the 
   *  start and end point.
   *  Degenerated edges become horizontal edges.
   */
  edge<C> &extend (C e) 
  {
    vector<double> dp;
    if (is_degenerate ()) {
      dp = vector<double> (e, 0.0);
    } else {
      dp = d () * (double (e) / double_length ());
    }
    *this = edge<C> (point<C> (point<double> (p1 ()) - dp), point<C> (point<double> (p2 ()) + dp));
    return *this;
  }

  /**
   *  @brief Returns the extended edge
   *
   *  The extension is applied parallel to the edge at the 
   *  start and end point.
   *  Degenerated edges become horizontal edges.
   */
  edge<C> extended (C e) const
  {
    vector<double> dp;
    if (is_degenerate ()) {
      dp = vector<double> (e, 0.0);
    } else {
      dp = d () * (double (e) / double_length ());
    }
    return edge<C> (point<C> (point<double> (p1 ()) - dp), point<C> (point<double> (p2 ()) + dp));
  }

  /**
   *  @brief Returns the shifted edge
   *
   *  The shift is applied perpendicular to the edge to the left of the edge 
   *  if the shift is positive and to the right if negative.
   *  Degenerated edges are not shifted.
   */
  edge<C> shifted (C e) const
  {
    if (is_degenerate ()) {
      return *this;
    } else {
      vector<double> dp = d () * (double (e) / double_length ());
      dp = vector<double> (-dp.y (), dp.x ());
      return edge<C> (point<C> (point<double> (p1 ()) + dp), point<C> (point<double> (p2 ()) + dp));
    }
  }

  /**
   *  @brief Shifts the edge
   *
   *  The shift is applied perpendicular to the edge to the left of the edge 
   *  if the shift is positive and to the right if negative.
   *  Degenerated edges are not shifted.
   */
  edge<C> &shift (C e) 
  {
    if (! is_degenerate ()) {
      vector<double> dp = d () * (double (e) / double_length ());
      dp = vector<double> (-dp.y (), dp.x ());
      *this = edge<C> (point<C> (point<double> (p1 ()) + dp), point<C> (point<double> (p2 ()) + dp));
    }
    return *this;
  }

  /**
   *  @brief Transform the edge.
   *
   *  Transforms the edge with the given transformation.
   *  Modifies the edge with the transformed edge.
   *  
   *  @param t The transformation to apply.
   *
   *  @return The transformed edge.
   */
  template <class Tr>
  edge<C> &transform (const Tr &t)
  {
    // [[ZH]] 修改：就地把本边替换为变换后的边。
    // [[ZH]] ★ 关键点：若变换含镜像（is_mirror，如 m0/m45/m90/m135），
    // [[ZH]]   则交换 p1/p2 的顺序再变换。原因见下面的英文 NOTE：
    // [[ZH]]   镜像会翻转手性，若照原顺序变换，「右侧=内部」的约定就会被破坏
    // [[ZH]]   （多边形会变成"里外颠倒"）。交换端点恰好把被翻转的手性扳回来。
    // [[ZH]] 坑：这是「边的方向携带语义」这一约定的直接体现，改动此处会让所有
    // [[ZH]]     布尔运算/DRC 结果错乱。dbEdgeProcessor 单元测试 TEST(135a/135b) 专门
    // [[ZH]]     在全部 8 种 db::Trans 变换下验证旋转/镜像不变性。
    if (t.is_mirror ()) {
      //  NOTE: in case of mirroring transformations we swap p1 and p2. The reasoning is that
      //  this way we maintain the orientation semantics: "right" of the edge is "inside" of
      //  an area.
      *this = edge<C> (t * m_p2, t * m_p1);
    } else {
      *this = edge<C> (t * m_p1, t * m_p2);
    }
    return *this;
  }

  /**
   *  @brief Transform the edge.
   *
   *  Transforms the edge with the given transformation.
   *  Does not modify the edge but returns the transformed edge.
   *  
   *  @param t The transformation to apply.
   *
   *  @return The transformed edge.
   */
  template <class Tr>
  edge<typename Tr::target_coord_type> transformed (const Tr &t) const
  {
    if (t.is_mirror ()) {
      //  NOTE: in case of mirroring transformations we swap p1 and p2. The reasoning is that
      //  this way we maintain the orientation semantics: "right" of the edge is "inside" of
      //  an area.
      return edge<typename Tr::target_coord_type> (t * m_p2, t * m_p1);
    } else {
      return edge<typename Tr::target_coord_type> (t * m_p1, t * m_p2);
    }
  }

  /**
   *  @brief Moves the edge.
   *
   *  Moves the edge by the given offset and returns the 
   *  moved edge. The edge is overwritten.
   *
   *  @param p The distance to move the edge.
   * 
   *  @return The moved edge.
   */
  edge<C> &move (const vector<C> &p)
  {
    m_p1 += p;
    m_p2 += p;
    return *this;
  }

  /**
   *  @brief Enlarges the edge.
   *
   *  Enlarges the edge by the given distance and returns the 
   *  enlarged edge. The edge is overwritten.
   *
   *  @param p The distance to move the edge points.
   * 
   *  @return The enlarged edge.
   */
  edge<C> &enlarge (const vector<C> &p)
  {
    m_p1 -= p;
    m_p2 += p;
    return *this;
  }

  /**
   *  @brief Set the first point.
   */
  void set_p1 (const point<C> &p) 
  {
    m_p1 = p;
  }

  /**
   *  @brief Set the second point.
   */
  void set_p2 (const point<C> &p) 
  {
    m_p2 = p;
  }

  /**
   *  @brief The first point.
   */
  const point<C> &p1 () const
  {
    return m_p1;
  }

  /**
   *  @brief The second point.
   */
  const point<C> &p2 () const
  {
    return m_p2;
  }

  /**
   *  @brief Returns the bounding box
   */
  box_type bbox () const
  {
    return box_type (m_p1, m_p2);
  }

  /**
   *  @brief The horizontal extend of the edge.
   */
  // [[ZH]] 功能：返回方向向量 d = p2 - p1（即边的「位移」）。
  // [[ZH]] 坑：上面这句英文 @brief 写的是 "horizontal extend"，但实现返回的是二维方向向量，
  // [[ZH]]     属于上游的复制粘贴笔误。以实现为准：d() 是向量，dx()/dy() 才是两个分量。
  vector_type d () const
  {
    return vector_type (dx (), dy ());
  }

  /**
   *  @brief The horizontal extend of the edge.
   */
  // [[ZH]] 功能：返回 x 方向分量 dx = p2.x - p1.x。可为负（表示边朝左）。
  // [[ZH]] 坑：EdgeProcessor 用它的符号判断边的走向（dy>0 为上边，见 north_edge/south_edge）。
  C dx () const
  {
    return m_p2.x () - m_p1.x ();
  }

  /**
   *  @brief The vertical extend of the edge.
   */
  // [[ZH]] 功能：返回 y 方向分量 dy = p2.y - p1.y。可为负。
  // [[ZH]] 坑：dy 的符号决定这条边在 scanline 扫描算法中被归入「北侧（向上）」还是「南侧（向下）」，
  // [[ZH]]     是布尔运算正确性的关键输入，见 dbEdgeProcessor.cc 的 north_edge/south_edge。
  C dy () const
  {
    return m_p2.y () - m_p1.y ();
  }

  /**
   *  @brief Shortcut for p1().x()
   */
  C x1 () const
  {
    return m_p1.x ();
  }

  /**
   *  @brief Shortcut for p1().y()
   */
  C y1 () const
  {
    return m_p1.y ();
  }

  /**
   *  @brief Shortcut for p2().x()
   */
  C x2 () const
  {
    return m_p2.x ();
  }

  /**
   *  @brief Shortcut for p2().y()
   */
  C y2 () const
  {
    return m_p2.y ();
  }

  /**
   *  @brief The absolute value of the horizontal extend of the edge.
   *
   *  This function is safe against coordinate overflow for int32
   *  types.
   */
  distance_type dx_abs () const
  {
    return m_p2.x () > m_p1.x () ? m_p2.x () - m_p1.x () : m_p1.x () - m_p2.x ();
  }

  /**
   *  @brief The vertical extend of the edge.
   *
   *  This function is safe against coordinate overflow for int32
   *  types.
   */
  distance_type dy_abs () const
  {
    return m_p2.y () > m_p1.y () ? m_p2.y () - m_p1.y () : m_p1.y () - m_p2.y ();
  }

  /**
   *  @brief Test if the edge is orthogonal (vertical or horizontal)
   */
  bool is_ortho () const
  {
    return m_p1.x () == m_p2.x () || m_p1.y () == m_p2.y ();
  }

  /**
   *  @brief Test for degenerated edge
   */
  bool is_degenerate () const 
  {
    return m_p1 == m_p2;
  }

  /**
   *  @brief The length of the edge
   */
  distance_type length () const
  {
    double ddx (m_p2.x () - m_p1.x ());
    double ddy (m_p2.y () - m_p1.y ());
    return coord_traits::rounded_distance (sqrt (ddx * ddx + ddy * ddy));
  }

  /**
   *  @brief The length of the edge
   */
  double double_length () const
  {
    double ddx (m_p2.x () - m_p1.x ());
    double ddy (m_p2.y () - m_p1.y ());
    return sqrt (ddx * ddx + ddy * ddy);
  }

  /**
   *  @brief The square of the length of the edge
   */
  area_type sq_length () const
  {
    return coord_traits::sq_length (m_p2.x (), m_p2.y (), m_p1.x (), m_p1.y ());
  }

  /**
   *  @brief The square of the length of the edge
   */
  double double_sq_length () const
  {
    double ddx (m_p2.x () - m_p1.x ());
    double ddy (m_p2.y () - m_p1.y ());
    return (ddx * ddx + ddy * ddy);
  }

  /**
   *  @brief The orthogonal length of the edge
   *
   *  @return The orthogonal length (abs(dx)+abs(dy))
   */
  distance_type ortho_length () const
  {
    return dx_abs () + dy_abs ();
  }

  /**
   *  @brief Conversion to a string.
   *
   *  If dbu is set, it determines the factor by which the coordinates are multiplied to render
   *  micron units. In addition, a micron format is chosen for output of these coordinates.
   */
  std::string to_string (double dbu = 0.0) const
  {
    return "(" + m_p1.to_string (dbu) + ";" + m_p2.to_string (dbu) + ")";
  }
  
  /**
   *  @brief Reduce the edge
   *
   *  Reduction of a edge normalizes the edge by extracting
   *  a suitable transformation and placing the edge in a unique
   *  way. In this implementation, p1 is set to zero.
   *
   *  @return The transformation that must be applied to render the original edge
   */
  void reduce (simple_trans<coord_type> &tr)
  {
    point_type d (m_p1);
    move (-d);
    tr = simple_trans<coord_type> (simple_trans<coord_type>::r0, d);
  }

  /**
   *  @brief Reduce the edge
   *
   *  Reduction of a edge normalizes the edge by extracting
   *  a suitable transformation and placing the edge in a unique
   *  way. In this implementation, p1 is set to zero.
   *
   *  @return The transformation that must be applied to render the original edge
   */
  void reduce (disp_trans<coord_type> &tr)
  {
    vector_type d (m_p1);
    move (-d);
    tr = disp_trans<coord_type> (d);
  }

  /**
   *  @brief Reduce the edge
   *
   *  Reduction of a edge normalizes the edge by extracting
   *  a suitable transformation and placing the edge in a unique
   *  way. In this implementation, p1 is set to zero.
   *
   *  @return The transformation that must be applied to render the original edge
   */
  void reduce (unit_trans<coord_type> & /*tr*/)
  {
    //  .. no reduction possible ..
  }

  /**
   *  @brief Test for being parallel
   *
   *  @param e The edge to test against
   *
   *  @return True if both edges are parallel
   */
  bool parallel (const db::edge<C> &e) const
  {
    return coord_traits::vprod_sign (m_p2.x () - m_p1.x (), m_p2.y () - m_p1.y (),
                                     e.m_p2.x () - e.m_p1.x (), e.m_p2.y () - e.m_p1.y (),
                                     0, 0) == 0;
  }

  /**
   *  @brief Test whether a point is on an edge.
   *
   *  A point is on a edge if it is on (at least closer 
   *  than a grid point) the edge.
   *
   *  @param The point to test with the edge.
   *
   *  @return True if the point is on the edge.
   */
  // [[ZH-BEGIN]]
  // 功能：判断点 p 是否「落在边（线段）上」—— 含端点，且带 ±1 DBU 容差。
  //
  // 参数：p 待测点。
  // 返回：true = p 在该线段上（含端点）。
  //
  // ★ 注意这是「模糊」判断，不是精确的共线判断。三个条件必须同时成立：
  //     ① distance_abs(p) < prec_distance()   —— 到直线的距离小于 1 个 DBU 的容差
  //          （prec_distance() 即"网格精度"，整数坐标下为 1）
  //     ② sprod_sign(p, p2, p1) >= 0   —— p 没有"跑出" p1 端点之外
  //     ③ sprod_sign(p, p1, p2) >= 0   —— p 没有"跑出" p2 端点之外
  //   ②③ 用的是标量积（投影）符号，等价于「p 在 p1p2 方向上的投影落在线段范围内」，
  //   它们把①的"在直线上"收紧为"在线段上"。
  //
  // 坑：1) ①②③ 用的是 **< 和 >=** 的组合 → 端点也被接受。
  //        若需要严格排除端点，用 contains_excl()（那里是 > 0）。
  //     2) 因为是容差判断，一个**不在**直线上的点（偏 1 DBU）也可能返回 true。
  //        这与 is_point_on_exact()（dbEdgeProcessor.cc 中的精确版本）形成对比：
  //        布尔运算内部需要严格区分"真共线"和"近共线"，所以那边另写了精确版本。
  //     3) 退化边（p1==p2）退化为精确的点相等比较 m_p1 == p（无容差）。
  // [[ZH-END]]
  bool contains (const db::point<C> &p) const
  {
    if (is_degenerate ()) {
      // [[ZH]] 退化边：只能判断点是否与它重合（这里用精确相等，没有容差）。
      return m_p1 == p;
    } else {
      // [[ZH]] ① 到直线距离在容差内  ②③ 投影落在线段范围内（>= 表示含端点）
      return distance_abs (p) < coord_traits::prec_distance () &&
             coord_traits::sprod_sign (p.x (), p.y (), m_p2.x (), m_p2.y (), m_p1.x (), m_p1.y ()) >= 0 &&
             coord_traits::sprod_sign (p.x (), p.y (), m_p1.x (), m_p1.y (), m_p2.x (), m_p2.y ()) >= 0;
    }
  }

  /**
   *  @brief Test whether a point is on an edge excluding the endpoints.
   *
   *  A point is on a edge if it is on (at least closer 
   *  than a grid point) the edge.
   *
   *  @param The point to test with the edge.
   *
   *  @return True if the point is on the edge but not equal p1 or p2.
   */
  // [[ZH]] 功能：与 contains 相同，但**排除两个端点**（要求 p 严格位于线段内部）。
  // [[ZH]] 实现差异：②③ 的判定由 `>= 0` 改为 `> 0`，即投影必须严格落在
  // [[ZH]]           p1、p2 之间，不接受与端点重合的情形。
  // [[ZH]] 坑：退化边恒返回 false（退化边没有"内部"可言）。
  // [[ZH]]     这个函数常用于「点在边中间」的场景，例如把交点插入到边的切割点列表时，
  // [[ZH]]     需要排除交点恰好等于端点的情况，避免产生零长碎片边。
  bool contains_excl (const db::point<C> &p) const
  {
    if (is_degenerate ()) {
      // [[ZH]] 退化边没有内部，恒不满足"排除端点后仍在边上"。
      return false;
    } else {
      // [[ZH]] 与 contains 唯一的区别：> 0 而非 >= 0，从而排除端点。
      return distance_abs (p) < coord_traits::prec_distance () &&
             coord_traits::sprod_sign (p.x (), p.y (), m_p2.x (), m_p2.y (), m_p1.x (), m_p1.y ()) > 0 &&
             coord_traits::sprod_sign (p.x (), p.y (), m_p1.x (), m_p1.y (), m_p2.x (), m_p2.y ()) > 0;
    }
  }

  /**
   *  @brief Coincidence check.
   *
   *  Checks whether a edge is coincident with another edge. 
   *  Coincidence is defined by being parallel, oriented the same way and that 
   *  both edges share more than one point.
   *
   *  @param e the edge to test with
   *
   *  @return True if the edges are coincident.
   */
  // [[ZH-BEGIN]]
  // 功能：判断两条边是否「重合（共线且重叠）」。
  //       按英文文档的定义，重合需同时满足：
  //         ① 平行且方向一致（同向，不是反向）
  //         ② 共享不止一个点（即真正重叠，而非仅端点相接）
  //
  // 参数：e 另一条边。
  // 返回：true = 重合。
  //
  // 实现要点（比看上去微妙）：
  //   用 distance_abs 做 ±1 DBU 容差判断（不是精确共线测试），
  //   然后用 sprod_sign(*this, e) 判断两边的方向关系：
  //     · 同向  → 直接检查 e.p1 落在 p1p2 内部 且 e.p2 也落在 p1p2 内部
  //     · 反向  → 因为 e 的端点顺序逆着，需改用 e.p2 对 p1p2 的判断
  //               （源码里的三元表达式正是在做这个补偿）
  //   注意判断用的是"严格落在内部"，所以**仅端点相接不算重合**。
  //
  // 坑：1) 方向是判定的一部分：a.coincident(b) 与 b.coincident(a) 结果一致，
  //        但 a.coincident(b.swapped_points()) 为 false（反向不算重合）。
  //        若你只关心"是否共线重叠"而与方向无关，需要自己处理两种朝向。
  //     2) 退化边（任一方）直接返回 false。
  //     3) 容差 ±1 DBU 意味着数值上"几乎共线"的边也可能被判为重合 ——
  //        这对布尔运算是必要的（要把近共线边当作重合边消解），
  //        但也意味着它不是严格的几何判定。
  // [[ZH-END]]
  bool coincident (const db::edge<C> &e) const
  {
    return ! is_degenerate () && ! e.is_degenerate () &&
           distance_abs (e.p1 ()) < coord_traits::prec_distance () && 
           distance_abs (e.p2 ()) < coord_traits::prec_distance () && 
           (sprod_sign (*this, e) < 0 ? 
            (coord_traits::sprod_sign (e.p2 ().x (), e.p2 ().y (), p1 ().x (), p1 ().y (), p2 ().x (), p2 ().y ()) > 0 && 
             coord_traits::sprod_sign (e.p1 ().x (), e.p1 ().y (), p2 ().x (), p2 ().y (), p1 ().x (), p1 ().y ()) > 0) :
            (coord_traits::sprod_sign (e.p1 ().x (), e.p1 ().y (), p1 ().x (), p1 ().y (), p2 ().x (), p2 ().y ()) > 0 && 
             coord_traits::sprod_sign (e.p2 ().x (), e.p2 ().y (), p2 ().x (), p2 ().y (), p1 ().x (), p1 ().y ()) > 0));
  }

  /**
   *  @brief Intersection test. 
   *
   *  Returns true if the edges intersect. 
   *  If the edges coincide, they also intersect.
   *  For degenerated edges, the intersection is mapped to
   *  point containment tests.
   *
   *  @param e The edge to test.
   */
  // [[ZH-BEGIN]]
  // 功能：真正的「线段相交」测试 —— 两条边（作为有限线段）是否有公共点。
  //       共线的两条边若重叠，也算相交。
  //       退化边（长度为 0 的点）按其点包含关系处理。
  //
  // 参数：e 另一条边。
  // 返回：true = 两条线段有公共点。
  //
  // 实现：一条精心设计的「快速排除」阶梯，从便宜到昂贵依次判断：
  //       ① 自分退化   → 退化为「点是否在 e 上」
  //       ② e 退化     → 退化为「e 的点是否在自身上」
  //       ③ 包围盒不相接 → 直接 false（最常见的快速失败路径，成本极低）
  //       ④ 两条都是水平/垂直正交边 → 包围盒相接即相交，直接 true（免做叉积）
  //       ⑤ 一般情形   → 互相跨越（双向 crossed_by）才算相交
  //       这个阶梯是性能关键：版图数据绝大多数是曼哈顿图形，第 ④ 步能省掉大量叉积运算。
  //
  // 坑：1) 第 ④ 步对共线但不重叠的两条正交边也可能返回 true（因为只看包围盒），
  //        对布尔运算是可接受的近似，但若你需要严格判定请用 intersect_point。
  //     2) 判定用的都是精确整数叉积（vprod_sign），不含浮点误差。
  // [[ZH-END]]
  bool intersect (const db::edge<C> &e) const
  {
    if (is_degenerate ()) {
      // [[ZH]] ① 自己是点：退化为点包含测试。
      return e.contains (p1 ());
    } else if (e.is_degenerate ()) {
      // [[ZH]] ② 对方是点：对称处理。
      return contains (e.p1 ());
    } else if (! box_type (p1 (), p2 ()).touches (box_type (e.p1 (), e.p2 ()))) {
      // [[ZH]] ③ 包围盒都不相接，不可能相交 —— 最快且最常命中的排除分支。
      return false;
    } else if (is_ortho () && e.is_ortho ()) {
      // [[ZH]] ④ 两条正交（水平或垂直）边且包围盒相接 → 必相交。
      // [[ZH]]    这是曼哈顿版图的最常见情形，跳过叉积计算。
      return true;
    } else {
      // [[ZH]] ⑤ 一般情形：双向跨越才是真相交（单向只说明直线穿越）。
      return crossed_by (e) && e.crossed_by (*this);
    }
  }

  /**
   *  @brief Intersection test with intersect point. 
   *
   *  Returns true if the edges intersect and returns the
   *  intersection point if true. For coinciding edges one
   *  of the points that coincide is returned.
   *
   *  @param e The edge to test.
   *
   *  @return A pair <bool,point> with true as the first element
   *  if the edges intersect and the intersection point as the second.
   *  If the edges do not intersect, returns <false,undef.>.
   */
  // [[ZH-BEGIN]]
  // 功能：求两条边（作为有限线段）的交点。相交则返回 {true, 交点坐标}，
  //       不相交返回 {false, (0,0)}（注意：失败时返回的是 (0,0) 而非"未定义值"，
  //       调用方必须靠 bool 分量判断，不能靠坐标判断）。
  //
  // 参数：e 另一条边。
  // 返回：std::pair<bool, db::point<C>>，见上。
  //
  // ★ 为什么这个函数对 EdgeProcessor 至关重要：
  //   EdgeProcessor 求布尔运算时要先把所有相交的边在交点处「打断」(cut)，
  //   再按 scanline 重新排序、成对消解。若两次调用同一对边得到不同的交点坐标，
  //   排序就会不稳定，进而导致结果多边形拓扑错误。
  //   因此本库约定：交点坐标必须是「精确、可复现」的整数结果。
  //   dbEdgeProcessor.cc 里为此专门包了一层 safe_intersect_point() 来消除
  //   「参数顺序不同 → 结果不同」的不对称性（见该文件注释）。
  //
  // 实现：与 intersect 相同的快速阶梯，但多了交点计算：
  //       ① 自分退化 / ② e 退化 → 用 contains 判断，返回已存在的端点
  //       ③ 包围盒不相接 → false
  //       ④ 两条正交边 → 交点就是两个包围盒的交集矩形的左下角，直接取 max/min
  //       ⑤ 一般情形 ↓
  //          a. 若 e 的某个端点恰好落在自身上（ends_on_edge），直接返回该端点，
  //             避免用除法算出理论上相同但舍入后可能不同的坐标。
  //          b. 否则用叉积比值参数化求交：
  //                x = p1.x + dx * vxa / (vxa + vxb)
  //                y = p1.y + dy * vxa / (vxa + vxb)
  //             其中 vxa、vxb 是 e 的端点到直线 *this 的有向面积（已取绝对值）。
  //             关键：除法用 div_exact()，保证 .5 的舍入方向固定、结果可复现。
  //
  // 坑：1) 共线（coincident）时"交点"不唯一，这里返回其中一个重合端点，属约定行为。
  //     2) 若你需要的是「两条无限长直线的交点」，用 cut_point()，不要用本函数。
  // [[ZH-END]]
  std::pair <bool, db::point<C> > intersect_point (const db::edge<C> &e) const
  {
    if (is_degenerate ()) {
      if (e.contains (p1 ())) {
        return std::make_pair (true, p1 ());
      } else {
        return std::make_pair (false, db::point<C> (0, 0));
      }
    } else if (e.is_degenerate ()) {
      if (contains (e.p1 ())) {
        return std::make_pair (true, e.p1 ());
      } else {
        return std::make_pair (false, db::point<C> (0, 0));
      }
    } else if (! box_type (p1 (), p2 ()).touches (box_type (e.p1 (), e.p2 ()))) {
      return std::make_pair (false, db::point<C> (0, 0));
    } else if (is_ortho () && e.is_ortho ()) {
      coord_type x = std::max (std::min (p1 ().x (), p2 ().x ()), std::min (e.p1 ().x (), e.p2 ().x ()));
      coord_type y = std::max (std::min (p1 ().y (), p2 ().y ()), std::min (e.p1 ().y (), e.p2 ().y ()));
      return std::make_pair (true, db::point<C> (x, y));
    } else if (! crossed_by (e)) {
      return std::make_pair (false, db::point<C> (0, 0));
    } else {

      bool res = true;
      bool ends_on_edge = false;

      std::pair<area_type, int> vsa = coord_traits::vprod_with_sign (e.p2 ().x (), e.p2 ().y (), m_p1.x (), m_p1.y (), e.p1 ().x (), e.p1 ().y ());
      area_type vxa = vsa.first;
      if (vsa.second < 0) {
        res = false;
      } else if (vsa.second == 0) {
        ends_on_edge = true;
      }

      std::pair<area_type, int> vsb = coord_traits::vprod_with_sign (e.p2 ().x (), e.p2 ().y (), m_p2.x (), m_p2.y (), e.p1 ().x (), e.p1 ().y ());
      area_type vxb = -vsb.first;
      if (vsb.second > 0) {
        res = !res;
      } else if (vsb.second == 0) {
        ends_on_edge = true;
      }

      if (ends_on_edge) {

        if (contains (e.p1 ())) {
          return std::make_pair (true, e.p1 ());
        } else if (contains (e.p2 ())) {
          return std::make_pair (true, e.p2 ());
        } else if (e.contains (p1 ())) {
          return std::make_pair (true, p1 ());
        } else if (e.contains (p2 ())) {
          return std::make_pair (true, p2 ());
        } else {
          return std::make_pair (false, db::point<C> (0, 0));
        }

      } else if (res) {

        if (vxa < 0) {
          vxa = -vxa;
        }
        if (vxb < 0) {
          vxb = -vxb;
        }

        coord_type x = m_p1.x () + div_exact (dx (), vxa, vxa + vxb);
        coord_type y = m_p1.y () + div_exact (dy (), vxa, vxa + vxb);

        return std::make_pair (true, db::point<C> (x, y));

      } else {
        return std::make_pair (false, db::point<C> (0, 0));
      }

    }
  }

  /**
   *  @brief Distance between the edge and a point.
   *
   *  Returns the distance between the edge and the point. The 
   *  distance is signed which is negative if the point is to the
   *  "right" of the edge and positive if the point is to the "left".
   *  The distance is measured by projecting the point onto the
   *  line through the edge. If the edge is degenerated, the distance
   *  is not defined.
   *
   *  The distance is through as a distance of the point from the line
   *  through the edge.
   *
   *  @param p The point to test.
   *
   *  @return The distance
   */
  // [[ZH-BEGIN]]
  // 功能：点到「边所在直线」的带符号距离。
  //       符号与 side_of 一致：点在左侧为正，右侧为负。
  //       距离值 = 叉积 / 边长 = ((p2-p1) × (p-p1)) / |p2-p1|。
  //
  // 参数：p 待测点。
  // 返回：coord_type（整数）—— 已按坐标类型舍入回网格。
  //
  // 坑：1) 测的是到「无限长直线」的距离，不是到「线段」的距离。
  //        若需要到线段的最近距离，用 euclidian_distance()。
  //     2) 退化边（p1==p2）直接返回 0，结果无意义，调用方需自行排除。
  //     3) 内部用 double 做除法再 rounded() 回整数，因此不是精确有理数运算；
  //        对精度敏感的场景请用 side_of()（纯整数精确）。
  // [[ZH-END]]
  coord_type distance (const db::point<C> &p) const
  {
    //  the distance is computed from 
    //    d = (a x b) / |a|
    //  where b = p - p1, a = p2 - p1
    if (is_degenerate ()) {
      //  for safety handle this case - without a reasonable result
      // [[ZH]] 退化边无方向，无法定义带符号距离，按约定返回 0。
      return 0;
    } else {
      //  compute the distance as described above 
      // [[ZH]] axb = 叉积 (p2-p1) × (p-p1)，即平行四边形面积的两倍，其符号即左右侧。
      area_type axb = coord_traits::vprod (m_p2.x (), m_p2.y (), p.x (), p.y (), m_p1.x (), m_p1.y ()); 
      // [[ZH]] 除以边长得到「高」，即垂距。用 double 做除法。
      double d = double (axb) / double_length ();
      //  and round
      // [[ZH]] 舍入回坐标类型，避免浮点值泄漏到整数几何体系。
      return coord_traits::rounded (d);
    }
  }

  /**
   *  @brief Gets the distance of the point from the edge.
   *
   *  The distance is computed as the minimum distance of the point to any of the edge's
   *  points.
   *
   *  @param p The point whose distance is to be computed
   *
   *  @return The distance
   */
  distance_type euclidian_distance (const db::point<C> &p)
  {
    if (db::sprod_sign (p - p1 (), d ()) < 0) {
      return p1 ().distance (p);
    } else if (db::sprod_sign (p - p2 (), d ()) > 0) {
      return p2 ().distance (p);
    } else {
      return std::abs (distance (p));
    }
  }

  /**
   *  @brief Side of the point
   *
   *  Returns 1 if the point is "left" of the edge, 0 if on
   *  and -1 if the point is "right" of the edge.
   *
   *  @param p The point to test.
   *
   *  @return The side value
   */
  // [[ZH-BEGIN]]
  // 功能：判断点 p 位于本边的哪一侧 —— 返回 1 表示「左侧」，0 表示「在边上」，
  //       -1 表示「右侧」。
  //
  //       ★ 这是全库最核心的谓词。结合「边的右侧 == 多边形内部」这一约定可知：
  //            side_of(p) == -1  →  p 在多边形内部
  //            side_of(p) ==  0  →  p 在轮廓上
  //            side_of(p) == +1  →  p 在多边形外部
  //       因此「点在多边形内」的判断，本质上就是看该点在所有轮廓边的同侧。
  //
  // 参数：p 待测点。
  // 返回：+1 / 0 / -1，见上。
  //
  // 实现：取叉积 (p2-p1) × (p-p1) 的符号，即三角形 (p1, p2, p) 有向面积的符号。
  //       符号为正 = p 在前进方向左侧（逆时针）。数学上就是 vprod_sign。
  //
  // 坑：1) 对退化边（p1==p2）直接返回 0，此时结果无几何意义 —— 调用方需自行排除退化边。
  //     2) 这里用的是「精确」叉积符号（coord_traits::vprod_sign），不是浮点近似，
  //        因此对整数坐标可给出严格一致的结果，这是布尔运算可复现的基础。
  // [[ZH-END]]
  int side_of (const db::point<C> &p) const
  {
    //  the distance is computed from 
    //    d = (a x b) / sqrt (a * a)
    //  where b = p - p1, a = p2 - p1
    // [[ZH]] 修改：无。纯查询函数，不改变任何成员。
    if (is_degenerate ()) {
      //  for safety handle this case - without a reasonable result
      // [[ZH]] 退化边没有方向，无法定义左右，按约定返回 0（视为"在边上"）。
      return 0;
    } else {
      //  compute the side as the sign of the distance as in "distance"
      // [[ZH]] vprod_sign(p2, p, p1) = 叉积 (p2-p1) × (p-p1) 的符号。
      // [[ZH]] 正 → p 在左侧；负 → 在右侧；零 → 共线（在直线上，但不一定在线段内）。
      return coord_traits::vprod_sign (m_p2.x (), m_p2.y (), p.x (), p.y (), m_p1.x (), m_p1.y ());
    }
  }

  /**
   *  @brief Absolute distance between the edge and a point.
   *
   *  Returns the distance between the edge and the point. 
   *
   *  @param p The point to test.
   *
   *  @return The distance
   */
  distance_type distance_abs (const db::point<C> &p) const
  {
    //  the distance is computed from 
    //    d = (a x b) / |a|
    //  where b = p - p1, a = p2 - p1
    if (is_degenerate ()) {
      //  for safety handle this case - without a reasonable result
      return 0;
    } else {
      //  compute the distance as described above 
      area_type axb = coord_traits::vprod (m_p2.x (), m_p2.y (), p.x (), p.y (), m_p1.x (), m_p1.y ()); 
      double d = fabs (double (axb)) / double_length ();
      //  and round
      return coord_traits::rounded_distance (d);
    }
  }

  /**
   *  @brief Swaps the points of the edge
   */
  // [[ZH-BEGIN]]
  // 功能：就地把 p1 与 p2 互换，即**翻转边的方向**。
  //
  // 参数：无。
  // 返回：*this（引用），便于链式调用，如 e.swap_points().swap_points()。
  //
  // ★ 为什么这个平凡的操作值得单独注解：
  //   结合「边的右侧 = 内部」这一约定，翻转方向会把内/外语义对调。
  //   因此 swap_points() 不是无害的"规范化"，而是一个**语义变更操作**。
  //   库中大量地方用它来做规范化（例如 edge_xaty 里统一成 dy>0），
  //   此时前提是「后续只做对称的计算，不依赖内外侧」。
  //   若你在依赖 side_of / distance 符号的代码里随手 swap，会导致内外判断反转。
  //
  // 相关：带镜像的变换会自动交换端点，见 transform()/transformed() 的注释。
  // [[ZH-END]]
  edge<C> &swap_points () 
  {
    // [[ZH]] 交换两个端点，其余无状态可维护（edge 只有这两个成员）。
    std::swap (m_p1, m_p2);
    return *this;
  }

  /**
   *  @brief Returns the edge with swapped points
   */
  // [[ZH]] 功能：返回一个方向相反的新边（= 先拷贝再 swap_points）。
  // [[ZH]] 与 swap_points() 的区别：本函数**不修改**原对象，是 const 操作。
  // [[ZH]] 用途：需要在表达式里临时获得反向边时使用，避免为交换而拷贝一份可变副本。
  edge<C> swapped_points () const
  {
    // [[ZH]] 先复制，再对副本交换 —— 因此原边 e 不受影响。
    edge<C> e = *this;
    e.swap_points ();
    return e;
  }

  /**
   *  @brief Clip the line given by the edge at the given box
   *
   *  Determines the part of the line (given by the edge) that runs through the given box.
   *  If the line does not hit the box, false is returned in the first member of the 
   *  return value.
   *
   *  @return first: false if the line does not hit the box, second: the part of the line inside the box
   */
  std::pair<bool, edge> clipped_line (const db::box<C> &box) const
  {
    if (box.empty ()) {
      return std::make_pair (false, db::edge<C> ());
    }

    std::pair <bool, db::point<C> > pc1 (false, db::point<C> ()), pc2 (false, db::point<C> ());

    pc1 = cut_point (db::edge<C> (box.p1 (), db::point<C> (box.p1 ().x (), box.p2 ().y ())));
    if (pc1.first) {
      pc2 = cut_point (db::edge<C> (db::point<C> (box.p2 ().x (), box.p1 ().y ()), box.p2 ()));
    }

    if (! pc1.first || ! pc2.first) {
      pc1 = cut_point (db::edge<C> (box.p1 (), db::point<C> (box.p2 ().x (), box.p1 ().y ())));
      if (pc1.first) {
        pc2 = cut_point (db::edge<C> (db::point<C> (box.p1 ().x (), box.p2 ().y ()), box.p2 ()));
      }
    }

    if (pc1.first && pc2.first) {
      return db::edge<C> (pc1.second, pc2.second).clipped (box);
    } else {
      return std::make_pair (false, db::edge<C> ());
    }
  }

  /**
   *  @brief Clip at rectangle
   *
   *  Clips the edge at the box provided. Maintains the orientation of the
   *  edge.
   *
   *  @return first: false if edge disappears. second: the clipped edge
   */
  std::pair<bool, edge> clipped (const db::box<C> &box) const
  {
    if (box.empty ()) {
      return std::make_pair (false, db::edge<C> ());
    }

    bool swapped = false;

    db::point<C> p1 (m_p1), p2 (m_p2);

    if (p1.x () > p2.x ()) {
      std::swap (p1, p2);
      swapped = !swapped;
    }

    if (p2.x () < box.left ()) {
      return std::make_pair (false, db::edge<C>());
    } else if (p1.x () < box.left ()) {
      p1 = db::point<C> (box.left (), m_p1.y () + db::coord_traits<C>::rounded((double)(box.left () - m_p1.x ()) * double (dy ()) / double (dx ())));
    }
    if (p1.x () > box.right ()) {
      return std::make_pair (false, db::edge<C> ());
    } else if (p2.x () > box.right ()) {
      p2 = db::point<C> (box.right (), m_p1.y () + db::coord_traits<C>::rounded((double)(box.right () - m_p1.x ()) * double (dy ()) / double (dx ())));
    }
    
    if (p1.y () > p2.y ()) {
      std::swap (p1, p2);
      swapped = !swapped;
    }

    if (p2.y () < box.bottom ()) {
      return std::make_pair (false, db::edge<C> ());
    } else if (p1.y () < box.bottom ()) {
      p1 = db::point<C> (std::max (box.left (), std::min (box.right (), m_p1.x () + db::coord_traits<C>::rounded((double)(box.bottom () - m_p1.y ()) * double (dx ()) / double (dy ())))), box.bottom ());
    }
    if (p1.y () > box.top ()) {
      return std::make_pair (false, db::edge<C> ());
    } else if (p2.y () > box.top ()) {
      p2 = db::point<C> (std::max (box.left (), std::min (box.right (), m_p1.x () + db::coord_traits<C>::rounded((double)(box.top () - m_p1.y ()) * double (dx ()) / double (dy ())))), box.top ());
    }

    if (swapped) {
      return std::make_pair (true, db::edge<C> (p2, p1));
    } else {
      return std::make_pair (true, db::edge<C> (p1, p2));
    }
  }

  /** 
   *  @brief Check, if an edge is cut by a line (given by an edge)
   *
   *  This method returns true if p1 is in one semispace 
   *  while p2 is in the other or one of them is on the line
   *  through the edge
   */
  // [[ZH-BEGIN]]
  // 功能：判断「穿过」。以 *this 所在直线为界，检查 e 的两个端点是否分居两侧
  //       （或至少有一个恰好落在该直线上 → 也算 true）。
  //
  //       ★ 关键理解：它测的是 e 相对 *this 这条「直线」的位置，而不是两条线段是否相交。
  //         *this 被当作无限长直线使用，其自身端点不参与判断。
  //
  // 参数：e 被测试的那条边（它的两个端点才是被测对象）。
  // 返回：true = e 跨越了 *this 所在直线；false = e 完全在直线一侧。
  //
  // ★ 与 intersect 的关系（这是理解库内相交判断的关键）：
  //       intersect(e) == crossed_by(e) && e.crossed_by(*this)
  //   即「互相跨越」才是真正的线段相交。单方向的 crossed_by 只说明射线关系。
  //
  // 实现：下面注释里给出的等价伪码 —— 用叉积符号（side）判断两个端点是否异号，
  //       遇到符号为 0（点在直线上）立即返回 true。
  //
  // 坑：对退化边（p1==p2）没有显式保护，结果取决于 vprod_sign 的行为，调用方应先排除。
  // [[ZH-END]]
  bool crossed_by (const db::edge <C> &e) const
  {
    //  this is basically this algorithm:
    //  res = true;
    //  if (side_of (e.p1 ()) < 0) {
    //    res = false
    //  } else if (side_of (e.p1 ()) == 0) {
    //    return true;
    //  } 
    //  if (side_of (e.p2 ()) > 0) {
    //    res = !res
    //  } else if (side_of (e.p2 ()) == 0) {
    //    return true;
    //  }
    //  return res;

    // [[ZH]] res 的初值 true 表示「e.p1 在直线左侧（含），且 e.p2 也在左侧」这一默认情形。
    // [[ZH]] 下面两步分别修正 e.p1、e.p2 的位置，最终 res 表示「两端点分居两侧」。
    bool res = true;

    // [[ZH]] vsa = e.p1() 相对 *this 所在直线的叉积符号（即 side_of(e.p1())）。
    int vsa = coord_traits::vprod_sign (m_p2.x (), m_p2.y (), e.p1 ().x (), e.p1 ().y (), m_p1.x (), m_p1.y ());
    if (vsa < 0) {
      // [[ZH]] e.p1 在右侧 → 翻转判定基准。
      res = false;
    } else if (vsa == 0) {
      // [[ZH]] e.p1 恰在直线上 → 视为「穿过」，提前返回。
      return true;
    }

    // [[ZH]] vsb = e.p2() 相对同一直线的叉积符号。
    int vsb = coord_traits::vprod_sign (m_p2.x (), m_p2.y (), e.p2 ().x (), e.p2 ().y (), m_p1.x (), m_p1.y ());
    if (vsb > 0) {
      // [[ZH]] e.p2 在左侧，与 e.p1（此时必在右侧）异号 → 确实跨越。
      // [[ZH]] 这里用 res = !res 而非直接赋值，是为了兼容两端点同为 0 等边界情形。
      res = !res;
    } else if (vsb == 0) {
      // [[ZH]] e.p2 恰在直线上 → 同样视为「穿过」。
      return true;
    }

    // [[ZH]] 返回「两端点是否分居直线两侧」。
    return res;
  }

  /** 
   *  @brief Check, if an edge is cut by a line (given by an edge)
   *
   *  This method returns true if p1 is in one semispace 
   *  while p2 is in the other or one of them is on the line
   *  through the edge. In addition to "crossed_by", also returns
   *  the point at which the edge is crossed.
   */
  std::pair <bool, db::point<C> > crossed_by_point (const db::edge <C> &e) const
  {
    bool res = true;

    std::pair<area_type, int> vsa = coord_traits::vprod_with_sign (m_p2.x (), m_p2.y (), e.p1 ().x (), e.p1 ().y (), m_p1.x (), m_p1.y ());
    area_type vxa = vsa.first;
    if (vsa.second < 0) {
      res = false;
    } else if (vsa.second == 0) {
      return std::make_pair (true, e.p1 ());
    }

    std::pair<area_type, int> vsb = coord_traits::vprod_with_sign (m_p2.x (), m_p2.y (), e.p2 ().x (), e.p2 ().y (), m_p1.x (), m_p1.y ());
    area_type vxb = -vsb.first;
    if (vsb.second > 0) {
      res = !res;
    } else if (vsb.second == 0) {
      return std::make_pair (true, e.p2 ());
    }

    if (res) {

      if (vxa < 0) {
        vxa = -vxa;
      }
      if (vxb < 0) {
        vxb = -vxb;
      }

      coord_type x = e.p1 ().x () + div_exact (e.dx (), vxa, vxa + vxb);
      coord_type y = e.p1 ().y () + div_exact (e.dy (), vxa, vxa + vxb);

      return std::make_pair (true, db::point<C> (x, y));

    } else {
      return std::make_pair (false, db::point<C> (0, 0));
    }
  }

  /** 
   *  @brief Compute the projection of a point on the edge
   *
   *  This method returns true in the first member of the return
   *  value if the point can be projected on the edge. In this case,
   *  the second member will return the point projected on the edge.
   *
   *  @param point The point to be projected
   */
  // [[ZH-BEGIN]]
  // 功能：把点 pt 垂直投影到**线段** *this 上。
  //       返回 {true, 垂足} —— 仅当垂足确实落在线段范围内（含两端）；
  //       若垂足落在线段之外（即在延长线上），返回 {false, (0,0)}。
  //
  // 参数：pt 待投影的点。
  // 返回：{bool 是否投影成功, point 垂足坐标}。
  //
  // ★ 实现思路很巧妙，值得一看：
  //   它不做点积/除法，而是**构造一条过 pt 且垂直于 *this 的临时边**，
  //   再求这条临时边与 *this 的交点 —— 那个交点就是垂足。
  //       临时边 = (pt, pt + (dy, -dx))
  //   注意 (dy, -dx) 正是把方向向量 (dx, dy) 逆时针旋转 90° 得到的，
  //   因此这条临时边垂直于 *this，且必然过 pt。
  //   然后调用 crossed_by_point 直接拿到交点（并复用其相交判定）。
  //   整个过程中除 crossed_by_point 内部的交比计算外没有额外的开方/三角函数。
  //
  // 坑：1) 返回 false 时坐标是 (0,0) 而非"未定义"，必须靠 bool 判断。
  //     2) 退化边（*this 长度为 0）时，(dy,-dx) == (0,0)，临时边退化为一个点，
  //        行为由 crossed_by_point 的退化处理决定，结果无几何意义。
  //     3) 这里投影到的是**线段**（有端点限制）；若要投影到无限长直线，
  //        应使用 cut_point 或自行放宽判定。
  // [[ZH-END]]
  std::pair <bool, db::point<C> > projected (const db::point<C> &pt) const
  {
    // [[ZH]] 构造过 pt、垂直于 *this 的临时边，再求它与 *this 的交点 = 垂足。
    // [[ZH]] (dy(), -dx()) 是方向向量逆时针转 90°，故临时边 ⊥ *this。
    return db::edge <C> (pt, pt + db::vector<C> (dy (), -dx ())).crossed_by_point (*this);
  }

  /**
   *  @brief Compute cut point of two lines (given by edges)
   *
   *  This method returns true in the first member if both edges cut.
   *  If this is the case, the second member of the returned pair is
   *  the point at which the edges would intersect, given they are 
   *  extended beyond their ends.
   */
  // [[ZH-BEGIN]]
  // 功能：求「两条边所在无限长直线」的交点，**不考虑端点限制**。
  //
  // 参数：e2 另一条边。
  // 返回：{true, 交点}；仅当两条直线平行时返回 {false, (0,0)}。
  //       ★ 即使交点远在线段之外也返回 true。
  //
  // ★ 与 intersect_point 的区别（最容易混淆的一对，务必分清）：
  //       intersect_point  把两边当**有限线段**：只在线段真正相交时给交点。
  //       cut_point        把两边当**无限直线**：交点在延长线上也给。
  //   EdgeProcessor 的相交处理阶段需要的正是后者（先判断延长后是否相交）。
  //
  // 实现：叉积参数化。设
  //       vps = e2 的方向 × *this 的方向   （有符号面积，其符号即 vps.second）
  //       pr1 = e2.p1 到直线 *this 的有向面积
  //       pr2 = vps.first（叉积的绝对值部分）
  //   则交点 = e2.p1 - e2方向 * (pr1 / pr2)
  //   直观理解：从 e2.p1 出发沿 e2 方向外推，外推比例由"离目标直线还差多少面积"决定，
  //   推到面积归零时正好落在 *this 所在直线上。
  //
  // 坑：1) pr1/pr2 是 **double 除法**，再乘方向向量构造整数 point ——
  //        因此结果会经过舍入，不是精确有理交点。
  //        需要**精确可复现**的交点时必须改用 intersect_point（内部用 div_exact）。
  //     2) 平行（vps.second == 0）返回 (0,0)，必须靠 bool 分量判断，
  //        不能因为坐标是 (0,0) 就当作"交点在原点"。
  //     3) 共线（平行且重叠）也走平行分支 → 返回 false，尽管它们有无穷多交点。
  // [[ZH-END]]
  std::pair <bool, db::point<C> > cut_point (const db::edge<C> &e2) const
  {
    // [[ZH]] vps.first = 两边方向向量的叉积（有符号面积）；vps.second = 其符号。
    // [[ZH]] 符号为 0 即平行（或退化），此时无唯一交点。
    std::pair<typename coord_traits::area_type, int> vps = coord_traits::vprod_with_sign (e2.dx (), e2.dy (), this->dx (), this->dy (), 0, 0);
    if (vps.second != 0) {
      // [[ZH]] pr1 = e2.p1 相对直线 *this 的有向面积（用叉积算，无需开方）。
      double pr1 = double (coord_traits::vprod (e2.p1 ().x (), e2.p1 ().y (), this->p2 ().x (), this->p2 ().y (), this->p1 ().x (), this->p1 ().y ()));
      // [[ZH]] pr2 = 叉积绝对值，作为分母把面积比换算成沿 e2 方向的位移比例。
      double pr2 = double (vps.first);
      // [[ZH]] 沿 e2 方向外推 pr1/pr2 比例，落到 *this 所在直线上 → 交点。
      // [[ZH]] 注意 vector * double 会把结果舍入成整数坐标(C)。
      db::point<C> p = e2.p1 () - db::vector<C> ((e2.p2 () - e2.p1 ()) * (pr1 / pr2));
      return std::make_pair (true, p);
    } else {
      // [[ZH]] 平行/退化：无唯一交点，返回 false（坐标占位为 (0,0)）。
      return std::make_pair (false, db::point<C> (0, 0));
    }
  }

private:
  // [[ZH-BEGIN]]
  // 这是 edge 的**全部**数据成员 —— 只有两个端点，别处没有隐藏状态。
  //
  // m_p1 : 边的起点（"from"）。★ 方向语义的承载者之一。
  // m_p2 : 边的终点（"to"）。  方向为 p1 → p2。
  //
  // 重要含义：
  //   1) edge 是**纯值类型**（value type）：两个 point 就是全部状态，
  //      sizeof(edge<Coord>) == 8 字节。拷贝、比较、放进 vector 都非常廉价，
  //      所以库中大量函数按值传递 edge（例如 edge_xaty(db::edge<C> e, C y)）。
  //   2) 因为不含索引/句柄，「翻转方向」只能通过交换 p1 与 p2 实现
  //      （见 swap_points / swapped_points）。
  //   3) 没有缓存字段：dx()/dy()/length() 等每次调用都重新计算。
  //      这保证了「改了 m_p1/m_p2 之后不可能出现陈旧缓存」这一安全性，
  //      代价是重复计算（编译器通常能内联优化掉）。
  //   4) 默认构造（edge()）把两点都置为 (0,0)，得到一个**退化边**，
  //      其 is_degenerate() == true，此时方向/左右侧都无意义。
  // [[ZH-END]]
  point<C> m_p1, m_p2;
};

/**
 *  @brief "intersect" binary predicate
 */
template <class Edge>
struct edges_intersect
{
  bool operator() (const Edge &e1, const Edge &e2) const
  {
    return e1.intersect (e2);
  }
};

/**
 *  @brief Scaling of an edge
 *
 *  @param e The edge to scale.
 *  @param s The scaling factor
 *
 *  @return The scaled edge
 */ 
template <class C>
inline edge<db::DCoord>
operator* (const edge<C> &e, double s)
{
  return edge<db::DCoord> (e.p1 () * s, e.p2 () * s);
}

/**
 *  @brief Binary * operator (transformation)
 *
 *  Transforms the edge with the given transformation and 
 *  returns the result.
 *
 *  @param t The transformation to apply
 *  @param e The edge to transform
 *  @return t * e
 */
template <class Tr>
inline edge<typename Tr::target_coord_type> 
operator* (const Tr &t, const edge<typename Tr::coord_type> &e)
{
  return e.transformed (t);
}

/**
 *  @brief Output stream insertion operator
 */
template <class C>
inline std::ostream &
operator<< (std::ostream &os, const edge<C> &e)
{
  return (os << e.to_string ());
}

/**
 *  @brief The standard edge typedef
 */
// [[ZH-BEGIN]]
// ★ 新人最需要先看清的一件事：db::Edge 不是一个独立的类，它只是 edge<Coord> 的别名。
//
//   db::Edge  = db::edge<db::Coord>   → 坐标类型 int32_t，整数网格坐标。
//                                        数据库存储、布尔运算、DRC/LVS 全部用它。
//   db::DEdge = db::edge<db::DCoord>  → 坐标类型 double，浮点坐标。
//                                        用于需要亚网格精度的几何计算（如近似圆、加密曲线）；
//                                        转换到整数时会按数据库精度(DBU)舍入。
//
// 两者**接口完全一致**（同一个模板），只是数值类型不同。
// 因此本文件里所有 `edge<C>` 的说明，对 Edge 和 DEdge 同样适用。
//
// 坑：DBU (database unit) 是版图坐标的最小单位（通常 1nm）。
//     db::Coord 表示的就是「多少个 DBU」的整数，不是微米。换算见 db::Layout::dbu()。
// [[ZH-END]]
typedef edge<db::Coord>  Edge;

/**
 *  @brief The double coordinate edge typedef
 */
// [[ZH]] 功能：浮点坐标版本的边，见上面对 db::Edge 的说明。
typedef edge<db::DCoord> DEdge;

/**
 *  @brief Convenience wrappers for coord_traits functions: vector product: p x q
 */
template <class C>
typename db::coord_traits<C>::area_type vprod (const db::edge<C> &p, const db::edge<C> &q)
{
  return db::coord_traits<C>::vprod (p.dx (), p.dy (), q.dx (), q.dy (), 0, 0);
}

/**
 *  @brief Convenience wrappers for coord_traits functions: vector product sign: sign(p x q)
 */
template <class C>
int vprod_sign (const db::edge<C> &p, const db::edge<C> &q)
{
  return db::coord_traits<C>::vprod_sign (p.dx (), p.dy (), q.dx (), q.dy (), 0, 0);
}

/**
 *  @brief Convenience wrappers for coord_traits functions: scalar product: 0->p x 0->q
 */
template <class C>
typename db::coord_traits<C>::area_type sprod (const db::edge<C> &p, const db::edge<C> &q)
{
  return db::coord_traits<C>::sprod (p.dx (), p.dy (), q.dx (), q.dy (), 0, 0);
}

/**
 *  @brief Convenience wrappers for coord_traits functions: scalar product sign: sign(0->p x 0->q)
 */
template <class C>
int sprod_sign (const db::edge<C> &p, const db::edge<C> &q)
{
  return db::coord_traits<C>::sprod_sign (p.dx (), p.dy (), q.dx (), q.dy (), 0, 0);
}

/**
 *  @brief Determines the lower bound of the edge
 */
// [[ZH]] 功能：返回边的 y 下界 min(p1.y, p2.y)。
// [[ZH]] 用途：与 edge_ymax 一起构成「按 y 的包围盒」，是 EdgeProcessor 把边
// [[ZH]]     分桶/排序（按 ymin 升序）时的主键，也是 scanline 扫描的推进依据。
// [[ZH]] 注意：这里是「下界」而非「起点」—— 与边的方向无关。
template <class C>
inline C edge_ymin (const db::edge<C> &e) 
{
  return std::min (e.p1 ().y (), e.p2 ().y ());
}

/**
 *  @brief Determines the upper bound of the edge
 */
// [[ZH]] 功能：返回边的 y 上界 max(p1.y, p2.y)。与 edge_ymin 配对使用。
template <class C>
inline C edge_ymax (const db::edge<C> &e) 
{
  return std::max (e.p1 ().y (), e.p2 ().y ());
}

/**
 *  @brief Determines the left bound of the edge
 */
// [[ZH]] 功能：返回边的 x 下界 min(p1.x, p2.x)。用于 x 方向的包围盒与裁剪。
template <class C>
inline C edge_xmin (const db::edge<C> &e) 
{
  return std::min (e.p1 ().x (), e.p2 ().x ());
}

/**
 *  @brief Determines the right bound of the edge
 */
// [[ZH]] 功能：返回边的 x 上界 max(p1.x, p2.x)。与 edge_xmin 配对使用。
template <class C>
inline C edge_xmax (const db::edge<C> &e) 
{
  return std::max (e.p1 ().x (), e.p2 ().x ());
}

/**
 *  @brief Computes the x value of an edge at the given y value
 *
 *  HINT: for application in the scanline algorithm 
 *  it is important that this method delivers exactly (!) the same x for the same edge 
 *  (after normalization to dy()>0) and same y!
 */
// [[ZH-BEGIN]]
// 功能：求边在 y = const 这条水平线上的 x 坐标（即扫描线交点）。
//       这是扫描线算法的核心原语：给定当前扫过的 y，算出每条活跃边的横向位置，
//       以便按 x 排序、判断遮挡与内外关系。
//
// 参数：e 目标边（★ 按值传递，因为内部可能要交换端点）；
//       y 扫描线的 y 坐标。
// 返回：double 类型的 x 值。
//       注意：返回 double 而非整数 —— 这是刻意的，见下面「坑」。
//
// ★★ 上面英文 HINT 是**契约**，不是建议，必须严格遵守：
//     同一对 (边, y) 反复调用，必须返回**完全相等**的 double 值。
//     原因：EdgeProcessor 用这个 x 给同一 scanline 上的边排序。
//           如果同一位置算出两个略有差异的 double，排序结果就不稳定，
//           边的相对次序会随机化，布尔运算结果随之出错（且难以复现）。
//     为满足该契约：
//       ① 先把边归一化为 dy() > 0（保证同一条边只有一种参数化）；
//       ② y 落在端点之外时直接返回端点 x（不进入浮点除法）；
//       ③ 只有线段内部才做浮点插值，且表达式固定为
//              x1 + dx * (y - y1) / dy
//          运算次序不可改写（编译器重结合会破坏可复现性，
//          故 dbEdgeProcessor.cc 中相关变量声明为 volatile double）。
//
// 坑：1) 归一化只调整了 *副本* e（按值传递），不会影响调用者的边；
//        但这也意味着「同一条边、方向相反」会被归一化成同一结果，这是所需的。
//     2) 返回 double 意味着超出端点时会「夹住」到端点 x（clamp），
//        即它把边当作有限线段处理，而非无限长直线。
// [[ZH-END]]
template <class C>
inline double edge_xaty (db::edge<C> e, C y)
{
  // [[ZH]] 归一化：统一成 p1.y <= p2.y（即 dy >= 0 的方向）。
  // [[ZH]] 这是让「同一条边」只有唯一参数化的关键，交换的是副本，不影响调用方。
  if (e.p1 ().y () > e.p2 ().y ()) {
    e.swap_points ();
  }

  if (y <= e.p1 ().y ()) {
    // [[ZH]] 扫描线在起点之下 → 夹住，直接返回起点 x（避免无谓的浮点除法）。
    return e.p1 ().x ();
  } else if (y >= e.p2 ().y ()) {
    // [[ZH]] 扫描线在终点之上 → 夹住，返回终点 x。
    return e.p2 ().x ();
  } else {
    // [[ZH]] 线段内部：线性插值。
    // [[ZH]] ★ 这个表达式的运算次序是「契约」的一部分，不可重排/优化掉，
    // [[ZH]]   否则不同翻译单元可能算出不同的 double，破坏排序稳定性。
    return double (e.p1 ().x ()) + double (e.dx ()) * double (y - e.p1 ().y ()) / double (e.dy ());
  }
}

// [[ZH-BEGIN]]
// 功能：这一组四个**比较仿函数 (comparator)**，用于按边的包围盒给边排序。
//       四者结构完全相同 —— 「先按主键比较；主键相等时用 edge 自身的 operator< 兜底」：
//         edge_ymin_compare → 主键 = y 下界 min(p1.y, p2.y)
//         edge_ymax_compare → 主键 = y 上界 max(p1.y, p2.y)
//         edge_xmin_compare → 主键 = x 下界 min(p1.x, p2.x)
//         edge_xmax_compare → 主键 = x 上界 max(p1.x, p2.x)
//
// 参数：a、b 两条边（const 引用，不产生拷贝）。
// 返回：bool，满足 std::sort/std::set 要求的「严格弱序 (strict weak ordering)」。
//
// ★ 为什么末尾必须 `return a < b;`（这是最容易被忽略、也最容易写错的地方）：
//   只比较主键不构成严格弱序 —— 主键相同的元素之间既不"小于"也不"大于"，
//   排序算法会认为它们等价，其相对次序就变成**未定义**（取决于排序实现的内部细节）。
//   而 scanline 算法要求：同样的输入必须产生**完全相同的处理顺序**，
//   否则布尔运算结果会随编译环境/数据规模而变（且极难复现）。
//   用 operator<（依次比较 p1、p2）兜底可给出全序，从而保证结果确定。
//
// 坑：1) 比较的是**包围盒边界**，不是边的几何位置。对同一 ymin 上的多条边，
//        最终次序由 operator< 决定，可能与人的几何直觉不符 —— 这是刻意的。
//     2) 这些比较基于整数坐标，无浮点误差；但它只能用于「按 y 分批」的粗排序。
//        同一 scanline 内按 x 精排必须用 edge_xaty()（double），那是另一套比较器，
//        见 dbEdgeProcessor.cc 中的 EdgeXAtYCompare2。
// [[ZH-END]]
/**
 *  @brief Functor that compares two edges by their lower bound.
 */
template <class C>
class edge_ymin_compare
{
public:
  bool operator() (const db::edge<C> &a, const db::edge<C> &b) const
  {
    C ya = edge_ymin (a);
    C yb = edge_ymin (b);
    if (ya != yb) {
      return ya < yb;
    } else {
      return a < b;
    }
  }
};

/**
 *  @brief Functor that compares two edges by their upper bound.
 */
template <class C>
class edge_ymax_compare
{
public:
  bool operator() (const db::edge<C> &a, const db::edge<C> &b) const
  {
    C ya = edge_ymax (a);
    C yb = edge_ymax (b);
    if (ya != yb) {
      return ya < yb;
    } else {
      return a < b;
    }
  }
};

/**
 *  @brief Functor that compares two edges by their left bound.
 */
template <class C>
class edge_xmin_compare
{
public:
  bool operator() (const db::edge<C> &a, const db::edge<C> &b) const
  {
    C ya = edge_xmin (a);
    C yb = edge_xmin (b);
    if (ya != yb) {
      return ya < yb;
    } else {
      return a < b;
    }
  }
};

/**
 *  @brief Functor that compares two edges by their right bound.
 */
template <class C>
class edge_xmax_compare
{
public:
  bool operator() (const db::edge<C> &a, const db::edge<C> &b) const
  {
    C ya = edge_xmax (a);
    C yb = edge_xmax (b);
    if (ya != yb) {
      return ya < yb;
    } else {
      return a < b;
    }
  }
};

/**
 *  @brief Computes the left bound of the edge geometry for a given band [y1..y2].
 */
template <class C>
inline C edge_xmin_at_yinterval (const db::edge<C> &e, C y1, C y2) 
{
  if (e.dx () == 0) {
    return e.p1 ().x ();
  } else if (e.dy () == 0) {
    return std::min (e.p1 ().x (), e.p2 ().x ());
  } else {
    return C (floor (edge_xaty (e, ((e.dy () < 0) ^ (e.dx () < 0)) == 0 ? y1 : y2)));
  }
}

/**
 *  @brief Computes the right bound of the edge geometry for a given band [y1..y2].
 */
template <class C>
inline C edge_xmax_at_yinterval (const db::edge<C> &e, C y1, C y2) 
{
  if (e.dx () == 0) {
    return e.p1 ().x ();
  } else if (e.dy () == 0) {
    return std::max (e.p1 ().x (), e.p2 ().x ());
  } else {
    return C (ceil (edge_xaty (e, ((e.dy () < 0) ^ (e.dx () < 0)) != 0 ? y1 : y2)));
  }
}

/**
 *  @brief Functor that compares two edges by their left bound for a given interval [y1..y2].
 *
 *  This function is intended for use in scanline scenarios to determine what edges are 
 *  interacting in a certain y interval.
 */
template <class C>
struct edge_xmin_at_yinterval_compare
{
  edge_xmin_at_yinterval_compare (C y1, C y2)
    : m_y1 (y1), m_y2 (y2)
  {
    // .. nothing yet ..
  }

  bool operator() (const db::edge<C> &a, const db::edge<C> &b) const
  {
    if (edge_xmax (a) < edge_xmin (b)) {
      return true;
    } else if (edge_xmin (a) >= edge_xmax (b)) {
      return false;
    } else {
      C xa = edge_xmin_at_yinterval (a, m_y1, m_y2);
      C xb = edge_xmin_at_yinterval (b, m_y1, m_y2);
      if (xa != xb) {
        return xa < xb;
      } else {
        return a < b;
      }
    }
  }

public:
  C m_y1, m_y2;
};

} // namespace db

/**
 *  @brief Special extractors for the edges
 */

namespace tl 
{
  template<> DB_PUBLIC void extractor_impl (tl::Extractor &ex, db::Edge &b);
  template<> DB_PUBLIC void extractor_impl (tl::Extractor &ex, db::DEdge &b);

  template<> DB_PUBLIC bool test_extractor_impl (tl::Extractor &ex, db::Edge &b);
  template<> DB_PUBLIC bool test_extractor_impl (tl::Extractor &ex, db::DEdge &b);

} // namespace tl

#endif

