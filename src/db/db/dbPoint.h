
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



#ifndef HDR_dbPoint
#define HDR_dbPoint

#include "dbCommon.h"

#include "dbTypes.h"
#include "dbObjectTag.h"
#include "tlString.h"
#include "tlTypeTraits.h"
#include "tlVector.h"

#include <string>

namespace db {

template <class C> class vector;
template <class C, class R = C> struct box;
template <class C> class generic_repository;
class ArrayRepository;

// [[ZH-BEGIN]]
// ============================================================================
//  dbPoint.h —— 二维点（几何体系的最底层积木）
// ============================================================================
//
// 【位置】本库几何体系的最底层。
//     point  ─┐
//     vector ─┼─→ edge / box / polygon / path / text ... 都是由它们构成的
//     box    ─┘
//   因此先把 point / vector / box 三个搞清楚，后面所有几何类都会容易很多。
//
// 【点与向量的区别（重要，两者代码几乎一样但语义不同）】
//   db::point  —— 位置（一个坐标）
//   db::vector —— 位移（两个点之间的差）
//   ★ 库中**严格区分**二者，不用同一个类型。好处是类型系统能阻止
//     “把两个点相加”这类无意义的操作（因为 point + point 没有重载）。
//     转换是显式的：p1 - p2 得到 vector，point + vector 得到 point。
//
// 【与坐标类型的关系】
//   point<C> 的 C 是 db::Coord（整数 DBU）或 db::DCoord（double）。
//   → db::Point  = db::point<db::Coord>   （最常用）
//   → db::DPoint = db::point<db::DCoord>  （浮点版本）
//   两者可互相构造/赋值，而转换时**必须经过 coord_traits::rounded()**（见下）。
//
// 【内部结构】
//   只有两个成员 m_x、m_y —— 纯值类型，没有指针/缓存。
//   因此拷贝极廉价，库中大量按值传递 point。
// [[ZH-END]]
/**
 *  @brief A point class
 */

template <class C>
class DB_PUBLIC point
{
public:
  // [[ZH]] 与 dbEdge 同样的“配套类型别名”模式，供容器/模板取出相关类型。
  // [[ZH]] coord_type     → 坐标类型本身（C）
  // [[ZH]] coord_traits   → C 的数值策略（取整、精度等，见 dbTypes.h）
  // [[ZH]] vector_type    → 对应的向量类型（用于 p - q 的结果）
  // [[ZH]] distance_type  → 距离类型（比 C 宽，防溢出）
  // [[ZH]] area_type      → 面积/叉积类型（比 C 宽，防溢出）
  // [[ZH]] box_type       → 对应的矩形类型
  typedef C coord_type;
  typedef db::coord_traits<C> coord_traits;
  typedef db::vector<C> vector_type;
  typedef typename coord_traits::distance_type distance_type; 
  typedef typename coord_traits::area_type area_type; 
  typedef db::object_tag< point<C> > tag;
  typedef db::box<C> box_type;
  typedef db::point<C> point_type;

  /** 
   *  @brief Default constructor
   *
   *  Creates a point at 0,0
   */
  // [[ZH]] 功能：构造原点 (0,0)。
  point () : m_x (0), m_y (0) { }

  /**
   *  @brief Standard constructor
   *
   *  @param x The x coordinate
   *  @param y The y coordinate
   */
  // [[ZH]] 功能：用两个坐标构造。参数必须已是 C 类型（不做转换）。
  point (C x, C y) : m_x (x), m_y (y) { }

  /**
   *  @brief Standard constructor from a different type
   *
   *  @param x The x coordinate
   *  @param y The y coordinate
   */
  // [[ZH]] 功能：用**其它类型**的坐标构造（例如用 double 构造 Point）。
  // [[ZH]] ★ 协调：这里会调 coord_traits::rounded() 做**四舍五入**，而不是直接截断 ——
  // [[ZH]]   若用 static_cast，1.9999999 会变成 1 而非 2，
  // [[ZH]]   这种系统性偏左会让几何结果逐渐错位。这就是 dbTypes.h 里 coord_converter 存在的理由。
  template <class D>
  point (D x, D y) : m_x (coord_traits::rounded (x)), m_y (coord_traits::rounded (y)) { }

  /**
   *  @brief The copy constructor 
   *
   *  @param d The source from which to copy
   */
  point (const point<C> &d) : m_x (d.x ()), m_y (d.y ()) { }

  /**
   *  @brief Assignment
   *
   *  @param d The source from which to take the data
   */
  point &operator= (const point<C> &d) 
  {
    m_x = d.x ();
    m_y = d.y ();
    return *this;
  }

  /**
   *  @brief The copy constructor that also converts
   *
   *  The copy constructor allows one to convert between different
   *  coordinate types, if possible.
   *
   *  @param d The source from which to copy
   */
  // [[ZH]] 功能：**跨坐标类型**的转换构造（如 DPoint → Point）。
  // [[ZH]] 说明：标了 explicit，因此必须显式书写类型（如 Point p(dpt)），不能隐式转换 ——
  // [[ZH]]   这是刻意的：跨类型转换会丢精度，不应在不经意间发生。
  // [[ZH]] 又：同样走 rounded() 而非截断。
  template <class D>
  explicit point (const point<D> &d) : m_x (coord_traits::rounded (d.x ())), m_y (coord_traits::rounded (d.y ())) { }

  /**
   *  @brief Assignment which also converts
   *
   *  This assignment operator will convert the coordinate types if possible
   *
   *  @param d The source from which to take the data
   */
  template <class D>
  point &operator= (const point<C> &d) 
  {
    m_x = coord_traits::rounded (d.x ());
    m_y = coord_traits::rounded (d.y ());
    return *this;
  }

  /**
   *  @brief Add to operation
   */
  point<C> &operator+= (const vector<C> &v)
  {
    m_x += v.x ();
    m_y += v.y ();
    return *this;
  }

  /**
   *  @brief Move (equivalent to +=)
   *  This alias is needed for compatibility with other shapes
   */
  point<C> &move (const vector<C> &v)
  {
    m_x += v.x ();
    m_y += v.y ();
    return *this;
  }

  /**
   *  @brief method version of operator+ (mainly for automation purposes)
   */
  point<C> add (const vector<C> &v) const
  {
    point<C> r (*this);
    r += v;
    return r;
  }

  /**
   *  @brief Returns the scaled point
   */
  point<db::DCoord> scaled (double s) const
  {
    return point<db::DCoord> (m_x * s, m_y * s);
  }

  /**
   *  @brief Moved point (equivalent to add)
   *  This alias is needed for compatibility with other shapes
   */
  point<C> moved (const vector<C> &v) const
  {
    point<C> r (*this);
    r += v;
    return r;
  }

  /**
   *  @brief Subtract from operation
   */
  point<C> &operator-= (const vector<C> &v)
  {
    m_x -= v.x ();
    m_y -= v.y ();
    return *this;
  }
  
  /**
   *  @brief method version of operator- (mainly for automation purposes)
   */
  point<C> subtract (const vector<C> &v) const
  {
    return *this - v;
  }

  /**
   *  @brief method version of operator- (mainly for automation purposes)
   */
  vector<C> subtract (const point<C> &p) const
  {
    return *this - p;
  }

  /**
   *  @brief "less" comparison operator
   *
   *  This operator is provided to establish a sorting
   *  order
   */
  bool operator< (const point<C> &p) const
  {
    return m_y < p.m_y || (m_y == p.m_y && m_x < p.m_x);
  }

  /**
   *  @brief Equality test operator
   */
  bool operator== (const point<C> &p) const
  {
    return m_x == p.m_x && m_y == p.m_y;
  }

  /**
   *  @brief Inequality test operator
   */
  bool operator!= (const point<C> &p) const
  {
    return !operator== (p);
  }

  /**
   *  @brief Const transform
   *
   *  Transforms the point with the given transformation
   *  without modifying the point.
   *
   *  @param t The transformation to apply
   *  @return The transformed point
   */
  template <class Tr>
  point<typename Tr::target_coord_type> transformed (const Tr &t) const
  {
    return t (*this);
  }

  /**
   *  @brief In-place transformation
   *
   *  Transforms the point with the given transformation
   *  and writes the result back to the point.
   *
   *  @param t The transformation to apply
   *  @return The transformed point
   */
  template <class Tr>
  point &transform (const Tr &t)
  {
    *this = t (*this);
    return *this;
  }

  /**
   *  @brief Accessor to the x coordinate
   */
  C x () const
  {
    return m_x;
  }

  /**
   *  @brief Accessor to the y coordinate
   */
  C y () const
  {
    return m_y;
  }

  /**
   *  @brief Write accessor to the x coordinate
   */
  void set_x (C _x)
  {
    m_x = _x;
  }

  /**
   *  @brief Write accessor to the y coordinate
   */
  void set_y (C _y)
  {
    m_y = _y;
  }

  /**
   *  @brief Scaling self by some factor
   *
   *  Scaling involves rounding which in our case is simply handled
   *  with the coord_traits scheme.
   */
  point<C> &operator*= (double s)
  {
    m_x = coord_traits::rounded (m_x * s);
    m_y = coord_traits::rounded (m_y * s);
    return *this;
  }

  /**
   *  @brief Scaling self by some integer factor
   */
  point<C> &operator*= (long s)
  {
    m_x = coord_traits::rounded (m_x * s);
    m_y = coord_traits::rounded (m_y * s);
    return *this;
  }

  /**
   *  @brief Division by some divisor.
   *
   *  Scaling involves rounding which in our case is simply handled
   *  with the coord_traits scheme.
   */

  point<C> &operator/= (double s)
  {
    double mult = 1.0 / static_cast<double>(s);
    *this *= mult;
    return *this;
  }

  /**
   *  @brief Dividing self by some integer divisor
   */
  point<C> &operator/= (long s)
  {
    double mult = 1.0 / static_cast<double>(s);
    *this *= mult;
    return *this;
  }

  /**
   *  @brief The euclidian distance to another point
   *
   *  @param d The other to compute the distance to.
   */
  distance_type distance (const point<C> &p) const
  {
    double ddx (p.x ());
    double ddy (p.y ());
    ddx -= double (x ());
    ddy -= double (y ());
    return coord_traits::rounded_distance (sqrt (ddx * ddx + ddy * ddy));
  }

  /**
   *  @brief The euclidian distance of the point to (0,0)
   */
  // [[ZH]] 功能：两点间**欧氏距离**，结果舍入回 distance_type。
  // [[ZH]] 实现：转为 double 算 sqrt(dx²+dy²)，再用 rounded_distance 圆整。
  // [[ZH]] ★ 舍入的含义：距离是有小数的（如 √2），返回的是整数近似值。
  // [[ZH]]   因此**不要用 distance() 做精确比较**（两条略差的距离可能相等）——
  // [[ZH]]   需要精确比较时用 double_distance() 或 sq_distance()（后者免开方且无舍入）。
  // [[ZH]] 注意：内部转 double 是为了避免整数平方导致溢出（dx*dx 可达 2^62）。
  distance_type distance () const
  {
    // [[ZH]] 先转 double 再平方，避开整数溢出。
    double ddx (x ());
    double ddy (y ());
    return coord_traits::rounded_distance (sqrt (ddx * ddx + ddy * ddy));
  }

  /**
   *  @brief The euclidian distance to another point as double value
   *
   *  @param d The other to compute the distance to.
   */
  // [[ZH]] 功能：到另一点的欧氏距离，**不做舍入**（保持 double 精度）。
  // [[ZH]] 选用建议：需要精确比较/取最近点时用这个 ——
  // [[ZH]] 整数版的 distance() 会把接近的距离舍入成相等，导致“最近点”选择不确定。
  double double_distance (const point<C> &p) const
  {
    double ddx (p.x ());
    double ddy (p.y ());
    ddx -= double (x ());
    ddy -= double (y ());
    return sqrt (ddx * ddx + ddy * ddy);
  }

  /**
   *  @brief The euclidian distance of the point to (0,0) as double value
   */
  double double_distance () const
  {
    double ddx (x ());
    double ddy (y ());
    return sqrt (ddx * ddx + ddy * ddy);
  }

  /**
   *  @brief The square euclidian distance to another point
   *
   *  @param d The other to compute the distance to.
   */
  // [[ZH]] 功能：到另一点距离的**平方**（无舍入、无开方）。
  // [[ZH]] ★ 为什么这是比较距离的**首选**方式：
  // [[ZH]]   比较 d1 与 d2 的大小，等价于比较 d1² 与 d2²（距离非负）。
  // [[ZH]]   而平方版本全是整数乘加，**精确且无舍入**，还省掉一次 sqrt。
  // [[ZH]]   因此“找最近点”这类逻辑应比 sq_distance 而不是 distance。
  // [[ZH]] 返回类型：area_type（比坐标类型宽，因为平方的星纲与面积相同）。
  area_type sq_distance (const point<C> &p) const
  {
    return coord_traits::sq_length (p.x (), p.y (), x (), y ());
  }

  /**
   *  @brief The square euclidian distance to point (0,0)
   *
   *  @param d The other to compute the distance to.
   */
  area_type sq_distance () const
  {
    return coord_traits::sq_length (0, 0, x (), y ());
  }

  /**
   *  @brief The square of the euclidian distance to another point as double value
   *
   *  @param d The other to compute the distance to.
   */
  double sq_double_distance (const point<C> &p) const
  {
    double ddx (p.x ());
    double ddy (p.y ());
    ddx -= double (x ());
    ddy -= double (y ());
    return ddx * ddx + ddy * ddy;
  }

  /**
   *  @brief The square of the euclidian distance of the point to (0,0) as double value
   */
  double sq_double_distance () const
  {
    double ddx (x ());
    double ddy (y ());
    return ddx * ddx + ddy * ddy;
  }

  /**
   *  @brief String conversion
   *
   *  If dbu is set, it determines the factor by which the coordinates are multiplied to render
   *  micron units. In addition, a micron format is chosen for output of these coordinates.
   */
  // [[ZH-BEGIN]]
  // 功能：把点转成字符串（三个重载行为分支）。
  //
  // 参数：dbu 决定输出的**单位**，这是本函数唯一需要注意的地方：
  //         dbu == 0.0（默认）→ 直接输出 DBU 整数值，形如 "1000,2000"
  //         dbu == 1.0        → 同上但走 db_to_string（带数据库数值格式）
  //         dbu >  0.0 且 ≠1  → 把坐标乘 dbu 后按**微米**格式化（带单位/小数）
  //
  // 返回：坐标字符串，格式固定为 "x,y"（分号分隔是 edge 的做法，点是逗号）。
  //
  // 提示：db::Layout::dbu() 给出的就是这里该传的值。
  //       调试打印时传 dbu 能直接看到微米值，比对着 DBU 整数猜方便很多。
  //
  // 坑：输出格式会影响脚本/测试的字符串比对 —— 所以不要随意改格式。
  // [[ZH-END]]
  std::string
  to_string (double dbu = 0.0) const
  {
    if (dbu == 1.0) {
      return tl::db_to_string (m_x) + "," + tl::db_to_string (m_y);
    } else if (dbu > 0.0) {
      return tl::micron_to_string (dbu * m_x) + "," + tl::micron_to_string (dbu * m_y);
    } else {
      return tl::to_string (m_x) + "," + tl::to_string (m_y);
    }
  }

  /**
   *  @brief Fuzzy comparison of points
   */
  // [[ZH-BEGIN]]
  // 功能：**模糊**相等 —— 两个坐标均在容差内相等则视为同一点。
  //
  // ★★ 与 operator== 的区别（关键，切勿混用）：
  //     operator==      → **精确**相等（逐坐标严格相等）。
  //                       用于容器键/排序，必须构成严格全序。
  //     equal()         → **模糊**相等，带容差（整数下为 1 DBU）。
  //                       用于几何去重：把浮点误差导致的“几乎重合点”当作重合点。
  //
  // 坑：不要用 equal()/less() 做 std::map/std::set 的键 ——
  //     容差会破坏“传递性”直觉，导致容器行为异常。那类场景必须用 operator< / operator==。
  // [[ZH-END]]
  bool equal (const point<C> &p) const
  {
    // [[ZH]] 两维都经过 coord_traits::equal（内部带 prec_distance 容差）。
    return coord_traits::equal (x (), p.x ()) && coord_traits::equal (y (), p.y ());
  }

  /**
   *  @brief Fuzzy comparison of points for inequality
   */
  // [[ZH]] 功能：模糊不等（即 !equal）。
  bool not_equal (const point<C> &p) const
  {
    return ! equal (p);
  }

  /**
   *  @brief Fuzzy "less" comparison of points
   */
  // [[ZH-BEGIN]]
  // 功能：**模糊**“小于” —— 先把近乎相等的坐标归为相等，再定序。
  //
  // ★ 注意比较顺序是“**先 y 后 x**”（y 优先）—— 与 operator< 的“先 x 后 y”相反！
  //   这不是笔误：这是整数几何中常用的“扫描线优先”习惯
  //   （先按 y 分行，再在行内按 x 排），与 dbEdge 的 edge_ymin_compare 同一直觉。
  //   但正因为两者顺序不同，绝不能用 less() 替代 operator< 做容器键。
  //
  // 用途：几何去重（把容差内的点视为同一个），如多边形顶点合并。
  // [[ZH-END]]
  bool less (const point<C> &p) const
  {
    // [[ZH]] 先看 y（与 operator< 的顺序相反），容差内相等则继续看 x。
    if (! coord_traits::equal (y (), p.y ())) {
      return y () < p.y ();
    }
    if (! coord_traits::equal (x (), p.x ())) {
      return x () < p.x ();
    }
    // [[ZH]] 两维都在容差内 → 视为同一点，不构成“小于”。
    return false;
  }

  /**
   *  @brief The (dummy) translation operator
   */
  void translate (const point<C> &d, db::generic_repository<C> &, db::ArrayRepository &)
  {
    *this = d;
  }

  /**
   *  @brief The (dummy) translation operator
   */
  template <class T>
  void translate (const point<C> &d, const T &t, db::generic_repository<C> &, db::ArrayRepository &)
  {
    *this = d;
    transform (t);
  }

private:
  // [[ZH-BEGIN]]
  // 这是 point 的**全部**数据成员 —— 只有两个坐标，没有隐藏状态。
  //
  // m_x : x 坐标（类型 C，即 Coord 或 DCoord）
  // m_y : y 坐标
  //
  // 重要含义（与 dbEdge 的设计一致）：
  //   1) 纯**值类型**：sizeof(point<Coord>) == 8 字节，拷贝/比较极廉价，
  //      所以库中大量函数按值传递 point。
  //   2) 无缓存：x()/y()/distance() 每次调用重新计算，
  //      好处是改了坐标后不可能出现陈旧缓存。
  //   3) 默认构造是原点 (0,0) —— ★ 注意这与 dbEdge 不同：
  //      edge 默认构造得到**退化边**（p1==p2，无方向，需特别处理），
  //      point 默认构造得到正常的原点，没有任何“无效态”。
  //      因此在库中遇到“未初始化的点”时不用担心它是退化的。
  // [[ZH-END]]
  C m_x, m_y;
};

template <class C>
inline point<double> 
operator* (const db::point<C> &p, double s) 
{
  return point<double> (p.x () * s, p.y () * s);
}

template <class C>
inline point<C> 
operator* (const db::point<C> &p, long s) 
{
  return point<C> (p.x () * s, p.y () * s);
}

template <class C>
inline point<C> 
operator* (const db::point<C> &p, unsigned long s) 
{
  return point<C> (p.x () * s, p.y () * s);
}

template <class C>
inline point<C> 
operator* (const db::point<C> &p, int s) 
{
  return point<C> (p.x () * s, p.y () * s);
}

template <class C>
inline point<C> 
operator* (const db::point<C> &p, unsigned int s) 
{
  return point<C> (p.x () * s, p.y () * s);
}

template <class C, typename Number>
inline point<C>
operator/ (const db::point<C> &p, Number s)
{
  double mult = 1.0 / static_cast<double>(s);
  return point<C> (p.x () * mult, p.y () * mult);
}

/**
 *  @brief The binary + operator (addition point and vector)
 *
 *  @param p The first point
 *  @param v The second point
 *  @return p + v
 */
template <class C>
inline point<C>
operator+ (point<C> p, const vector<C> &v)
{
  p += v;
  return p;
}

/**
 *  @brief The binary - operator (addition of points)
 *
 *  @param p1 The first point
 *  @param p2 The second point
 *  @return p1 - p2
 */
template <class C>
inline point<C>
operator- (const point<C> &p, const vector<C> &v)
{
  return point<C> (p.x () - v.x (), p.y () - v.y ());
}

/**
 *  @brief The binary - operator (addition of points)
 *
 *  @param p1 The first point
 *  @param p2 The second point
 *  @return p1 - p2
 */
template <class C>
inline vector<C>
operator- (const point<C> &p1, const point<C> &p2)
{
  return vector<C> (p1.x () - p2.x (), p1.y () - p2.y ());
}

/**
 *  @brief The unary - operator 
 *
 *  @param p The point 
 *  @return -p = (-p.x, -p.y)
 */
template <class C>
inline point<C> 
operator- (const point<C> &p)
{
  return point<C> (-p.x (), -p.y ());
}

/**
 *  @brief The stream insertion operator
 */
template <class C>
inline std::ostream &
operator<< (std::ostream &os, const point<C> &p)
{
  return (os << p.to_string ());
}

/**
 *  @brief The short integer point
 */
// [[ZH]] ShortPoint：用 short 存坐标的点 —— 压缩存储用（如掩模数据），省内存。
// [[ZH]] 见 db::short_box 一类的短型体系；一般几何计算不用它。
typedef point <short> ShortPoint;

/**
 *  @brief The standard point
 */
// [[ZH-BEGIN]]
// ★ 与 db::Edge / db::DEdge 同样的模式：db::Point 不是独立类，而是模板的别名。
//
//   db::Point       = db::point<db::Coord>   → 整数坐标（单位 DBU），最常用
//   db::DPoint      = db::point<db::DCoord>  → 浮点坐标
//   db::ShortPoint  = db::point<short>       → 缩短存储（见上）
//
// 三者接口完全一致（同一模板），仅数值类型不同；可互相显式转换（转换时**带舍入**）。
// 因此本文件中所有 `point<C>` 的说明对它们同样适用。
// [[ZH-END]]
typedef point <db::Coord> Point;

/**
 *  @brief The standard double coordinate point
 */
// [[ZH]] DPoint：浮点坐标版本（D 前缀 = Double）。参见 dbTypes.h 中 coord_traits<double> 的说明。
typedef point <db::DCoord> DPoint;

/**
 *  @brief A generic conversion operator from double point to any type
 */
template <class D, class C>
struct point_coord_converter
{
  db::point<D> operator() (const db::point<C> &dp) const
  {
    return db::point<D> (dp);
  }
};

/**
 *  A fuzzy "less" operator for point lists
 */
template <class C>
inline bool less (const tl::vector<point<C> > &a, const tl::vector<point<C> > &b)
{
  if (a.size () != b.size ()) {
    return a.size () < b.size ();
  }

  for (typename tl::vector<point<C> >::const_iterator i = a.begin (), j = b.begin (); i != a.end (); ++i, ++j) {
    if (! i->equal (*j)) {
      return i->less (*j);
    }
  }

  return false;
}

/**
 *  A fuzzy "equal" operator for point lists
 */
template <class C>
inline bool equal (const tl::vector<point<C> > &a, const tl::vector<point<C> > &b)
{
  if (a.size () != b.size ()) {
    return false;
  }

  for (typename tl::vector<point<C> >::const_iterator i = a.begin (), j = b.begin (); i != a.end (); ++i, ++j) {
    if (! i->equal (*j)) {
      return false;
    }
  }

  return true;
}

}

/**
 *  @brief Special extractors for the points
 */

namespace tl 
{
  template <> DB_PUBLIC void extractor_impl (tl::Extractor &ex, db::Point &p);
  template <> DB_PUBLIC void extractor_impl (tl::Extractor &ex, db::DPoint &p);

  template <> DB_PUBLIC bool test_extractor_impl (tl::Extractor &ex, db::Point &p);
  template <> DB_PUBLIC bool test_extractor_impl (tl::Extractor &ex, db::DPoint &p);

} // namespace tl

#endif

