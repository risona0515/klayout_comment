
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


#ifndef HDR_dbTypes
#define HDR_dbTypes

#include <stdint.h>
#define _USE_MATH_DEFINES // for MSVC
#include <math.h>
#include <stdio.h>
#include <algorithm>

namespace db {

// [[ZH-BEGIN]]
// ============================================================================
//  dbTypes.h —— 全库最底层的类型与数值策略定义
// ============================================================================
//
// 【这个文件为什么重要】
//   本库所有几何对象都是**关于坐标类型的模板**（point<C> / edge<C> / polygon<C> ...）。
//   而“坐标类型”的行为（怎么取整、怎么防溢出、精度是多少）由本文件的
//   coord_traits 提供。因此**看不懂本文件，就看不懂任何几何类的数值行为**。
//
// 【两个坐标类型】
//   db::Coord  —— 整数坐标，单位是 **DBU**（数据库最小单位，通常 1nm）。
//                 默认是 int32_t；若编译时定义 HAVE_64BIT_COORD 则是 int64_t。
//   db::DCoord —— double 坐标（浮点）。
//                 用来表达 DBU 以下的精度（例如近似圆、加密曲线）；
//                 转回整数时按 DBU 舍入。
//
// 【为什么需要 coord_traits（而不是直接用 int/double）】
//   ★ 关键在于**溢出**与**精度**必须显式管理：
//     · 两个 int32 坐标相乘（求叉积/面积）会超出 int32 → 必须用 int64_t，
//       这就是 area_type 存在的理由。
//     · int32 坐标之差的绝对值在极端值下会溢出 → 用更宽的 distance_type。
//     · 面积相乘后再相加可能超过 int64 → 64 位坐标用 __int128（见下方特化）。
//   类型选择是硬需求，不是风格问题 —— 用错类型就会静默溢出、算出错误的几何结果。
//
// 【本文件的三个核心概念】
//   1) coord_traits<C>          —— C 的数值策略（类型别名 + 取整 + 叉积/标积等）
//   2) coord_converter<D,C>      —— 坐标类型之间的转换（带舍入）
//   3) epsilon / epsilon_f       —— 浮点比较的模糊度常量
//
// 【阅读建议】
//   先看 Coord / DCoord 两个 typedef，再看 coord_traits 的四个特化
//   （int32 / int16 / int64 / double）各自的类型选择，最后看 generic_coord_traits
//   里那些运算函数的实现约定。
// [[ZH-END]]
/**
 *  @brief A generic constant describing the "fuzzyness" of a double comparison of a value around 1
 */
// [[ZH]] epsilon：double 比较的"模糊度"（相对误差量级，围绕 1 的值）。
// [[ZH]] 用途：判断两个 double 是否"相等"时用 `fabs(a-b) < epsilon` 而不是 `a == b`。
// [[ZH]] 为什么需要：浮点运算有舍入误差，精确相等在数学上等值的量之间常常不成立。
// [[ZH]] 取值 1e-10 是针对 double 约 15~16 位有效数字留出的余量。
// [[ZH]] 对比：几何**精确**判断（如共线）不用它，而用整数叉积（见 dbEdge.h 的 vprod_sign）。
const double epsilon = 1e-10;

/**
 *  @brief A generic constant describing the "fuzzyness" of a float comparison of a value around 1
 */
// [[ZH]] fepsilon：float 比较的模糊度。取值比 epsilon 宽松 4 个数量级，
// [[ZH]] 因为 float 只有约 7 位有效数字，能用 float 的地方本来就不要求高精度。
const double fepsilon = 1e-6;

/**
 *  @brief The standard integer coordinate type
 */
// [[ZH-BEGIN]]
// 功能：定义标准整数坐标类型 Coord —— **全库几何的默认坐标单位**。
//
// ★★ 最重要的概念：Coord 的单位不是微米，而是 **DBU**（database unit，
//    数据库最小单位），通常是 1nm。它的值 = "这个位置距原点多少个 DBU"。
//    微米 ↔ DBU 的换算由 db::Layout::dbu() 给出（1 微米 = 1/dbu 个 DBU）。
//    因此在版图语境下，坐标 1000 通常意味着 1000 DBU = 1 微米。
//
// 为何用整数而不是浮点：
//   · 版图数据量极大，整数省一半内存（对比 double）；
//   · 布尔运算要求"完全可复现"，整数叉积没有舍入误差，浮点做不到（见 dbEdge.h）。
//
// 编译期开关 HAVE_64BIT_COORD：
//   默认 int32_t（能表示 ±2.1e9 DBU，即约 ±2 米，对单个版图够用且省内存）；
//   定义该宏则用 int64_t（超大版图 / 拼接全芯片时需要）。
//   ★ 注意这是**编译期**决定，不是运行期配置 —— 它改变了所有几何类的二进制布局，
//     因此库依赖它的 ABI（build.sh 里对应 `-with-64bit-coord` 选项）。
// [[ZH-END]]
#if defined(HAVE_64BIT_COORD)
typedef int64_t Coord;
#else
typedef int32_t Coord;
#endif

/** 
 *  @brief The standard floating-point coordinate type
 */
// [[ZH]] DCoord：浮点坐标类型（就是 double）。
// [[ZH]] 用途：需要 DBU 以下精度的几何（近似圆/曲线、加密后的近似图形）。
// [[ZH]] 与整数坐标的关系：通过 coord_traits<DCoord>::rounded() 转回 Coord 时**按 DBU 舍入**，
// [[ZH]]       因此浮点精度最终会被量化掉 —— 只在中途计算保持精度。
// [[ZH]] 命名：D 前缀 = Double，因此 db::DPoint / db::DBox / db::DPolygon 都是浮点版本。
typedef double DCoord;

/**
 *  @brief Coordinate types traits (for integer types)
 *
 *  Defines associated types for a certain coordinate type:
 *  coord_type (the coord_type itself), area_type (the type 
 *  of the area associated), dist_type (the type of the distance
 *  between two coordinates).
 *  Also declares other properties like precision (epsilon),
 *  conversion methods etc.
 */

template <class C, class A, class D, class P, class S> 
struct generic_coord_traits 
{
  /** 
   *  @brief The coordinate type itself 
   */
  typedef C coord_type;

  /**
   *  @brief The associated area type
   */
  typedef A area_type;

  /**
   *  @brief The associated distance type
   */
  typedef D distance_type;

  /**
   *  @brief The associated perimeter type
   */
  typedef P perimeter_type;

  /**
   *  @brief The "short" coordinate type
   *
   *  This is a special type mainly used to represent "short" boxes (i.e. small ones) with 
   *  a small memory footprint. It is used mainly for 32bit coordinates and mask data.
   */
  typedef S short_coord_type;

  /**
   *  @brief The precision (resolution) of the coordinate type
   */
  static coord_type prec () { return 1; }

  /**
   *  @brief The precision (resolution) of the distance type
   */
  static distance_type prec_distance () { return 1; }

  /**
   *  @brief The precision (resolution) of the area type
   */
  static area_type prec_area () { return 1; }

  /**
   *  @brief The rounding method
   *  Specialization for double: Converts the given double to the coordinate type.
   */
  static coord_type rounded (double v) { return coord_type (v > 0 ? v + 0.5 : v - 0.5); }

  /**
   *  @brief The rounding method
   *  Specialization for float: Converts the given double to the coordinate type.
   */
  static coord_type rounded (float v) { return coord_type (v > 0 ? v + 0.5 : v - 0.5); }

  /**
   *  @brief The rounding method
   *  Converts the given value to the coordinate type.
   */
  template <class X>
  static coord_type rounded (X v) { return coord_type (v); }

  /**
   *  @brief The rounding method (up)
   */
  static coord_type rounded_up (double v) { return coord_type (ceil (v)); }

  /**
   *  @brief The rounding method (down)
   */
  static coord_type rounded_down (double v) { return coord_type (floor (v)); }

  /**
   *  @brief The rounding method for distances
   */
  static distance_type rounded_distance (double v) { return distance_type (v > 0 ? v + 0.5 : v - 0.5); }

  /**
   *  @brief The rounding method for perimeters
   */
  static perimeter_type rounded_perimeter (double v) { return perimeter_type (v > 0 ? v + 0.5 : v - 0.5); }

  /**
   *  @brief (Fuzzy) equality of coordinates
   */
  static bool equal (coord_type c1, coord_type c2)
  {
    return c1 == c2;
  }

  /**
   *  @brief (Fuzzy) less comparison of coordinates
   */
  static bool less (coord_type c1, coord_type c2)
  {
    return c1 < c2;
  }

  /**
   *  @brief The test for equality with a double
   */
  static bool equals (coord_type c, double v) 
  { 
    return fabs (double (c) - v) < 0.5;
  }  

  /**
   *  @brief The test for equality of the area with a double
   */
  static bool equals_area (area_type a, double v) 
  { 
    return fabs (double (a) - v) < 0.5;
  }  

  /** 
   *  @brief The square length of a vector
   *  
   *  Computes the square length of a vectors.
   *  The vector is a - b, where a and b are points.
   *
   *  @param ax The first points's x component
   *  @param ay The first points's y component
   *  @param bx The second points's x component
   *  @param by The second points's y component
   *
   *  @return The square length: (ax - bx) * (ay - by)
   */
  static area_type sq_length (coord_type ax, coord_type ay, 
                              coord_type bx, coord_type by) 
  {
    return ((area_type) ax - (area_type) bx) * ((area_type) ax - (area_type) bx) + ((area_type) ay - (area_type) by) * ((area_type) ay - (area_type) by);
  }

  /** 
   *  @brief The sign of the scalar product of two vectors.
   *  
   *  Computes the scalar product of two vectors.
   *  The first vector is a - c, the second b - c,
   *  where a, b and c are points.
   *
   *  @param ax The first points's x component
   *  @param ay The first points's y component
   *  @param bx The second points's x component
   *  @param by The second points's y component
   *  @param cx The third points's x component
   *  @param cy The third points's y component
   *
   *  @return The scalar product: (ax - cx) * (bx - cx) + (ay - cy) * (by - cy)
   */
  static area_type sprod (coord_type ax, coord_type ay, 
                          coord_type bx, coord_type by, 
                          coord_type cx, coord_type cy) 
  {
    return ((area_type) ax - (area_type) cx) * ((area_type) bx - (area_type) cx) + ((area_type) ay - (area_type) cy) * ((area_type) by - (area_type) cy);
  }

  /** 
   *  @brief The sign of the scalar product of two vectors with two 
   *  coordinates.
   *  
   *  Computes the sign of the scalar product of two vectors.
   *  The first vector is a - c, the second b - c,
   *  where a, b and c are points.
   *
   *  @param ax The first points's x component
   *  @param ay The first points's y component
   *  @param bx The second points's x component
   *  @param by The second points's y component
   *  @param cx The third points's x component
   *  @param cy The third points's y component
   *
   *  @return The sign of the scalar product (-1: negative,
   *  0: zero, +1: positive)
   */
  static int sprod_sign (coord_type ax, coord_type ay, 
                         coord_type bx, coord_type by, 
                         coord_type cx, coord_type cy) 
  {
    area_type p1 = ((area_type) ax - (area_type) cx) * ((area_type) bx - (area_type) cx);
    area_type p2 = -(((area_type) ay - (area_type) cy) * ((area_type) by - (area_type) cy));
    if (p1 > p2) {
      return 1;
    } else if (p1 == p2) {
      return 0;
    } else {
      return -1;
    }
  }

  /**
   *  @brief A combination of sign and value of sprod.
   */
  static std::pair<area_type, int> sprod_with_sign (coord_type ax, coord_type ay,
                                                    coord_type bx, coord_type by,
                                                    coord_type cx, coord_type cy)
  {
    area_type p1 = ((area_type) ax - (area_type) cx) * ((area_type) bx - (area_type) cx);
    area_type p2 = -(((area_type) ay - (area_type) cy) * ((area_type) by - (area_type) cy));
    if (p1 > p2) {
      return std::make_pair (p1 - p2, 1);
    } else if (p1 == p2) {
      return std::make_pair (0, 0);
    } else {
      return std::make_pair (p1 - p2, -1);
    }
  }

  /**
   *  @brief the vector product of two vectors.
   *  
   *  Computes the vector product of two vectors.
   *  The first vector is a - c, the second b - c,
   *  where a, b and c are points.
   *
   *  @param ax The first points's x component
   *  @param ay The first points's y component
   *  @param bx The second points's x component
   *  @param by The second points's y component
   *  @param cx The third points's x component
   *  @param cy The third points's y component
   *
   *  @return The vector product: (ax - cx) * (by - cy) - (ax - cx) * (by - cy)
   */
  static area_type vprod (coord_type ax, coord_type ay, 
                          coord_type bx, coord_type by, 
                          coord_type cx, coord_type cy) 
  {
    return ((area_type) ax - (area_type) cx) * ((area_type) by - (area_type) cy) - ((area_type) ay - (area_type) cy) * ((area_type) bx - (area_type) cx);
  }

  /** 
   *  @brief The sign of the vector product of two vectors with two 
   *  coordinates.
   *  
   *  Computes the sign of the vector product of two vectors.
   *  The first vector is a - c, the second b - c,
   *  where a, b and c are points.
   *
   *  @param ax The first points's x component
   *  @param ay The first points's y component
   *  @param bx The second points's x component
   *  @param by The second points's y component
   *  @param cx The third points's x component
   *  @param cy The third points's y component
   *
   *  @return The sign of the vector product (-1: negative,
   *  0: zero, +1: positive)
   */
  static int vprod_sign (coord_type ax, coord_type ay, 
                         coord_type bx, coord_type by, 
                         coord_type cx, coord_type cy) 
  {
    area_type p1 = ((area_type) ax - (area_type) cx) * ((area_type) by - (area_type) cy);
    area_type p2 = ((area_type) ay - (area_type) cy) * ((area_type) bx - (area_type) cx);
    if (p1 > p2) {
      return 1;
    } else if (p1 == p2) {
      return 0;
    } else {
      return -1;
    }
  }

  /**
   *  @brief A combination of sign and value of vprod.
   */
  static std::pair<area_type, int> vprod_with_sign (coord_type ax, coord_type ay,
                                                    coord_type bx, coord_type by,
                                                    coord_type cx, coord_type cy)
  {
    area_type p1 = ((area_type) ax - (area_type) cx) * ((area_type) by - (area_type) cy);
    area_type p2 = ((area_type) ay - (area_type) cy) * ((area_type) bx - (area_type) cx);
    if (p1 > p2) {
      return std::make_pair (p1 - p2, 1);
    } else if (p1 == p2) {
      return std::make_pair (0, 0);
    } else {
      return std::make_pair (p1 - p2, -1);
    }
  }
};

/** 
 *  @brief Coord_traits template declaration
 */
// [[ZH]] coord_traits<C>：某个坐标类型 C 的**数值策略**（所有几何类都依赖它）。
// [[ZH]] 主模板故意留空 —— 没有任何默认行为，未特化的类型会在编译期报错。
// [[ZH]] 这是刻意的：坐标类型必须显式声明其数值特性，不能被默默推导出来。
// [[ZH]] 下面提供四个特化：int32_t / int16_t / int64_t / double。
template <class C>
struct coord_traits
{
};

// [[ZH-BEGIN]]
// 功能：int32_t（默认 Coord）的数值策略。
//
// ★ 类型选择的理由（每个都是防溢出的硬需求，不是风格）：
//   area_type = **int64_t**
//     叉积/面积 = 两个 int32 坐标之积（最坏 ~2^31 * 2^31 = 2^62），
//     用 int32 必溢出。int64 是能装下它的最小可行类型。
//     ★ 这就是为什么 dbEdge 的叉积函数都返回 area_type。
//   distance_type = **uint32_t**
//     坐标之差的绝对值：int32 的 MIN..MAX 跨度是 2^32 量级，
//     无符号 32 位刚好能完整表示（int32 不行）。
//   perimeter_type = **uint64_t**
//     周长是多个 distance 之和，可能超过 uint32。
//   short_coord_type = **int16_t**
//     用于“短坐标”：小尺寸的 box 用短类型存，节省内存（见 dbBox/dbShapes 的短型变体）。
// [[ZH-END]]
/** 
 *  @brief Coord_traits specialization for 32 bit coordinates 
 */
template <>
struct coord_traits<int32_t>
  : public generic_coord_traits<int32_t, int64_t/*area*/, uint32_t/*dist*/, uint64_t/*perimeter*/, int16_t/*short*/>
{
};

/** 
 *  @brief Coord_traits specialization for 16 bit coordinates 
 */
// [[ZH]] int16_t 的策略：用于掩模数据等“坐标本来就不大”的场景（省内存）。
// [[ZH]] ★ 注意这里的 area_type 是 **int32_t**（而不是 int64）：
// [[ZH]]   因为 16 位坐标相乘最大 ~2^30，int32 够用 —— 类型选择按实际范围裁剪，
// [[ZH]]   不一刀切地都用最宽类型（这也是为了让短型数据保持瘦小）。
template <>
struct coord_traits<int16_t>
  : public generic_coord_traits<int16_t, int32_t/*area*/, uint32_t/*dist*/, uint32_t/*perimeter*/, int16_t/*short*/>
{
};

#if defined(HAVE_64BIT_COORD)
/** 
 *  @brief Coord_traits specialization for 64 bit coordinates 
 */
// [[ZH]] int64_t 的策略：仅当定义了 HAVE_64BIT_COORD 时才启用（超大版图）。
// [[ZH]] ★★ 关键：area_type 用 **__int128**（GCC/Clang 扩展类型）。
// [[ZH]]   两个 int64 坐标相乘可达 2^126，int64 远远装不下；
// [[ZH]]   如果没有 128 位类型，64 位坐标模式的叉积会溢出 —— 这就是它只在
// [[ZH]]   GCC/Clang 下可用的原因（MSVC 没有 __int128）。
// [[ZH]] 注意：这是全文件唯一会改变**编译可用性**的地方，不是纯配置。
template <>
struct coord_traits<int64_t>
  : public generic_coord_traits<int64_t, __int128/*area*/, uint64_t/*dist*/, uint64_t/*perimeter*/, int32_t/*short*/>
{
};
#endif

/** 
 *  @brief Coord_traits specialization for double coordinates 
 *
 *  The precision is chosen such that the double coordinate
 *  can represent "micrometers" with a physical resolution limit of 0.01 nm.
 *  The area precision will render reliable vector product signs for vectors
 *  of roughly up to 60 mm length.
 */
// [[ZH-BEGIN]]
// 功能：double（DCoord）的数值策略。这是唯一**不继承** generic_coord_traits 的特化 ——
//       因为浮点的取整/精度语义与整数完全不同，无法共用一套默认实现。
//
// ★★ 上面英文注释给出了精度取值的**量化依据**（很好的工程文档范例，值得一读）：
//     • prec() = 1e-5 ：
//         约定 DCoord 的单位是**微米**，而 1e-5 微米 = 0.01 nm，
//         即“物理分辨率下限 0.01 nm”。这就是高精度比较的判据尺度。
//     • prec_area() = 1e-10 ：
//         由它可以推算出叉积符号可信的**最大向量长度约 60 mm**。
//         意思是：超过 ~60mm 的向量来做叉积，符号就可能因浮点误差而不可信。
//         ★ 这是理解“为何浮点几何不可用于大尺寸版图”的具体依据。
//
// ★ 与整数版的关键差异：
//     • area_type / distance_type 都是 **double**（不是更宽的整型）——
//       浮点不靠加宽类型防溢出，靠的是指数范围大。
//     • rounded() 系列基本是**恒等映射**（不做取整），
//       除非显式转回 Coord（那时由 coord_converter 调 coord_traits<Coord>::rounded）。
//     • prec_distance() 是 1e-5 而非 1 —— 因此 db::DEdge::contains() 的“模糊”容差
//       在浮点下是 0.01 nm 量级，而整数下是 1 DBU。两者**不可混用**。
//
// 用途：近似圆/曲线的加密封装、需要子 DBU 精度的几何运算。
// [[ZH-END]]
template <>
struct coord_traits<double>
{
  // [[ZH]] 浮点的五个类型别名 — 注意全是 double（面积/距离不加宽）。
  typedef double coord_type;
  typedef double area_type;
  typedef double distance_type;
  typedef double perimeter_type;
  typedef float short_coord_type;

  // [[ZH]] prec() = 1e-5 微米 = 0.01 nm：DCoord 的物理分辨率下限。
  static double prec ()                       { return 1e-5; }
  // [[ZH]] 距离精度同 1e-5 —— 因此浮点版 contains() 的容差是 0.01nm 量级（绝非 1 DBU）。
  static double prec_distance ()              { return 1e-5; }
  // [[ZH]] 面积精度 1e-10：由它决定叉积符号在多大范围内可信（约 60mm 向量）。
  static double prec_area ()                  { return 1e-10; }
  template <class X>
  static double rounded (X v)                 { return double (v); }
  static double rounded_up (double v)         { return v; }
  static double rounded_down (double v)       { return v; }
  static double rounded_distance (double v)   { return v; }
  static double rounded_perimeter (double v)  { return v; }

  static area_type sq_length (coord_type ax, coord_type ay, 
                              coord_type bx, coord_type by) 
  {
    return (ax - bx) * (ax - bx) + (ay - by) * (ay - by);
  }

  static area_type sprod (coord_type ax, coord_type ay, 
                          coord_type bx, coord_type by, 
                          coord_type cx, coord_type cy) 
  {
    return (ax - cx) * (bx - cx) + (ay - cy) * (by - cy);
  }


  static int sprod_sign (double ax, double ay, double bx, double by, double cx, double cy) 
  {
    double dx1 = ax - cx, dy1 = ay - cy;
    double dx2 = bx - cx, dy2 = by - cy;
    double pa = (sqrt (dx1 * dx1 + dy1 * dy1) + sqrt (dx2 * dx2 + dy2 * dy2)) * db::epsilon;
    area_type p1 = dx1 * dx2;
    area_type p2 = -dy1 * dy2;
    if (p1 <= p2 - pa) {
      return -1;
    } else if (p1 < p2 + pa) {
      return 0;
    } else {
      return 1;
    }  
  }

  static std::pair<area_type, int> sprod_with_sign (double ax, double ay, double bx, double by, double cx, double cy)
  {
    double dx1 = ax - cx, dy1 = ay - cy;
    double dx2 = bx - cx, dy2 = by - cy;
    double pa = (sqrt (dx1 * dx1 + dy1 * dy1) + sqrt (dx2 * dx2 + dy2 * dy2)) * db::epsilon;
    area_type p1 = dx1 * dx2;
    area_type p2 = -dy1 * dy2;
    if (p1 <= p2 - pa) {
      return std::make_pair (p1 - p2, -1);
    } else if (p1 < p2 + pa) {
      return std::make_pair (0, 0);
    } else {
      return std::make_pair (p1 - p2, 1);
    }
  }

  static area_type vprod (coord_type ax, coord_type ay, 
                          coord_type bx, coord_type by, 
                          coord_type cx, coord_type cy) 
  {
    return (ax - cx) * (by - cy) - (ay - cy) * (bx - cx);
  }

  static int vprod_sign (double ax, double ay, double bx, double by, double cx, double cy) 
  {
    double dx1 = ax - cx, dy1 = ay - cy;
    double dx2 = bx - cx, dy2 = by - cy;
    double pa = (sqrt (dx1 * dx1 + dy1 * dy1) + sqrt (dx2 * dx2 + dy2 * dy2)) * db::epsilon;
    area_type p1 = dx1 * dy2;
    area_type p2 = dy1 * dx2;
    if (p1 <= p2 - pa) {
      return -1;
    } else if (p1 < p2 + pa) {
      return 0;
    } else {
      return 1;
    }
  }

  static std::pair<area_type, int> vprod_with_sign (double ax, double ay, double bx, double by, double cx, double cy)
  {
    double dx1 = ax - cx, dy1 = ay - cy;
    double dx2 = bx - cx, dy2 = by - cy;
    double pa = (sqrt (dx1 * dx1 + dy1 * dy1) + sqrt (dx2 * dx2 + dy2 * dy2)) * db::epsilon;
    area_type p1 = dx1 * dy2;
    area_type p2 = dy1 * dx2;
    if (p1 <= p2 - pa) {
      return std::make_pair (p1 - p2, -1);
    } else if (p1 < p2 + pa) {
      return std::make_pair (0, 0);
    } else {
      return std::make_pair (p1 - p2, 1);
    }
  }

  static bool equal (double c1, double c2)
  {
    return fabs (c1 - c2) < prec (); 
  }

  static bool less (double c1, double c2)
  {
    return c1 < c2 - prec () * 0.5;
  }

  static bool equals (double c, double v) 
  { 
    return fabs (double (c) - v) < prec (); 
  }  

  static bool equals_area (double a, double v) 
  { 
    return fabs (double (a) - v) < prec_area (); 
  }  

};

/**
 *  @brief A generic conversion operator from double coordinates to any type
 */
// [[ZH-BEGIN]]
// 功能：坐标类型转换器 —— 把坐标从类型 C 转到类型 D，**带舍入**。
//
// 参数：模板参数 D = 目标坐标类型，C = 源坐标类型。
// 返回：D 类型的坐标值，已经把 C 的值按 D 的精度舍入。
//
// ★ 为何不用 static_cast 而需要它：
//   从 DCoord(double) 转 Coord(int32) **必须走 coord_traits<D>::rounded()**，
//   否则 C++ 的隐式转换会**截断**而非四舍五入，
//   导致 1.9999999 变成 1（而不是 2）—— 这种系统性偏移会让几何结果慢慢错位。
//   反向（int → double）则是精确的，rounded 退化为恒等。
//
// 用途：几何类的模板构造函数（例如 db::Point<double> p; db::Point<int> q(p);）
//       内部就是通过它完成坐标转换的。这就是“浮点版本与整数版本可以互相构造”的机制。
// [[ZH-END]]
template <class D, class C>
struct coord_converter
{
  D operator() (C c) const
  {
    return coord_traits<D>::rounded (c);
  }
};

/**
 *  @brief A very generic cast operator from T to U
 */
template <class U, class T>
struct cast_op 
{
  U operator() (const T &t) const
  {
    return U (t);
  }
};

/**
 *  @brief A functor wrapping the epsilon constant in a templatized form
 */
template <class F>
struct epsilon_f 
{
  operator double () const { return 0.0; } 
};

/**
 *  @brief And the specialization of epsilon_f for double 
 */
template <>
struct epsilon_f<double>
{
  operator double () const { return epsilon; }
};

/**
 *  @brief And the specialization of epsilon_f for float
 */
template <>
struct epsilon_f<float>
{
  operator double () const { return fepsilon; }
};

/**
 *  @brief The type of a cell index
 */
// [[ZH-BEGIN]]
// 功能：本组是库内部使用的**标识符 (ID) 类型**定义 —— 全部是无符号整数下标。
//
// ★ 重要理解：这些不是“句柄/指针”，而是**整数索引**，需配合各自的容器才有效，
//   单独持有一个 ID 没有意义（容器改变后可能失效）。
//
// 为何都用无符号：它们是下标/编号，不存在负值；
//   同时让“未初始化/无效”可以用 0 或最大值来表达（各容器自行约定）。
//
// 各类型的用途：
//   cell_index_type          cell 在 Layout 内的下标（db::Cell::cell_index()）
//   properties_id_type       属性集的 ID（db::PropertiesRepository 分配）—— 0 = 无属性
//   property_names_id_type   属性名集合的 ID
//   property_values_id_type  属性值集合的 ID
//   pcell_id_type            PCell（参数化单元）的注册 ID
//   lib_id_type              library 的 ID
// [[ZH-END]]
typedef unsigned int cell_index_type;

/**
 *  @brief The type of a properties id
 */
typedef size_t properties_id_type;

/**
 *  @brief The type of a properties name id
 */
typedef size_t property_names_id_type;

/**
 *  @brief The type of a properties value id
 */
typedef size_t property_values_id_type;

/**
 *  @brief The type of the PCell id
 */
typedef unsigned int pcell_id_type;

/**
 *  @brief The type of the library id
 */
typedef size_t lib_id_type;

} // namespace db

#endif

