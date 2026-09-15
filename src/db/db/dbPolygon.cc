
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


#include "dbPolygon.h"

namespace db
{

// [[ZH-BEGIN]]
// 功能：求一条边在**各向异性偏移**后的“支撑直线”（用一条 DEdge 表示）。
//
// 参数：e  原始边（★ 不允许退化，函数开头有 tl_assert）；
//       dx, dy  X/Y 方向的偏移量（可不同，这就是“各向异性”的来源）；
//       ext    沿边方向的额外延伸量（用于构造“测试直线”，见下）；
//       nsign  法线方向的符号（+1 / -1）—— 决定向边的哪一侧偏移。
// 返回：一条 db::DEdge（浮点），表示偏移后的直线。
//
// 【实现思路（三步）】
//   ① 把边方向 e.d() 归一化为单位向量 ec；
//      再取它的法线 nc = (-ec.y, ec.x)（即把 ec 逆时针转 90°）。
//      ★ 这与 dbEdge 的“右侧=内侧”约定一致：nc 的正负号决定内外。
//   ② “各向异性缩放”的关键两步：
//         ec *= sqrt(ec.x²·dx² + ec.y²·dy²) * ext
//         nc *= sqrt(nc.x²·dx² + nc.y²·dy²) * nsign
//      ★ 注意开方里是**椭圆型度量**而不是普通的 |dx|/|dy| ——
//        这正是在“X、Y 被不同缩放”时求真实偏移量的正确公式。
//        也是本文件区别于“简单垂直平移”的本质所在。
//   ③ 用 nc 做垂直偏移、用 ec 向两端各延伸一点，得到一条**比原边更长**的
//      “测试直线”。
//
// 【★ 为什么要造“测试直线”而不是直接给偏移后的边】
//   上面英文注释已说明：这条加长的直线用来**界定相邻边各自负责的区域**
//   （裁剪范围）—— 因为相邻边的偏移结果会互相影响（拐角处需要求交）。
//   多延伸半宽是为了避免“非锐角情形下偏移太短、算不出交点”的问题。
// [[ZH-END]]
// [[ZH-BEGIN]]
// ============================================================================
//  dbPolygon.cc —— 多边形的 **尺寸缩放 (sizing)** 的实现
// ============================================================================
//
// 【本文件在做什么】
//   实现 polygon_contour<C>::size (dx, dy, mode) —— 把轮廓向外扩张/向内收缩。
//   这是 DRC 里“宽度/间距检查”与版图修正（OPC、填充）的底层操作。
//
// 【★ 核心难点：各向异性（dx ≠ dy）时，“法线”不再是垂直方向】
//   当 dx == dy（各向同性）时，偏移就是把每条边沿其法线平移 dx —— 很直白。
//   但当 dx != dy（各向异性，X/Y 方向缩放量不同）时：
//     · “垂直于边的单位向量”**不再**是偏移方向；
//     · 偏移量在 X、Y 上被不同地缩放，因此正确的偏移方向需要重新推导。
//   本文件下面那几个辅助函数（compute_shifted / compute_normals / dpx）
//   全部是为解决这一件事而存在的。理解这一点，整个文件就好读了。
//
// 【★ 第二个难点：结果必须落回整数网格，但偏移量本质是小数】
//   偏移后的角点位置是√运算的结果（小数），而坐标是整数。
//   若简单四舍五入，**45° 的边**会被破成锯齿（不再精确是 45°）。
//   为此有 dpx<C> 这个“智能乘法”：它设法让水平/垂直/45° 的方向保持“贴着网格”，
//   而其它角度就正常按浮点处理。
//   ★ 这解释了为什么本文件里到处是 sqrt / M_SQRT2 / M_SQRT1_2。
//
// 【本文件的组织】
//   compute_shifted     求一条边在偏移后的“支撑直线”
//   dpx<C>              智能乘法（保持特殊角度贴网格；整数/浮点两套）
//   compute_normals     求边的方向向量与偏移方向（各向异性下需特殊推导）
//   vprod_sign_for / sprod_sign_for   按向量类型分派的叉积/标积符号
//   polygon_contour<C>::size (dx, dy, mode)   ★ 主体：约 290 行
//   namespace tl        多边形/简单多边形的文本解析特化
//
// 【一个需要注意的实现细节】
//   本文件大量使用 **db::DCoord（浮点）做中间计算**，最后才舍回 Coord。
//   即“**浮点算、整数存**”—— 这是全库处理几何构造时的通用做法：
//   确保内部精度，只在与存储交互处引入舍入。
// [[ZH-END]]
template <class C>
static
db::DEdge compute_shifted (const db::edge<C> &e, C dx, C dy, double ext, int nsign)
{
  tl_assert (! e.is_degenerate ()); // no coincident points allowed

  //  Compute the unit vector of the line and its normal (times width)
  db::DVector ec (e.d ());
  ec *= 1.0 / ec.double_length ();
  db::DVector nc (-ec.y (), ec.x ());

  ec *= sqrt (ec.x () * ec.x () * dx * dx + ec.y () * ec.y () * dy * dy) * ext;
  nc *= sqrt (nc.x () * nc.x () * dx * dx + nc.y () * nc.y () * dy * dy) * nsign;

  //  We create two test lines for the adjacent edges that extend somewhat further (i.e by half
  //  the width). These test lines define the limits of the area where the segments are responsible
  return db::DEdge (db::DPoint (e.p1 ()) + nc - ec, db::DPoint (e.p2 ()) + nc + ec);
}

/**
 *  @brief Smart multiplication of a vector with a distance
 *  This function tries to keep the length of the vector on grid if its
 *  a 45 degree or horizontal/vertical one.
 */
// [[ZH-BEGIN]]
// 功能：**智能乘法** —— 向量 p 乘以距离 d，但尽量让结果“贴着网格”。
//
// ★ 为什么需要它（本函数存在的全部理由）：
//   偏移后坐标需要落回整数。若对任意方向都简单地“乘完再四舍五入”，
//   水平/垂直/45° 这些**本应精确**的方向会被破成微小锯齿（不再精确）。
//   例如 45° 的单位向量是 (1/√2, 1/√2)，乘一个整数距离后两端都会引入
//   √2 的舍入误差 → 结果不再是“每步 X 与 Y 增量相等”的真 45° 边。
//   本函数对这两种特殊情形做**针对性取整**，从而保住这些角度。
//
// 参数：p 方向向量（**注意：约定它已是单位向量或近单位向量**）；
//       d 要乘的距离（可为负）。
// 返回：p * d 的“智能”结果（DVector，浮点）。
//
// 【实现：整数版分三种情形（float 版直接乘，无舍入问题）】
//   ① 水平或垂直（某一分量≈0）：
//        先把 d 四舍五入成整数再乘 → 保证沿轴向的偏移量是**整数**，
//        于是边仍精确保持水平/垂直。
//   ② 45°（两分量绝对值相等）：
//        用 `M_SQRT2 * rounded (d * M_SQRT1_2)` ——
//        即先“把 d 换算成沿轴分量再取整”，再乘回 √2。
//        ★ 这样 p*(d) 的每个分量都会落在网格上，45° 得以保持。
//        （M_SQRT1_2 = 1/√2，M_SQRT2 = √2，所以 M_SQRT2 是 M_SQRT1_2 的倒数）
//   ③ 其它角度：正常乘（几何上无法保证贴网格，也不值得尝试）。
//
// 【★ 上游注释提到的一个容忍点】
//   看英文注释：向上模式（“up” mode）用于计算延伸量，
//   而“取一个**略大**的值有助于避免延伸太短导致算不出交点”——
//   即取整方向在这里是**故意偏向“多延伸一点”**的，
//   宁可多留一点余量，也不要因延伸不足而丢失交点（那会破坏拐角几何）。
// [[ZH-END]]
template <class C>
inline db::DVector dpx (const db::DVector &p, double d);

template <>
inline db::DVector dpx<Coord> (const db::DVector &p, double d)
{
  //  Note: "up" mode is used for computing the extension. Some bigger value
  //  is helpful in avoiding the case of too short extensions causing 
  //  a missing intersection even in non-acute angle case
  if (fabs (p.x ()) < db::epsilon || fabs (p.y ()) < db::epsilon) {
    return p * double (coord_traits<Coord>::rounded (d));
  } else if (fabs (fabs (p.x ()) - fabs (p.y ())) < db::epsilon) {
    //  45 degree case: try to round d such that if p is on the grid it will be later
    return p * (M_SQRT2 * coord_traits<Coord>::rounded (d * M_SQRT1_2));
  } else {
    return p * d;
  }
}

// [[ZH]] 功能：dpx 的**浮点特化** —— 无需任何取整，直接相乘。
// [[ZH]] 理由：坐标本身就是 double，不存在“回落整数网格”的问题，
// [[ZH]]       因此智能取整在这里既无必要也无意义（上面英文注释也说明了）。
// [[ZH]] 这体现了库内常见做法：**同一算法在整数与浮点下分别特化**，
// [[ZH]]       只在整数侧处理“对齐网格”这类问题。
template <>
inline db::DVector dpx<DCoord> (const db::DVector &p, double d)
{
  //  No need to round in the double case
  return p * d;
}

// [[ZH-BEGIN]]
// 功能：求一条边的**方向单位向量 ed** 与**偏移方向向量 nd**（均为浮点）。
//
// 参数：d          边的方向向量（未归一化）：
//       dx, dy     X/Y 方向的缩放量（可不同，决定各向异性）；
//       nsign      偏移方向的符号（+1/-1，即朝边的哪一侧）；
// 输出：ed 方向单位向量；nd 偏移方向向量（★ 已含缩放与符号）。
//
// 【★ 为什么不能简单地“取垂线”】
//   各向异性下（dx ≠ dy），偏移方向**不是**垂直方向。
//   本函数先把 d 归一化得到 ed，再由 ed 取垂线，
//   但关键在最后一步：用 dpx 把垂线**按各向异性缩放**（见下 nd 的赋值）。
//   这就是为什么这里需要专门写一个函数，而不是在主流程里随手算。
//
// 【两处分支说明】
//   1) dx == dy（**各向同性**）：走上面的“简化处理”分支。
//      此时垂线方向即为偏移方向，只需按长度缩放。
//   2) 否则：走各向异性分支（在下面），需要额外处理。
//
// 【一个空值的极端情形（上游标了 TODO）】
//   若 d 的长度小于浮点精度（prec_distance），会把 ed/nd 置为空向量，
//   并带有 TODO 注释说“这本不该发生（前面已断言 d 非 0）”。
//   ★ 即：这是一个**防御性分支**，正常输入不会走到；
//     若真走到，ed/nd 为空会导致此处结果无几何意义。
// [[ZH-END]]
template <class C> 
static void
compute_normals (const db::vector<C> &d, C dx, C dy, int nsign, db::DVector &ed, db::DVector &nd)
{
  if (db::coord_traits<C>::equal (dx, dy)) {

    //  Simplified handling for the isotropic case
    double f = d.double_length ();

    if (f < db::coord_traits<DCoord>::prec_distance ()) {

      //  TODO: this should never happen (we assert before that d is not 0)
      ed = db::DVector ();
      nd = db::DVector ();

    } else {

      ed = db::DVector (d) * (1.0 / f);
      nd = db::DVector (-ed.y (), ed.x ());

      //  dpx is a smart multiplication trying to preserve 45 degree edges on grid
      nd = dpx<C> (nd, fabs (double (dx)) * nsign);

    }

  } else {

    double f = sqrt(double (dx) * double (dx) * double (d.y()) * double (d.y()) + double (dy) * double (dy) * double (d.x()) * double (d.x()));
    if (f < db::coord_traits<DCoord>::prec_area ()) {

      if (dx == 0) {
        ed = db::DVector (0.0, 1.0);
      } else if (dy == 0) {
        ed = db::DVector (1.0, 0.0);
      } else {
        ed = db::DVector ();
      }

      nd = db::DVector();

    } else {

      ed = db::DVector (d) * (double (dx) * double (dy) / f);

      nd = db::DVector (double (-d.y ()) * double (dx) * double (dx), double (d.x ()) * double (dy) * double (dy));
      nd *= nsign / f;

    }

  }
}

/**
 *  @brief Provides a special DVector vprod sign for the purpose of representing integer-coordinate vectors
 *  The "zero" criterion is somewhat tighter than that of the normal integer value vectors.
 *  Hence, parallelity is somewhat more strict which makes the size function produce a
 *  better approximation to the desired target contour.
 */
static inline int
vprod_sign_for (const db::DVector &a, const db::DVector &b, const db::Vector &)
{
  double vp = db::vprod (a, b);
  if (vp <= -1e-2) {
    return -1;
  } else if (vp < 1e-2) {
    return 0;
  } else {
    return 1;
  }
}

/**
 *  @brief Fallback to the default vprod sign in the double-coordinate case
 */
static inline int
vprod_sign_for (const db::DVector &a, const db::DVector &b, const db::DVector &)
{
  return db::vprod_sign (a, b);
}

/**
 *  @brief Provides a special DVector sprod sign for the purpose of representing integer-coordinate vectors
 *  The "zero" criterion is somewhat tighter than that of the normal integer value vectors.
 *  Hence, orthogonality is somewhat more strict which makes the size function produce a
 *  better approximation to the desired target contour.
 */
static inline int
sprod_sign_for (const db::DVector &a, const db::DVector &b, const db::Vector &)
{
  double sp = db::sprod (a, b);
  if (sp <= -1e-2) {
    return -1;
  } else if (sp < 1e-2) {
    return 0;
  } else {
    return 1;
  }
}

/**
 *  @brief Fallback to the default sprod sign in the double-coordinate case
 */
static inline int
sprod_sign_for (const db::DVector &a, const db::DVector &b, const db::DVector &)
{
  return db::sprod_sign (a, b);
}

template <class C>
void polygon_contour<C>::size (C dx, C dy, unsigned int mode)
{
  if (dx == 0 && dy == 0) {
    return;
  }
  if (size () < 2) {
    return;
  }

  double ext = 100.0;
  if (mode == 0) {
    ext = 0.0;
  } else if (mode == 1) {
    ext = sqrt(2.0) - 1.0;
  } else if (mode == 2) {
    ext = 1.0; 
  } else if (mode == 3) {
    ext = sqrt(2.0) + 1.0;
  } else if (mode == 4) {
    ext = 10.0;
  }

  bool outside = (dx + dy) > 0;
  int nsign = outside ? 1 : -1;
  dx *= nsign;
  dy *= nsign;

#if 1
  //  New algorithm: trying to preserve 45 degree angles

  std::vector<point_type> new_points;
  new_points.reserve (size () * 2);

  simple_iterator p0 (this, 0);
  simple_iterator pn (this, size ());

  simple_iterator p = p0;

  simple_iterator pp = p;
  ++pp;

  std::back_insert_iterator<std::vector<point_type> > pts (new_points);

  tl_assert (*pp != *p); // no coincident points allowed

  db::vector<C> d (*pp - *p);
  db::DVector ed, nd;
  compute_normals (d, dx, dy, nsign, ed, nd);

  do {

    simple_iterator ppp = pp;
    ++ppp;
    if (ppp == pn) {
      ppp = p0;
    }

    tl_assert (*ppp != *pp); // no coincident points allowed

    db::vector<C> dd (*ppp - *pp);
    db::DVector eed, nnd;
    compute_normals (dd, dx, dy, nsign, eed, nnd);

    int vpsign = vprod_sign_for (eed, ed, dd) * nsign;

    if (vpsign <= 0) {

      if (nd.double_length () < db::epsilon) {

        //  no shift implied by second edge: simply shift the point in the
        //  direction implied by the second edge and connect to the vertex
        *pts++ = *pp;
        *pts++ = *pp + vector<C> (nnd);

      } else if (nnd.double_length () < db::epsilon) {

        //  no shift implied by second edge: simply shift the point in the
        //  direction implied by the first edge and connect to the vertex
        *pts++ = *pp + vector<C> (nd);
        *pts++ = *pp;

      } else if (vpsign == 0 && sprod_sign_for (nd, nnd, dd) > 0) {

        //  colinear edges: simply shift the point
        *pts++ = *pp + vector<C> (nd);

      } else {

        //  inner corner -> create a loop of three points which define the area
        //  in self-overlapping way but confined to the resulting contour
        *pts++ = *pp + vector<C> (nd);
        *pts++ = *pp;
        *pts++ = *pp + vector<C> (nnd);

      }

    } else {

      double l1max = ext * nd.double_length () / ed.double_length ();
      double l2max = ext * nnd.double_length () / eed.double_length ();

      double dv = db::vprod (ed, eed);

      double l1 = db::vprod (nnd - nd, eed) / dv;
      double l2 = db::vprod (nd - nnd, ed) / dv;

      if ((l1 < -db::epsilon) != (l2 < -db::epsilon)) {

        //  No well-formed intersection (reflecting edge) ->
        //  create a direct connection
        *pts++ = *pp + vector<C> (nd);
        *pts++ = *pp + vector<C> (nnd);

      } else if (l1 < l1max + db::epsilon && l2 < l2max + db::epsilon) {

        //  well-formed corner
        *pts++ = *pp + vector<C> (nd + ed * l1);

      } else {

        //  cut-off corner: produce two points connecting the edges 
        *pts++ = *pp + vector<C> (nd + ed * std::min (l1max, l1));
        *pts++ = *pp + vector<C> (nnd - eed * std::min (l2max, l2));

      }

    }
    
    p = pp;
    pp = ppp;

    ed = eed;
    nd = nnd;

    d = dd;

  } while (p != p0);

  //  assign the results
  assign (new_points.begin (), new_points.end (), db::unit_trans<C> (), is_hole (), true /*compress*/, false /*don't normalize*/);

#else

  size_t npts = size ();

  if (npts < 2) {
    return;
  }

  //  create a vector for the output points
  std::vector<point_type> new_points;
  new_points.reserve (npts);

  //  create a vector where we remember what edge is obsolete since it became inverted
  std::vector<short> inverted;
  inverted.resize (npts, 0);
  size_t nvalid = npts;
  size_t nvalid_last = 0;

  while (nvalid >= 2 && nvalid_last != nvalid) {

    nvalid_last = nvalid;

    new_points.clear ();

    //  find the first and second valid edge:
    //  lie = last input edge, cie = current input edges

    unsigned int i = 0; 

    while (i < npts && inverted[i]) { ++i; }

    tl_assert (i != npts);
    db::edge<C> lie ((*this)[i], (*this)[(i + 1) % npts]); 
    db::DEdge lie_s (compute_shifted (lie, dx, dy, ext, nsign));
    unsigned int lie_index = i;

    do { ++i; } while (i < npts && inverted[i]);

    tl_assert (i != npts);
    db::edge<C> cie ((*this)[i], (*this)[(i + 1) % npts]);
    db::DEdge cie_s (compute_shifted (cie, dx, dy, ext, nsign));
    unsigned int cie_index = i;

    //  Do an intersection test on these lines
    std::pair <bool, db::DPoint> ip = lie_s.intersect_point (cie_s);

    //  last output point
    db::point<C> lop;

    //  If the lines intersect, we have a well-formed inner or outer corner
    if (ip.first) {
      lop = point<C>::from_double (ip.second);
    } else {
      //  If the test lines to not cross, we have the case of an acute angle bend.
      //  This is a normal outer bend: we insert both points to define the contour in a 
      //  confined, cut-off fashion.
      lop = point<C>::from_double (cie_s.p1 ());
    }

    //  start with the next edge
    i = (i + 1) % npts;
    unsigned int ii;

    unsigned int llie_index;

    for (unsigned int j = 0; j < npts; ++j, i = ii) {

      ii = ((i + 1) >= npts ? 0 : (i + 1));

      // ignore inverted edges now.
      if (inverted[i]) {
        continue;
      }

      llie_index = lie_index;

      lie = cie;
      lie_s = cie_s;
      lie_index = cie_index;

      cie = db::edge<C> ((*this)[i], (*this)[ii]);
      cie_s = compute_shifted (cie, dx, dy, ext, nsign);
      cie_index = i;

      //  Do an intersection test on these lines
      std::pair <bool, db::DPoint> ip = lie_s.intersect_point (cie_s);

      //  If the lines intersect, we have a well-formed inner or outer corner
      if (ip.first && ! lie_s.parallel (cie_s)) {

        //  compute next output edge (corresponding to last input edge)
        db::edge<C> o (lop, point<C>::from_double (ip.second));

        int s = sprod_sign (o, lie);
        if (s > 0) {
          //  No inversion: output new point
          if (new_points.empty () || new_points.back () != lop) {
            new_points.push_back (lop);
          }
          new_points.push_back (o.p2 ());
        } else if (s < 0) {
          //  mark this edge as inverted
          inverted[lie_index] = 1;
          --nvalid;
        }

        lop = o.p2 ();

      } else {

        //  compute next output edge (corresponding to last input edge)
        db::edge<C> o (lop, point<C>::from_double (lie_s.p2 ()));

        int s = sprod_sign (o, lie);
        if (s > 0) {
          //  No inversion: output new point
          if (! new_points.empty () && new_points.back () != lop) {
            new_points.push_back (lop);
          }
        } else if (s < 0) {
          //  mark this edge as inverted
          inverted[lie_index] = 1;
          --nvalid;
        } 

        new_points.push_back (o.p2 ());

        lop = point<C>::from_double (cie_s.p1 ());
        new_points.push_back (lop);

      }

    }

  }

  //  assign the results
  assign (new_points.begin (), new_points.end (), db::unit_trans (), is_hole (), true /*compress*/, false /*don't normalize*/);

#endif

}

// explicit instantiations for polygon<T> and simple_polygon<T>
template class polygon_contour<db::Coord>;
template class polygon_contour<db::DCoord>;

}

namespace tl
{

template<> DB_PUBLIC void extractor_impl (tl::Extractor &ex, db::Polygon &p)
{
  if (! test_extractor_impl (ex, p)) {
    ex.error (tl::to_string (tr ("Expected a polygon specification")));
  }
}

template<> DB_PUBLIC void extractor_impl (tl::Extractor &ex, db::DPolygon &p)
{
  if (! test_extractor_impl (ex, p)) {
    ex.error (tl::to_string (tr ("Expected a polygon specification")));
  }
}

template<> DB_PUBLIC void extractor_impl (tl::Extractor &ex, db::SimplePolygon &p)
{
  if (! test_extractor_impl (ex, p)) {
    ex.error (tl::to_string (tr ("Expected a polygon specification")));
  }
}

template<> DB_PUBLIC void extractor_impl (tl::Extractor &ex, db::DSimplePolygon &p)
{
  if (! test_extractor_impl (ex, p)) {
    ex.error (tl::to_string (tr ("Expected a polygon specification")));
  }
}


template<class C> DB_PUBLIC bool _test_extractor_impl (tl::Extractor &ex, db::polygon<C> &p)
{
  typedef db::point<C> point_type;
  std::vector <point_type> points;

  if (ex.test ("(")) {

    p.clear ();

    point_type pt;
    while (ex.try_read (pt)) {
      points.push_back (pt);
      ex.test (";");
    }

    p.assign_hull (points.begin (), points.end (), false, false);

    while (ex.test ("/")) {

      points.clear ();

      point_type pt;
      while (ex.try_read (pt)) {
        points.push_back (pt);
        ex.test (";");
      }

      p.insert_hole (points.begin (), points.end (), false, false);

    }

    ex.expect (")");

    return true;

  } else {
    return false;
  }
}

template<> DB_PUBLIC bool test_extractor_impl (tl::Extractor &ex, db::Polygon &p)
{
  return _test_extractor_impl (ex, p);
}

template<> DB_PUBLIC bool test_extractor_impl (tl::Extractor &ex, db::DPolygon &p)
{
  return _test_extractor_impl (ex, p);
}

template<class C> DB_PUBLIC bool _test_extractor_impl (tl::Extractor &ex, db::simple_polygon<C> &p)
{
  typedef db::point<C> point_type;
  std::vector <point_type> points;

  if (ex.test ("(")) {

    point_type pt;
    while (ex.try_read (pt)) {
      points.push_back (pt);
      ex.test (";");
    }

    p.assign_hull (points.begin (), points.end (), false, false);

    ex.expect (")");

    return true;

  } else {
    return false;
  }
}

template<> DB_PUBLIC bool test_extractor_impl (tl::Extractor &ex, db::SimplePolygon &p)
{
  return _test_extractor_impl (ex, p);
}

template<> DB_PUBLIC bool test_extractor_impl (tl::Extractor &ex, db::DSimplePolygon &p)
{
  return _test_extractor_impl (ex, p);
}

}



