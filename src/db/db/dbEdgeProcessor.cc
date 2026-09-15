
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



#include "dbEdgeProcessor.h"
#include "dbPolygonGenerators.h"
#include "dbLayout.h"
#include "tlTimer.h"
#include "tlProgress.h"
#include "gsi.h"

#include <vector>
#include <deque>
#include <memory>

#if 0
#define DEBUG_MERGEOP
#define DEBUG_BOOLEAN
#define DEBUG_EDGE_PROCESSOR
#endif

// #define DEBUG_SIZE_INTERMEDIATE

namespace db
{

// [[ZH-BEGIN]]
// ============================================================================
//  dbEdgeProcessor.cc —— scanline 布尔运算引擎的实现
// ============================================================================
//
// 建议先读 dbEdgeProcessor.h 的文件头注释（那里有整体背景、四阶段划分、
// wrap count 与 property 两个核心概念）。本文件的注释侧重"实现细节与数据结构"。
//
// 【本文件的组织顺序】
//   §1 工具与比较器        本行以下 ~ 550 行
//        NonZeroInsideFunc / ProjectionCompare / PolyMapCompare(已废弃)
//        is_point_on_exact / is_point_on_fuzzy     精确 vs 模糊的"点在边上"
//        safe_intersect_point                      顺序无关的求交（重要）
//        CutPoints / WorkEdge                      阶段 1-3 的核心数据结构
//        EdgePropCompare / EdgeXAtYCompare2        scanline 排序所需的比较器
//   §2 交点计算            ~1080-1590
//        add_hparallel_cutpoints                   共线水平边之间的切点
//        get_intersections_per_band_90             正交（曼哈顿）快速路径
//        get_intersections_per_band_any            任意角度通用路径
//   §3 扫描线状态          ~1600-2160
//        EdgeProcessorState                        单 (sink,evaluator) 的扫描线状态机
//        EdgeProcessorStates + SkipInfo            单/多路统一 + skip 优化
//   §4 主驱动板            redo_or_process
//        阶段 1 prep → 阶段 2 intersections → 阶段 3 split → 阶段 4 production
//   §5 公开 API 实现       insert / process / redo / simple_merge / merge / size / boolean
//
// 【两个贯穿全文件的优化思想，读代码时留意】
//   ① **正交快速路径**：版图绝大多数图形是曼哈顿的（边水平或垂直）。
//      两条正交边求交只需 max/min，无需叉积与除法，因而单独写了 *_90 版本。
//      判断入口会先看"是否全是正交边"，是则走快速路径。
//   ② **一致性优先于绝对精度**：多处刻意使用 `volatile double` 阻止编译器
//      把浮点值缓存在寄存器里重结合（见文件内相关注释）——
//      目的不是数值精度，而是**保证同一位置两次计算得到逐位相同的 double**，
//      从而使排序稳定、结果可复现。改动这些 volatile 可能引入极难排查的错误。
//
// 【调试开关】
//   把上面 `#if 0` 改为 `#if 1` 可打开 DEBUG_MERGEOP / DEBUG_BOOLEAN /
//   DEBUG_EDGE_PROCESSOR，在 stderr 打印内部状态（注意会非常啰嗦）。
// [[ZH-END]]
// [[ZH]] fill_factor：扫描线"分带(band)"时的容量增长经验系数。
// [[ZH]] 含义：每扩展一个 y 带，预估还要容纳多少比例的边，用于预分配容器，
// [[ZH]]       减少处理中的重新分配。1.5 是纯经验值，调大只影响内存占用/速度，
// [[ZH]]       不影响计算结果（改它不会改变输出的几何）。
const double fill_factor = 1.5;

// -------------------------------------------------------------------------------
//  Some utilities ..

// [[ZH-BEGIN]]
// 功能：最朴素的"内部"谓词 —— 环绕数非零即为内部。
//       即经典的**非零环绕规则 (non-zero winding rule)**。
//
//       ★ 它是 BooleanOp 的**默认**谓词：BooleanOp::edge() 与 BooleanOp::compare_ns()
//         内部直接构造一个 NonZeroInsideFunc 并把同一实例同时当作 A、B 两个操作数的
//         谓词（见本文件 BooleanOp::edge 的实现）。
//         而 BooleanOp2 则改用 ParametrizedInsideFunc，以便给 A、B **分别**指定模式
//         （这就是 BooleanOp2 相对 BooleanOp 的唯一增强）。
//
// 与 ParametrizedInsideFunc 的关系：后者是它的泛化版本（带 mode 参数），
//       mode = -1 时两者行为完全一致。
//
// 参数：wc 环绕数（可为负）。
// 返回：true = 该位置算作"在图形内部"。
// [[ZH-END]]
struct NonZeroInsideFunc 
{
  inline bool operator() (int wc) const
  {
    // [[ZH]] 只看是否为零：非零即内部（负环绕数也算内部）。
    return wc != 0;
  }
};

// [[ZH-BEGIN]]
// 功能：把一个点序列按"沿某条边的投影长度"排序的比较器。
//
// ★ 用在哪里：阶段 3（split）中，把一条边上的切点(cut points)按沿线方向排序，
//   然后**依次相邻两点**切成小段（见本文件 std::sort(..., ProjectionCompare(e)) 处）。
//   不排序就无法正确地"沿边切分"。
//
// 参数：构造时传入参考边 m_e；operator() 接收两个待比较的点 a、b。
// 返回：a 在 b 之前（即 a 的投影更靠 m_e.p1 一侧）时为 true。
//
// 实现：用标量积 sprod 计算 a、b 相对 m_e.p1 的投影长度（沿 m_e 方向），
//       投影小者在前。投影相同时用点的字典序 (a < b) 打破平局 ——
//       ★ 这个兜底是必要的，否则排序不构成严格弱序，切分顺序会不确定。
//
// 坑：sprod 是整数运算，结果精确无浮点误差；但正因如此，它对"投影恰好相等"
//     的情形依赖 a < b 的确定性来保证可复现。
// [[ZH-END]]
struct ProjectionCompare
{
  // [[ZH]] 参数：e 参考边，投影沿 e 的方向（以 e.p1 为原点）度量。
  ProjectionCompare (const db::Edge &e)
    : m_e (e) { }

  // [[ZH]] 功能：比较 a、b 沿 m_e 的投影长度。返回 true 表示 a 更靠近 m_e.p1。
  bool operator () (const db::Point &a, const db::Point &b) const
  {
    // [[ZH]] sp1/sp2 = a、b 相对 m_e.p1 的投影（标量积），整数精确。
    db::coord_traits<db::Coord>::area_type sp1 = db::sprod (m_e, db::Edge (m_e.p1 (), a));
    db::coord_traits<db::Coord>::area_type sp2 = db::sprod (m_e, db::Edge (m_e.p1 (), b));
    if (sp1 != sp2) {
      // [[ZH]] 投影不同：小者在前。
      return sp1 < sp2;
    } else {
      // [[ZH]] 投影相同（例如两点对称）：用字典序兜底，保证严格弱序与结果确定。
      return a < b;
    }
  }

private:
  // [[ZH]] m_e：参考边，决定投影方向与原点。构造时固定，之后不变。
  db::Edge m_e;
};

// [[ZH-BEGIN]]
// ⚠ 死代码提示：PolyMapCompare 在**整个代码库中已无任何引用**（定义在此，无人使用）。
//   它是早期实现遗留的比较器，功能与现在在用的 EdgeXAtYCompare2 重叠。
//   阅读时可以直接跳过；若将来要清理无用代码，这是一个候选。
//   （用 grep "PolyMapCompare" 可自行确认只有本处定义。）
// [[ZH-END]]
struct PolyMapCompare
{
  PolyMapCompare (db::Coord y)
    : m_y (y) { }

  bool operator() (const std::pair<db::Edge, size_t> &a, const std::pair<db::Edge, size_t> &b) const
  {
    //  simple cases ..
    if (a.first.dx () == 0 && b.first.dx () == 0) {
      return a.first.p1 ().x () < b.first.p1 ().x ();
    } else if (edge_xmax (a.first) < edge_xmin (b.first)) {
      return true;
    } else if (edge_xmin (a.first) > edge_xmax (b.first)) {
      return false;
    } else {

      //  complex case:
      double xa = edge_xaty (a.first, m_y);
      double xb = edge_xaty (b.first, m_y);

      if (xa != xb) {
        return xa < xb;
      } else {

        //  compare by angle of normalized edges

        db::Edge ea (a.first);
        db::Edge eb (b.first);

        if (ea.dy () < 0) {
          ea.swap_points ();
        }
        if (eb.dy () < 0) {
          eb.swap_points ();
        }

        return db::vprod_sign (ea, eb) < 0;

      }
    }
  }

private:
  db::Coord m_y;
};

// [[ZH-BEGIN]]
// 功能：**精确**判断点 pt 是否在边 e 上（含端点，无容差）。
//
// ★ 与 db::edge::contains() 的区别（重要）：
//     db::edge::contains()  带 ±1 DBU 容差，是"模糊"判断。
//     本函数                 是**严格**判断 —— 一般位置的边要求叉积严格为 0。
//   布尔运算内部必须区分"真正共线"与"仅仅近似共线"，否则会把不该合并的边合并，
//   因此这里单独实现了精确版本。
//
// 参数：e 目标边；pt 待测点。
// 返回：true = 点严格落在边（线段）上。
//
// 实现：
//   ① 先用包围盒快速排除 —— 绝大多数点在此被排除，成本极低。
//   ② 正交边（水平/垂直）走捷径：既然①已确认点在包围盒内，
//      且边是水平或垂直的，那么点必然在线段上，直接返回 true（免去叉积）。
//   ③ 一般情形：叉积符号为 0 即共线（配合①的包围盒约束，共线即在线段上）。
//
// 坑：对一般（非正交）边，这里的"精确"仅指**共线判断精确**；
//     由于①的包围盒约束，判定的是"在线段上"而非"在无限直线上"。
// [[ZH-END]]
inline bool
is_point_on_exact (const db::Edge &e, const db::Point &pt)
{
  if (pt.x () < db::edge_xmin (e) || pt.x () > db::edge_xmax (e) ||
      pt.y () < db::edge_ymin (e) || pt.y () > db::edge_ymax (e)) {

    return false;

  } else if (e.dy () == 0 || e.dx () == 0) {

    //  shortcut for orthogonal edges
    return true;

  } else {

    return db::vprod_sign (pt - e.p1 (), e.p2 () - e.p1 ()) == 0;

  }
}

// [[ZH-BEGIN]]
// 功能：**模糊**判断点 pt 是否在边 e 上 —— 允许 ±1 DBU 的偏差。
//
// ★ 与 is_point_on_exact 的分工：
//   is_point_on_exact  → "真正共线"
//   is_point_on_fuzzy  → "近共线"，即点在边旁边 1 个网格单位以内也算"在边上"
//   阶段 2（求交）需要处理"看起来该相交、但整数网格上差了 1 DBU"的边，
//   此时用模糊判定把它们当作相交，"吸附"到一起，避免出现几乎重合却不相交的碎片。
//
// 参数：e 目标边；pt 待测点。
// 返回：true = 点在边的 ±1 DBU 邻域内（**不含端点**）。
//
// 实现：构造一个 ±1 的方向偏移向量 offset，比较"点到边的有向面积"与
//       "offset 到边的有向面积"的大小关系，从而判定点是否落在
//       边两侧 1 DBU 的**锥形邻域**内。
//       当边朝左上/右下时用 offset = (1,1)，否则用 (-1,1)，
//       以保证偏移方向指向边的"内侧"。
//
// 坑：1) ★ 本函数**排除端点**（开头就有 pt == e.p1() || pt == e.p2() 的检查）——
//        这与 is_point_on_exact 不同。若你把两者混用，会在端点处得到不一致的结论。
//     2) 这里的"1 DBU 锥形邻域"是**有方向性**的（依赖 offset 的选取），
//        不是严格的欧氏圆邻域 —— 这是为了配合整数网格的取舍而做的设计，
//        也是设计上最微妙的一处近似。
//     3) 正交边同样走捷径直接返回 true（因为包围盒已确认点在边上）。
// [[ZH-END]]
inline bool
is_point_on_fuzzy (const db::Edge &e, const db::Point &pt)
{
  //  exclude the start and end point
  if (pt == e.p1 () || pt == e.p2 ()) {

    return false;

  } else if (pt.x () < db::edge_xmin (e) || pt.x () > db::edge_xmax (e) ||
             pt.y () < db::edge_ymin (e) || pt.y () > db::edge_ymax (e)) {

    return false;

  } else if (e.dy () == 0 || e.dx () == 0) {

    //  shortcut for orthogonal edges
    return true;

  } else {

    bool with_equal = false;

    db::Vector offset;
    if ((e.dx () < 0 && e.dy () > 0) || (e.dx () > 0 && e.dy () < 0)) {
      offset = db::Vector (1, 1);
      with_equal = true;
    } else {
      offset = db::Vector (-1, 1);
    }

    db::Vector pp1 = pt - e.p1 ();

    typedef db::coord_traits<db::Point::coord_type>::area_type area_type;
    area_type a1 = 2 * db::vprod (pp1, e.d ());
    area_type a2 = db::vprod (offset, e.d ());

    if ((a1 < 0) == (a2 < 0)) {
      with_equal = false;
    }

    if (a1 < 0) { a1 = -a1; }
    if (a2 < 0) { a2 = -a2; }

    return a1 < a2 || (a1 == a2 && with_equal);

  }
}

//  A intersection test that is more robust numerically.
//  In some cases (i.e. (3,-3;-8,-1) and (-4,-2;13,-4)), the intersection test gives different results
//  for the intersection point if the edges are swapped. This test is robust since it operates on ordered edges.
// [[ZH-BEGIN]]
// 功能：求两边的交点，且**保证结果与参数顺序无关**。
//
// ★ 为什么需要这个包装（这是本文件最容易被低估的一个函数）：
//   db::edge::intersect_point() 的实现在某些输入下**不是对称的** ——
//   e1.intersect_point(e2) 与 e2.intersect_point(e1) 可能给出不同的坐标
//   （上面的英文注释举了具体例子：(3,-3;-8,-1) 与 (-4,-2;13,-4)）。
//   原因是内部走的是"以某条边为基准算叉积比值"的不对称公式。
//
//   后果：如果一个地方按 (e1,e2) 调用、另一个地方按 (e2,e1) 调用，
//   同一个交点会得到两个不同坐标 → 边在同一处被切成错位的两段 →
//   scanline 排序不稳定 → 布尔运算结果出现拓扑错误（且极难复现）。
//
//   解决：统一按**边的全序**（operator<）固定调用顺序。
//   这样无论调用者传参顺序如何，内部总是以同一个顺序求交，结果必然一致。
//
// 参数：e1、e2 两条边（顺序无关）。
// 返回：std::pair<bool, db::Point>，语义同 db::edge::intersect_point()。
//
// 用法约定：本文件（以及全库）在阶段 2 求交时**一律**通过本函数求交，
//           不要直接调用 intersect_point，以免破坏上述一致性保证。
// [[ZH-END]]
inline std::pair<bool, db::Point> safe_intersect_point (const db::Edge &e1, const db::Edge &e2)
{
  if (e1 < e2) {
    return e1.intersect_point (e2);
  } else {
    return e2.intersect_point (e1);
  }
}

/**
 *  @brief A structure for storing data in the first phase of the scanline algorithm
 *
 *  This data are the cut points (intersection points with other edges). The has_cutpoints
 *  flag indicates that the edge has cutpoints which the edge must follow. If the edge has strong
 *  cutpoints (strong_cutpoints), they will change the edge, hence non-cutpoints (attractors) must be
 *  included. 
 */
// [[ZH-BEGIN]]
// ============================================================================
//  CutPoints —— 一条边上的"待切分点"清单（阶段 1-3 的核心数据结构）
// ============================================================================
//
// 【背景】阶段 2 求交后，必须把每条边在交点处"打断"，使所有小段互不相交，
//   阶段 4 的扫描线才能按顺序处理。CutPoints 就是"某条边要在哪些点被打断"的记录。
//
// 【两种点：strong 与 weak(attractor)】★ 这是本结构最微妙的设计
//   cut_points    强切点：**确定**要在此处打断这条边。
//   attractors    弱切点（吸引点）：暂时只是"记下"，**先不打断**。
//                 每个元素是 (点, 另一条边的 CutPoints 下标)。
//
//   为什么需要 weak？考虑两条边 A、B 在整数网格下"几乎相交但差 1 DBU"：
//   求交时可能只能判定"B 的端点落在 A 的附近"，却无法确认交点。
//   如果立刻把 A 打断，会切断一段本不该断的边（产生碎片）；
//   如果完全不记，之后 B 真的被别的边打断时，A 这边就漏了一个切点。
//   于是采用"延迟决策"：先在 A 上记一个 attractor 指向 B。
//
// 【提升(promotion)机制】当 A 上出现了一个**强**切点（strong_cutpoints 变为 true），
//   说明 A 确实要被切分了；此时 `add()` 会把此前攒下的所有 attractors
//   **反向传播**回它们各自所属的那条边（用记录的下标找到对方），把它们提升为强切点。
//   这样两边的切分保持一致，不会出现"一边断了、另一边没断"的错位。
//
// 【两个标志位】
//   has_cutpoints    : 这条边是否至少记录过一个点（无论强弱）。
//   strong_cutpoints : 这条边是否已有**强**切点（即确定要被打断）。
//                      它决定 add_attractor() 是"直接成为切点"还是"先记为弱切点"。
//
// 注意：本结构只服务于阶段 1-3；阶段 4 会复用 WorkEdge::data 字段存别的东西。
// [[ZH-END]]
struct CutPoints
{
  // [[ZH]] cut_points：强切点列表 —— 这条边将在这些点处被切断。
  // [[ZH]]             阶段 3 会沿边方向排序后，在相邻两点间生成小段。
  std::vector <db::Point> cut_points;
  // [[ZH]] attractors：弱切点列表，元素为 (点, 对方边的 CutPoints 下标)。
  // [[ZH]]             记下"可能需要的切点"以及"该回溯通知谁"，
  // [[ZH]]             等本方出现强切点时再统一提升，见 add()。
  std::vector <std::pair<db::Point, size_t> > attractors;
  // [[ZH]] has_cutpoints：是否记录过任何点（强弱皆算）。
  // [[ZH]] 注意：它用位域 : 8 声明（占 8 bit），是历史写法，语义上就是一个 bool。
  bool has_cutpoints : 8;
  // [[ZH]] strong_cutpoints：是否已有强切点（即确定要被打断）。
  // [[ZH]] 决定 add_attractor() 的行为分支，见下。
  bool strong_cutpoints : 8;

  // [[ZH]] 功能：构造空的切点清单（两个标志位均为 false）。
  CutPoints ()
    : has_cutpoints (false), strong_cutpoints (false)
  { }

  // [[ZH-BEGIN]]
  // 功能：登记一个**弱**切点（attractor）。
  // 参数：p    待记录的点；
  //       next 对方边的 CutPoints 下标 —— 用于日后反向提升时找到对方。
  // 行为：· 若本方**已有强切点**（strong_cutpoints == true），
  //         说明本方确定要被打断，这个点应当直接成为真正的切点 → 进 cut_points。
  //       · 否则先存进 attractors，连同 next 下标，等待日后提升。
  // 副作用：可能修改 cut_points 或 attractors。
  // [[ZH-END]]
  void add_attractor (const db::Point &p, size_t next)
  {
    if (strong_cutpoints) {
      // [[ZH]] 本方已在切分状态 → 该点直接成为正式切点。
      cut_points.push_back (p);
    } else {
      // [[ZH]] 否则先"挂起"，记录对方下标，等本方出现强切点时再提升。
      attractors.push_back (std::make_pair (p, next));
    }
  }

  // [[ZH-BEGIN]]
  // 功能：登记一个切点 p，并（在必要时）触发弱切点的反向提升。
  //
  // 参数：p        切点坐标；
  //       cpvector 全体边的 CutPoints 数组 —— 提升时需要它按下标找到对方边；
  //       strong   该点是否为**强**切点（默认 true）。
  //
  // 副作用（★ 这是本函数的关键，不只是"加一个点"）：
  //   ① 置 has_cutpoints = true。
  //   ② 若这是本方的**第一个强切点**（strong && !strong_cutpoints）：
  //        置 strong_cutpoints = true，
  //        并把此前攒下的**所有 attractors 反向提升**为各自所属边的强切点 ——
  //        通过 attractor 里记录的下标，在 cpvector 中找到对方边并对其调用 add(...)。
  //        这样保证"一方确定切分"时，另一方对应的点也同步成为切分点，两边不错位。
  //        注意实现里先把 attractors 交换到局部变量 attr 再遍历，
  //        以**避免边遍历边修改自身容器**导致迭代器失效/无限递归。
  //   ③ 去重：若 cut_points 中已存在同坐标的点，直接返回（不重复插入）。
  //        这一步是必要的，否则重复切点会产生零长度碎片边。
  //
  // 坑：提升是**递归**的（对方边 add 时也可能触发它自己的提升），
  //     但因③的去重与②的"仅首个强切点触发"两个条件，递归会收敛。
  // [[ZH-END]]
  void add (const db::Point &p, std::vector <CutPoints> *cpvector, bool strong = true)
  {
    has_cutpoints = true;
    if (strong && !strong_cutpoints) {

      strong_cutpoints = true;
      if (! attractors.empty ()) {

        // [[ZH]] 先整体取出，避免遍历 attractors 的同时又修改它（自我修改风险）。
        std::vector <std::pair<db::Point, size_t> > attr;
        attractors.swap (attr);

        cut_points.reserve (cut_points.size () + attr.size ());
        // [[ZH]] 反向提升：按记录的下标找到对方边的 CutPoints，把点也加入对方。
        for (std::vector <std::pair<db::Point, size_t> >::const_iterator a = attr.begin (); a != attr.end (); ++a) {
          (*cpvector) [a->second].add (a->first, cpvector, true);
        }

      }

    } 

    //  do not insert points twice
    // [[ZH]] ③ 去重：同坐标的切点只保留一个，避免产生零长度碎片边。
    for (auto c = cut_points.begin (); c != cut_points.end (); ++c) {
      if (*c == p) {
        return;
      }
    }

    cut_points.push_back (p);

  }

};

/**
 *  @brief A data object for the scanline algorithm
 */
// [[ZH-BEGIN]]
// 功能：算法内部使用的边记录 —— 在 db::Edge（两个端点）之上挂了两项附加数据。
//       阶段 2、3、4 处理的都是 WorkEdge 而不是裸的 db::Edge。
//
// 继承关系：public db::Edge，因此所有边的几何操作（dx/dy/swap_points/
//           intersect_point/operator< ...）都可以直接调用，无需转换。
//
// 【★ 两个附加字段，注意 data 的含义会"换岗"】
//   prop : 这条边的来源标签（property）。全生命周期含义固定，
//          由 insert() 时传入，评估器靠它区分不同输入多边形/图层。
//
//   data : ★ 含义随阶段变化，这是本结构最容易读错的地方：
//            · 阶段 1-3：存放本边 CutPoints 在 mp_cpvector 中的**下标 + 1**
//                        （0 表示"尚无 CutPoints"，故用 +1 让 0 可表示"无"）。
//                        由 make_cutpoints() 惰性分配。
//            · 阶段 4  ：改存 skip 优化信息的下标（SkipInfo 的条目号）。
//            所以不能跨阶段假设 data 的语义；阶段 4 开始前代码里还有
//            `tl_assert (future->data == 0)` 之类的断言来确认这一点。
//
// 【为什么用"下标 + 1"而不是指针】
//   CutPoints 存放在 std::vector 中，扩容会使其中的元素地址失效。
//   用下标可以安全地在 vector 增长后重新寻址；且 vector 紧凑、缓存友好。
//
// 坑：拷贝构造与赋值**都被显式实现**了 —— 因为默认的成员逐一拷贝对本类是正确的，
//     但作者仍显式写出以明确意图（并确保 data/prop 一并复制）。
//     注意 operator= (const db::Edge &) 的重载**只复制几何**，
//     会把 data/prop 留在原值 —— 这是刻意的（用于仅替换几何的场合）。
// [[ZH-END]]
struct WorkEdge
  : public db::Edge
{
  // [[ZH]] 功能：默认构造 —— 退化的零长度边，data = 0（无 CutPoints），prop = 0。
  WorkEdge () 
    : db::Edge (), data (0), prop (0)
  { }

  // [[ZH]] 功能：由几何边 + 来源标签构造。
  // [[ZH]] 参数：e 几何；p 来源标签(property)；d 初始 data（通常是 CutPoints 下标+1，默认 0=无）。
  WorkEdge (const db::Edge &e, EdgeProcessor::property_type p = 0, size_t d = 0) 
    : db::Edge (e), data (d), prop (p)
  { }

  // [[ZH]] 功能：拷贝构造 —— 几何、data、prop 全部复制。
  WorkEdge (const WorkEdge &d)
    : db::Edge (d), data (d.data), prop (d.prop)
  { }

  // [[ZH]] 功能：赋值 —— 复制全部三项。带自赋值检查。
  WorkEdge &operator= (const WorkEdge &d)
  { 
    if (this != &d) {
      db::Edge::operator= (d);
      data = d.data;
      prop = d.prop;
    }
    return *this;
  }

  // [[ZH]] 功能：**只**替换几何部分，保留 data / prop 不变。
  // [[ZH]] 用途：当只想更新边的位置（例如打断后替换成某一段）而保留其
  // [[ZH]]       来源标签与切点关联时使用。
  WorkEdge &operator= (const db::Edge &d)
  { 
    db::Edge::operator= (d);
    return *this;
  }

  // [[ZH-BEGIN]]
  // 功能：取得本边对应的 CutPoints 对象；若尚未分配则**惰性分配**一个。
  //
  // 参数：cutpoints 全局的 CutPoints 数组（每条边一项，惰性增长）。
  // 返回：指向本边 CutPoints 的**指针**。
  //
  // 实现：data == 0 表示"尚未分配" → 向 cutpoints 追加一个空 CutPoints，
  //       并把 `下标 + 1` 存进 data（+1 是为了让 0 能表示"无"）。
  //       data != 0 时直接用 data - 1 索引。
  //
  // 注意：返回的是 vector 内部元素的指针 —— 若之后 cutpoints 再次扩容，
  //       该指针可能失效。因此调用者应在拿到的指针上**立即**使用，
  //       不要长期持有（这也是本文件在阶段 3 尽快用掉它的原因）。
  // [[ZH-END]]
  CutPoints *make_cutpoints (std::vector <CutPoints> &cutpoints)
  {
    if (! data) {
      // [[ZH]] 惰性分配：追加一项，并记录 下标+1（使 0 可表示"无"）。
      cutpoints.push_back (CutPoints ());
      data = cutpoints.size ();
    }
    return &cutpoints [data - 1];
  }

  // [[ZH]] data：阶段 1-3 = CutPoints 下标+1（0 = 无）；阶段 4 = skip 信息下标。见上面说明。
  size_t data;
  // [[ZH]] prop：边的来源标签(property)，全生命周期语义固定，供评估器区分不同输入。
  db::EdgeProcessor::property_type prop;
};

/**
 *  @brief Compare edges by property ID
 */
struct EdgePropCompare
{
  bool operator() (const db::WorkEdge &a, const db::WorkEdge &b) const
  {
    return a.prop < b.prop;
  }
};

/**
 *  @brief Compare edges by property ID (reverse)
 */
struct EdgePropCompareReverse
{
  bool operator() (const db::WorkEdge &a, const db::WorkEdge &b) const
  {
    return b.prop < a.prop;
  }
};

/**
 *  @brief A compare operator for edged
 *  This operator will compare edges by their x position on the scanline
 */
struct EdgeXAtYCompare
{
  EdgeXAtYCompare (db::Coord y)
    : m_y (y) { }

  bool operator() (const db::Edge &a, const db::Edge &b) const
  {
    //  simple cases ..
    if (a.dx () == 0 && b.dx () == 0) {
      return a.p1 ().x () < b.p1 ().x ();
    } else if (edge_xmax (a) < edge_xmin (b)) {
      return true;
    } else if (edge_xmin (a) > edge_xmax (b)) {
      return false;
    } else {

      //  complex case:
      //  HINT: "volatile" forces xa and xb into memory and disables FPU register optimisation.
      //  That way, we can exactly compare doubles afterwards.
      volatile double xa = edge_xaty (a, m_y);
      volatile double xb = edge_xaty (b, m_y);

      if (xa != xb) {
        return xa < xb;
      } else if (a.dy () == 0) {
        return false;
      } else if (b.dy () == 0) {
        return true;
      } else {

        //  compare by angle of normalized edges

        db::Edge ea (a);
        db::Edge eb (b);

        if (ea.dy () < 0) {
          ea.swap_points ();
        }
        if (eb.dy () < 0) {
          eb.swap_points ();
        }

        return db::vprod_sign (ea, eb) < 0;

      }
    }
  }

  bool equal (const db::Edge &a, const db::Edge &b) const
  {
    //  simple cases ..
    if (a.dx () == 0 && b.dx () == 0) {
      return a.p1 ().x () == b.p1 ().x ();
    } else if (edge_xmax (a) < edge_xmin (b)) {
      return false;
    } else if (edge_xmin (a) > edge_xmax (b)) {
      return false;
    } else {

      //  complex case:
      //  HINT: "volatile" forces xa and xb into memory and disables FPU register optimisation.
      //  That way, we can exactly compare doubles afterwards.
      volatile double xa = edge_xaty (a, m_y);
      volatile double xb = edge_xaty (b, m_y);

      if (xa != xb) {
        return false;
      } else if (a.dy () == 0 || b.dy () == 0) {
        return (a.dy () == 0) == (b.dy () == 0);
      } else {

        //  compare by angle of normalized edges

        db::Edge ea (a);
        db::Edge eb (b);

        if (ea.dy () < 0) {
          ea.swap_points ();
        }
        if (eb.dy () < 0) {
          eb.swap_points ();
        }

        return db::vprod_sign (ea, eb) == 0;

      }
    }
  }

private:
  db::Coord m_y;
};

/**
 *  @brief A compare operator for edges used within EdgeXAtYCompare2
 *  This operator is an extension of db::edge_xaty and will deliver
 *  the minimum x if the edge is horizontal.
 */
static inline double edge_xaty2 (db::Edge e, db::Coord y)
{
  if (e.p1 ().y () > e.p2 ().y ()) {
    e.swap_points ();
  }

  if (y <= e.p1 ().y ()) {
    if (y == e.p2 ().y ()) {
      return std::min (e.p1 ().x (), e.p2 ().x ());
    } else {
      return e.p1 ().x ();
    }
  } else if (y >= e.p2 ().y ()) {
    return e.p2 ().x ();
  } else {
    return double (e.p1 ().x ()) + double (e.dx ()) * double (y - e.p1 ().y ()) / double (e.dy ());
  }
}

/**
 *  @brief A compare operator for edges
 *  This operator will compare edges by their x position on the scanline
 *  In Addition to EdgeXAtYCompare, this operator will also compare the 
 *  direction of the edges. Edges are equal if the position at which they cross
 *  the scanline and their direction is identical.
 */
struct EdgeXAtYCompare2
{
  EdgeXAtYCompare2 (db::Coord y)
    : m_y (y) { }

  bool operator() (const db::Edge &a, const db::Edge &b) const
  {
    //  simple cases ..
    if (a.dx () == 0 && b.dx () == 0) {
      return a.p1 ().x () < b.p1 ().x ();
    } else if (edge_xmax (a) < edge_xmin (b)) {
      return true;
    } else if (edge_xmin (a) > edge_xmax (b)) {
      return false;
    } else {

      //  complex case:
      //  HINT: "volatile" forces xa and xb into memory and disables FPU register optimisation.
      //  That way, we can exactly compare doubles afterwards.
      volatile double xa = edge_xaty2 (a, m_y);
      volatile double xb = edge_xaty2 (b, m_y);

      if (xa != xb) {
        return xa < xb;
      } else if (a.dy () == 0) {
        return false;
      } else if (b.dy () == 0) {
        return true;
      } else {

        //  In that case the edges will not intersect but rather touch in one point. This defines
        //  a sorting which preserves the scanline order of the edges when y advances.

        db::Edge ea (a);
        db::Edge eb (b);

        if (ea.dy () < 0) {
          ea.swap_points ();
        }
        if (eb.dy () < 0) {
          eb.swap_points ();
        }

        bool fa = ea.p2 ().y () > m_y;
        bool fb = eb.p2 ().y () > m_y;

        if (fa && fb) {
          //  Both edges advance
          return db::vprod_sign (ea, eb) < 0;
        } else if (fa || fb) {
          //  Only one edge advances - equality
          return false;
        } else {
          return db::vprod_sign (ea, eb) > 0;
        }

      }
    }
  }

  bool equal (const db::Edge &a, const db::Edge &b) const
  {
    //  simple cases ..
    if (a.dx () == 0 && b.dx () == 0) {
      return a.p1 ().x () == b.p1 ().x ();
    } else if (edge_xmax (a) < edge_xmin (b)) {
      return false;
    } else if (edge_xmin (a) > edge_xmax (b)) {
      return false;
    } else {

      //  complex case:
      //  HINT: "volatile" forces xa and xb into memory and disables FPU register optimisation.
      //  That way, we can exactly compare doubles afterwards.
      volatile double xa = edge_xaty2 (a, m_y);
      volatile double xb = edge_xaty2 (b, m_y);

      if (xa != xb) {
        return false;
      } else if (a.dy () == 0 || b.dy () == 0) {
        return (a.dy () == 0) == (b.dy () == 0);
      } else {

        //  compare by angle of normalized edges

        db::Edge ea (a);
        db::Edge eb (b);

        if (ea.dy () < 0) {
          ea.swap_points ();
        }
        if (eb.dy () < 0) {
          eb.swap_points ();
        }

        return db::vprod_sign (ea, eb) == 0;

      }
    }
  }

private:
  db::Coord m_y;
};

// -------------------------------------------------------------------------------
//  EdgePolygonOp implementation

EdgePolygonOp::EdgePolygonOp (EdgePolygonOp::mode_t mode, bool include_touching, int polygon_mode)
  : m_mode (mode), m_include_touching (include_touching),
    m_function (polygon_mode),
    m_wcp_n (0), m_wcp_s (0)
{
}

void EdgePolygonOp::reset () 
{ 
  m_wcp_n = m_wcp_s = 0;
}

int EdgePolygonOp::select_edge (bool horizontal, property_type p)
{
  if (p == 0) {
    return 0;
  }

  bool inside;

  if (horizontal) {
    if (m_include_touching) {
      inside = (m_function (m_wcp_n) || m_function (m_wcp_s));
    } else {
      inside = (m_function (m_wcp_n) && m_function (m_wcp_s));
    }
  } else {
    inside = m_function (m_wcp_n);
  }

  if (m_mode == Inside) {
    return inside ? 1 : 0;
  } else if (m_mode == Outside) {
    return inside ? 0 : 1;
  } else {
    return inside ? 1 : 2;
  }
}

int EdgePolygonOp::edge (bool north, bool enter, property_type p) 
{ 
  if (p == 0) {
    int *wc = north ? &m_wcp_n : &m_wcp_s;
    if (enter) {
      ++*wc;
    } else {
      --*wc;
    }
  }

  return 0; 
}

bool EdgePolygonOp::is_reset () const 
{ 
  return (m_wcp_n == 0 && m_wcp_s == 0);
}

bool EdgePolygonOp::prefer_touch () const 
{ 
  return m_include_touching; 
}

bool EdgePolygonOp::selects_edges () const 
{ 
  return true; 
}

// -------------------------------------------------------------------------------
//  InteractionDetector implementation

InteractionDetector::InteractionDetector (int mode, property_type primary_id)
  : m_mode (mode), m_include_touching (true), m_last_primary_id (primary_id)
{
  // .. nothing yet ..
}

void
InteractionDetector::reset ()
{
  m_wcv_n.clear ();
  m_wcv_s.clear ();
  m_inside_n.clear ();
  m_inside_s.clear ();
}

void 
InteractionDetector::reserve (size_t n)
{
  m_wcv_n.clear ();
  m_wcv_s.clear ();
  m_wcv_n.resize (n, 0);
  m_wcv_s.resize (n, 0);
  m_inside_n.clear ();
  m_inside_s.clear ();
}

int 
InteractionDetector::edge (bool north, bool enter, property_type p)
{
  tl_assert (p < m_wcv_n.size () && p < m_wcv_s.size ());

  int *wcv = north ? &m_wcv_n [p] : &m_wcv_s [p];

  bool inside_before = (*wcv != 0);
  *wcv += (enter ? 1 : -1);
  bool inside_after = (*wcv != 0);

  //  In "interacting" and "enclosing" mode we need to handle both north and south events because
  //  we have to catch interactions between objects north and south to the scanline
  if (north || (m_mode == 0 && m_include_touching) || (m_mode < -1 && m_include_touching)) {

    std::set <property_type> *inside = north ? &m_inside_n : &m_inside_s;

    if (inside_after < inside_before) {

      inside->erase (p);

      //  the primary objects are delivered last of all coincident edges
      //  (due to prefer_touch == true and the sorting of coincident edges by property id)
      //  hence every remaining parts count as non-interacting (outside)
      if (p <= m_last_primary_id) {
        for (std::set <property_type>::const_iterator i = inside->begin (); i != inside->end (); ++i) {
          if (*i > m_last_primary_id) {
            m_non_interactions.insert (*i);
          }
        }
      }

    } else if (inside_after > inside_before) {

      if (m_mode != 0) {

        //  enclosing/inside/outside mode
        if (p > m_last_primary_id) {

          //  note that the primary parts will be delivered first of all coincident
          //  edges hence we can check whether the primary is present even for coincident
          //  edges
          bool any = false;
          for (std::set <property_type>::const_iterator i = inside->begin (); i != inside->end (); ++i) {
            if (*i <= m_last_primary_id) {
              any = true;
              m_interactions.insert (std::make_pair (*i, p));
            }
          }
          if (! any) {
            m_non_interactions.insert (p);
          }

        } else {

          for (std::set <property_type>::const_iterator i = inside->begin (); i != inside->end (); ++i) {
            if (*i > m_last_primary_id) {
              if (m_mode < -1) {
                //  enclosing mode: an opening primary (= enclosing one) with open secondaries means the secondary
                //  has been opened before and did not close. Because we sort by property ID this must have happened
                //  before, hence the secondary is overlapping. Make them non-interactions. We still have to record them
                //  as interactions because this is how we skip the primaries later.
                m_non_interactions.insert (*i);
              }
              m_interactions.insert (std::make_pair (p, *i));
            }
          }

        }

      } else {

        for (std::set <property_type>::const_iterator i = m_inside_n.begin (); i != m_inside_n.end (); ++i) {
          if (*i < p) {
            m_interactions.insert (std::make_pair (*i, p));
          } else if (p < *i) {
            m_interactions.insert (std::make_pair (p, *i));
          }
        }

        for (std::set <property_type>::const_iterator i = m_inside_s.begin (); i != m_inside_s.end (); ++i) {
          if (*i < p) {
            m_interactions.insert (std::make_pair (*i, p));
          } else if (p < *i) {
            m_interactions.insert (std::make_pair (p, *i));
          }
        }

      }

      inside->insert (p);

    }

  }

  return 0;
}

int 
InteractionDetector::compare_ns () const
{
  return 0;
}

void
InteractionDetector::finish ()
{
  if (m_mode < -1) {

    //  In enclosing mode remove those objects which have an interaction with a secondary having a non-interaction:
    //  these are the ones where secondaries overlap and stick to the outside.
    std::set<property_type> primaries_to_delete;
    for (std::set<std::pair<property_type, property_type> >::iterator i = m_interactions.begin (); i != m_interactions.end (); ++i) {
      if (m_non_interactions.find (i->second) != m_non_interactions.end ()) {
        primaries_to_delete.insert (i->first);
      }
    }

    for (std::set<std::pair<property_type, property_type> >::iterator i = m_interactions.begin (); i != m_interactions.end (); ) {
      std::set<std::pair<property_type, property_type> >::iterator ii = i;
      ++ii;
      if (primaries_to_delete.find (i->first) != primaries_to_delete.end ()) {
        m_interactions.erase (i);
      }
      i = ii;
    }

  } else if (m_mode == -1) {

    //  In inside mode remove those objects which have a non-interaction with a primary
    for (std::set<std::pair<property_type, property_type> >::iterator i = m_interactions.begin (); i != m_interactions.end (); ) {
      std::set<std::pair<property_type, property_type> >::iterator ii = i;
      ++ii;
      if (m_non_interactions.find (i->second) != m_non_interactions.end ()) {
        m_interactions.erase (i);
      }
      i = ii;
    }

  } else if (m_mode > 0) {

    //  In outside mode leave those objects which don't participate in an interaction
    for (iterator pp = begin (); pp != end (); ++pp) {
      m_non_interactions.erase (pp->second);
    }

    m_interactions.clear ();
    for (std::set<property_type>::const_iterator p = m_non_interactions.begin (); p != m_non_interactions.end (); ++p) {
      m_interactions.insert (m_interactions.end (), std::make_pair (m_last_primary_id, *p));
    }

  }

  m_non_interactions.clear ();
}

// -------------------------------------------------------------------------------
//  MergeOp implementation

MergeOp::MergeOp (unsigned int min_wc)
  : m_wc_n (0), m_wc_s (0), m_min_wc (min_wc), m_zeroes (0)
{
  //  .. nothing yet ..
}

void  
MergeOp::reset ()
{
  m_wcv_n.clear ();
  m_wcv_s.clear ();
  m_wc_n = 0;
  m_wc_n = 0;
  m_zeroes = 0;
}

void 
MergeOp::reserve (size_t n)
{
  m_wcv_n.clear ();
  m_wcv_s.clear ();
  m_wcv_n.resize (n, 0);
  m_wcv_s.resize (n, 0);
  m_zeroes = 2 * n;
}

static inline 
bool result_by_mode (int wc, unsigned int min_wc)
{
  return wc > int (min_wc);
}

int 
MergeOp::edge (bool north, bool enter, property_type p)
{
  tl_assert (p < m_wcv_n.size () && p < m_wcv_s.size ());

  int *wcv = north ? &m_wcv_n [p] : &m_wcv_s [p];
  int *wc = north ? &m_wc_n : &m_wc_s;

  bool inside_before = (*wcv != 0);
  *wcv += (enter ? 1 : -1);
  bool inside_after = (*wcv != 0);
  m_zeroes += (!inside_after) - (!inside_before);
#ifdef DEBUG_MERGEOP
  printf ("north=%d, enter=%d, prop=%d -> %d\n", north, enter, int (p), int (m_zeroes));
#endif
  tl_assert (long (m_zeroes) >= 0);

  bool res_before = result_by_mode (*wc, m_min_wc);
  if (inside_before != inside_after) {
    *wc += (inside_after - inside_before);
  }
  bool res_after = result_by_mode (*wc, m_min_wc);

  return res_after - res_before;
}

int 
MergeOp::compare_ns () const
{
  return result_by_mode (m_wc_n, m_min_wc) - result_by_mode (m_wc_s, m_min_wc);
}

// -------------------------------------------------------------------------------
//  BooleanOp implementation

BooleanOp::BooleanOp (BoolOp mode)
  : m_wc_na (0), m_wc_nb (0), m_wc_sa (0), m_wc_sb (0), m_mode (mode), m_zeroes (0)
{
  //  .. nothing yet ..
}

void  
BooleanOp::reset ()
{
  m_wcv_n.clear ();
  m_wcv_s.clear ();
  m_wc_na = m_wc_sa = 0;
  m_wc_nb = m_wc_sb = 0;
  m_zeroes = 0;
}

void 
BooleanOp::reserve (size_t n)
{
  m_wcv_n.clear ();
  m_wcv_s.clear ();
  m_wcv_n.resize (n, 0);
  m_wcv_s.resize (n, 0);
  m_zeroes = 2 * n;
}

template <class InsideFunc>
inline bool 
BooleanOp::result (int wca, int wcb, const InsideFunc &inside_a, const InsideFunc &inside_b) const
{
  switch (m_mode) {
  case BooleanOp::And:
    return inside_a (wca) && inside_b (wcb);
  case BooleanOp::ANotB:
    return inside_a (wca) && ! inside_b (wcb);
  case BooleanOp::BNotA:
    return ! inside_a (wca) && inside_b (wcb);
  case BooleanOp::Xor:
    return (inside_a (wca) && ! inside_b (wcb)) || (! inside_a (wca) && inside_b (wcb));
  case BooleanOp::Or:
    return inside_a (wca) || inside_b (wcb);
  default:
    return false;
  }
}

template <class InsideFunc>
inline int 
BooleanOp::edge_impl (bool north, bool enter, property_type p, const InsideFunc &inside_a, const InsideFunc &inside_b) 
{
  tl_assert (p < m_wcv_n.size () && p < m_wcv_s.size ());

  int *wcv = north ? &m_wcv_n [p] : &m_wcv_s [p];
  int *wca = north ? &m_wc_na : &m_wc_sa;
  int *wcb = north ? &m_wc_nb : &m_wc_sb;

  bool inside_before = ((p % 2) == 0 ? inside_a (*wcv) : inside_b (*wcv));
  *wcv += (enter ? 1 : -1);
  bool inside_after = ((p % 2) == 0 ? inside_a (*wcv) : inside_b (*wcv));
  m_zeroes += (!inside_after) - (!inside_before);
#ifdef DEBUG_BOOLEAN
  printf ("north=%d, enter=%d, prop=%d -> %d\n", north, enter, int (p), int (m_zeroes));
#endif
  tl_assert (long (m_zeroes) >= 0);

  bool res_before = result (*wca, *wcb, inside_a, inside_b);
  if (inside_before != inside_after) {
    if ((p % 2) == 0) {
      *wca += (inside_after - inside_before);
    } else {
      *wcb += (inside_after - inside_before);
    }
  }
  bool res_after = result (*wca, *wcb, inside_a, inside_b);

  return res_after - res_before;
}

template <class InsideFunc> 
inline int 
BooleanOp::compare_ns_impl (const InsideFunc &inside_a, const InsideFunc &inside_b) const
{
  return result (m_wc_na, m_wc_nb, inside_a, inside_b) - result (m_wc_sa, m_wc_sb, inside_a, inside_b);
}

int 
BooleanOp::edge (bool north, bool enter, property_type p)
{
  NonZeroInsideFunc inside;
  return edge_impl (north, enter, p, inside, inside);
}

int 
BooleanOp::compare_ns () const
{
  NonZeroInsideFunc inside;
  return compare_ns_impl (inside, inside);
}

// -------------------------------------------------------------------------------
//  BooleanOp2 implementation

BooleanOp2::BooleanOp2 (BoolOp op, int wc_mode_a, int wc_mode_b)
  : BooleanOp (op), m_wc_mode_a (wc_mode_a), m_wc_mode_b (wc_mode_b)
{
  //  .. nothing yet ..
}

int 
BooleanOp2::edge (bool north, bool enter, property_type p)
{
  ParametrizedInsideFunc inside_a (m_wc_mode_a);
  ParametrizedInsideFunc inside_b (m_wc_mode_b);
  return edge_impl (north, enter, p, inside_a, inside_b);
}

int 
BooleanOp2::compare_ns () const
{
  ParametrizedInsideFunc inside_a (m_wc_mode_a);
  ParametrizedInsideFunc inside_b (m_wc_mode_b);
  return compare_ns_impl (inside_a, inside_b);
}

// -------------------------------------------------------------------------------
//  EdgeProcessor implementation

EdgeProcessor::EdgeProcessor (bool report_progress, const std::string &progress_desc)
  : m_report_progress (report_progress), m_progress_desc (progress_desc), m_base_verbosity (30)
{
  mp_work_edges = new std::vector <WorkEdge> ();
  mp_cpvector = new std::vector <CutPoints> ();
}

EdgeProcessor::~EdgeProcessor ()
{
  if (mp_work_edges) {
    delete mp_work_edges;
    mp_work_edges = 0;
  }
  if (mp_cpvector) {
    delete mp_cpvector;
    mp_cpvector = 0;
  }
}

void 
EdgeProcessor::disable_progress ()
{
  m_report_progress = false;
}

void 
EdgeProcessor::enable_progress (const std::string &progress_desc)
{
  m_report_progress = true;
  m_progress_desc = progress_desc;
}

void
EdgeProcessor::set_base_verbosity (int bv)
{
  m_base_verbosity = bv;
}

void 
EdgeProcessor::reserve (size_t n)
{
  mp_work_edges->reserve (n);
}

size_t
EdgeProcessor::count () const
{
  return mp_work_edges->size ();
}

void
EdgeProcessor::insert (const db::Edge &e, property_type p)
{
  if (e.p1 () != e.p2 ()) {
    mp_work_edges->push_back (WorkEdge (e, p));
  }
}

void
EdgeProcessor::clear ()
{
  mp_work_edges->clear ();
  mp_cpvector->clear ();
}

static void
add_hparallel_cutpoints (WorkEdge &e1, WorkEdge &e2, const db::Box &cell, std::vector <CutPoints> &cutpoints)
{
  db::Coord e1_xmin = std::min (e1.x1 (), e1.x2 ());
  db::Coord e1_xmax = std::max (e1.x1 (), e1.x2 ());
  if (e2.x1 () > e1_xmin && e2.x1 () < e1_xmax && cell.contains (e2.p1 ())) {
    e1.make_cutpoints (cutpoints)->add (e2.p1 (), &cutpoints, false);
  }
  if (e2.x2 () > e1_xmin && e2.x2 () < e1_xmax && cell.contains (e2.p2 ())) {
    e1.make_cutpoints (cutpoints)->add (e2.p2 (), &cutpoints, false);
  }
}

static void
get_intersections_per_band_90 (std::vector <CutPoints> &cutpoints, std::vector <WorkEdge>::iterator current, std::vector <WorkEdge>::iterator future, db::Coord y, db::Coord yy, bool with_h)
{
  std::sort (current, future, edge_xmin_compare<db::Coord> ());

#ifdef DEBUG_EDGE_PROCESSOR
  printf ("y=%d..%d (90 degree)\n", y, yy);
  printf ("edges:"); 
  for (std::vector <WorkEdge>::iterator c1 = current; c1 != future; ++c1) { 
    printf (" %s", c1->to_string().c_str ()); 
  } 
  printf ("\n");
#endif
  db::Coord x = edge_xmin (*current);

  std::vector <WorkEdge>::iterator f = current;
  for (std::vector <WorkEdge>::iterator c = current; c != future; ) {

    size_t n = 0;
    db::Coord xx = x;

    //  fetch as many cells as to fill in roughly 50% more
    //  (this is an empirical performance improvement factor)
    do {

      while (f != future && edge_xmin (*f) <= xx) {
        ++f;
      }

      if (f != future) {
        xx = edge_xmin (*f);
      } else {
        xx = std::numeric_limits <db::Coord>::max ();
      }

      if (n == 0) {
        n = std::distance (c, f);
      }

    } while (f != future && std::distance (c, f) < long (n * fill_factor));

#ifdef DEBUG_EDGE_PROCESSOR
    printf ("edges %d..%d:", x, xx); 
    for (std::vector <WorkEdge>::iterator c1 = c; c1 != f; ++c1) { 
      printf (" %s", c1->to_string().c_str ()); 
    } 
    printf ("\n");
#endif

    if (std::distance (c, f) > 1) {

      db::Box cell (x, y, xx, yy);

      for (std::vector <WorkEdge>::iterator c1 = c; c1 != f; ++c1) {

        bool c1p1_in_cell = cell.contains (c1->p1 ());
        bool c1p2_in_cell = cell.contains (c1->p2 ());

        for (std::vector <WorkEdge>::iterator c2 = c; c2 != f; ++c2) {

          if (c1 == c2) {
            continue;
          }

          if (c2->dy () == 0) {

            if ((with_h || c1->dy () != 0) && c1 < c2) {

              if (c1->dy () == 0) {

                //  parallel horizontal edges: produce the end points of each other edge as cutpoints
                if (c1->p1 ().y () == c2->p1 ().y ()) {
                  add_hparallel_cutpoints (*c1, *c2, cell, cutpoints);
                  add_hparallel_cutpoints (*c2, *c1, cell, cutpoints);
                }

              } else if (c1->p1 () != c2->p1 () && c1->p2 () != c2->p1 () &&
                         c1->p1 () != c2->p2 () && c1->p2 () != c2->p2 ()) {

                std::pair <bool, db::Point> cp = c1->intersect_point (*c2);
                if (cp.first && cell.contains (cp.second)) {

                  //  add a cut point to c1 and c2 (c2 only if necessary)
                  c1->make_cutpoints (cutpoints)->add (cp.second, &cutpoints, true);
                  if (with_h) {
                    c2->make_cutpoints (cutpoints)->add (cp.second, &cutpoints, true);
                  }

#ifdef DEBUG_EDGE_PROCESSOR
                  printf ("intersection point %s between %s and %s (1).\n", cp.second.to_string ().c_str (), c1->to_string ().c_str (), c2->to_string ().c_str ());
#endif

                }

              }

            } 
          
          } else if (c1->dy () == 0) {
            
            if (c1 < c2 && c1->p1 () != c2->p1 () && c1->p2 () != c2->p1 () &&
                           c1->p1 () != c2->p2 () && c1->p2 () != c2->p2 ()) {

              std::pair <bool, db::Point> cp = c1->intersect_point (*c2);
              if (cp.first && cell.contains (cp.second)) {
                
                //  add a cut point to c1 and c2
                c2->make_cutpoints (cutpoints)->add (cp.second, &cutpoints, true);
                if (with_h) {
                  c1->make_cutpoints (cutpoints)->add (cp.second, &cutpoints, true);
                }

#ifdef DEBUG_EDGE_PROCESSOR
                printf ("intersection point %s between %s and %s (2).\n", cp.second.to_string ().c_str (), c1->to_string ().c_str (), c2->to_string ().c_str ()); 
#endif

              }

            }

          } else if (c1->p1 ().x () == c2->p1 ().x ()) {

            //  both edges are coincident - produce the ends of the edges involved as cut points
            if (c1p1_in_cell && c1->p1 ().y () > db::edge_ymin (*c2) && c1->p1 ().y () < db::edge_ymax (*c2)) {
              c2->make_cutpoints (cutpoints)->add (c1->p1 (), &cutpoints, true);
            }
            if (c1p2_in_cell && c1->p2 ().y () > db::edge_ymin (*c2) && c1->p2 ().y () < db::edge_ymax (*c2)) {
              c2->make_cutpoints (cutpoints)->add (c1->p2 (), &cutpoints, true);
            }

          }

        }

      }

    }

    x = xx;
    for (std::vector <WorkEdge>::iterator cc = c; cc != f; ++cc) {
      if (edge_xmax (*cc) < x) {
        if (c != cc) {
          std::swap (*cc, *c);
        }
        ++c;
      }
    }

  }
}

/**
 *  @brief Computes the x value of an edge at the given y value
 *
 *  HINT: for application in the scanline algorithm 
 *  it is important that this method delivers exactly (!) the same x for the same edge 
 *  (after normalization to dy()>0) and same y!
 */
template <class C>
inline double edge_xaty_double (db::edge<C> e, double y)
{
  if (e.p1 ().y () > e.p2 ().y ()) {
    e.swap_points ();
  }

  if (y <= e.p1 ().y ()) {
    return e.p1 ().x ();
  } else if (y >= e.p2 ().y ()) {
    return e.p2 ().x ();
  } else {
    return double (e.p1 ().x ()) + double (e.dx ()) * (y - double (e.p1 ().y ())) / double (e.dy ());
  }
}

/**
 *  @brief Computes the left bound of the edge geometry for a given band [y1..y2].
 */
template <class C>
inline C edge_xmin_at_yinterval_double (const db::edge<C> &e, double y1, double y2) 
{
  if (e.dx () == 0) {
    return e.p1 ().x ();
  } else if (e.dy () == 0) {
    return std::min (e.p1 ().x (), e.p2 ().x ());
  } else {
    return C (floor (edge_xaty_double (e, ((e.dy () < 0) ^ (e.dx () < 0)) == 0 ? y1 : y2)));
  }
}

/**
 *  @brief Computes the right bound of the edge geometry for a given band [y1..y2].
 */
template <class C>
inline C edge_xmax_at_yinterval_double (const db::edge<C> &e, double y1, double y2) 
{
  if (e.dx () == 0) {
    return e.p1 ().x ();
  } else if (e.dy () == 0) {
    return std::max (e.p1 ().x (), e.p2 ().x ());
  } else {
    return C (ceil (edge_xaty_double (e, ((e.dy () < 0) ^ (e.dx () < 0)) != 0 ? y1 : y2)));
  }
}

/**
 *  @brief Functor that compares two edges by their left bound for a given interval [y1..y2].
 *
 *  This function is intended for use in scanline scenarios to determine what edges are 
 *  interacting in a certain y interval.
 */
template <class C>
struct edge_xmin_at_yinterval_double_compare
{
  edge_xmin_at_yinterval_double_compare (double y1, double y2)
    : m_y1 (y1), m_y2 (y2)
  {
    // .. nothing yet ..
  }

  bool operator() (const db::edge<C> &a, const db::edge<C> &b) const
  {
    if (edge_xmax (a) < edge_xmin (b)) {
      return true;
    } else if (edge_xmin (a) > edge_xmax (b)) {
      return false;
    } else {
      C xa = edge_xmin_at_yinterval_double (a, m_y1, m_y2);
      C xb = edge_xmin_at_yinterval_double (b, m_y1, m_y2);
      if (xa != xb) {
        return xa < xb;
      } else {
        return a < b;
      }
    }
  }

public:
  double m_y1, m_y2;
};

static void 
get_intersections_per_band_any (std::vector <CutPoints> &cutpoints, std::vector <WorkEdge>::iterator current, std::vector <WorkEdge>::iterator future, db::Coord y, db::Coord yy, bool with_h)
{
  double dy = y - 0.5;
  double dyy = yy + 0.5;
  std::vector <std::pair<const WorkEdge *, WorkEdge *> > p1_weak;   // holds weak interactions of edge endpoints with other edges

  std::sort (current, future, edge_xmin_at_yinterval_double_compare<db::Coord> (dy, dyy));

#ifdef DEBUG_EDGE_PROCESSOR
  printf ("y=%d..%d\n", y, yy);
  printf ("edges:"); 
  for (std::vector <WorkEdge>::iterator c1 = current; c1 != future; ++c1) { 
    printf (" %s", c1->to_string().c_str ()); 
  } 
  printf ("\n");
#endif
  db::Coord x = edge_xmin_at_yinterval_double (*current, dy, dyy);

  std::vector <WorkEdge>::iterator f = current;
  for (std::vector <WorkEdge>::iterator c = current; c != future; ) {

    size_t n = 0;
    db::Coord xx = x;

    //  fetch as many cells as to fill in roughly 50% more
    //  (this is an empirical performance improvement factor)
    do {

      while (f != future && edge_xmin_at_yinterval_double (*f, dy, dyy) <= xx) {
        ++f;
      }

      if (f != future) {
        xx = edge_xmin_at_yinterval_double (*f, dy, dyy);
      } else {
        xx = std::numeric_limits <db::Coord>::max ();
      }

      if (n == 0) {
        n = std::distance (c, f);
      }

    } while (f != future && std::distance (c, f) < long (n * fill_factor));

#ifdef DEBUG_EDGE_PROCESSOR
    printf ("edges %d..%d:", x, xx); 
    for (std::vector <WorkEdge>::iterator c1 = c; c1 != f; ++c1) { 
      printf (" %s", c1->to_string().c_str ()); 
    } 
    printf ("\n");
#endif

    if (std::distance (c, f) > 1) {

      db::Box cell (x, y, xx, yy);

      std::set<db::Point> weak_points;    // holds points that need to go in all other edges
      p1_weak.clear ();

      for (std::vector <WorkEdge>::iterator c1 = c; c1 != f; ++c1) {

        bool c1p1_in_cell = cell.contains (c1->p1 ());
        bool c1p2_in_cell = cell.contains (c1->p2 ());

        for (std::vector <WorkEdge>::iterator c2 = c; c2 != f; ++c2) {

          if (c1 == c2) {
            continue;
          }

          if (c2->dy () == 0) {

            if ((with_h || c1->dy () != 0) && c1 < c2) {

              if (c1->dy () == 0) {

                //  parallel horizontal edges: produce the end points of each other edge as cutpoints
                if (c1->p1 ().y () == c2->p1 ().y ()) {
                  add_hparallel_cutpoints (*c1, *c2, cell, cutpoints);
                  add_hparallel_cutpoints (*c2, *c1, cell, cutpoints);
                }

              } else if (c1->p1 () != c2->p1 () && c1->p2 () != c2->p1 () &&
                         c1->p1 () != c2->p2 () && c1->p2 () != c2->p2 ()) {

                std::pair <bool, db::Point> cp = safe_intersect_point (*c1, *c2);
                if (cp.first && cell.contains (cp.second)) {
                  //  Stash the cutpoint as it must be inserted into other edges as well.
                  weak_points.insert (cp.second);
                }

              }

            } 
          
          } else if (c1->parallel (*c2) && c1->side_of (c2->p1 ()) == 0) {

#ifdef DEBUG_EDGE_PROCESSOR
            printf ("%s and %s are parallel.\n", c1->to_string ().c_str (), c2->to_string ().c_str ()); 
#endif

            //  both edges are coincident - produce the ends of the edges involved as cut points
            if (c1p1_in_cell && c2->contains (c1->p1 ()) && c2->p1 () != c1->p1 () && c2->p2 () != c1->p1 ()) {
              c2->make_cutpoints (cutpoints)->add (c1->p1 (), &cutpoints, !is_point_on_exact(*c2, c1->p1 ()));
#ifdef DEBUG_EDGE_PROCESSOR
              if (! is_point_on_exact(*c2, c1->p1 ())) {
                printf ("intersection point %s between %s and %s.\n", c1->p1 ().to_string ().c_str (), c1->to_string ().c_str (), c2->to_string ().c_str ()); 
              } else {
                printf ("weak intersection point %s between %s and %s.\n", c1->p1 ().to_string ().c_str (), c1->to_string ().c_str (), c2->to_string ().c_str ()); 
              }
#endif
            }
            if (c1p2_in_cell && c2->contains (c1->p2 ()) && c2->p1 () != c1->p2 () && c2->p2 () != c1->p2 ()) {
              c2->make_cutpoints (cutpoints)->add (c1->p2 (), &cutpoints, !is_point_on_exact(*c2, c1->p2 ()));
#ifdef DEBUG_EDGE_PROCESSOR
              if (! is_point_on_exact(*c2, c1->p2 ())) {
                printf ("intersection point %s between %s and %s.\n", c1->p2 ().to_string ().c_str (), c1->to_string ().c_str (), c2->to_string ().c_str ()); 
              } else {
                printf ("weak intersection point %s between %s and %s.\n", c1->p2 ().to_string ().c_str (), c1->to_string ().c_str (), c2->to_string ().c_str ()); 
              }
#endif
            }

          } else {

            if (c1 < c2 && c1->p1 () != c2->p1 () && c1->p2 () != c2->p1 () &&
                           c1->p1 () != c2->p2 () && c1->p2 () != c2->p2 ()) {

              std::pair <bool, db::Point> cp = safe_intersect_point (*c1, *c2);
              if (cp.first && cell.contains (cp.second)) {
                //  Stash the cutpoint as it must be inserted into other edges as well.
                weak_points.insert (cp.second);
              }

            } 

            //  The endpoints of the other edge must be inserted into the edge 
            //  if they are within the modification range (but only then).
            //  We first collect these endpoints because we have to decide whether that can be 
            //  a weak attractor or, if it affects two or more edges in which case it will become a strong attractor. 
            //  It's sufficient to do this for p1 only because we made sure we caught all edges
            //  in the +-0.5DBU vicinity by choosing the cell large enough (.._double operators).
            //  For end points exactly on the line we insert a cutpoint to ensure we use the
            //  endpoints as cutpoints in any case.
            if (c1p1_in_cell && is_point_on_fuzzy (*c2, c1->p1 ())) {
              if (is_point_on_exact (*c2, c1->p1 ())) {
#ifdef DEBUG_EDGE_PROCESSOR
                printf ("end point %s gives intersection point between %s and %s.\n", c1->p1 ().to_string ().c_str (), c1->to_string ().c_str (), c2->to_string ().c_str ()); 
#endif
                c2->make_cutpoints (cutpoints)->add (c1->p1 (), &cutpoints, true);
              } else {
                p1_weak.push_back (std::make_pair (c1.operator-> (), c2.operator-> ()));
              }
            }

          }

        }

      }

      //  insert weak intersection points into all relevant edges - weak into edges
      //  where the point is on and strong into edges where the point is on in a fuzzy way.

      for (auto wp = weak_points.begin (); wp != weak_points.end (); ++wp) {

        for (std::vector <WorkEdge>::iterator cc = c; cc != f; ++cc) {
          if ((with_h || cc->dy () != 0) && is_point_on_fuzzy (*cc, *wp)) {
            bool on_edge = is_point_on_exact (*cc, *wp);
            cc->make_cutpoints (cutpoints)->add (*wp, &cutpoints, !on_edge);
#ifdef DEBUG_EDGE_PROCESSOR
            if (!on_edge) {
              printf ("intersection point %s gives strong cutpoint in %s.\n", wp->to_string ().c_str (), cc->to_string ().c_str ());
            } else {
              printf ("intersection point %s gives weak cutpoint in %s.\n", wp->to_string ().c_str (), cc->to_string ().c_str ());
            }
#endif
          }
        }

      }

      //  go through the list of "p1 to other edges" and insert p1 either as cutpoint
      //  (if there are other strong cutpoints already) or weak attractor.

      auto p1w_from = p1_weak.begin ();
      while (p1w_from != p1_weak.end ()) {

        bool strong = false;
        auto p1w_to = p1w_from;
        while (p1w_to != p1_weak.end () && p1w_to->first == p1w_from->first) {
          if (p1w_to->second->data > 0 && cutpoints [p1w_to->second->data - 1].strong_cutpoints) {
            strong = true;
          }
          ++p1w_to;
        }

        db::Point p1 = p1w_from->first->p1 ();

        p1w_to [-1].second->make_cutpoints (cutpoints);
        size_t n = p1w_to [-1].second->data - 1;

        for (auto cp = p1w_from; cp != p1w_to; ++cp) {

          cp->second->make_cutpoints (cutpoints);
          size_t nn = cp->second->data - 1;
          if (strong) {
            cutpoints [nn].add (p1, &cutpoints);
#ifdef DEBUG_EDGE_PROCESSOR
            printf ("Insert strong attractor %s in %s.\n", cp->first->p1 ().to_string ().c_str (), cp->second->to_string ().c_str ());
#endif
          } else {
            cutpoints [nn].add_attractor (p1, n);
#ifdef DEBUG_EDGE_PROCESSOR
            printf ("Insert weak attractor %s in %s.\n", cp->first->p1 ().to_string ().c_str (), cp->second->to_string ().c_str ());
#endif
          }

          n = nn;

        }

        p1w_from = p1w_to;

      }

    }

    x = xx;
    for (std::vector <WorkEdge>::iterator cc = c; cc != f; ++cc) {
      if (edge_xmax (*cc) < x || edge_xmax_at_yinterval_double (*cc, dy, dyy) < x) {
        if (c != cc) {
          std::swap (*cc, *c);
        }
        ++c;
      }
    }

  }
}

void 
EdgeProcessor::process (db::EdgeSink &es, EdgeEvaluatorBase &op)
{
  std::vector<std::pair<db::EdgeSink *, db::EdgeEvaluatorBase *> > procs;
  procs.push_back (std::make_pair (&es, &op));
  process (procs);
}

void
EdgeProcessor::redo (db::EdgeSink &es, EdgeEvaluatorBase &op)
{
  std::vector<std::pair<db::EdgeSink *, db::EdgeEvaluatorBase *> > procs;
  procs.push_back (std::make_pair (&es, &op));
  redo (procs);
}

namespace
{

class EdgeProcessorState
{
public:
  EdgeProcessorState (db::EdgeSink *es, db::EdgeEvaluatorBase *op)
    : mp_es (es), mp_op (op), m_vertex (false),
      m_x (0), m_y (0), m_hx (0), m_ho (0), m_pn (0), m_ps (0)
  { }

  void start ()
  {
    mp_es->start ();
  }

  void flush ()
  {
    mp_es->flush ();
  }

  void reset ()
  {
    mp_es->reset_stop ();
    mp_op->reset ();
  }

  bool is_reset ()
  {
    return mp_op->is_reset ();
  }

  bool can_stop ()
  {
    return mp_es->can_stop ();
  }

  void reserve (size_t n)
  {
    mp_op->reserve (n);
  }

  void begin_scanline (db::Coord y)
  {
    m_y = y;
    m_x = 0;
    m_hx = 0;
    m_ho = 0;
    m_vertex = false;
    mp_es->begin_scanline (y);
  }

  void end_scanline (db::Coord y)
  {
    mp_es->end_scanline (y);
  }

  void next_vertex (double x)
  {
    m_x = db::coord_traits<db::Coord>::rounded (x);
    m_vertex = false;
  }

  void end_vertex ()
  {
    if (m_vertex) {
      m_hx = m_x;
      m_ho = mp_op->compare_ns ();
    }
  }

  void next_coincident ()
  {
    m_pn = m_ps = 0;
  }

  void end_coincident ()
  {
    if (! m_vertex && (m_ps != 0 || m_pn != 0)) {

      if (m_ho != 0) {
        db::Edge he (db::Point (m_hx, m_y), db::Point (db::coord_traits<db::Coord>::rounded (m_x), m_y));
        if (m_ho > 0) {
          he.swap_points ();
        }
        mp_es->put (he);
#ifdef DEBUG_EDGE_PROCESSOR
        printf ("put(%s)\n", he.to_string ().c_str ());
#endif
      }

      m_vertex = true;

    }
  }

  void north_edge (bool prefer_touch, EdgeEvaluatorBase::property_type prop)
  {
    m_pn += mp_op->edge (true, prefer_touch, prop);
  }

  void south_edge (bool prefer_touch, EdgeEvaluatorBase::property_type prop)
  {
    m_ps += mp_op->edge (false, prefer_touch, prop);
  }

  void select_edge (const WorkEdge &e)
  {
    int tag = mp_op->select_edge (e.dy () == 0, e.prop);
    if (tag > 0) {
      mp_es->put (e, (unsigned int) tag);
#ifdef DEBUG_EDGE_PROCESSOR
      printf ("put(%s, %d)\n", e.to_string().c_str(), tag);
#endif
    }
  }

  bool push_edge (const db::Edge &e)
  {
    if (m_pn != 0) {

      db::Edge edge (e);
      if ((m_pn > 0 && edge.dy () < 0) || (m_pn < 0 && edge.dy () > 0)) {
        edge.swap_points ();
      }

      if (edge_ymin (edge) == m_y) {
        mp_es->put (edge);
#ifdef DEBUG_EDGE_PROCESSOR
        printf ("put(%s)\n", edge.to_string().c_str());
#endif
      } else {
        mp_es->crossing_edge (edge);
#ifdef DEBUG_EDGE_PROCESSOR
        printf ("xing(%s)\n", edge.to_string().c_str());
#endif
      }

      return true;

    } else {
      return false;
    }
  }

  void skip_n (size_t n)
  {
    mp_es->skip_n (n);
  }

private:
  db::EdgeSink *mp_es;
  db::EdgeEvaluatorBase *mp_op;

  bool m_vertex;
  db::Coord m_x, m_y, m_hx;
  int m_ho;
  int m_pn, m_ps;
};

//  NOTE: set this to 0 to force memory-allocation storage for SkipInfo always (testing)
const size_t skip_info_storage_threshold = 1;

/**
 *  @brief Encapsulates the state of the edge processor's generation stage
 *
 *  The generation state may involve multiple generators and output sinks. This
 *  class provides a single interface to handle the case of single and multiple
 *  receivers in a uniform way.
 */
class EdgeProcessorStates
{
public:
  /**
   *  @brief A structure holding the "skip information"
   *
   *  Skipping intervals with a known behavior is an optimization to improve
   *  the scanner's performance. This object keeps the information required to
   *  properly implement the skipping. It keeps both the edge skip count per
   *  interval ("skip") as well as the corresponding skip count for the
   *  generated edges. As multiple edge receivers can be supplied, the result
   *  skip count is a individual one per generator and edge receiver.
   */
  struct SkipInfo
  {
    SkipInfo (size_t _skip, const std::vector<size_t> &_skip_res)
      : skip (_skip), m_skip_res_n (0), m_skip_res (0)
    {
      set_skip_res (_skip_res.begin (), _skip_res.end ());
    }

    SkipInfo ()
      : skip (0), m_skip_res_n (0), m_skip_res (0)
    { }

    SkipInfo (const SkipInfo &si)
      : skip (0), m_skip_res_n (0), m_skip_res (0)
    {
      operator= (si);
    }

    ~SkipInfo ()
    {
      if (m_skip_res_n > skip_info_storage_threshold) {
        delete[] skip_res ();
      }
    }

    SkipInfo &operator= (const SkipInfo &si)
    {
      if (&si != this) {
        skip = si.skip;
        const size_t *n = si.skip_res ();
        set_skip_res (n, n + si.m_skip_res_n);
      }
      return *this;
    }

    template <class Iter>
    void set_skip_res (Iter b, Iter e)
    {
      if (m_skip_res_n > skip_info_storage_threshold) {
        delete[] (reinterpret_cast<size_t *> (m_skip_res));
      }

      m_skip_res_n = e - b;
      if (m_skip_res_n <= skip_info_storage_threshold) {
        if (b == e) {
          m_skip_res = 0;
        } else {
          m_skip_res = *b;
        }
      } else {
        size_t *t = new size_t[m_skip_res_n];
        m_skip_res = reinterpret_cast<size_t> (t);
        for (Iter i = b; i != e; ++i) {
          *t++ = *i;
        }
      }
    }

    const size_t *skip_res () const
    {
      if (m_skip_res_n <= skip_info_storage_threshold) {
        return &m_skip_res;
      } else {
        return reinterpret_cast<const size_t *> (m_skip_res);
      }
    }

    size_t skip;

  private:
    size_t m_skip_res_n;
    size_t m_skip_res;
  };

  /**
   *  @brief Creates a generator stage state object from the given sinks and operators
   */
  EdgeProcessorStates (const std::vector<std::pair<db::EdgeSink *, db::EdgeEvaluatorBase *> > &procs)
    : m_selects_edges (false), m_prefer_touch (false)
  {
    m_states.reserve (procs.size ());
    for (std::vector<std::pair<db::EdgeSink *, db::EdgeEvaluatorBase *> >::const_iterator p = procs.begin (); p != procs.end (); ++p) {

      m_states.push_back (EdgeProcessorState (p->first, p->second));

      if (p->second->selects_edges ()) {
        m_selects_edges = true;
      }

      if (p->second->prefer_touch ()) {
        m_prefer_touch = true;
      }

    }
  }

  /**
   *  @brief Returns true if the processors want to select edges
   */
  bool selects_edges () const
  {
    return m_selects_edges;
  }

  /**
   *  @brief Returns true if the processors prefer touching mode
   */
  bool prefer_touch () const
  {
    return m_prefer_touch;
  }

  /**
   *  @brief Initial event
   *  This method is called when the scan is initiated
   */
  void start ()
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->start ();
    }
  }

  /**
   *  @brief Final event
   *  This method is called after the scan terminated
   */
  void flush ()
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->flush ();
    }
  }

  /**
   *  @brief Reset status event
   *  This method is to ensure the state of the operator is reset.
   */
  void reset ()
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->reset ();
    }
  }

  /**
   *  @brief Gets a value indicating whether all operators are reset
   */
  bool is_reset ()
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      if (! s->is_reset ()) {
        return false;
      }
    }
    return true;
  }

  /**
   *  @brief Gets a value indicating whether the generator wants to stop
   */
  bool can_stop ()
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      if (s->can_stop ()) {
        return true;
      }
    }
    return false;
  }

  /**
   *  @brief Reserve memory n edges
   */
  void reserve (size_t n)
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->reserve (n);
    }
  }

  /**
   *  @brief Begin scanline event
   *  This method is called at the beginning of a new scanline
   */
  void begin_scanline (db::Coord y)
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->begin_scanline (y);
    }
  }

  /**
   *  @brief End scanline event
   *  This method is called at the end of a scanline
   */
  void end_scanline (db::Coord y)
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->end_scanline (y);
    }
  }

  /**
   *  @brief Announces a batch of edges crossing the same point
   */
  void next_vertex (double x)
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->next_vertex (x);
    }
  }

  /**
   *  @brief Finishes the vertex
   */
  void end_vertex ()
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->end_vertex ();
    }
  }

  /**
   *  @brief Announces a batch of edges crossing the same point and begin coincident
   *  This event is a sub-event of "next_vertex".
   */
  void next_coincident ()
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->next_coincident ();
    }
  }

  /**
   *  @brief Announces a batch of edges crossing the same point and begin coincident
   *  This event is a sub-event of "next_vertex".
   */
  void end_coincident ()
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->end_coincident ();
    }
  }

  /**
   *  @brief Announces an edge north to the scanline
   */
  void north_edge (bool prefer_touch, EdgeEvaluatorBase::property_type prop)
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->north_edge (prefer_touch, prop);
    }
  }

  /**
   *  @brief Announces an edge south of the scanline
   */
  void south_edge (bool prefer_touch, EdgeEvaluatorBase::property_type prop)
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->south_edge (prefer_touch, prop);
    }
  }

  /**
   *  @brief Gives the generators an opportunity to select the given edge
   */
  void select_edge (const WorkEdge &e)
  {
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->select_edge (e);
    }
  }

  /**
   *  @brief Delivers an edge to the edge sink if present
   *
   *  This method will return true if at least one of the edge sinks received the edge
   */
  void push_edge (const db::Edge &e)
  {
    size_t i = 0;
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s, ++i) {
      if (s->push_edge (e)) {
        ++m_nres [i];
      }
    }
  }

  /**
   *  @brief Skips n edges on the edge sink
   *
   *  This is for optimization of the polygon generation. Stitching of edges does not happen if
   *  there are no news.
   */
  void skip_n (const SkipInfo &si)
  {
    const size_t *n = si.skip_res ();
    for (std::vector<EdgeProcessorState>::iterator s = m_states.begin (); s != m_states.end (); ++s) {
      s->skip_n (*n++);
    }
  }

  /**
   *  @brief Gets the SkipInfo for a given index
   */
  const SkipInfo &skip_info (size_t n)
  {
    if (n == 0) {
      static SkipInfo empty;
      return empty;
    } else {
      return m_skip_info [n - 1];
    }
  }

  /**
   *  @brief Releases a SkipInfo entry
   */
  void release_skip_entry (size_t n)
  {
    m_skip_queue.push_front (n - 1);
  }

  /**
   *  @brief Resets a SkipInfo entry
   *
   *  A convenience function to reset and release a SkipInfo entry
   */
  void reset_skip_entry (size_t &n)
  {
    if (n != 0) {
      release_skip_entry (n);
      n = 0;
    }
  }

  /**
   *  @brief Begins an interval that can potentially be skipped
   */
  void begin_skip_interval ()
  {
    m_nres.clear ();
    m_nres.resize (m_states.size (), size_t (0));
  }

  /**
   *  @brief Finishes an interval that can potentially be skipped
   *
   *  Returns the index of a new skip interval entry containing the skip information.
   */
  size_t end_skip_interval (size_t skip)
  {
    size_t n = 0;

    if (! m_skip_queue.empty ()) {
      n = m_skip_queue.front ();
      m_skip_queue.pop_front ();
    } else {
      n = m_skip_info.size ();
      m_skip_info.push_back (SkipInfo ());
    }

    m_skip_info[n].skip = skip;
    m_skip_info[n].set_skip_res (m_nres.begin (), m_nres.end ());
    return n + 1;
  }

private:
  std::vector<EdgeProcessorState> m_states;
  bool m_selects_edges, m_prefer_touch;
  std::vector<SkipInfo> m_skip_info;
  std::list<size_t> m_skip_queue;
  std::vector<size_t> m_nres;
};

}

void
EdgeProcessor::redo (const std::vector<std::pair<db::EdgeSink *, db::EdgeEvaluatorBase *> > &gen)
{
  redo_or_process (gen, true);
}

void
EdgeProcessor::process (const std::vector<std::pair<db::EdgeSink *, db::EdgeEvaluatorBase *> > &gen)
{
  redo_or_process (gen, false);
}

void
EdgeProcessor::redo_or_process (const std::vector<std::pair<db::EdgeSink *, db::EdgeEvaluatorBase *> > &gen, bool redo)
{
  tl::SelfTimer timer (tl::verbosity () >= m_base_verbosity, "EdgeProcessor: process");

  EdgeProcessorStates gs (gen);

  bool prefer_touch = gs.prefer_touch ();
  bool selects_edges = gs.selects_edges ();
  
  db::Coord y;
  std::vector <WorkEdge>::iterator future;

  //  step 1: preparation

  if (mp_work_edges->empty ()) {
    gs.start ();
    gs.flush ();
    return;
  }

  mp_cpvector->clear ();

  //  count the properties

  property_type n_props = 0;
  for (std::vector <WorkEdge>::iterator e = mp_work_edges->begin (); e != mp_work_edges->end (); ++e) {
    if (e->prop > n_props) {
      n_props = e->prop;
    }
  }
  ++n_props;

  //  prepare progress

  size_t todo_max = 1000000;

  std::unique_ptr<tl::AbsoluteProgress> progress;
  if (m_report_progress) {
    if (m_progress_desc.empty ()) {
      progress.reset (new tl::AbsoluteProgress (tl::to_string (tr ("Processing")), 1000));
    } else {
      progress.reset (new tl::AbsoluteProgress (m_progress_desc, 1000));
    }
    progress->set_format (tl::to_string (tr ("%.0f%%")));
    progress->set_unit (todo_max / 100);
  }

  size_t todo_next = 0;
  size_t todo = todo_next;
  todo_next += (todo_max - todo) / 5;


  if (redo) {

    //  redo mode: skip the intersection detection step and clear the data

    for (std::vector <WorkEdge>::iterator c = mp_work_edges->begin (); c != mp_work_edges->end (); ++c) {
      c->data = 0;
    }

    todo = todo_next;
    todo_next += (todo_max - todo) / 5;

  } else {

    //  step 2: find intersections
    std::sort (mp_work_edges->begin (), mp_work_edges->end (), edge_ymin_compare<db::Coord> ());

    y = edge_ymin ((*mp_work_edges) [0]);
    future = mp_work_edges->begin ();

    for (std::vector <WorkEdge>::iterator current = mp_work_edges->begin (); current != mp_work_edges->end (); ) {

      if (m_report_progress) {
        double p = double (std::distance (mp_work_edges->begin (), current)) / double (mp_work_edges->size ());
        progress->set (size_t (double (todo_next - todo) * p) + todo);
      }

      size_t n = 0;
      db::Coord yy = y;

      //  Use as many scanlines as to fetch approx. 50% new edges into the scanline (this
      //  is an empirically determined factor)
      do {

        while (future != mp_work_edges->end () && edge_ymin (*future) <= yy) {
          ++future;
        }

        if (future != mp_work_edges->end ()) {
          yy = edge_ymin (*future);
        } else {
          yy = std::numeric_limits <db::Coord>::max ();
        }

        if (n == 0) {
          n = std::distance (current, future);
        }

      } while (future != mp_work_edges->end () && std::distance (current, future) < long (n * fill_factor));

      bool is90 = true;

      if (current != future) {

        for (std::vector <WorkEdge>::iterator c = current; c != future && is90; ++c) {
          if (c->dx () != 0 && c->dy () != 0) {
            is90 = false;
          }
        }

        if (is90) {
          get_intersections_per_band_90 (*mp_cpvector, current, future, y, yy, selects_edges);
        } else {
          get_intersections_per_band_any (*mp_cpvector, current, future, y, yy, selects_edges);
        }

      }

      y = yy;
      for (std::vector <WorkEdge>::iterator c = current; c != future; ++c) {
        //  Hint: we have to keep the edges ending a y (the new lower band limit) in the all angle case because these edges
        //  may receive cutpoints because the enter the -0.5DBU region below the band
        if ((!is90 && edge_ymax (*c) < y) || (is90 && edge_ymax (*c) <= y)) {
          if (current != c) {
            std::swap (*current, *c);
          }
          ++current;
        }
      }

    }

    //  step 3: create new edges from the ones with cutpoints
    //
    //  Hint: when we create the edges from the cutpoints we use the projection to sort the cutpoints along the
    //  edge. However, we have some freedom to connect the points which we use to avoid "z" configurations which could
    //  create new intersections in a 1x1 pixel box.

    todo = todo_next;
    todo_next += (todo_max - todo) / 5;

    size_t n_work = mp_work_edges->size ();
    size_t nw = 0;
    for (size_t n = 0; n < n_work; ++n) {

      if (m_report_progress) {
        double p = double (n) / double (n_work);
        progress->set (size_t (double (todo_next - todo) * p) + todo);
      }

      WorkEdge &ew = (*mp_work_edges) [n];

      CutPoints *cut_points = ew.data ? & ((*mp_cpvector) [ew.data - 1]) : 0;
      ew.data = 0;

      if (ew.dy () == 0 && ! selects_edges) {

        //  don't care about horizontal edges

      } else if (cut_points) {

        if (cut_points->has_cutpoints && ! cut_points->cut_points.empty ()) {

          db::Edge e = ew;
          property_type p = ew.prop;
          std::sort (cut_points->cut_points.begin (), cut_points->cut_points.end (), ProjectionCompare (e));

          db::Point pll = e.p1 ();
          db::Point pl = e.p1 ();

          for (std::vector <db::Point>::iterator cp = cut_points->cut_points.begin (); cp != cut_points->cut_points.end (); ++cp) {
            if (*cp != pl) {
              WorkEdge ne = WorkEdge (db::Edge (pl, *cp), p);
              if (pl.y () == pll.y () && ne.p2 ().x () != pl.x () && ne.p2 ().x () == pll.x ()) {
                ne = db::Edge (pll, ne.p2 ());
              } else if (pl.x () == pll.x () && ne.p2 ().y () != pl.y () && ne.p2 ().y () == pll.y ()) {
                ne = db::Edge (ne.p1 (), pll);
              } else {
                pll = pl;
              }
              pl = *cp;
              if (selects_edges || ne.dy () != 0) {
                if (nw <= n) {
                  (*mp_work_edges) [nw++] = ne;
                } else {
                  mp_work_edges->push_back (ne);
                }
              }
            }
          }

          if (cut_points->cut_points.back () != e.p2 ()) {
            WorkEdge ne = WorkEdge (db::Edge (pl, e.p2 ()), p);
            if (pl.y () == pll.y () && ne.p2 ().x () != pl.x () && ne.p2 ().x () == pll.x ()) {
              ne = db::Edge (pll, ne.p2 ());
            } else if (pl.x () == pll.x () && ne.p2 ().y () != pl.y () && ne.p2 ().y () == pll.y ()) {
              ne = db::Edge (ne.p1 (), pll);
            }
            if (selects_edges || ne.dy () != 0) {
              if (nw <= n) {
                (*mp_work_edges) [nw++] = ne;
              } else {
                mp_work_edges->push_back (ne);
              }
            }
          }

        } else {

          if (nw < n) {
            (*mp_work_edges) [nw] = (*mp_work_edges) [n];
          }
          ++nw;

        }

      } else {

        if (nw < n) {
          (*mp_work_edges) [nw] = (*mp_work_edges) [n];
        }
        ++nw;

      }

    }

    if (nw != n_work) {
      mp_work_edges->erase (mp_work_edges->begin () + nw, mp_work_edges->begin () + n_work);
    }

#ifdef DEBUG_EDGE_PROCESSOR
    printf ("Output edges:\n");
    for (std::vector <WorkEdge>::iterator c1 = mp_work_edges->begin (); c1 != mp_work_edges->end (); ++c1) {
      printf ("%s\n", c1->to_string().c_str ());
    }
#endif

  }


  tl::SelfTimer timer2 (tl::verbosity () >= m_base_verbosity + 10, "EdgeProcessor: production");

  //  step 4: compute the result edges 
  
  gs.start (); // call this as late as possible. This way, input containers can be identical with output containers ("clear" is done after the input is read)

  gs.reset ();
  gs.reserve (n_props);

  std::sort (mp_work_edges->begin (), mp_work_edges->end (), edge_ymin_compare<db::Coord> ());

  y = edge_ymin ((*mp_work_edges) [0]);

  future = mp_work_edges->begin ();
  for (std::vector <WorkEdge>::iterator current = mp_work_edges->begin (); current != mp_work_edges->end () && ! gs.can_stop (); ) {

    if (m_report_progress) {
      double p = double (std::distance (mp_work_edges->begin (), current)) / double (mp_work_edges->size ());
      progress->set (size_t (double (todo_max - todo_next) * p) + todo_next);
    }

    std::vector <WorkEdge>::iterator f0 = future;
    while (future != mp_work_edges->end () && edge_ymin (*future) <= y) {
      tl_assert (future->data == 0); // HINT: for development
      ++future;
    }
    std::sort (f0, future, EdgeXAtYCompare2 (y));

    db::Coord yy = std::numeric_limits <db::Coord>::max ();
    if (future != mp_work_edges->end ()) {
      yy = edge_ymin (*future);
    }
    for (std::vector <WorkEdge>::const_iterator c = current; c != future; ++c) {
      if (edge_ymax (*c) > y) {
        yy = std::min (yy, edge_ymax (*c));
      }
    }

    db::Coord ysl = y;
    gs.begin_scanline (y);

    tl_assert (gs.is_reset ()); // HINT: for development

    if (current != future) {

      std::inplace_merge (current, f0, future, EdgeXAtYCompare2 (y));
#ifdef DEBUG_EDGE_PROCESSOR
      printf ("y=%d ", y);
      for (std::vector <WorkEdge>::iterator c = current; c != future; ++c) { 
        printf ("%ld-", long (c->data)); 
      } 
      printf ("\n");
#endif

      for (std::vector <WorkEdge>::iterator c = current; c != future; ) {

        const EdgeProcessorStates::SkipInfo &skip_info = gs.skip_info (c->data);
#ifdef DEBUG_EDGE_PROCESSOR
        printf ("X %ld->%d\n", long (c->data), int (skip_info.skip));
#endif

        if (skip_info.skip != 0 && (c + skip_info.skip >= future || (c + skip_info.skip)->data != 0)) {

          tl_assert (c + skip_info.skip <= future);

          gs.skip_n (skip_info);

          //  skip this interval - has not changed
          c += skip_info.skip;

        } else {

          std::vector <WorkEdge>::iterator c0 = c;
          gs.begin_skip_interval ();

          do {

            gs.reset_skip_entry (c->data);

            std::vector <WorkEdge>::iterator f = c + 1;

            //  HINT: "volatile" forces x and xx into memory and disables FPU register optimisation.
            //  That way, we can exactly compare doubles afterwards.
            volatile double x = edge_xaty (*c, y);

            while (f != future) {
              volatile double xx = edge_xaty (*f, y);
              if (xx != x) {
                break;
              }
              gs.reset_skip_entry (f->data);
              ++f;
            }

            //  compute edges that occur at this vertex
            
            gs.next_vertex (x);
            
            //  treat all edges crossing the scanline in a certain point
            for (std::vector <WorkEdge>::iterator cc = c; cc != f; ) {

              gs.next_coincident ();

              std::vector <WorkEdge>::iterator e = mp_work_edges->end ();

              std::vector <WorkEdge>::iterator cc0 = cc;

              std::vector <WorkEdge>::iterator fc = cc;
              do {
                ++fc;
              } while (fc != f && EdgeXAtYCompare2 (y).equal (*fc, *cc));

              //  sort the coincident edges by property ID - that will
              //  simplify algorithms like "inside" and "outside".
              if (fc - cc > 1) {
                //  for prefer_touch we first deliver the opening edges in ascending
                //  order, in the other case we the other way round so that the opening
                //  edges are always delivered with ascending property ID order.
                if (prefer_touch) {
                  std::sort (cc, fc, EdgePropCompare ());
                } else {
                  std::sort (cc, fc, EdgePropCompareReverse ());
                }
              }

              //  treat all coincident edges
              do {

                if (cc->dy () != 0) {

                  if (e == mp_work_edges->end () && edge_ymax (*cc) > y) {
                    e = cc;
                  }
                  
                  if ((cc->dy () > 0) == prefer_touch) {
                    if (edge_ymax (*cc) > y) {
                      gs.north_edge (prefer_touch, cc->prop);
                    }
                    if (edge_ymin (*cc) < y) {
                      gs.south_edge (prefer_touch, cc->prop);
                    }
                  }

                }

                ++cc;

              } while (cc != fc);

              //  Give the edge selection operator a chance to select edges now
              if (selects_edges) {
                for (std::vector <WorkEdge>::iterator sc = cc0; sc != fc; ++sc) {
                  if (edge_ymin (*sc) == y) {
                    gs.select_edge (*sc);
                  }
                }
              }

              //  report the closing or opening edges in the opposite order 
              //  than the other ones (see previous loop). Hence we have some
              //  symmetry of events which simplify implementation of the 
              //  InteractionDetector for example.
              do {

                --fc;

                if (fc->dy () != 0 && (fc->dy () > 0) != prefer_touch) {
                  if (edge_ymax (*fc) > y) {
                    gs.north_edge (! prefer_touch, fc->prop);
                  }
                  if (edge_ymin (*fc) < y) {
                    gs.south_edge (! prefer_touch, fc->prop);
                  }
                }

              } while (fc != cc0);

              gs.end_coincident ();

              if (e != mp_work_edges->end ()) {
                gs.push_edge (*e);
              }

            }

            gs.end_vertex ();

            c = f;

          } while (c != future && ! gs.is_reset ());

          //  TODO: assert that there is no overflow here:
          c0->data = gs.end_skip_interval (std::distance (c0, c));

        }

      }

      y = yy;

#ifdef DEBUG_EDGE_PROCESSOR
      for (std::vector <WorkEdge>::iterator c = current; c != future; ++c) {
        printf ("%ld-", long (c->data)); 
      } 
      printf ("\n");
#endif
      std::vector <WorkEdge>::iterator c0 = current;
      current = future;

      bool valid = true;

      for (std::vector <WorkEdge>::iterator c = future; c != c0; ) {

        --c;

        size_t data = c->data;
        c->data = 0;

        db::Coord ymax = edge_ymax (*c);
        if (ymax >= y) {
          --current;
          if (current != c) {
            std::swap (*current, *c);
          }
        }
        if (ymax <= y) {
          //  an edge ends now. The interval is not valid, i.e. cannot be skipped easily.
          valid = false;
        }

        if (data != 0 && current != future) {
          if (valid) {
            current->data = data;
            data = 0;
          } else {
            current->data = 0;
          }
          valid = true;
        }

        if (data) {
          gs.release_skip_entry (data);
        }

      }
#ifdef DEBUG_EDGE_PROCESSOR
      for (std::vector <WorkEdge>::iterator c = current; c != future; ++c) {
        printf ("%ld-", long (c->data)); 
      } 
      printf ("\n");
#endif
    
    }

    tl_assert (gs.is_reset ()); // HINT: for development (second)

    gs.end_scanline (ysl);

  }

  gs.flush ();

}

void
EdgeProcessor::simple_merge (const std::vector<db::Edge> &in, std::vector <db::Edge> &edges, int mode)
{
  clear ();
  reserve (in.size ());
  insert_sequence (in.begin (), in.end ());

  db::SimpleMerge op (mode);
  db::EdgeContainer out (edges);
  process (out, op);
}

void
EdgeProcessor::simple_merge (const std::vector<db::Edge> &in, std::vector <db::Polygon> &polygons, bool resolve_holes, bool min_coherence, int mode)
{
  clear ();
  reserve (in.size ());
  insert_sequence (in.begin (), in.end ());

  db::SimpleMerge op (mode);
  db::PolygonContainer pc (polygons);
  db::PolygonGenerator out (pc, resolve_holes, min_coherence);
  process (out, op);
}

void
EdgeProcessor::simple_merge (const std::vector<db::Polygon> &in, std::vector <db::Edge> &edges, int mode)
{
  clear ();
  reserve (count_edges (in));
  for (std::vector<db::Polygon>::const_iterator q = in.begin (); q != in.end (); ++q) {
    insert (*q);
  }

  db::SimpleMerge op (mode);
  db::EdgeContainer out (edges);
  process (out, op);
}

void
EdgeProcessor::simple_merge (const std::vector<db::Polygon> &in, std::vector <db::Polygon> &out, bool resolve_holes, bool min_coherence, int mode)
{
  clear ();
  reserve (count_edges (in));

  if (&in == &out) {
    while (! out.empty ()) {
      insert (out.back ());
      out.pop_back ();
    }
  } else {
    for (std::vector<db::Polygon>::const_iterator q = in.begin (); q != in.end (); ++q) {
      insert (*q);
    }
  }

  db::SimpleMerge op (mode);
  db::PolygonContainer pc (out);
  db::PolygonGenerator pg (pc, resolve_holes, min_coherence);
  process (pg, op);
}

void
EdgeProcessor::merge (const std::vector<db::Polygon> &in, std::vector <db::Edge> &edges, unsigned int min_wc)
{
  clear ();
  reserve (count_edges (in));

  size_t n = 0;
  for (std::vector<db::Polygon>::const_iterator q = in.begin (); q != in.end (); ++q, ++n) {
    insert (*q, n);
  }

  db::MergeOp op (min_wc);
  db::EdgeContainer out (edges);
  process (out, op);
}

void
EdgeProcessor::merge (const std::vector<db::Polygon> &in, std::vector <db::Polygon> &out, unsigned int min_wc, bool resolve_holes, bool min_coherence)
{
  clear ();
  reserve (count_edges (in));

  if (&in == &out) {
    size_t n = 0;
    while (! out.empty ()) {
      insert (out.back (), n);
      out.pop_back ();
      ++n;
    }
  } else {
    size_t n = 0;
    for (std::vector<db::Polygon>::const_iterator q = in.begin (); q != in.end (); ++q, ++n) {
      insert (*q, n);
    }
  }

  db::MergeOp op (min_wc);
  db::PolygonContainer pc (out);
  db::PolygonGenerator pg (pc, resolve_holes, min_coherence);
  process (pg, op);
}

void
EdgeProcessor::size (const std::vector<db::Polygon> &in, db::Coord dx, db::Coord dy, std::vector <db::Edge> &out, unsigned int mode)
{
  clear ();
  reserve (count_edges (in));

  size_t n = 0;
  for (std::vector<db::Polygon>::const_iterator q = in.begin (); q != in.end (); ++q, n += 2) {
    insert (*q, n);
  }

  //  Merge the polygons and feed them into the sizing filter
  db::EdgeContainer ec (out);
  db::SizingPolygonFilter siz (ec, dx, dy, mode);
  db::PolygonGenerator pg (siz, false /*don't resolve holes*/, false /*min. coherence*/);
  db::BooleanOp op (db::BooleanOp::Or);
  process (pg, op);
}

void
EdgeProcessor::size (const std::vector<db::Polygon> &in, db::Coord dx, db::Coord dy, std::vector <db::Polygon> &out, unsigned int mode, bool resolve_holes, bool min_coherence)
{
  clear ();
  reserve (count_edges (in));

  if (&in == &out) {
    size_t n = 0;
    while (! out.empty ()) {
      insert (out.back (), n);
      out.pop_back ();
      n += 2;
    }
  } else {
    size_t n = 0;
    for (std::vector<db::Polygon>::const_iterator q = in.begin (); q != in.end (); ++q, n += 2) {
      insert (*q, n);
    }
  }

  //  Merge the polygons and feed them into the sizing filter
#if ! defined(DEBUG_SIZE_INTERMEDIATE)
  db::PolygonContainer pc (out);
  db::PolygonGenerator pg2 (pc, resolve_holes, min_coherence);
  db::SizingPolygonFilter siz (pg2, dx, dy, mode);
  db::PolygonGenerator pg (siz, false /*don't resolve holes*/, false /*min. coherence*/);
  db::BooleanOp op (db::BooleanOp::Or);
  process (pg, op);
#else
  //  Intermediate output for debugging 
  db::PolygonContainer pc (out);
  db::PolygonGenerator pg2 (pc, false, false);
  db::BooleanOp op (db::BooleanOp::Or);
  process (pg2, op);
  for (std::vector <db::Polygon>::iterator p = out.begin (); p != out.end (); ++p) {
    *p = p->sized (dx, dy, mode);
  }
#endif
}

void 
EdgeProcessor::boolean (const std::vector<db::Polygon> &a, const std::vector<db::Polygon> &b, std::vector <db::Edge> &out, int mode)
{
  clear ();
  reserve (count_edges (a) + count_edges (b));

  size_t n;
  
  n = 0;
  for (std::vector<db::Polygon>::const_iterator q = a.begin (); q != a.end (); ++q, n += 2) {
    insert (*q, n);
  }

  n = 1;
  for (std::vector<db::Polygon>::const_iterator q = b.begin (); q != b.end (); ++q, n += 2) {
    insert (*q, n);
  }

  db::BooleanOp op ((db::BooleanOp::BoolOp) mode);
  db::EdgeContainer ec (out);
  process (ec, op);
}

void 
EdgeProcessor::boolean (const std::vector<db::Polygon> &a, const std::vector<db::Polygon> &b, std::vector <db::Polygon> &out, int mode, bool resolve_holes, bool min_coherence)
{
  clear ();
  reserve (count_edges (a) + count_edges (b));

  size_t n;
  
  n = 0;
  if (&a == &out && &b != &out) {
    while (! out.empty ()) {
      insert (out.back (), n);
      out.pop_back ();
      n += 2;
    }
  } else {
    for (std::vector<db::Polygon>::const_iterator q = a.begin (); q != a.end (); ++q, n += 2) {
      insert (*q, n);
    }
  }

  n = 1;
  if (&b == &out) {
    while (! out.empty ()) {
      insert (out.back (), n);
      out.pop_back ();
      n += 2;
    }
  } else {
    for (std::vector<db::Polygon>::const_iterator q = b.begin (); q != b.end (); ++q, n += 2) {
      insert (*q, n);
    }
  }

  db::BooleanOp op ((db::BooleanOp::BoolOp) mode);
  db::PolygonContainer pc (out);
  db::PolygonGenerator pg (pc, resolve_holes, min_coherence);
  process (pg, op);
}

void 
EdgeProcessor::boolean (const std::vector<db::Edge> &a, const std::vector<db::Edge> &b, std::vector <db::Edge> &out, int mode)
{
  clear ();
  reserve (a.size () + b.size ());

  insert_sequence (a.begin (), a.end (), 0);
  insert_sequence (b.begin (), b.end (), 1);

  db::BooleanOp op ((db::BooleanOp::BoolOp) mode);
  db::EdgeContainer ec (out);
  process (ec, op);
}

void 
EdgeProcessor::boolean (const std::vector<db::Edge> &a, const std::vector<db::Edge> &b, std::vector <db::Polygon> &out, int mode, bool resolve_holes, bool min_coherence)
{
  clear ();
  reserve (a.size () + b.size ());

  insert_sequence (a.begin (), a.end (), 0);
  insert_sequence (b.begin (), b.end (), 1);

  db::BooleanOp op ((db::BooleanOp::BoolOp) mode);
  db::PolygonContainer pc (out);
  db::PolygonGenerator pg (pc, resolve_holes, min_coherence);
  process (pg, op);
}

} // namespace db

