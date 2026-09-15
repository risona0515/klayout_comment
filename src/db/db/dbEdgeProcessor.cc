
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
//   ⚠ 注意：本文件的注释**并非均匀覆盖**。下面的标注说明了每个区域的注释情况，
//     避免误以为"列出来的都已注释"。标记含义：
//       [已注] = 函数/结构级说明完整
//       [部分] = 只有类头或关键片段有注释
//       [待补] = 目前尚无函数级注释（仅靠 .h 里的声明级文档支撑）
//
//   §1 工具与比较器                                        [已注]
//        NonZeroInsideFunc / ProjectionCompare / PolyMapCompare(已废弃)
//        is_point_on_exact / is_point_on_fuzzy     精确 vs 模糊的"点在边上"
//        safe_intersect_point                      顺序无关的求交（重要）
//        CutPoints / WorkEdge                      阶段 1-3 的核心数据结构
//        EdgePropCompare / EdgeXAtYCompare2        scanline 排序所需的比较器
//   §2 交点计算                                            [已注]
//        add_hparallel_cutpoints                   共线水平边之间的切点
//        get_intersections_per_band_90             正交（曼哈顿）快速路径
//        edge_xaty_double / edge_xmin|xmax_at_yinterval_double
//                                                  放宽带 [y-0.5, y+0.5] 的界计算
//        get_intersections_per_band_any            任意角度通用路径
//   §3 扫描线状态                                          [部分]
//        EdgeProcessorState                        类头有说明，**各方法体待补**
//        EdgeProcessorStates + SkipInfo            类头有说明，**各方法体待补**
//   §4 主驱动板 redo_or_process                            [已注]
//        阶段 1 prep → 阶段 2 intersections → 阶段 3 split → 阶段 4 production
//   §5 公开 API 实现                                       [部分]
//        insert / clear / reserve / count          已注
//        process / redo / redo_or_process          已注（转发与主驱动）
//        评估器实现体（EdgePolygonOp / InteractionDetector /
//        MergeOp / BooleanOp / BooleanOp2）        **待补**
//        simple_merge / merge / size / boolean      **待补**（语义见 .h，实现体未注）
//
// 【为什么 §3 与 §5 的部分还有缺口】
//   这不是刻意省略，而是当前进度的真实状态：已优先完成"数据结构 + 主驱动 + 求交数学"
//   这三块最难读懂也最关键的部分，以及 .h 的全部声明级文档。
//   剩余的评估器实现体与状态机方法体语义相对直白（多为转发或计数维护），
//   其**契约**已在 .h 与对应类头中写清，但缺少逐函数说明。
//
// 【两个贯穿全文件的优化思想，读代码时留意】
//   ① **正交快速路径**：版图绝大多数图形是曼哈顿的（边水平或垂直）。
//      两条正交边求交只需 max/min，无需叉积与除法，因而单独写了 *_90 版本。
//      判断入口会先看"是否全是正交边"，是则走快速路径。
//      注意：只有一般（斜边）路径才需要 safe_intersect_point 包装，
//      _90 路径可直接用 intersect_point —— 因为正交求交的公式天然对称。
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
// [[ZH-BEGIN]]
// ⚠ 死代码提示：EdgeXAtYCompare 在**整个代码库中已无任何引用**（只有本处定义，
//   外加一处文字提及）。它是被 EdgeXAtYCompare2 取代的旧版比较器 ——
//   EdgeXAtYCompare2 额外比较边的方向，而本版只比较 x。
//   阅读时可以跳过；它是清理无用代码时的候选。
//   （可用 grep "EdgeXAtYCompare" 自行确认：只有定义处与注释提及，没有使用处。）
// [[ZH-END]]
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
// [[ZH-BEGIN]]
// 功能：db::edge_xaty 的加强版 —— 求边在扫描线 y 处的 x，且**对水平边返回最小 x**。
//
// 与 db::edge_xaty 的唯一区别：当扫描线恰好落在水平边的两个端点高度之间时，
// 本函数返回 min(x1, x2) 而不是插值结果。因为水平边没有"唯一的"扫描线交点 x，
// 取较小的 x 可以让它在排序中稳定地落在左侧。
//
// 参数：e 目标边（按值传递，内部可能交换端点做归一化）；
//       y 扫描线 y。
// 返回：double 的 x 值。
//
// 与 db::edge_xaty 相同的契约：同一 (边, y) 必须返回**逐位相同**的 double，
// 否则阶段 4 的排序会不稳定（见 EdgeXAtYCompare2 的 volatile 说明）。
// [[ZH-END]]
static inline double edge_xaty2 (db::Edge e, db::Coord y)
{
  // [[ZH]] 归一化为 p1.y <= p2.y，使同一条边只有一种参数化。
  if (e.p1 ().y () > e.p2 ().y ()) {
    e.swap_points ();
  }

  if (y <= e.p1 ().y ()) {
    if (y == e.p2 ().y ()) {
      // [[ZH]] ★ 本函数的关键分支：水平边（p1.y == p2.y == y）没有唯一交点，
      // [[ZH]] 取最小 x 以保证排序稳定、确定。
      return std::min (e.p1 ().x (), e.p2 ().x ());
    } else {
      return e.p1 ().x ();
    }
  } else if (y >= e.p2 ().y ()) {
    return e.p2 ().x ();
  } else {
    // [[ZH]] 线段内部插值。运算次序是契约的一部分，不可重排。
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
  // [[ZH]] 参数：y 当前扫描线的 y 坐标。比较结果依赖于它（边的 x 位置随 y 变化）。
  EdgeXAtYCompare2 (db::Coord y)
    : m_y (y) { }

  // [[ZH-BEGIN]]
  // 功能：★ 阶段 4 的**核心比较器** —— 决定同一扫描线上各边的处理次序。
  //       先按"边与扫描线的交点 x"升序；x 相同时再按边的方向/上下关系定序。
  //
  // 参数：a、b 两条待比较的边。
  // 返回：true 表示 a 应排在 b 之前。
  //
  // 【为什么需要一个"加强版"比较器，而不是简单比 x】
  //   扫描线自下而上推进时，两条边可能**恰好相交于一点**（相切）。
  //   此时它们的 x 相等，若不定序，处理顺序不定，环绕数就可能算错。
  //   因此 x 相等时还必须考虑两条边的走向（dy 的符号、端点在上方还是下方），
  //   以保证"y 前进后扫描线上的左右次序保持一致"（源码注释称之为
  //   "preserves the scanline order"）。这是布尔运算正确性的关键细节。
  //
  // 【★ volatile double 是**正确性**手段，不是性能优化 —— 不要删】
  //   见下面 HINT 注释：声明为 volatile 会强制 xa/xb 写回内存，
  //   从而阻止编译器把浮点计算保持在 x87 寄存器中。
  //   原因：x87 浮点寄存器是 80 位扩展精度，写回内存会截断为 64 位。
  //   若不 volatile，同一个值可能在"寄存器中比较"和"内存中比较"时得到不同结果，
  //   或在不同优化级别/不同编译单元下产生不同尾数 →
  //   `xa != xb` 的结果变得不稳定 → 排序不稳定 → 结果不可复现。
  //   volatile 把两条边都降级为 64 位 double 再比较，使得比较**逐位确定**。
  //   删除 volatile 可能不会立刻出错，但会引入极难排查的偶发错误。
  // [[ZH-END]]
  bool operator() (const db::Edge &a, const db::Edge &b) const
  {
    //  simple cases ..
    // [[ZH]] 快速分支①：两条都是垂直边（dx == 0）→ 直接比 x，无需插值。
    if (a.dx () == 0 && b.dx () == 0) {
      return a.p1 ().x () < b.p1 ().x ();
    } else if (edge_xmax (a) < edge_xmin (b)) {
      // [[ZH]] 快速分支②：a 的 x 范围完全在 b 左侧 → a 在前（无需插值）。
      return true;
    } else if (edge_xmin (a) > edge_xmax (b)) {
      // [[ZH]] 快速分支③：a 完全在 b 右侧 → a 在后。
      return false;
    } else {

      //  complex case:
      //  HINT: "volatile" forces xa and xb into memory and disables FPU register optimisation.
      //  That way, we can exactly compare doubles afterwards.
      // [[ZH]] ★ volatile 是刻意的：强制 64 位精度比较，保证结果逐位确定。
      // [[ZH]] 这不是防编译器乱序的性能考量，而是**可复现性**的必要条件。见上方说明。
      volatile double xa = edge_xaty2 (a, m_y);
      volatile double xb = edge_xaty2 (b, m_y);

      if (xa != xb) {
        // [[ZH]] x 不同：常规情形，小者在前。
        return xa < xb;
      } else if (a.dy () == 0) {
        // [[ZH]] x 相同且 a 是水平边：让 a 排在后面（水平边不改变环绕数，
        // [[ZH]] 放在竖直边之后处理可保持次序语义清晰）。
        return false;
      } else if (b.dy () == 0) {
        // [[ZH]] 对称地：b 是水平边 → a 在前。
        return true;
      } else {

        //  In that case the edges will not intersect but rather touch in one point. This defines
        //  a sorting which preserves the scanline order of the edges when y advances.
        // [[ZH]] ★ 两条斜/竖边在扫描线上 x 相同 → 它们在此点相切。
        // [[ZH]] 必须给出确定次序，否则 y 前进后左右关系可能反转，导致环绕数算错。

        db::Edge ea (a);
        db::Edge eb (b);

        // [[ZH]] 归一化方向为 dy >= 0，使"方向比较"有统一基准。
        if (ea.dy () < 0) {
          ea.swap_points ();
        }
        if (eb.dy () < 0) {
          eb.swap_points ();
        }

        // [[ZH]] fa/fb：该边是否向上延伸越过当前扫描线（决定"继续活跃"与否）。
        bool fa = ea.p2 ().y () > m_y;
        bool fb = eb.p2 ().y () > m_y;

        if (fa && fb) {
          //  Both edges advance
          // [[ZH]] 两边都继续向上：用叉积符号比较斜率，保证"更左的边"有确定次序。
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

// [[ZH]] 功能：构造。只是把参数存为成员（m_function 由 polygon_mode 构造）。
// [[ZH]] 参数含义见 .h 中对 mode_t / include_touching / polygon_mode 的说明。
EdgePolygonOp::EdgePolygonOp (EdgePolygonOp::mode_t mode, bool include_touching, int polygon_mode)
  : m_mode (mode), m_include_touching (include_touching),
    m_function (polygon_mode),
    m_wcp_n (0), m_wcp_s (0)
{
}

// [[ZH]] 功能：把多边形自身的环绕数（上/下侧）归零，回到确定初始状态。
// [[ZH]] 修改：m_wcp_n = m_wcp_s = 0。
void EdgePolygonOp::reset () 
{ 
  m_wcp_n = m_wcp_s = 0;
}

// [[ZH-BEGIN]]
// ============================================================================
//  select_edge —— ★ 本类**唯一**的产出通道（这是理解本类的关键）
// ============================================================================
// 功能：判断一条候选边相对于多边形是"在内"还是"在外"，并返回对应的**标签(tag)**。
//
// 参数：horizontal 该边是否水平；
//       p          该边的 property。
// 返回：tag > 0 → 引擎会调用 sink->put(edge, tag) 投递这条边；
//       tag = 0 → 不投递。
//         具体取值：Inside 模式选中→1；Outside 模式选中→1；
//                   Both 模式 内部→1、外部→2（用标签区分，供 sink 分流）。
//
// ★★ 为什么本类不用常规的 edge() 返回值通道：
//   普通评估器靠 edge() 返回 ±1 来"构造"输出边界（重新生成几何）。
//   而 EdgePolygonOp 的语义是"从已有边里**挑选**"—— 它要把输入的边原样输出，
//   而不是重新合成轮廓。因此它让 edge() 恒返回 0（见下），
//   改用 select_edge + selects_edges()==true 这条专门通道。
//   这也解释了为何本类的 selects_edges() 恒为 true。
//
// ★ 两个判据细节：
//   1) `p == 0` 直接返回 0 —— property 0 是多边形**自身**的边，
//      它只用来建立环绕数，不是被挑选的对象。这对应 .h 里的输入约定。
//   2) 水平边要**同时看两侧**（m_wcp_n 与 m_wcp_s）：
//      因为水平边恰好躺在扫描线上，它到底算"在内部"还是"在外部"是有歧义的。
//      因此：include_touching 时只要**任一侧**在内部就算内部（取"或"）；
//            否则要求**两侧都在**内部才算（取"且"）。
//      这是刻意处理退化情形，不是笔误。垂直边无此歧义，只看北侧即可。
//
// 实现：先用 m_function（由 polygon_mode 决定的内外规则）把环绕数转成 bool，
//       再根据 m_mode 映射成标签。
// [[ZH-END]]
int EdgePolygonOp::select_edge (bool horizontal, property_type p)
{
  if (p == 0) {
    // [[ZH]] property 0 = 多边形自身的边，只参与建环绕数，不作为被挑选对象。
    return 0;
  }

  bool inside;

  if (horizontal) {
    // [[ZH]] 水平边：躺在扫描线上，内外有歧义 → 需要看两侧。
    if (m_include_touching) {
      // [[ZH]] 允许相切：任一侧在内部就算内部。
      inside = (m_function (m_wcp_n) || m_function (m_wcp_s));
    } else {
      // [[ZH]] 不允许相切：要求两侧都在内部。
      inside = (m_function (m_wcp_n) && m_function (m_wcp_s));
    }
  } else {
    // [[ZH]] 非水平边：位置明确，只看北侧环绕数。
    inside = m_function (m_wcp_n);
  }

  // [[ZH]] 根据模式把 bool 映射为标签：Inside 选内部；Outside 选外部；
  // [[ZH]] Both 则内部→1、外部→2（用标签区分供 sink 分流）。
  if (m_mode == Inside) {
    return inside ? 1 : 0;
  } else if (m_mode == Outside) {
    return inside ? 0 : 1;
  } else {
    return inside ? 1 : 2;
  }
}

// [[ZH-BEGIN]]
// 功能：维护多边形的环绕数 —— ★ 但**恒返回 0**，不产生任何输出边界。
//
// 参数：north 上/下侧；enter ±1；p 该边 property。
// 返回：恒为 0（本类不走 edge() 输出通道，见 select_edge 的说明）。
//
// 修改：**只对 property == 0**（多边形自身的边）累加对应侧的环绕数 m_wcp_n/s。
//       其它 property 的边完全不理会 —— 它们不是"边界"，只是待挑选的对象。
//
// 对比：普通评估器（如 SimpleMerge）的 edge() 会算"状态迁移量"返回 ±1/0，
//       因为它们要靠返回值重建几何。本类只做计数，输出交给 select_edge。
// [[ZH-END]]
int EdgePolygonOp::edge (bool north, bool enter, property_type p) 
{ 
  if (p == 0) {
    // [[ZH]] 只统计多边形自身边的环绕数，其它 property 忽略。
    int *wc = north ? &m_wcp_n : &m_wcp_s;
    if (enter) {
      ++*wc;
    } else {
      --*wc;
    }
  }

  return 0; 
}

// [[ZH]] 功能：两侧环绕数是否都已归零（即不再位于任何多边形内部）。
// [[ZH]] 用途：供引擎判定"整段无变化"以启用 skip_n 优化。
bool EdgePolygonOp::is_reset () const 
{ 
  return (m_wcp_n == 0 && m_wcp_s == 0);
}

// [[ZH]] 功能：把 include_touching 作为"相切偏好"上报给引擎。
// [[ZH]] 重要连带作用：引擎会把它当作 edge() 的 **enter** 实参传下去，
// [[ZH]] 因此它直接影响环绕数的加减方向（见 .h 文件头"易踩的坑"）。
bool EdgePolygonOp::prefer_touch () const 
{ 
  return m_include_touching; 
}

// [[ZH]] 功能：恒返回 true —— 启用 select_edge 通道。
// [[ZH]] ★ 这是本类产出结果的唯一途径，因此**必须**为 true，不可改成 false。
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

// [[ZH]] 功能：清空所有状态 —— 每个 property 的环绕数、两侧"当前在内部"的集合。
// [[ZH]] 修改：m_wcv_n/m_wcv_s 与 m_inside_n/m_inside_s 全部清空。
// [[ZH]] 注意：**不清空 m_interactions/m_non_interactions**（结果集）——
// [[ZH]] 结果是在整个扫描过程中累积的，不能随每行扫描线被清掉。
void
InteractionDetector::reset ()
{
  m_wcv_n.clear ();
  m_wcv_s.clear ();
  m_inside_n.clear ();
  m_inside_s.clear ();
}

// [[ZH]] 功能：按 property 总数 n 预分配两侧环绕数数组，并清空内部集合。
// [[ZH]] 参数：n 需覆盖的 property 个数（= 最大 property + 1）。
// [[ZH]] 为何必须：edge() 里有 tl_assert(p < m_wcv_n.size())，未 reserve 会断言失败。
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

// [[ZH-BEGIN]]
// ============================================================================
//  InteractionDetector::edge —— 记录多边形间的相互作用（本文件最绕的一段逻辑）
// ============================================================================
// 功能：根据这条边更新对应 property 的环绕数，并在"张开/闭合"的时刻
//       把发现的相互作用记入 m_interactions / m_non_interactions。
//
// 参数：north 上/下侧；enter ±1；p 边的 property。
// 返回：恒为 0 —— 本类**不输出几何**，只记录配对关系（见 .h 的类说明）。
//
// 修改：m_wcv_n[p] 或 m_wcv_s[p]、m_inside_n/m_inside_s、
//       m_interactions、m_non_interactions。
//
// 【第一步：本事件是否需要处理】
//   条件：north || (mode==0 && include_touching) || (mode<-1 && include_touching)
//   源码注释解释了原因："interacting"（0）与 "enclosing"（-2）模式必须
//   **南北两侧都看**，才能捕获扫掠线两侧对象之间的交互（例如上下相邻相切）。
//   其余情形的南侧事件可以跳过，这是纯优化。
//
// 【第二步：判断张开还是闭合，以及由此推导关系】
//   环绕数由 0 变非零 = **张开**（进入该多边形）；由非零变 0 = **闭合**（离开）。
//
//   —— 闭合分支（inside_after < inside_before）：
//      把 p 从"在内部集合"中移除。若 p 是 **primary**，则剩下仍在集合里的
//      secondary 都意味着"与 p 不相交"→ 记入 m_non_interactions。
//      为何能这么断言：源码注释指出，由于 prefer_touch==true 且重合边按
//      property 排序，**primary 对象在所有重合边中最后被处理**，
//      因此此刻仍"张开"的东西必然在其外部。
//
//   —— 张开分支（inside_after > inside_before）：
//      · mode != 0（inside/enclosing/outside 模式）：
//          若 p 是 secondary → 遍历当前所有 primary，逐个记 (primary, p)；
//            若一个 primary 都没找到 → p 记入 non_interactions（在外部）。
//          若 p 是 primary   → 遍历当前所有 secondary，逐个记 (p, secondary)；
//            在 enclosing(-2) 模式下额外把该 secondary 记入 non_interactions
//            （因为 primary 张开时 secondary 已开着且未闭合 → 二者重叠而非包含）。
//      · mode == 0（重叠/相切模式）：
//          把 p 与当前**南北两侧**所有张开的对象配对，
//          并把每对规范化为"较小的 property 在前"—— 这使输出顺序确定。
//          这是 result 里说的"配对中较小者为第一元素"约定的实现处。
//      最后把 p 插入本侧的"在内部集合"。
//
// 坑：本函数多次用"在集合里找 primary/secondary"的线性遍历（集合通常很小）；
//     不要误以为它做了排序或索引 —— 复杂度靠对象数量小来保证。
// [[ZH-END]]
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
  // [[ZH]] 决定本事件是否处理：interacting(0) 与 enclosing(-2) 模式需要南北都看，
  // [[ZH]] 否则会漏掉扫描线两侧对象之间的交互。这是逻辑必要，不是优化。
  if (north || (m_mode == 0 && m_include_touching) || (m_mode < -1 && m_include_touching)) {

    std::set <property_type> *inside = north ? &m_inside_n : &m_inside_s;

    if (inside_after < inside_before) {

      // [[ZH]] ===== 闭合分支：p 离开该区域 =====
      inside->erase (p);

      //  the primary objects are delivered last of all coincident edges
      //  (due to prefer_touch == true and the sorting of coincident edges by property id)
      //  hence every remaining parts count as non-interacting (outside)
      if (p <= m_last_primary_id) {
        // [[ZH]] p 是 primary：此刻仍张开的 secondary 都在 p 外部 → 记为 non-interaction。
        // [[ZH]] 依据：primary 在重合边中最后被处理，故剩余者必在其外。
        for (std::set <property_type>::const_iterator i = inside->begin (); i != inside->end (); ++i) {
          if (*i > m_last_primary_id) {
            m_non_interactions.insert (*i);
          }
        }
      }

    } else if (inside_after > inside_before) {

      // [[ZH]] ===== 张开分支：p 进入该区域 =====
      if (m_mode != 0) {

        //  enclosing/inside/outside mode
        if (p > m_last_primary_id) {

          //  note that the primary parts will be delivered first of all coincident
          //  edges hence we can check whether the primary is present even for coincident
          //  edges
          // [[ZH]] p 是 secondary → 找当前张开的 primary，逐个记录配对。
          bool any = false;
          for (std::set <property_type>::const_iterator i = inside->begin (); i != inside->end (); ++i) {
            if (*i <= m_last_primary_id) {
              any = true;
              m_interactions.insert (std::make_pair (*i, p));
            }
          }
          if (! any) {
            // [[ZH]] 一个 primary 都没有 → 该 secondary 在外部。
            // [[ZH]] （outside 模式的伪配对由此推导，见 finish()）
            m_non_interactions.insert (p);
          }

        } else {

          // [[ZH]] p 是 primary → 找当前张开的 secondary，逐个记录配对。
          for (std::set <property_type>::const_iterator i = inside->begin (); i != inside->end (); ++i) {
            if (*i > m_last_primary_id) {
              if (m_mode < -1) {
                //  enclosing mode: an opening primary (= enclosing one) with open secondaries means the secondary
                //  has been opened before and did not close. Because we sort by property ID this must have happened
                //  before, hence the secondary is overlapping. Make them non-interactions. We still have to record them
                //  as interactions because this is how we skip the primaries later.
                // [[ZH]] enclosing 模式：primary 张开时 secondary 已开着且未闭合 →
                // [[ZH]] 说明二者重叠而非包含，故记为 non-interaction；
                // [[ZH]] 但仍要记入 interactions（finish() 靠它反推需要剔除哪些 primary）。
                m_non_interactions.insert (*i);
              }
              m_interactions.insert (std::make_pair (p, *i));
            }
          }

        }

      } else {

        // [[ZH]] mode == 0（重叠/相切）：与南北**两侧**当前张开的对象全部配对，
        // [[ZH]] 并把每对规范化为"较小 property 在前"以固定输出顺序。
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

      // [[ZH]] 最后把 p 加入本侧的"在内部集合"，供后续事件参考。
      inside->insert (p);

    }

  }

  return 0;
}

// [[ZH]] 功能：恒返回 0 —— 本类不输出几何，不需要比较南北差异。
int 
InteractionDetector::compare_ns () const
{
  return 0;
}

// [[ZH-BEGIN]]
// ============================================================================
//  finish —— 把扫描期间累积的原始记录**后处理**成最终结果（按 mode 分流）
// ============================================================================
// 功能：对 mode != 0 的情形，在读取结果**之前**必须调用（见 .h 说明）。
//       它用 m_non_interactions 这个"排除集"去修正 m_interactions。
//
// 修改：重写 m_interactions，并清空 m_non_interactions。
//
// 【为什么必须延迟到扫描结束】
//   在扫描过程中我们只能知道"某个对象当下是否在某处内部"，
//   但"谁包含谁""谁完全在外部"这类全局结论要等所有边处理完才能定。
//   因此扫描期先把正/反两类证据分别记下，最后在这里做集合运算得出答案。
//
// 【三种模式的不同后处理】
//   mode < -1（enclosing，谁包含谁）：
//     先找出"与某个会被判为外部的 secondary 有交互"的那些 primary，
//     把它们整批删掉（它们的"包含"是被重叠污染的，不可信）。
//   mode == -1（inside，谁在谁里面）：
//     直接删除那些 secondary 被判为 non-interaction 的配对条目。
//   mode > 0（outside，谁在谁外面）：
//     保留**从未参与过任何交互**的 secondary —— 它们就是"在外部"的。
//     并把它们与 m_last_primary_id 配成对（★ 这是伪配对：
//     按定义外部的多边形与 primary 并不相交，因此 primary_id 恒定
//     为 last_primary_id，相当于一个代表"背景"的合成 id）。
//     上游 .h 文档已就此提醒使用者不要误认为是真实几何关系，这里再标一次。
//
// 坑：本函数会**清空 m_non_interactions**。若需要多次读取结果，
//     应只调用一次 finish() 然后反复遍历 begin()/end()。
// [[ZH-END]]
void
InteractionDetector::finish ()
{
  if (m_mode < -1) {

    //  In enclosing mode remove those objects which have an interaction with a secondary having a non-interaction:
    //  these are the ones where secondaries overlap and stick to the outside.
    // [[ZH]] 先收集需要被剔除的 primary。
    std::set<property_type> primaries_to_delete;
    for (std::set<std::pair<property_type, property_type> >::iterator i = m_interactions.begin (); i != m_interactions.end (); ++i) {
      if (m_non_interactions.find (i->second) != m_non_interactions.end ()) {
        primaries_to_delete.insert (i->first);
      }
    }

    // [[ZH]] 再删除这些 primary 参与的全部配对。
    // [[ZH]] 注意这里用"先自增副本 ii、再 erase(i)、最后 i=ii"的写法，
    // [[ZH]] 避免 erase 后迭代器失效（std::set::erase 只失效被删元素）。
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
    // [[ZH]] inside 模式：删除 secondary 已被判为外部的配对。
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
    // [[ZH]] outside 模式：先从未交互集合中剔除掉"曾参与交互"的对象。
    for (iterator pp = begin (); pp != end (); ++pp) {
      m_non_interactions.erase (pp->second);
    }

    // [[ZH]] 然后用剩余的"从未交互"对象重建结果集 —— 它们就是"在外部"的。
    // [[ZH]] ★ 配对的 primary 恒为 m_last_primary_id（伪配对，见上方说明）。
    m_interactions.clear ();
    for (std::set<property_type>::const_iterator p = m_non_interactions.begin (); p != m_non_interactions.end (); ++p) {
      m_interactions.insert (m_interactions.end (), std::make_pair (m_last_primary_id, *p));
    }

  }

  // [[ZH]] 排除集已完成使命，清空（避免影响后续可能的重复 finish 调用）。
  m_non_interactions.clear ();
}

// -------------------------------------------------------------------------------
//  MergeOp implementation

// [[ZH]] 功能：构造。min_wc 即最小重叠阈值（语义见 .h 中 MergeOp 的说明）。
// [[ZH]] 初始化 m_zeroes = 0 —— 注意它在 reserve() 之前不是真实值，
// [[ZH]] 必须先 reserve(n_props) 才能得到正确的 is_reset() 判定。
MergeOp::MergeOp (unsigned int min_wc)
  : m_wc_n (0), m_wc_s (0), m_min_wc (min_wc), m_zeroes (0)
{
  //  .. nothing yet ..
}

// [[ZH]] 功能：清空所有计数，回到确定初始状态。
// [[ZH]] 修改：两侧 m_wcv 清空；m_wc_n/m_wc_s 归零；m_zeroes 归零。
// [[ZH]] 坑（上游遗留）：这里写了两遍 `m_wc_n = 0;` 而**没有重置 m_wc_s**。
// [[ZH]]     单纯看容易以为是笔误。实际影响很小：reserve() 会重设全部计数，
// [[ZH]]     而引擎在每次处理前都会先 reserve()，因此 m_wc_s 总会被重新赋值。
// [[ZH]]     这里仅作记录，不改动代码。
void  
MergeOp::reset ()
{
  m_wcv_n.clear ();
  m_wcv_s.clear ();
  m_wc_n = 0;
  m_wc_n = 0;
  m_zeroes = 0;
}

// [[ZH]] 功能：按 property 总数 n 预分配两侧的"每 property 环绕数"数组。
// [[ZH]] 参数：n = 最大 property + 1。
// [[ZH]] 修改：m_wcv_n/m_wcv_s 尺寸置 n 且全 0；m_zeroes = 2*n（两侧全部归零）。
// [[ZH]] 为何 m_zeroes 是 2*n：它统计"两侧共 2n 个槽位中仍为零的个数"，
// [[ZH]] 因此初始时全部为零。is_reset() 即比较它与两侧总槽位数。
void 
MergeOp::reserve (size_t n)
{
  m_wcv_n.clear ();
  m_wcv_s.clear ();
  m_wcv_n.resize (n, 0);
  m_wcv_s.resize (n, 0);
  m_zeroes = 2 * n;
}

// [[ZH-BEGIN]]
// 功能：MergeOp 的判定核心 —— 给定"当前张开的多边形个数 wc"，判断该位置是否算内部。
//
// 参数：wc 当前张开（重叠）的多边形个数；min_wc 构造时给定的阈值。
// 返回：true = 算作内部。
//
// ★★ 判定是**严格大于**：`wc > min_wc`（而不是 >=）。
//    代入可得：min_wc = 0 → wc >= 1 → "至少 1 个"→ 输出全部多边形（普通融合）
//              min_wc = 1 → wc >= 2 → "至少 2 个重叠处才输出"
//              min_wc = n → wc >= n+1
//    .h 中类文档写的 "0: all polygons, 1: at least two overlapping" **是对的**，
//    与这里一致（我早先曾怀疑文档差一，核对后确认是我错了）。
//    但请留意"严格大于"这个细节，因为 wc 是**个数**，初看容易数错。
//
// 注意：本类同时只统计了 m_wc_n/m_wc_s 两个"张开个数"，
//       配合 compare_ns() 即可得出"上/下两侧是否在内部"的差异。
// [[ZH-END]]
static inline 
bool result_by_mode (int wc, unsigned int min_wc)
{
  // [[ZH]] ★ 严格大于。min_wc 为 unsigned，故显式转 int 再比较。
  return wc > int (min_wc);
}

// [[ZH-BEGIN]]
// ============================================================================
//  MergeOp::edge —— ★ 本文件里"累加式返回值"机制最清楚的范例
// ============================================================================
// 功能：更新该 property 的环绕数，并在必要时修正"张开个数"，
//       再返回**状态迁移量**（而非布尔值）。
//
// 参数：north 上/下侧；enter ±1；
//       p     该边的 property —— ★ 本类会用它，按 property 分开计数。
// 返回：+1 = 由"不满足重叠条件"变为满足（比如第 2 个多边形开始重叠）；
//       -1 = 由满足变为不满足；
//        0 = 判定结果未变。
//
// 修改：m_wcv_n[p] / m_wcv_s[p]（±1）、m_wc_n / m_wc_s（张开个数）、m_zeroes。
//
// 【★ 为什么 m_wc 不能直接对每个事件加一】
//   同一个多边形可能多次穿越同一条扫描线（z 字形边），因此 wcv 可能从 +1 变到 +2 甚至回负。
//   而"这个多边形是否张开"只看 wcv **是否为零**。
//   所以只有在"零/非零"这一跳变真的发生时，才去调整 m_wc。
//   这就是下面 inside_before != inside_after 判断的意义 —— 它把
//   "环绕数变化"过滤成"张开状态变化"，避免重复计数同一个多边形。
//
// 【返回值的用途】
//   引擎会把返回值**累加**到 m_pn/m_ps（见 EdgeProcessorState::north_edge），
//   非零即代表此处存在结果边界，符号决定输出边的方向。
//   因此这里返回 res_after - res_before（bool 相减得 -1/0/+1）。
// [[ZH-END]]
int 
MergeOp::edge (bool north, bool enter, property_type p)
{
  tl_assert (p < m_wcv_n.size () && p < m_wcv_s.size ());

  // [[ZH]] 按 north 选中"该 property 的环绕数"与"张开个数"两个计数器。
  int *wcv = north ? &m_wcv_n [p] : &m_wcv_s [p];
  int *wc = north ? &m_wc_n : &m_wc_s;

  // [[ZH]] 更新该 property 的环绕数（enter 决定加减），并判断"张开"状态是否跳变。
  bool inside_before = (*wcv != 0);
  *wcv += (enter ? 1 : -1);
  bool inside_after = (*wcv != 0);
  // [[ZH]] m_zeroes：仍为零的槽位数。张开一个则零槽位-1，闭合一个则+1。
  m_zeroes += (!inside_after) - (!inside_before);
#ifdef DEBUG_MERGEOP
  printf ("north=%d, enter=%d, prop=%d -> %d\n", north, enter, int (p), int (m_zeroes));
#endif
  tl_assert (long (m_zeroes) >= 0);

  // [[ZH]] 记录变更前的判定结果。
  bool res_before = result_by_mode (*wc, m_min_wc);
  if (inside_before != inside_after) {
    // [[ZH]] ★ 只有"张开状态"真的跳变时才调整张开个数 ——
    // [[ZH]] 这样同一个多边形的多次穿越不会被数成多个多边形。
    *wc += (inside_after - inside_before);
  }
  // [[ZH]] 变更后的判定结果。二者之差就是累加式返回值。
  bool res_after = result_by_mode (*wc, m_min_wc);

  // [[ZH]] bool 相减 → -1 / 0 / +1，即状态迁移量。
  return res_after - res_before;
}

// [[ZH]] 功能：比较“下方”与“上方”的重叠判定之差，供引擎给合成的水平边定方向。
// [[ZH]] 返回：result_by_mode(north) - result_by_mode(south)，即 -1 / 0 / +1。
int 
MergeOp::compare_ns () const
{
  return result_by_mode (m_wc_n, m_min_wc) - result_by_mode (m_wc_s, m_min_wc);
}

// -------------------------------------------------------------------------------
//  BooleanOp implementation

// [[ZH]] 功能：构造。mode 取 BooleanOp::BoolOp（And/ANotB/BNotA/Xor/Or）。
// [[ZH]] 初始化四个操作数计数为 0（wca/wcb × north/south）。
BooleanOp::BooleanOp (BoolOp mode)
  : m_wc_na (0), m_wc_nb (0), m_wc_sa (0), m_wc_sb (0), m_mode (mode), m_zeroes (0)
{
  //  .. nothing yet ..
}

// [[ZH]] 功能：清空所有计数，回到确定初始状态。
// [[ZH]] 修改：两侧 m_wcv 清空；四个 A/B 计数归零；m_zeroes 归零。
void  
BooleanOp::reset ()
{
  m_wcv_n.clear ();
  m_wcv_s.clear ();
  m_wc_na = m_wc_sa = 0;
  m_wc_nb = m_wc_sb = 0;
  m_zeroes = 0;
}

// [[ZH]] 功能：按 property 总数 n 预分配两侧的"每 property 环绕数"数组。
// [[ZH]] 参数：n = 最大 property + 1。
// [[ZH]] 为何必须：edge_impl() 里有 tl_assert(p < m_wcv_n.size())，未 reserve 会断言失败。
void 
BooleanOp::reserve (size_t n)
{
  m_wcv_n.clear ();
  m_wcv_s.clear ();
  m_wcv_n.resize (n, 0);
  m_wcv_s.resize (n, 0);
  m_zeroes = 2 * n;
}

// [[ZH-BEGIN]]
// 功能：布尔运算的**真值表** —— 给定 A、B 两侧的"是否在内部"，算出结果是否为内部。
//
// 参数：wca A 操作数当前的张开个数；wcb B 操作数当前的张开个数；
//       inside_a / inside_b 把各自的个数转成 bool 的谓词（通常就是 NonZeroInsideFunc）。
// 返回：该位置是否算在结果内部。
//
// ★ 注意这里对 A 和 B **分别**应用谓词，而不是先把两边合并再判断 ——
//   这正是 BooleanOp 能实现集合运算的关键。
//
// 五种运算的一一对应（可当作真值表背下）：
//   And   (交集)   inside_a && inside_b
//   ANotB (差集)   inside_a && !inside_b
//   BNotA (差集)   !inside_a && inside_b
//   Xor   (对称差) 两者异或
//   Or    (并集)   两者取或
//   default        false（未知模式安全地当作空集）
//
// 模板参数 InsideFunc 允许换用不同内外规则 ——
//   BooleanOp  传 NonZeroInsideFunc（固定非零环绕）
//   BooleanOp2 传 ParametrizedInsideFunc（可分别指定 mode）
//   两者共用本模板，这就是 BooleanOp2 能复用的原因。
// [[ZH-END]]
template <class InsideFunc>
inline bool 
BooleanOp::result (int wca, int wcb, const InsideFunc &inside_a, const InsideFunc &inside_b) const
{
  // [[ZH]] 按 m_mode 查表。注意判据用的是"张开个数"（wca/wcb），
  // [[ZH]] 而不是原始环绕数 —— 因为每个多边形先各自归一化过（见 edge_impl）。
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
    // [[ZH]] 未知模式：安全地当作空集，不输出任何东西。
    return false;
  }
}

// [[ZH-BEGIN]]
// ============================================================================
//  BooleanOp::edge_impl —— ★ 本文件的两层计数机制（理解布尔运算的核心）
// ============================================================================
// 功能：更新对应 property 的环绕数、必要时修正 A/B 各自的"张开个数"，
//       并返回布尔运算结果的状态迁移量。
//
// 参数：north/enter 同惯例；p 该边 property；inside_a/inside_b 两侧的内外谓词。
// 返回：+1 = 结果由外变内；-1 = 由内变外；0 = 未变。
//
// 修改：m_wcv_n[p]/m_wcv_s[p]、m_wc_na/m_wc_sa 或 m_wc_nb/m_wc_sb、m_zeroes。
//
// 【★★ 两层计数：本类最容易看漏的设计】
//   第一层 m_wcv_*[p]  ：**每个 property（即每个多边形）各自的**环绕数。
//                        它只用于判断"这个多边形自己是否张开"（wcv != 0）。
//                        —— 这就是"先各自归一化"，使自相交/自重叠不污染计数。
//   第二层 m_wc_na / m_wc_nb：**A 组 / B 组当前各有多少个多边形张开**。
//                        这才是送进 result() 真值表的 wca/wcb。
//   两层的桥梁：只有当某个 property 的张开状态**发生跳变**时，
//   才把它的值 ±1 计入所属组（m_wca 或 m_wcb）。
//   —— 这与 MergeOp 的思路完全一致，只是分成两组。
//
// 【★ 操作数归属：靠 property 的奇偶性】
//   源码里用 `(p % 2) == 0` 判断：
//        偶数 property → 操作数 **A**（对应 m_wc_na / m_wc_sa）
//        奇数 property → 操作数 **B**（对应 m_wc_nb / m_wc_sb）
//   这个约定必须在**入口处**就遵守：
//        boolean() 的多边形版本会自动分配 A=0,2,4... / B=1,3,5...
//        边输入版本则只分 A=0 / B=1
//   若你手工 insert 并自行填 property，不符合奇偶约定就会把结果算错。
//
// 实现顺序：
//   ① 用 inside_a/inside_b 根据 p 的奇偶取得该 property 变更前的判定；
//   ② 更新 m_wcv[p]（±1），得到变更后的判定；
//   ③ 维护 m_zeroes；
//   ④ 记录变更前 result() 值；若张开状态跳变 → 把 ±1 加到所属组计数；
//   ⑤ 返回 result() 的新旧之差。
// [[ZH-END]]
template <class InsideFunc>
inline int 
BooleanOp::edge_impl (bool north, bool enter, property_type p, const InsideFunc &inside_a, const InsideFunc &inside_b) 
{
  tl_assert (p < m_wcv_n.size () && p < m_wcv_s.size ());

  // [[ZH]] 三组计数器：该 property 自己的环绕数，以及 A 组、B 组的张开个数。
  int *wcv = north ? &m_wcv_n [p] : &m_wcv_s [p];
  int *wca = north ? &m_wc_na : &m_wc_sa;
  int *wcb = north ? &m_wc_nb : &m_wc_sb;

  // [[ZH]] ★ 靠 property 奇偶选择该归哪一组判定：偶数→A，奇数→B。
  bool inside_before = ((p % 2) == 0 ? inside_a (*wcv) : inside_b (*wcv));
  *wcv += (enter ? 1 : -1);
  bool inside_after = ((p % 2) == 0 ? inside_a (*wcv) : inside_b (*wcv));
  m_zeroes += (!inside_after) - (!inside_before);
#ifdef DEBUG_BOOLEAN
  printf ("north=%d, enter=%d, prop=%d -> %d\n", north, enter, int (p), int (m_zeroes));
#endif
  tl_assert (long (m_zeroes) >= 0);

  // [[ZH]] 变更前的布尔运算结果（用 A/B 两组张开个数算）。
  bool res_before = result (*wca, *wcb, inside_a, inside_b);
  if (inside_before != inside_after) {
    // [[ZH]] ★ 只有该 property 的张开状态跳变，才把它计入所属组 ——
    // [[ZH]] 避免同一个多边形多次穿越被数成多个。
    if ((p % 2) == 0) {
      *wca += (inside_after - inside_before);
    } else {
      *wcb += (inside_after - inside_before);
    }
  }
  bool res_after = result (*wca, *wcb, inside_a, inside_b);

  // [[ZH]] bool 相减 → -1/0/+1，即状态迁移量。
  return res_after - res_before;
}

// [[ZH]] 功能：比较下方与上方的布尔运算结果之差，供引擎给合成的水平边定方向。
// [[ZH]] 返回：result(north) - result(south)，即 -1/0/+1。
template <class InsideFunc> 
inline int 
BooleanOp::compare_ns_impl (const InsideFunc &inside_a, const InsideFunc &inside_b) const
{
  return result (m_wc_na, m_wc_nb, inside_a, inside_b) - result (m_wc_sa, m_wc_sb, inside_a, inside_b);
}

// [[ZH]] 功能：BooleanOp 的 edge —— 固定用 NonZeroInsideFunc 作为 A、B **共同**的谓词。
// [[ZH]] 注意这里把同一个 inside 实例同时传给 A 和 B。这正是 BooleanOp 的限制：
// [[ZH]]       它无法给 A、B 指定不同的内外规则 —— 那个能力由 BooleanOp2 提供。
int 
BooleanOp::edge (bool north, bool enter, property_type p)
{
  NonZeroInsideFunc inside;
  return edge_impl (north, enter, p, inside, inside);
}

// [[ZH]] 功能：BooleanOp 的 compare_ns —— 同样固定用 NonZeroInsideFunc。
int 
BooleanOp::compare_ns () const
{
  NonZeroInsideFunc inside;
  return compare_ns_impl (inside, inside);
}

// -------------------------------------------------------------------------------
//  BooleanOp2 implementation

// [[ZH]] 功能：构造。在 BooleanOp 基础上额外保存 A、B 各自的融合模式。
// [[ZH]] 参数：wc_mode_a / wc_mode_b 语义同 ParametrizedInsideFunc（-1 非零／0 奇偶／+n／-n）。
BooleanOp2::BooleanOp2 (BoolOp op, int wc_mode_a, int wc_mode_b)
  : BooleanOp (op), m_wc_mode_a (wc_mode_a), m_wc_mode_b (wc_mode_b)
{
  //  .. nothing yet ..
}

// [[ZH-BEGIN]]
// 功能：★ BooleanOp2 相对 BooleanOp 的**唯一实质变化** —— 给 A、B 分别构造谓词。
//
// 说明：这里为每次调用新建两个 ParametrizedInsideFunc（很轻量，只是一个 int），
//       然后复用 BooleanOp 的 edge_impl 模板，从而得到"A、B 各用自己模式"的效果。
//       整个类没有自己的计数逻辑 —— 全部继承自 BooleanOp。
//
// 典型用途（见 .h）：对已做过 sizing 的多边形做布尔运算，
//   需要把自相交产生的重叠圈按"重叠计数"解释（mode > 0）。
//       实例参考：单元测试 TEST(27) 用 BooleanOp2(Xor, -1, -1) 把 4 个角碎片
//                 合并为 1 个带孔多边形。
// [[ZH-END]]
int 
BooleanOp2::edge (bool north, bool enter, property_type p)
{
  // [[ZH]] 用各自的 mode 构造谓词，其余逻辑全交给基类的 edge_impl。
  ParametrizedInsideFunc inside_a (m_wc_mode_a);
  ParametrizedInsideFunc inside_b (m_wc_mode_b);
  return edge_impl (north, enter, p, inside_a, inside_b);
}

// [[ZH]] 功能：BooleanOp2 的 compare_ns —— 同样用各自模式构造谓词。
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

// [[ZH-BEGIN]]
// 功能：把一条边加入处理队列，并标记其来源 property。
//
// 参数：e 待插入的边；p 来源标签（见 dbEdgeProcessor.h 文件头对 property 的说明）。
//
// ★ 注意：**退化边（零长度，p1 == p2）会被静默丢弃**。
//   没有任何返回值、日志或异常提示 —— 这是刻意的（退化边没有方向，
//   无法参与环绕数计算，留着只会制造麻烦），但对调用者来说是"看不见的行为"。
//   如果你发现"插入的边数比 count() 少"，原因通常就在这里。
//
// 其他说明：
//   · 本函数**不做任何几何合法性校验**：自相交、非闭合的边序列都会被照单全收，
//     只会产生奇怪的结果而不会报错。调用者需自行保证输入有意义。
//   · 不预分配容量。批量插入前请先调用 reserve() 以避免反复扩容。
//   · 插入的顺序不重要 —— 阶段 2 会重新排序（除非只调用 redo()）。
// [[ZH-END]]
void
EdgeProcessor::insert (const db::Edge &e, property_type p)
{
  if (e.p1 () != e.p2 ()) {
    // [[ZH]] 只插入非退化边。零长度边在此被静默忽略（见上方说明）。
    mp_work_edges->push_back (WorkEdge (e, p));
  }
}

void
EdgeProcessor::clear ()
{
  mp_work_edges->clear ();
  mp_cpvector->clear ();
}

// [[ZH-BEGIN]]
// 功能：处理**两条共线的水平边** —— 把 e2 的两个端点作为切点登记到 e1 上。
//
// 参数：e1 接收切点的边（将被切分的那条）；
//       e2 提供端点的邻居边（**必须与 e1 同 y**，调用方已检查）；
//       cell 当前 x/y 单元格 —— 只有落在格内的点才登记；
//       cutpoints 全局切点表。
//
// 为什么需要它：两条水平边共线时，它们的"相交"不是一个点，而是**一段重叠区间**。
//   无法用普通交点表示，因此改用一个约定：把 e2 的端点打进 e1，
//   使 e1 在那些位置被切开，从而重叠区间的边界变得明确。
//
// ★ 两个关键细节：
//   1) 判定用的是**严格内部**（`> e1_xmin && < e1_xmax`）—— 端点重合的情况不算，
//      因为那种情形由调用方的 "端点是否相同" 检查另行排除，避免产生零长碎片。
//   2) 登记时 `strong = false` → 写入的是**弱切点(attractor)**，不是强切点。
//      这是刻意的：仅有一对平行水平边并不足以证明必须切分 ——
//      是否真的切，取决于别处是否出现强切点（届时会通过 CutPoints::add 的
//      提升机制把这里的弱切点一并提升）。见 CutPoints 的说明。
// [[ZH-END]]
static void
add_hparallel_cutpoints (WorkEdge &e1, WorkEdge &e2, const db::Box &cell, std::vector <CutPoints> &cutpoints)
{
  db::Coord e1_xmin = std::min (e1.x1 (), e1.x2 ());
  db::Coord e1_xmax = std::max (e1.x1 (), e1.x2 ());
  // [[ZH]] e2 的左端点落在 e1 **内部**（严格不等，排端点）且在格内 → 登记为弱切点。
  if (e2.x1 () > e1_xmin && e2.x1 () < e1_xmax && cell.contains (e2.p1 ())) {
    e1.make_cutpoints (cutpoints)->add (e2.p1 (), &cutpoints, false);
  }
  // [[ZH]] 右端点同理。
  if (e2.x2 () > e1_xmin && e2.x2 () < e1_xmax && cell.contains (e2.p2 ())) {
    e1.make_cutpoints (cutpoints)->add (e2.p2 (), &cutpoints, false);
  }
}

// [[ZH-BEGIN]]
// ============================================================================
//  get_intersections_per_band_90 —— 阶段 2 的**正交快速路径**
// ============================================================================
//
// 功能：在一个 y 带 [y, yy] 内，找出 [current, future) 这批边之间的所有交点，
//       并把它们作为切点登记进 cutpoints。
//
// 适用前提（由调用方判断，本函数不再检查）：★ 本带内的边**全部是正交的**
//       （即每条边要么水平要么垂直，没有斜边）。
//       版图绝大多数图形满足此条件，因此这是最常走的路径。
//
// 参数：cutpoints 全局切点表（输出，会被追加）；
//       current / future 本带边的区间（半开区间 [current, future)）；
//       y / yy  本带的 y 范围；
//       with_h  ★ 是否也要给**水平边**登记切点。
//               为 false 时水平边只作为"切别人的刀"，自己不被切 ——
//               因为阶段 3 默认会把水平边丢弃（除非评估器要逐边处理），
//               切它纯属浪费。对应 EdgeProcessor 的 selects_edges()。
//
// 返回：无（结果通过 cutpoints 输出）。
//
// 【算法结构：三层嵌套的"扫描 + 分格"】
//   ① 先按 xmin 排序，使边可以沿 x 方向有序推进。
//   ② 外层扫 x：维护一个**活跃边集合**，把 xmin ≤ 当前 x 的边纳入；
//      每轮取一个 x 区间 [x, xx)，形成一个矩形单元格 cell = (x, y, xx, yy)。
//      xx 的选取沿用 fill_factor 启发式（与 redo_or_process 里 y 分带同思路）。
//   ③ 在 cell 内做两两配对，只处理**落在同一格内**的边对 —— 这是空间剪枝，
//      避免 O(n²) 的全量两两比较。
//   ④ 每轮结束把 xmax < x 的边从活跃集合中移除（就地压实，见函数末尾）。
//
// ★★ 一个非常重要的细节：本函数用裸的 `intersect_point`，
//    而不是 safe_intersect_point。这是**安全**的，因为两条正交边求交时，
//    db::edge::intersect_point 走的是 "两个包围盒取交集" 的分支：
//        x = max(min(p1x,p2x), min(e1x,e2x))
//        y = max(min(p1y,p2y), min(e1y,e2y))
//    该表达式关于两条边**完全对称**，交换实参不会改变结果。
//    而一般（斜边）路径就不对称了，因此 _any 版本必须用 safe_intersect_point。
//    这解释了为什么两个版本的求交调用方式不同 —— 不是笔误。
//
// 坑：本函数**不检查**边是否真的正交；若本带混入斜边，结果将不正确。
//     判定由调用方（redo_or_process 中的 is90 标志）负责。
// [[ZH-END]]
static void
get_intersections_per_band_90 (std::vector <CutPoints> &cutpoints, std::vector <WorkEdge>::iterator current, std::vector <WorkEdge>::iterator future, db::Coord y, db::Coord yy, bool with_h)
{
  // [[ZH]] ① 按 xmin 排序，使下面的活跃边集合可以沿 x 有序推进。
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
// [[ZH-BEGIN]]
// 功能：db::edge_xaty 的**double y** 版本 —— 求边在 y（可为小数）处的 x。
//
// ★ 何时需要它：_any 版本会把扫描带到**故意放宽 0.5 DBU**，
//   即用 y-0.5 / y+0.5 这样的**分数** y 去求 x（见 get_intersections_per_band_any）。
//   整数版的 edge_xaty 接不了小数 y，故另写此版。
//
// 参数：e 目标边（按值传递，内部会归一化方向）；
//       y 扫描线 y（double，可为小数）。
// 返回：double 的 x 值。
//
// 与 edge_xaty 完全相同的**可复现性契约**：
//   同一 (边, y) 必须返回逐位相同的 double —— 否则排序不稳定。
//   因此这里的插值表达式顺序同样不可重排。
//
// 注意：本函数**没有** edge_xaty2 那个"水平边取最小 x"的特殊处理。
//   它只用于计算带的左右边界（那里水平边有单独的 if 分支处理），
//   而非用于最终的扫描线排序。
// [[ZH-END]]
template <class C>
inline double edge_xaty_double (db::edge<C> e, double y)
{
  // [[ZH]] 归一化为 p1.y <= p2.y，使同一条边只有一种参数化。
  if (e.p1 ().y () > e.p2 ().y ()) {
    e.swap_points ();
  }

  // [[ZH]] 三段式：低于起点→夹到起点 x；高于终点→夹到终点 x；否则插值。
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
// [[ZH-BEGIN]]
// 功能：求该边在 y 区间 [y1, y2] 内的**最左 x**（即 x 的下界），并向下取整。
//
// ★ 用途（为什么需要它）：_any 路径的扫描带被放宽到 [y-0.5, y+0.5]，
//   因此一条斜边在带内可能跨越不同的 x。要判断"哪些边可能有交互"，
//   必须用边在整个带内的 x 范围（而不是某一条扫描线上的瞬时 x）来做排序与剪枝。
//   本函数提供这个范围的下界。
//
// 参数：e 目标边；y1 / y2 区间下/上界（double，带 0.5 偏移）。
// 返回：C 型的 x 下界（已 floor，保证是**偏保守**的左界）。
//
// 实现：分三种情况，关键是斜边那一条 ——
//   · dx == 0（垂直边）：x 恒定，直接返回 p1().x()。
//   · dy == 0（水平边）：x 在两端之间，取 min。
//   · 否则（斜边）：x 随 y 单调变化，因此最左 x 必然落在 y1 或 y2 **之一**。
//     选哪个由 x 的增减方向决定：x 随 y 递增 ⟺ dx 与 dy **同号**。
//     源码用 `((e.dy () < 0) ^ (e.dx () < 0)) == 0` 表达"同号"（异或为 0 即同号）：
//         同号 → x 递增 → 最小值在区间**下端 y1**
//         异号 → x 递减 → 最小值在区间**上端 y2**
//     最后套 floor() 把 double 结果向下取整为整数坐标 ——
//     向下取整是刻意的：使得到的左界**不会偏右**，宁可宽一点也不能漏掉可能的相交。
// [[ZH-END]]
template <class C>
inline C edge_xmin_at_yinterval_double (const db::edge<C> &e, double y1, double y2) 
{
  // [[ZH]] 垂直边：x 恒定，与 y 无关。
  if (e.dx () == 0) {
    return e.p1 ().x ();
  } else if (e.dy () == 0) {
    // [[ZH]] 水平边：x 在两端点之间，取下界。
    return std::min (e.p1 ().x (), e.p2 ().x ());
  } else {
    // [[ZH]] 斜边：同号（异或==0）→ x 递增 → 取 y1；否则取 y2。再 floor 保守化。
    return C (floor (edge_xaty_double (e, ((e.dy () < 0) ^ (e.dx () < 0)) == 0 ? y1 : y2)));
  }
}

/**
 *  @brief Computes the right bound of the edge geometry for a given band [y1..y2].
 */
// [[ZH-BEGIN]]
// 功能：求该边在 y 区间 [y1, y2] 内的**最右 x**（即 x 的上界），并向上取整。
//
// 参数与语义均与 edge_xmin_at_yinterval_double 对称（同为"偏保守"的界）。
//
// ★ 与 xmin 版的两处差异（容易看漏）：
//   1) 选择区间的条件由 `== 0` 改为 `!= 0` —— 因为求最大值时方向判断正好相反：
//        同号（x 递增）→ 最大值在区间**上端 y2**；异号 → 在**下端 y1**。
//   2) 用 ceil() 而非 floor() —— 上界要向上取整，同样是为了保守
//        （宁可宽一点，不能把可能相交的边排除掉）。
// [[ZH-END]]
template <class C>
inline C edge_xmax_at_yinterval_double (const db::edge<C> &e, double y1, double y2) 
{
  // [[ZH]] 垂直边：x 恒定。
  if (e.dx () == 0) {
    return e.p1 ().x ();
  } else if (e.dy () == 0) {
    // [[ZH]] 水平边：取上界。
    return std::max (e.p1 ().x (), e.p2 ().x ());
  } else {
    // [[ZH]] 斜边：注意条件是 != 0（与 xmin 版相反），并用 ceil 保守化。
    return C (ceil (edge_xaty_double (e, ((e.dy () < 0) ^ (e.dx () < 0)) != 0 ? y1 : y2)));
  }
}

/**
 *  @brief Functor that compares two edges by their left bound for a given interval [y1..y2].
 *
 *  This function is intended for use in scanline scenarios to determine what edges are 
 *  interacting in a certain y interval.
 */
// [[ZH-BEGIN]]
// 功能：_any 路径的排序比较器 —— 按"边在 y 区间 [y1,y2] 内的左界"给边排序。
//
// 参数：构造时传入区间 y1/y2（对应放宽后的 dy = y-0.5 与 dyy = yy+0.5）。
// 返回：a 应排在 b 之前时为 true。
//
// 为什么不能直接用 xmin 排序：_any 的带被放宽了 0.5 DBU，斜边在带内 x 会变化，
//   必须用"带内左界"而非"某点 x"来决定次序，否则相邻带之间边的相对次序会错乱。
//
// 结构：与 EdgeXAtYCompare2 同样的三段式 ——
//   ① 包围盒快速排除（两种相对位置各一分支，免去求界计算）；
//   ② 用带内左界比较；
//   ③ 相等时用 `a < b` 兑底。
//
// ★ 兑底那一步（`return a < b;`）与其它比较器一样是**必需**的：
//   没有它就不构成严格弱序，std::sort 对"相等"元素的相对次序未定义，
//   结果不可复现。
// [[ZH-END]]
template <class C>
struct edge_xmin_at_yinterval_double_compare
{
  // [[ZH]] 参数：y1/y2 限定比较所用的 y 区间（放宽后的带边界）。
  edge_xmin_at_yinterval_double_compare (double y1, double y2)
    : m_y1 (y1), m_y2 (y2)
  {
    // .. nothing yet ..
  }

  // [[ZH]] 功能：按带内左界比较。见上方类注释。
  bool operator() (const db::edge<C> &a, const db::edge<C> &b) const
  {
    if (edge_xmax (a) < edge_xmin (b)) {
      // [[ZH]] ① a 整体在 b 左侧（用普通包围盒即可判定）→ a 在前，免去求界。
      return true;
    } else if (edge_xmin (a) > edge_xmax (b)) {
      // [[ZH]] ① a 整体在 b 右侧 → a 在后。
      return false;
    } else {
      // [[ZH]] ② 区间重叠，必须算带内左界才能定序。
      C xa = edge_xmin_at_yinterval_double (a, m_y1, m_y2);
      C xb = edge_xmin_at_yinterval_double (b, m_y1, m_y2);
      if (xa != xb) {
        return xa < xb;
      } else {
        // [[ZH]] ③ 左界相等：用边自身的全序兑底，保证严格弱序与结果可复现。
        return a < b;
      }
    }
  }

public:
  // [[ZH]] m_y1 / m_y2：比较所用的 y 区间（即放宽后的带边界 dy..dyy）。
  double m_y1, m_y2;
};

// [[ZH-BEGIN]]
// ============================================================================
//  get_intersections_per_band_any —— 阶段 2 的**通用（任意角度）路径**
// ============================================================================
//
// 功能与调用方式和 get_intersections_per_band_90 完全对应
//   （找出本带内边的所有交点、登记为切点），
//   但处理的是**含斜边**的输入，因此无法用包围盒相交那种简单判据。
//
// 参数：含义同 _90 版（cutpoints / current / future / y / yy / with_h）。
//
// ★★ 本函数最关键的设计：把扫描带**故意放宽 ±0.5 DBU**
//       double dy  = y  - 0.5;
//       double dyy = yy + 0.5;
//    为何要这么做（这是整个"模糊"机制的核心）：
//      整数网格上，两条边可能在数学上“差半个单位就相交”。若按精确边界划分扫描带，
//      这种“几乎相交”会被拆到两个不同的带里，于是两边各自都看不到对方，
//      结果留下一个几乎重合却未打通的碎片。
//      把带向外放宽 0.5 DBU，就可以把这种临界情形**拉进同一个带**一并处理；
//      再用 is_point_on_fuzzy（±1 DBU 锥形邻域）做判定，把“几乎共线”当作共线。
//    代价是要用带内左界（edge_xmin_at_yinterval_double）来排序，
//      因为斜边在这个放宽后的带里 x 会变化。
//
// 【与 _90 版的四个具体差异】
//   1) 排序与剪枝用 edge_xmin_at_yinterval_double_compare（带内左界），
//      而非 _90 的 edge_xmin_compare。
//   2) 求交一律走 **safe_intersect_point**，而非裸的 intersect_point。
//      原因：斜边的求交公式**关于实参顺序不对称**，
//      若不固定顺序，同一交点会得到两个不同坐标，破坏扫描线排序的稳定。
//      详见 safe_intersect_point 的说明。（_90 版能省去这层包装，因为
//      两条正交边的求交结果是 max/min 形式、天然对称。）
//   3) 交点不能当场就当作切点，必须先收进 weak_points **暂存**，
//      留到两两配对全部跑完后再统一插入 —— 因为一个交点往往同时落在多条边上，
//      而它写入每条边时的强弱属性要一致。
//   4) 多了 p1_weak（端点级弱交互）的**延迟决策**机制，见下。
//
// 【p1_weak 机制：为何端点也要延迟决策】
//   若 c1 的端点 c1.p1 落在 c2 的模糊邻域内（但不精确在 c2 上），
//   则这个端点可能需要在 c2 上产生一个切点。但是否真的需要，
//   取决于它是否实际影响了**两条以上**的边（源码注释明确说了这一点）。
//   于是先把 (c1, c2) 这种候选对缓存在 p1_weak，等到本格配对结束后：
//     · 若任一目标边已经有强切点 → 把该端点作为**强切点**插入所有目标边；
//     · 否则 → 用 add_attractor 登记为**弱切点**，
//              并用对方 CutPoints 的下标把几条边**串成链**（变量 n 不断前推），
//              这样将来只要其中一条被提升，链上的其它边也会被一并提升。
//   这与 CutPoints 的 attractor 提升机制是同一套思路的两个层次。
//
// 【结构】与 _90 一致：排序 → 扫 x 分格 → 格内两两配对 → 移除失效边（压实）。
//   唯一的额外差异在末尾：压实判据同时看 edge_xmax 与 edge_xmax_at_yinterval_double
//   （因为"已被移出带"也可能由放宽后的右界决定）。
//
// 坑：本函数依赖调用方正确设置 y / yy（已放宽前的原值，函数内部自己算 ±0.5）。
// [[ZH-END]]
static void 
get_intersections_per_band_any (std::vector <CutPoints> &cutpoints, std::vector <WorkEdge>::iterator current, std::vector <WorkEdge>::iterator future, db::Coord y, db::Coord yy, bool with_h)
{
  // [[ZH]] ★ 把带向外放宽 0.5 DBU —— 本函数全部逻辑的基础，见上方说明。
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

// [[ZH-BEGIN]]
// ============================================================================
//  EdgeProcessorState —— 单路 (sink, evaluator) 的扫描线状态机
// ============================================================================
//
// 【角色】阶段 4 的"执行单元"。每有一个 (sink, evaluator) 对，就有一个本对象。
//   它负责把"工作边表"翻译成对评估器的调用，再把评估器的裁决翻译成对 sink 的投递。
//   多路输出时由 EdgeProcessorStates 持有多个本对象。
//
// 【它维护的状态（见文件末尾成员）】
//   mp_es / mp_op : 输出端与评估器（裸指针，生命周期由调用者保证）。
//   m_y           : 当前扫描线的 y。
//   m_x           : 当前顶点的 x（double 舍入后的整数），随顶点推进。
//   m_hx          : ★ 合成水平边的起点 x —— 即"上一个顶点"的 x。
//   m_ho          : ★ compare_ns() 的结果，决定合成出的水平边朝向哪个方向。
//   m_pn / m_ps   : ★ 北侧/南侧的**累加器**，即评估器 edge() 返回值之和。
//                   非零表示"此处有结果边界"，符号决定输出边方向。
//   m_vertex      : 是否正处于一个顶点上（用于判断何时该合成水平边）。
//
// 【为什么需要"合成水平边"】
//   扫描线算法只处理"竖直方向的事件"，因此结果里天然缺少水平方向的边界段。
//   但多边形必须闭合，所以引擎要在两个相邻顶点之间**补出水平边**：
//       从 m_hx 到当前 m_x，在 y = m_y 处。
//   补边时方向由 m_ho（compare_ns 的结果）决定：
//       若 m_ho > 0 则交换端点（见 end_coincident）。
//   这就是 dbEdgeProcessor.h 里 compare_ns() 存在的全部理由。
//
// 【关键方法的调用序列】（阶段 4 每处理一个扫描线上的位置时）
//   begin_scanline(y) → next_vertex(x) → north_edge/south_edge(...)
//   → end_vertex() → [next_coincident() ... end_coincident()] → end_scanline(y)
//
// 提示：本类的 north_edge/south_edge 直接把形参 prefer_touch 传给了评估器的
//       enter 形参 —— 这正是 dbEdgeProcessor.h 文件头提到的那个"易踩的坑"。
// [[ZH-END]]
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
// [[ZH]] skip_info_storage_threshold：SkipInfo 用"内联存储"还是"堆分配"的切换阈值。
// [[ZH]] 设为 0 可强制总是走堆分配 —— 这是留给测试用的开关（注释里写明 "testing"），
// [[ZH]] 用于验证两种存储路径行为一致。生产环境保持 1。
const size_t skip_info_storage_threshold = 1;

/**
 *  @brief Encapsulates the state of the edge processor's generation stage
 *
 *  The generation state may involve multiple generators and output sinks. This
 *  class provides a single interface to handle the case of single and multiple
 *  receivers in a uniform way.
 */
// [[ZH-BEGIN]]
// 功能：阶段 4 的门面（多路复用器）——
//       把"单路"与"多路"两种情形统一成同一个调用界面。
//
// 【为什么需要这层包装】
//   EdgeProcessor 支持一次扫描同时喂多个 (sink, evaluator) 对。
//   若没有本类，阶段 4 的循环里到处都要写"是单路还是多路"的分支判断。
//   本类把这些分支收敛到一处：内部持有**一个或多个** EdgeProcessorState，
//   对外统一提供 start/reset/reserve/begin_scanline/... 等方法，
//   内部再逐个转发（或聚合，如 can_stop 取各路的"或"）。
//
// 【它聚合出的全局查询】
//   can_stop()      : 任意一路 sink 请求停止 → 整体停止。
//   prefer_touch()  : 任意一路评估器要求"相切算内部" → 整体按相切处理。
//                     注意：多路时这是一个"或"，会同时影响所有路。
//   selects_edges() : 任意一路需要逐边通道 → 整体启用该通道。
//
// 【SkipInfo 与 skip 优化】
//   嵌套的 SkipInfo 结构（见下）实现了一个**记忆化**优化：
//   若某一段边的处理没有改变任何一路的内部状态，就把"这一段跳过了多少条边"
//   记下来，下次 redo/process 时直接跳到段尾并调用 sink->skip_n()，
//   从而免去逐条边调用评估器的开销。
//   它用 WorkEdge::data 存放条目下标 —— 这正是 data 字段在阶段 4 的"用途"。
// [[ZH-END]]
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

// [[ZH-BEGIN]]
// ============================================================================
//  redo_or_process —— 算法主驱动板（process/redo 的共同实现）
// ============================================================================
//
// 【入参】
//   gen  : [(sink1, eval1), (sink2, eval2), ...] —— 支持一次扫描多路输出。
//          单路调用（process(es,op)）在更上层被包装成只含一项的 gen。
//   redo : ★ 关键开关，决定是否重跑昂贵的阶段 2、3：
//            false (= process) → 跑完整四阶段
//            true  (= redo)    → **跳过阶段 2、3**，只清空 data 后直接进入阶段 4
//          另外注意 redo 模式下不会重排 mp_work_edges，因为它已被 process 排好序了。
//
// 【四个阶段】
//   step 1  preparation    清空切点表、统计 property 个数、准备进度对象。
//                          redo 模式在此清空各 WorkEdge 的 data 字段
//                          （把阶段 4 遗留的 skip 下标清掉，同时也就丢弃了
//                           阶段 1-3 的 CutPoints 关联 —— 因为已经不需要了）。
//   step 2  find intersections
//                          按 edge_ymin 排序，然后**分带(band)**扫描，
//                          逐带调用 get_intersections_per_band_90 / _any 求交，
//                          把交点登记为 CutPoints。
//                          ★ 最耗时阶段。正交输入走 *_90 快速路径。
//   step 3  create new edges from the ones with cutpoints
//                          对每条有切点的边，沿边方向排序切点，生成若干小段
//                          （原来的 1 条边变成 N 条 WorkEdge），从而保证
//                          "所有边互不相交"这一前提在阶段 4 成立。
//   step 4  compute the result edges
//                          自下而上扫描线：对每条扫描线按 x 排序（用
//                          EdgeXAtYCompare2），把重合边按 property 分组，
//                          调用评估器的 edge()/select_edge()，把结果投递给 sink。
//                          ★ 唯一响应 can_stop() 的阶段。
//
// 【"分带(band)"是什么意思，为什么要它】
//   朴素做法是"每移动一条扫描线就重算一次活跃边集合"，代价高。
//   这里改为把 y 轴切成若干**带**：一次取出足够多的边（目标是把约 50% 的边
//   纳入当前带，经验系数 fill_factor），在这一带内统一处理。
//   带内只需处理"本带涉及的边"，减少了反复筛选的开销。
//   具体做法见下面 step 2 中那个 do/while：不断把 ymin ≤ yy 的边纳入，
//   直到纳入的新边数量达到 n * fill_factor 的规模。
//
// 【进度报告】用 tl::AbsoluteProgress，把估算出的工作量(todo_max)按阶段切分：
//   阶段 2 占约 1/5，阶段 3 占约 1/5，其余留给阶段 4。
//   只有 m_report_progress 为真时才创建进度对象 ——
//   因为进度更新本身有成本，批量运算时应关闭（见构造函数参数 report_progress）。
//
// 【空输入】mp_work_edges 为空时直接 start()+flush() 后返回
//   （仍要调用，让 sink 有机会初始化/收尾）。
// [[ZH-END]]
void
EdgeProcessor::redo_or_process (const std::vector<std::pair<db::EdgeSink *, db::EdgeEvaluatorBase *> > &gen, bool redo)
{
  // [[ZH]] SelfTimer：当 tl::verbosity() 达到 m_base_verbosity 时，
  // [[ZH]] 在作用域结束自动打印本次处理耗时。这是"性能观测"的轻量手段，
  // [[ZH]] 不改变任何计算结果。阈值可用 set_base_verbosity() 调整（默认 30）。
  tl::SelfTimer timer (tl::verbosity () >= m_base_verbosity, "EdgeProcessor: process");

  // [[ZH]] gs 把"单路/多路 sink+evaluator"统一成一个入口，
  // [[ZH]] 并聚合各路的 can_stop、prefer_touch、selects_edges 等查询。见 EdgeProcessorStates。
  EdgeProcessorStates gs (gen);

  // [[ZH]] 这两个标志由**所有**评估器统一决定（多路时取"或"）：
  // [[ZH]]   prefer_touch  → 会被当作 edge() 的 enter 实参传下去（见 EdgeProcessorState）
  // [[ZH]]   selects_edges → 是否启用逐边 select_edge 通道
  bool prefer_touch = gs.prefer_touch ();
  bool selects_edges = gs.selects_edges ();
  
  // [[ZH]] y     ：当前扫描线的 y 坐标。
  // [[ZH]] future：指向"尚未纳入当前带的下一条边"（边已按 ymin 升序排好）。
  db::Coord y;
  std::vector <WorkEdge>::iterator future;

  //  step 1: preparation
  // [[ZH]] ===== 阶段 1：准备 =====

  if (mp_work_edges->empty ()) {
    // [[ZH]] 空输入：仍要通知 sink 开始与结束（否则某些 sink 的内部状态无法收尾）。
    gs.start ();
    gs.flush ();
    return;
  }

  // [[ZH]] 清空切点表。注意 mp_cpvector 是复用的成员，上次处理的切点必须丢弃。
  mp_cpvector->clear ();

  //  count the properties

  // [[ZH]] 统计 property 的最大值，从而得知总共有多少种来源标签。
  // [[ZH]] 评估器的内部数组（如 BooleanOp 的 m_wcv_n/m_wcv_s）按 property 索引，
  // [[ZH]] 因此需要这个上界来 resize。注意这里取的是 max+1（下标从 0 开始）。
  property_type n_props = 0;
  for (std::vector <WorkEdge>::iterator e = mp_work_edges->begin (); e != mp_work_edges->end (); ++e) {
    if (e->prop > n_props) {
      n_props = e->prop;
    }
  }
  ++n_props;

  //  prepare progress

  // [[ZH]] 进度报告的总量刻度。下面把 1000000 按阶段切分作为各阶段的配额。
  size_t todo_max = 1000000;

  std::unique_ptr<tl::AbsoluteProgress> progress;
  if (m_report_progress) {
    // [[ZH]] 仅在启用时才创建进度对象（进度更新有性能开销）。
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
  // [[ZH]] 阶段 1（准备）占总量约 1/5。
  todo_next += (todo_max - todo) / 5;


  if (redo) {

    //  redo mode: skip the intersection detection step and clear the data

    // [[ZH]] ===== redo 捷径：跳过阶段 2、3 =====
    // [[ZH]] 把每条工作边的 data 清零。这一步同时完成了两件事：
    // [[ZH]]   ① 丢弃阶段 1-3 遗留的 CutPoints 下标（阶段 4 不再需要它们）；
    // [[ZH]]   ② 把阶段 4 上次写入的 skip 下标清掉，使本次从头计算 skip 信息。
    // [[ZH]] 正因为跳过了阶段 2、3，redo 远快于 process，但其正确性
    // [[ZH]] **完全依赖**此前已经跑过一次 process（边已被打断、已按 ymin 排序）。
    for (std::vector <WorkEdge>::iterator c = mp_work_edges->begin (); c != mp_work_edges->end (); ++c) {
      c->data = 0;
    }

    // [[ZH]] redo 模式下阶段 2、3 不执行，但进度配额照常推进，保持百分比一致。
    todo = todo_next;
    todo_next += (todo_max - todo) / 5;

  } else {

    //  step 2: find intersections
    // [[ZH]] ===== 阶段 2：求交点（最耗时的阶段）=====
    // [[ZH]] 先按 ymin 升序排序，使扫描可以自下而上推进。
    // [[ZH]] ★ 这里用的是 edge_ymin_compare（整数包围盒比较），
    // [[ZH]]   内部以 operator< 兜底保证全序 —— 排序必须是确定的，否则结果不可复现。
    std::sort (mp_work_edges->begin (), mp_work_edges->end (), edge_ymin_compare<db::Coord> ());

    // [[ZH]] y 从最小 ymin 开始；future 指向尚未纳入当前带的边。
    y = edge_ymin ((*mp_work_edges) [0]);
    future = mp_work_edges->begin ();

    // [[ZH]] 外层循环：每次处理一个"带(band)"。
    // [[ZH]] current 是当前带处理的起点，循环体结束后会推进到带尾。
    for (std::vector <WorkEdge>::iterator current = mp_work_edges->begin (); current != mp_work_edges->end (); ) {

      if (m_report_progress) {
        // [[ZH]] 按"已处理的边数占比"更新阶段 2 的进度。
        double p = double (std::distance (mp_work_edges->begin (), current)) / double (mp_work_edges->size ());
        progress->set (size_t (double (todo_next - todo) * p) + todo);
      }

      size_t n = 0;
      db::Coord yy = y;

      //  Use as many scanlines as to fetch approx. 50% new edges into the scanline (this
      //  is an empirically determined factor)
      // [[ZH]] ★ 决定本带的 y 范围：不断把 ymin ≤ yy 的边纳入，
      // [[ZH]]   直到新纳入的边数达到"本带起点处边数 n 的 fill_factor 倍"（约 1.5x）。
      // [[ZH]]   直觉：一次多纳入一些边，摊薄"每移动一条扫描线就重建活跃集"的开销。
      // [[ZH]]   n 只在第一轮确定（= 初始带内边数），后续轮以它为基准。
      do {

        // [[ZH]] 把 ymin ≤ yy 的边全部纳入本带。
        while (future != mp_work_edges->end () && edge_ymin (*future) <= yy) {
          ++future;
        }

        // [[ZH]] 取下一带的边界：下一条边的 ymin；若没有了则用 Coord 的最大值
        // [[ZH]] （相当于"一直延伸到无穷远"，把剩余全部纳入）。
        if (future != mp_work_edges->end ()) {
          yy = edge_ymin (*future);
        } else {
          yy = std::numeric_limits <db::Coord>::max ();
        }

        if (n == 0) {
          // [[ZH]] 第一轮：记下本带的初始边数，作为后续扩张的基准。
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
    // [[ZH-BEGIN]]
    // ===== 阶段 3：按切点把边打断 =====
    // 目的：把"有切点的边"拆成若干小段，使阶段 4 面对的所有边**互不相交**。
    //       这是扫描线算法的前提条件（见 dbEdgeProcessor.h 文件头）。
    //
    // 上面英文 Hint 说的是本阶段最微妙的一处：
    //   切点沿边的先后顺序由 ProjectionCompare 排序决定（唯一）；
    //   但**相邻切点之间的连线方式存在自由度** —— 当切点落在同一格的角上时，
    //   可以选择不同的拐法。这里刻意选择能避免"Z 字形"的连法，
    //   因为 Z 字形会在 1×1 DBU 的方格内制造新的自相交，
    //   而阶段 4 假定输入无自相交。下面那些
    //       `ne = db::Edge (pll, ne.p2 ());` / `ne = db::Edge (ne.p1 (), pll);`
    //   就是在调整拐法，消除 Z 形。（变量 pll = "previous of previous"）
    //
    // 实现要点：
    //   · 原地生成：用写入下标 nw 在同一个 vector 内"就地覆盖"已处理过的位置，
    //     只有确实变多时才 push_back。跑完后把尾部多余元素 erase 掉。
    //     这样避免了额外容器，是性能敏感的写法（读起来要小心 nw 与 n 的关系）。
    //   · ew.data 用完即清零（`ew.data = 0;`）—— 因为 data 要留给阶段 4 存 skip 下标。
    //   · **水平边被直接丢弃**（`ew.dy () == 0 && ! selects_edges`）：
    //     扫描线算法不需要水平边参与（它们不改变环绕数），除非评估器显式要求逐边投递。
    //     这解释了为什么"结果里见不到水平边"是正常的。
    // [[ZH-END]]

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
  
  // [[ZH-BEGIN]]
  // ===== 阶段 4：扫描线求值并输出 =====
  // 这是唯一真正"产出结果"的阶段，也是唯一响应停止请求的阶段。
  //
  // 【★ 为什么 gs.start() 放在这么靠后的位置】
  //   上面英文注释已经点明：start() 延迟到**读完输入之后**才调用。
  //   原因是 EdgeContainer 之类的 sink 在 start() 里会（按需）清空输出容器。
  //   如果输入容器和输出容器是**同一个**（就地运算，很常见的用法），
  //   过早清空就会把还没读完的输入一起清掉。
  //   因此约定：start() 必须晚于"输入已被完全读取"。
  //
  // 【本阶段的循环结构】
  //   按 y 自下而上推进，每一步处理"当前 y 处的所有边事件"：
  //     ① 把 ymin <= y 的边纳入本次处理的区间 [f0, future)
  //     ② 用 EdgeXAtYCompare2 在这个区间内**按 x 排序**
  //        （这是扫描线的核心：同一条扫描线上必须按 x 从左到右处理）
  //     ③ 计算下一个 y（yy）：取"活跃边的 ymax"与"下一条边的 ymin"中的较小者
  //     ④ 处理本 y 处的边事件，调用评估器，投递结果
  //     ⑤ y = yy，继续
  //   循环条件里的 `! gs.can_stop ()` 就是停止请求的唯一检查点 ——
  //   也就是说停止只在**扫描线边界**生效，当前扫描线内的边仍会被处理完。
  //
  // 【关于 tl_assert (future->data == 0)】
  //   这不是业务逻辑，而是**开发期自检**：它验证"进入阶段 4 时所有边的 data 都已清零"。
  //   因为 data 在阶段 1-3 被用作 CutPoints 下标，在阶段 4 要改用作 skip 下标，
  //   若忘记清零就会出现"把旧的切点下标误当 skip 下标用"的隐蔽错误。
  //   见 WorkEdge 对 data 字段双重复用的说明。
  // [[ZH-END]]
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

