
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



#ifndef HDR_dbEdgeProcessor
#define HDR_dbEdgeProcessor

#include "dbCommon.h"

#include "dbTypes.h"
#include "dbEdge.h"
#include "dbPolygon.h"

#include <vector>
#include <set>

namespace db
{

// [[ZH-BEGIN]]
// ============================================================================
//  dbEdgeProcessor.h —— 边的布尔运算引擎（scanline / 扫描线算法）
// ============================================================================
//
// 【这个文件解决什么问题】
//   给定一堆多边形（或一堆边），求它们的 并 / 交 / 差 / 异或，或做尺寸放大缩小
//   (sizing)，或判断哪些边落在哪些多边形内部……这些问题都可归约为同一个核心操作：
//   **求一组边的布尔组合**。EdgeProcessor 就是这个核心操作的实现。
//
// 【它在整个库中的位置】
//       db::Region / db::Edges          ← 高层抽象（图层、集合运算）
//              ↓ 委托
//       db::ShapeProcessor              ← 把 Shape/Polygon/Path 拆成边（前端包装）
//              ↓ 委托
//       db::EdgeProcessor   ★ 你在这里 ★ ← 只认边，做真正的布尔运算
//              ↓ 输出
//       db::EdgeSink 的实现类            ← EdgeContainer / PolygonGenerator / TrapezoidGenerator
//
//   ★ 最重要的一点认知：EdgeProcessor **不认识多边形**，它只认识边。
//     多边形在这里被"打散成一圈边"，算完后再由 sink 重新缝合成多边形。
//     这解释了两件事：(a) 为什么本文件通篇只有 Edge 没有 Polygon；
//     (b) 为什么输出端要做"缝合(stitching)"这种看起来额外的工作。
//
// 【算法总览 —— 四个阶段】
//   全部工作由 EdgeProcessor::redo_or_process() 驱动，分四个阶段：
//     阶段 1  prep            把插入的边整理成 WorkEdge 数组，统计 property 个数
//     阶段 2  intersections   求所有边的交点（关键阶段，最耗时）
//     阶段 3  split           按交点把边"打断"，使各段互不相交
//     阶段 4  production      用扫描线自下而上扫过，对每段边询问评估器，决定去留并投递
//
//   ★ 只有阶段 4 会响应"停止请求(can_stop)"；阶段 2、3 是不可中断的。
//
// 【核心概念一：wrap count（wc，环绕数 / 圈数）】
//   ⚠ 本文件最容易误解的地方：名字里的 "wc" 在不同类中含义**不同**！
//     · GenericMerge / SimpleMerge 中：wc = **有向环绕数 (winding number)**，
//         即"从该点向上引一条射线，穿过多边形边界的有向次数"。
//         正负号取决于边的方向 —— 这正是"边的右侧 = 内部"约定的用武之地。
//     · MergeOp 中：wc = **当前张开的（即重叠的）多边形个数**，不是环绕数。
//     · BooleanOp 中：同时维护"每个 property 的环绕数"以及"A 的张开的个数/B 的张开的个数"。
//   看到 wc 请先确认属于哪一类，否则模式参数的语义一定会理解错。
//
// 【核心概念二：property（属性 / 来源标签）】
//   每条边带一个整数 property（类型 size_t），用来**分辨这条边来自哪个输入**：
//     · BooleanOp：约定 bit0 表示操作数（偶数 → A，奇数 → B）
//     · merge()：  property = 输入多边形的下标
//     · size()：   property = 输入下标 * 2
//     · EdgePolygonOp：0 = 多边形自身，1 及以上 = 待过滤的边
//   换言之，property 是"边的身份证"，评估器靠它区分不同输入。
//
// 【如何上手阅读本文件】
//   建议顺序：
//     ① EdgeSink            —— 输出端接口，最简单，先搞清"结果怎么出去"
//     ② EdgeEvaluatorBase 及其子类 —— "每段边如何决定去留"
//     ③ EdgeProcessor 公开 API     —— insert / process / simple_merge / boolean / size
//   具体实现在 dbEdgeProcessor.cc，那里有按阶段划分的更细注释。
//
// 【命名撞车警告 —— 极易踩坑】
//   db::EdgeProcessor         = 本文件的 scanline 布尔运算引擎
//   db::EdgeProcessorBase     = shape_collection_processor<Edge,Edge> 的别名
//                               （定义在 dbEdgesDelegate.h）
//                               ★ 名字里多了 "Base"，但它**不是本类的基类**，
//                                 而是 Edges 集合的处理框架，与 scanline 算法无关。
//   db::ExtendedEdgeProcessor (dbEdgesUtils.h)、db::PolygonToEdgeProcessor
//                               (dbRegionProcessors.h) 同样与本类无关。
// [[ZH-END]]
struct WorkEdge;
struct CutPoints;
class EdgeSink;

/**
 *  @brief A destination for a (sorted) set of edges
 *
 *  This receiver can be used as destination for the edge processor.
 *  It will receive edge events in the scanline order, this is bottom to 
 *  top and left to right. Edges will be non-intersecting.
 *
 *  This is the base class for such edge receivers.
 */
// [[ZH-BEGIN]]
// 功能：EdgeProcessor 的**输出接收端**接口 —— 处理结果通过它投递出去。
//
//       把它理解为"结果的出口"：想知道布尔运算的结果，就实现一个 EdgeSink
//       （或直接用现成的），传给 process()。引擎本身不知道"结果该变成什么形状"，
//       一切由你选的 sink 决定：
//         · EdgeContainer       → 收集成 std::vector<db::Edge>
//         · PolygonGenerator    → 缝合成 std::vector<db::Polygon>
//         · TrapezoidGenerator  → 分解成梯形
//
// ★ 投递协议（调用顺序契约，写 sink 前必须了解）：
//       start()                 所有输入边已缓存完毕，即将开始投递
//       └ 对每一条扫描线 y：
//           begin_scanline(y)
//           ├ put(edge)           该边在此 y 处开始或结束
//           ├ put(edge, tag)      带标签版本（仅当评估器 selects_edges() 为真）
//           ├ crossing_edge(edge) 该边"穿过"本扫描线（不在此开始/结束）
//           ├ skip_n(n)           可整体跳过 n 条边（保证形成闭合序列）
//           └ end_scanline(y)
//       flush()                 全部投递完毕
//
// ★ 投递内容的保证（sink 实现者可以依赖这些）：
//       1) 顺序为 scanline 序：整体自下而上（y 递增），同一扫描线内自左而右（x 递增）。
//       2) 投递出的边**互不相交**（相交的边已在阶段 2/3 于交点处被打断）。
//       3) start() 被调用时所有边都已缓存完成 —— 因此可在 start() 里安全地
//          丢弃原始输入、释放临时内存（这正是 start() 存在的意义）。
//
// 坑：1) 所有 virtual 方法的**默认实现都是空操作**，不强制重写。
//        因此"只关心 put()"的 sink 只需重写 put() 即可。
//     2) skip_n(n) 是性能优化：表示"这一批边的处理产生了相同结果，可整段跳过"。
//        默认实现就是"忽略"。若你的 sink 必须看到**每一条**边，需要留意它的存在；
//        典型用法见 dbEdgeProcessor.cc 中的 SkipInfo 与 EdgeProcessorStates。
//     3) 引擎可能对同一个 sink 多次调用 start()/flush()（例如后面紧跟 redo()），
//        所以 sink 不应假设 start() 只被调用一次。
// [[ZH-END]]
class DB_PUBLIC EdgeSink 
{
public:
  /** 
   *  @brief Constructor
   */
  // [[ZH]] 功能：构造。唯一成员 m_can_stop 初始化为 false（未请求停止）。
  EdgeSink () : m_can_stop (false) { }

  /** 
   *  @brief Destructor
   */
  virtual ~EdgeSink () { }

  /**
   *  @brief Start event
   *
   *  This method is called shortly before the first edge is delivered.
   *  Specifically, all edges are already cached before this method is called.
   *  Thus, inside an implementation of this method, the original source can be
   *  discarded.
   */
  // [[ZH]] 功能：投递开始前的回调。★ 此刻所有输入边都已缓存完毕 —— 这正是它的价值：
  // [[ZH]]       你可以放心地在这里丢弃原始输入或释放临时内存（引擎不会再读它们）。
  // [[ZH]] 调用时机：在第一条边投递之前，且整个投递过程**可能被调用多次**。
  virtual void start () { }

  /**
   *  @brief End event
   *
   *  This method is called after the last edge has been delivered.
   */
  // [[ZH]] 功能：投递结束后的回调，用于收尾（例如开始缝合多边形、做统计）。
  // [[ZH]] 坑：与 start() 一样可能被多次调用（引擎重跑时），不要假设只来一次。
  virtual void flush () { }

  /**
   *  @brief Deliver an edge
   *
   *  This method delivers an edge that ends or starts at the current scanline.
   */
  // [[ZH]] 功能：投递一条**在当前扫描线处开始或结束**的边（即"顶点事件"）。
  // [[ZH]] 这是最常用的回调 —— 只想要结果边的 sink 覆盖它即可。
  // [[ZH]] 注意：穿过扫描线的边不走这里，而走 crossing_edge()。
  virtual void put (const db::Edge &) { }

  /**
   *  @brief Deliver a tagged edge
   *
   *  This method delivers an edge that ends or starts at the current scanline.
   *  This version includes a tag which is generated when using "select_edge".
   *  A tag is a value > 0 returned by "select_edge".
   */
  // [[ZH]] 功能：put 的带标签版本。tag 是评估器的 select_edge() 返回的正整数（>0），
  // [[ZH]]       用来给同一次投递的边分类（例如 EdgePolygonOp 用 tag 区分"内部/外部"）。
  // [[ZH]] 何时触发：仅当评估器 selects_edges() 返回 true 时才可能走这个重载。
  virtual void put (const db::Edge &, int /*tag*/) { }

  /**
   *  @brief Deliver an edge that crosses the scanline
   *
   *  This method is called to deliver an edge that is not starting or ending at the
   *  current scanline.
   *  Another delivery of a set of crossing edges may happen through the skip_n 
   *  method which delivers a set of (unspecified) edges which are guaranteed to form
   *  a closed sequence, that is one which is starting and ending at a wrap count of 0.
   */
  // [[ZH-BEGIN]]
  // 功能：投递一条**穿过**当前扫描线的边（该边的两个端点都不在本扫描线上）。
  //
  // ★ 与 put() 的区别：put() 投递"在此处开始/结束"的边（顶点事件），
  //   crossing_edge() 投递"横跨此处"的边（跨越事件）。
  //   对于"收集所有结果边"的简单 sink（如 EdgeContainer），两者往往做同样的收集动作。
  //
  // 注意：与 skip_n() 的关系 —— 另有一批"穿过的边"可能**不会**逐条投递，
  //       而是通过 skip_n(n) 一次性告知"这里跳过了 n 条边"。
  //       这批被跳过的边保证构成**闭合序列**（即环绕数从 0 出发回到 0），
  //       因此对"结果只取决于环绕数是否变化"的评估器来说，它们不影响输出，可以整段跳过。
  // [[ZH-END]]
  virtual void crossing_edge (const db::Edge &) { }

  /**
   *  @brief Deliver an edge set forming a closed sequence
   *
   *  See description of "crossing_edge" for details.
   */
  // [[ZH-BEGIN]]
  // 功能：告知"sink 可整体跳过 n 条边"—— 纯性能优化通道。
  //
  // 参数：n 被跳过的边数（这些边不逐条投递）。
  // 前提：这 n 条边保证构成**闭合序列**（wrap count 从 0 开始、回到 0），
  //       因此它们不会改变任何基于环绕数的评估结果，可以被安全忽略。
  //
  // 为什么需要它：在 redo()/重复处理时，大量边段的结果完全相同，
  //   逐条投递会产生可观的调用开销；用 skip_n 直接跳到下一处"状态有变化"的位置。
  //   实现见 dbEdgeProcessor.cc 中的 EdgeProcessorStates::SkipInfo。
  //
  // 坑：默认实现是**空操作**，即默认忽略这条优化。
  //     若你的 sink 需要看到每一条结果边（而不只是"结果集合"），
  //     收到 skip_n 就意味着你**拿不到**那些边的具体几何 —— 需要重新设计思路
  //     （例如改用不触发 skip 的处理路径，或让评估器 selects_edges() 为真）。
  // [[ZH-END]]
  virtual void skip_n (size_t /*n*/) { }

  /**
   *  @brief Signal the start of a scanline at the given y coordinate
   */
  // [[ZH]] 功能：即将开始处理 y 这条扫描线。参数 y 是当前扫描线的 y 坐标。
  // [[ZH]] 用途：需要按扫描线分批组织输出的 sink（如多边形缝合器）在此重置行状态。
  virtual void begin_scanline (db::Coord /*y*/) { }

  /**
   *  @brief Signal the end of a scanline at the given y coordinate
   */
  // [[ZH]] 功能：y 这条扫描线处理完毕。与 begin_scanline 配对。
  virtual void end_scanline (db::Coord /*y*/) { }

  /**
   *  @brief Gets a value indicating that the generator wants to stop
   */
  // [[ZH]] 功能：查询"是否已请求停止"。引擎在每个扫描线边界检查此标志（见 .cc 的阶段 4 循环）。
  // [[ZH]] 用途：算法上只需部分结果的场景（例如"找到第一个满足条件的目标就够"）可提前收工。
  bool can_stop () const
  {
    return m_can_stop;
  }

  /**
   *  @brief Resets the stop request
   */
  // [[ZH]] 功能：清除停止请求，让引擎可以继续。
  // [[ZH]] 谁会调用：引擎在开始新一轮处理前会调用它（避免上一次的请求残留影响下一次）。
  void reset_stop ()
  {
    m_can_stop = false;
  }

protected:
  /**
   *  @brief Sets the stop request
   *
   *  The scanner can choose to stop once the request is set.
   *  This is useful for implementing receivers that can stop once a
   *  specific condition is found.
   */
  // [[ZH-BEGIN]]
  // 功能：由 sink 主动发起"请停止处理"的请求（protected，只能从子类内部调用）。
  //
  // ★ 停止协议的重要限制（新手最容易误判的地方，务必读完）：
  //   1) 引擎**只在每个扫描线的边界**检查该标志（dbEdgeProcessor.cc 的阶段 4 循环条件）。
  //      因此调用 request_stop() 后，**当前扫描线内剩余的边仍会继续投递**，
  //      直到本行结束才真正停下。
  //   2) 阶段 2（求交点）与阶段 3（打断边）**完全不响应**停止请求 ——
  //      它们必须跑完。若你的输入很大，这两阶段的耗时无法通过停止请求规避。
  //   3) 引擎在新一轮处理前会调用 reset_stop()，所以请求不是粘性的。
  //
  // 全代码库中唯一实际调用本函数的地方是 dbEdgesUtils.cc 里的 DetectTagEdgeSink
  // （用于"找到带标签的边就停"）。
  //
  // 典型用法：
  //     void my_sink::put (const db::Edge &e) override {
  //       if (找到符合条件的结果) {
  //         request_stop ();   // 请求停止；当前扫描线跑完后引擎会退出
  //       }
  //     }
  // [[ZH-END]]
  void request_stop ()
  {
    m_can_stop = true;
  }

private:
  // [[ZH]] m_can_stop：是否已请求停止。唯一的状态成员。生命周期：reset_stop() 清零，
  // [[ZH]] request_stop() 置位，引擎在扫描线边界读取。见上面停止协议的说明。
  bool m_can_stop;
};

/**
 *  @brief A edge container that can be used as a receiver for edges
 *
 *  This class reimplements the EdgeSink interface.
 *  This receiver simply collects the edges in a container (a vector of edges)
 *  which is either kept internally or supplied from the outside.
 */
// [[ZH-BEGIN]]
// 功能：最常用的 EdgeSink 实现 —— 把收到的边简单地收集进一个 std::vector<db::Edge>。
//
//       用途：当你只想要"结果边的列表"，不需要缝合成多边形时，直接用它。
//       典型场景：布尔运算结果还需进一步后处理，或只关心边本身（如 DRC 的边检查）。
//
// 【两种存放方式（由构造函数选择）】
//   · 外部容器：EdgeContainer(vec)      —— 结果直接写进你提供的 vec
//   · 内部容器：EdgeContainer()         —— 结果存进对象内的 m_edges，用 edges() 取出
//   外部方式的好处是你能预先 reserve、复用缓冲区。
//
// 【两个可选行为参数】
//   · tag      —— 只收集带指定标签的边（0 表示"标签不限，全收"）
//   · chained  —— 链式转发：收到的边同时也投递给另一个 EdgeContainer
//                 （用于"既要汇总、又要分流"的场景）
//
// 坑：1) clear 参数只在**构造函数**里生效，而且实现上是在第一次 start() 时清空一次
//        然后立刻把标志清零（见 start() 里的注释）——这是为了兼容"多次 start/flush"的
//        分帧投递（例如尺寸过滤器），不会把后续帧的数据误清。
//     2) 本类**不做去重、不排序、不缝合**：它按引擎投递的原样顺序追加，
//        因此可能包含未闭合的边序列。想得到闭合多边形请用 PolygonGenerator。
// [[ZH-END]]
class DB_PUBLIC EdgeContainer 
  : public EdgeSink
{
public:
  /**
   *  @brief Constructor connecting this receiver to an external edge vector
   */
  // [[ZH]] 功能：把本容器绑定到**外部**的边数组上，结果直接追加进 `edges`。
  // [[ZH]] 参数：edges 外部容器（引用，须比本对象活得久）；
  // [[ZH]]       clear 首次 start() 时是否先清空 edges；
  // [[ZH]]       tag   只收集该标签的边，0 = 不限；
  // [[ZH]]       chained 同时把边转发给这个链式容器（可为空）。
  EdgeContainer (std::vector<db::Edge> &edges, bool clear = false, int tag = 0, EdgeContainer *chained = 0)
    : EdgeSink (), mp_edges (&edges), m_clear (clear), m_tag (tag), mp_chained (chained)
  { }

  /**
   *  @brief Constructor using an internal edge vector
   */
  // [[ZH]] 功能：不提供外部容器，结果存进对象自带的内部数组，用 edges() 读取。
  EdgeContainer (int tag = 0, EdgeContainer *chained = 0)
    : EdgeSink (), mp_edges (&m_edges), m_clear (false), m_tag (tag), mp_chained (chained)
  { }

  /**
   *  @brief Get the edges collected so far (const version)
   */
  const std::vector<db::Edge> &edges () const 
  {
    return *mp_edges;
  }

  /**
   *  @brief Get the edges collected so far (non-const version)
   */
  std::vector<db::Edge> &edges () 
  {
    return *mp_edges;
  }

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  // [[ZH]] 功能：投递开始 —— 在首次 start() 时按需清空输出容器，并转发给链式容器。
  // [[ZH]] ★ 「单次生效(single-shot)」机制：清空后立刻把 m_clear 置为 false，
  // [[ZH]]   因此**只清一次**。原因见行内英文注释：尺寸过滤器之类的处理会分多帧
  // [[ZH]]   投递（多次 start/flush），若每次 start 都清空就会把前几帧的结果抹掉。
  // [[ZH]] 修改：mp_edges 指向的内容（可能被清空）、m_clear 置 false、
  // [[ZH]]       并递归调用 mp_chained->start()。
  virtual void start () 
  {
    if (m_clear) {
      // [[ZH]] 只在第一次清空，随后立刻关闭该标志 —— 这就是 single-shot 的含义。
      mp_edges->clear ();
      //  The single-shot scheme is a easy way to overcome problems with multiple start/flush brackets (i.e. on size filter)
      m_clear = false;
    }
    if (mp_chained) {
      // [[ZH]] 链式转发：先让下游容器也进入 start 状态。
      mp_chained->start ();
    }
  }

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  // [[ZH]] 功能：收取一条边 —— 无条件追加，并转发给链式容器。
  // [[ZH]] 修改：向 mp_edges 追加元素。
  // [[ZH]] 注意：本重载**不检查** m_tag（因为该版本没有标签信息可比较）。
  virtual void put (const db::Edge &e) 
  {
    mp_edges->push_back (e);
    if (mp_chained) {
      mp_chained->put (e);
    }
  }

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  // [[ZH]] 功能：收取一条**带标签**的边，按 m_tag 过滤后才追加，并转发给链式容器。
  // [[ZH]] 修改：仅当 (m_tag == 0 表示不限) 或 (tag == m_tag) 时向 mp_edges 追加。
  // [[ZH]] 坑：★ 链式转发**不受过滤影响** —— 无论是否通过过滤，都会调
  // [[ZH]]     mp_chained->put(e, tag)。即下游容器会看到全部边，可能自行过滤。
  virtual void put (const db::Edge &e, int tag)
  {
    if (m_tag == 0 || tag == m_tag) {
      // [[ZH]] 通过标签过滤（m_tag == 0 表示"标签不限"）。
      mp_edges->push_back (e);
    }
    if (mp_chained) {
      // [[ZH]] 注意：这一句在 if 之外 —— 过滤只影响本容器，不影响下游。
      mp_chained->put (e, tag);
    }
  }

private:
  // [[ZH]] m_edges  ：**内部**存储容器，仅在用第二个构造函数时被使用。
  // [[ZH]] mp_edges ：实际写入目标 —— 指向外部容器，或指向 m_edges（外部为空的兜底）。
  // [[ZH]]           用指针间接一层，使两种构造方式共用同一套写入代码。
  // [[ZH]] m_clear  ：是否需要在首次 start() 时清空（single-shot 标志，用完即关）。
  // [[ZH]] m_tag    ：只收集该标签的边；0 表示不限。
  // [[ZH]] mp_chained：可选的链式下游容器，收到的边会同步转发给它；0 表示无。
  std::vector<db::Edge> m_edges;
  std::vector<db::Edge> *mp_edges;
  bool m_clear;
  int m_tag;
  EdgeContainer *mp_chained;
};

/**
 *  @brief The edge set operator base class
 *
 *  This class is an internal class that specifies how the output is formed from 
 *  a set of intersecting-free input edges. 
 *  Basically, this object receives events for the edges along the scan line.
 *  At the beginning of the scan line, the "reset" method is called to bring the
 *  evaluator into a defined state. Each edge has an integer property that can be
 *  used to distinguish edges from different polygons or layers.
 */
// [[ZH-BEGIN]]
// ============================================================================
//  EdgeEvaluatorBase —— 评估器基类（"每段边如何决定去留"的协议定义）
// ============================================================================
//
// 【角色】
//   EdgeProcessor 在扫描过程中，每遇到一段边就问评估器一个问题：
//       "这条边要不要算作结果的一部分？"
//   评估器的回答决定了该段边被投递给 sink 还是被丢弃。
//   不同子类给出不同回答，从而实现 merge / boolean / size / interaction 等各种操作：
//       SimpleMerge / GenericMerge / MergeOp  → 按环绕数决定
//       BooleanOp / BooleanOp2                → 按 A、B 两操作数的环绕数组合决定
//       EdgePolygonOp                         → 按边与多边形的关系决定
//       InteractionDetector                   → 不输出边，只记录多边形间的相互作用
//
// 【★ 最重要的机制：edge() 的返回值是"累加"的，不是布尔值 ★】
//   引擎并不会把 edge() 的返回值当成 true/false，而是把它**累加**到
//   每个扫描线每一侧（north/south）的计数器上。精确地说（见 dbEdgeProcessor.cc）：
//         void north_edge (bool prefer_touch, property_type prop)
//         { m_pn += mp_op->edge (true, prefer_touch, prop); }
//   然后：
//         · 若累加器 m_pn == 0 → 该位置没有边界 → 不输出边
//         · 若 m_pn != 0      → 输出一条边，且**其符号决定输出边的方向**
//           （源码中：m_pn > 0 且 dy < 0，或 m_pn < 0 且 dy > 0，则交换端点）
//   所以返回值遵循这样的约定：
//          +1  这一段是"进入结果内部"的边界（逆着某方向）
//          -1  这一段是"离开结果内部"的边界（顺着反方向）
//           0  与结果无关，丢弃
//   这就是为什么各子类（如 GenericMerge）返回的是 1 / -1 / 0 而非 bool。
//   理解这一点，才能看懂各评估器的实现为何都在算"状态迁移"。
//
// 【★ 一个极易踩的坑：enter 参数与 prefer_touch 是同一个值 ★】
//   看上面 north_edge 的签名：第二个形参名叫 prefer_touch，却被直接传给了
//   EdgeEvaluatorBase::edge() 的 **enter** 形参。两个名字不同，但**运行时是同一个数**：
//         mp_op->edge (true, prefer_touch, prop)
//                              ^^^^^^^^^^^^ 就是 enter
//   之所以能成立，是因为引擎在投递前已按 prefer_touch() 对同位置的重合边分组，
//   保证"enter == prefer_touch"这一关系成立（见 .cc 中阶段 4 的重合边循环）。
//   读代码时若把 prefer_touch 和 enter 当成两件事，就会看不懂。
//
// 【与本类其余方法的关系】
//     reset()          每条扫描线开始时调用，把评估器恢复到确定状态
//     reserve(n)       预分配，避免处理中反复分配内存
//     edge(...)        ★ 核心，见上
//     select_edge(...) 可选通道：按"标签"直接挑选边投递（见下）
//     compare_ns()     比较"扫描线上方 vs 下方"的内外状态，用于给引擎合成的
//                      水平边定方向（见 dbEdgeProcessor.cc 的 end_vertex）
//     is_reset()       状态是否已回到初始 —— 引擎据此识别"整段无变化"区间，
//                      配合 skip_n() 做优化
//     prefer_touch()   是否把"相切/重合"视为内部（会影响 enter 的取值）
//     selects_edges()  是否启用 select_edge 通道
//
// 【默认实现全部是"空操作 / 返回 0/false"】
//   因此子类只需重写自己关心的少数几个方法。
// [[ZH-END]]
class DB_PUBLIC EdgeEvaluatorBase
{
public:
  // [[ZH]] property_type = size_t，是边上附带的"来源标签"类型（见文件头对 property 的说明）。
  typedef size_t property_type;

  EdgeEvaluatorBase () { }
  virtual ~EdgeEvaluatorBase () { }

  // [[ZH]] 功能：把评估器恢复到确定的初始状态（清空各环绕数计数）。
  // [[ZH]] 调用时机：每条扫描线开始时由引擎调用（见 EdgeProcessorState::reset）。
  virtual void reset () { }
  // [[ZH]] 功能：预分配内部容器，避免处理过程中反复分配。
  // [[ZH]] 参数：预期要处理的边数（仅作提示，实现可忽略）。
  virtual void reserve (size_t /*n*/) { }
  // [[ZH-BEGIN]]
  // 功能：★ 核心回调。引擎就"当前这段边"征询评估器。
  //
  // 参数：north  true = 更新扫描线**上方**的状态；false = 下方。
  //               （一次扫描线事件通常上下各问一次，用于比较两侧）
  //       enter  是否"增加"环绕数：true → +1，false → -1。
  //              典型实现是 `*wcv += (enter ? 1 : -1);`
  //              注意：引擎传入的实参是 prefer_touch()（见文件头"易踩的坑"）。
  //       p      该边的 property（来源标签），用于区分不同输入的多边形/图层。
  //
  // 返回：★ 不是布尔值！而是"对输出边方向的贡献"，会被引擎**累加**：
  //         +1 / -1  表示该段构成结果边界，符号决定输出边方向
  //          0       表示与结果无关（该段会被丢弃）
  //       详见文件头"最重要的机制"一节。
  //
  // 实现约定：返回值应当是"状态迁移量"，即比较调用前后的 inside/outside 状态：
  //       由外变内 → +1；由内变外 → -1；状态未变 → 0。
  // [[ZH-END]]
  virtual int edge (bool /*north*/, bool /*enter*/, property_type /*p*/) { return 0; }
  // [[ZH-BEGIN]]
  // 功能：可选的"按标签选边"通道 —— 让评估器直接指定哪条边要投递给 sink。
  //
  // 参数：horizontal 被询问的边是否水平（dy() == 0）；
  //       p          该边的 property。
  // 返回：tag > 0  → 引擎会调用 sink->put(edge, tag) 把这条边投递出去（tag 原样传递）；
  //       tag <= 0 → 该边不走这条通道。
  //
  // 何时被调用：仅当 selects_edges() 返回 true 时才启用本通道，
  //             且只对"edge_ymin == 当前扫描线 y"的边调用（见 .cc）。
  // 用途：EdgePolygonOp 用它实现"挑出位于多边形内部的边"，并用 tag 区分内部/外部。
  // [[ZH-END]]
  virtual int select_edge (bool /*horizontal*/, property_type /*p*/) { return 0; }
  // [[ZH-BEGIN]]
  // 功能：比较"扫描线上方"与"扫描线下方"的内外状态之差：
  //           返回 result(north) - result(south)
  //       即：站在当前 x 位置，往正上方看是否在结果内部、往正下方看是否在结果内部，二者之差。
  //
  // 返回：+1 = 上方在内、下方在外；-1 = 下方在内、上方在外；0 = 两侧一致。
  //
  // ★ 谁在用、为什么需要它：引擎在扫描线事件处理中需要**合成水平边**
  //   （因为扫描线算法天然产生水平方向的边界段）。合成时要知道这条水平边界
  //   应当朝哪个方向 —— 而"方向"正是由本函数的返回值决定的：
  //       若 compare_ns() > 0，则把合成出的水平边交换端点（翻转方向）。
  //   见 dbEdgeProcessor.cc 中 EdgeProcessorState::end_vertex 的 `m_ho = mp_op->compare_ns ()`。
  //
  // 注意：函数名里的 ns = north/south。
  // [[ZH-END]]
  virtual int compare_ns () const { return 0; }
  // [[ZH]] 功能：当前状态是否已回到初始（即所有环绕数归零）。
  // [[ZH]] 用途：引擎用它识别"这一段处理不会产生任何变化"的区间，从而走 skip_n 优化路径，
  // [[ZH]]       跳过整段边而不逐条处理。见 dbEdgeProcessor.cc 的 SkipInfo。
  virtual bool is_reset () const { return false; }
  // [[ZH]] 功能：询问评估器"相切/重合(touching)是否算作内部"。
  // [[ZH]] 为什么重要：引擎把这个返回值当作 edge() 的 enter 实参传下去（见文件头的坑）。
  // [[ZH]]              所以它会直接影响环绕数的加减方向。
  virtual bool prefer_touch () const { return false; }
  // [[ZH]] 功能：是否启用 select_edge() 通道。
  // [[ZH]] 返回 true 时，引擎会逐条对"起始于本扫描线"的边调用 select_edge()。
  virtual bool selects_edges () const { return false; }
};

/**
 *  @brief An intersection detector
 *
 *  This edge evaluator will not produce output edges but rather record the 
 *  property pairs of polygons intersecting or interacting in the specified
 *  way.
 *
 *  It will build a set of property pairs, where the lower property value
 *  is the first one of the pairs. 
 */
// [[ZH-BEGIN]]
// ============================================================================
//  InteractionDetector —— "相互作用检测器"（不产生几何，只报告配对关系）
// ============================================================================
//
// 【与其它评估器的本质区别】
//   前面那些评估器（SimpleMerge / BooleanOp ...）都通过 edge() 的返回值
//   **输出结果几何**。而本类**不输出任何边** —— 它把结果记录在内部集合里，
//   事后通过 begin()/end() 迭代读出"哪些输入多边形之间存在某种关系"。
//   因此它常配合一个空的 EdgeContainer 使用（sink 只是必需的占位）。
//
// 【报告什么】一组 (primary_id, secondary_id) 配对，用 std::set 去重且有序。
//   ★ 配对中**较小的 property 放前面**，所以迭代出来的顺序是确定的。
//
// 【★ property 分工 —— 使用本类的前提】
//   必须把输入分成"主要(primary)"与"次要(secondary)"两组：
//       property ∈ [0, last_primary_id]        → primary
//       property >  last_primary_id            → secondary
//   这个分界由构造参数 last_primary_id 指定。若不遵守，判定结果无意义。
//
// 【四种模式及其含义】
//   mode =  0  重叠或相切（overlapping / touching）
//              是否把"相切"算作相互作用由 set_include_touching() 决定。
//   mode = -1  次要组中**位于主要多边形内部**的所有多边形
//   mode = -2  主要组中**包围了次要多边形**的所有多边形
//   mode = +1  次要组中**位于主要多边形外部**的所有多边形
//
// 【★ 必须调用 finish() 的情形】
//   mode = -1、-2、+1 都需要在读取结果**之前**调用 finish()。
//   原因：这些模式判定的是"包含/排除"关系，必须等整条扫描线跑完、
//   `m_inside_n`/`m_inside_s` 这些"当前在谁内部"的集合稳定之后才能定论。
//   mode = 0 是即时的，不需要 finish()。
//
// 【上游文档中坦诚标注的已知限制】
//   单元测试 TEST(26c)/TEST(26d) 的注释里写着 **"does not work yet!"** ——
//   即 mode = -1 配合某些背景 property 组合时结果不正确。
//   这是上游已知且未修的问题，使用这些模式时要留意。
//
// 【报告格式的一处反直觉设计】
//   +1（外部）模式下的配对是"**伪配对**"：按定义外部的多边形与主要多边形
//   并不相交，所以不存在真正的相互作用点。此时报出的 primary_id 恒为
//   last_primary_id（即一个代表"背景"的合成 id）。上游文档明确说明了这一点，
//   以免使用者把它当作真实的几何关系。
// [[ZH-END]]
class DB_PUBLIC InteractionDetector
  : public EdgeEvaluatorBase
{
public:
  // [[ZH]] interactions_type：结果类型 —— property 配对的集合（std::set，去重且有序）。
  // [[ZH]] iterator          ：遍历结果的只读迭代器（begin()/end() 返回它）。
  typedef std::set<std::pair<property_type, property_type> > interactions_type;
  typedef interactions_type::const_iterator iterator;

  /**
   *  @brief Constructor
   *
   *  The mode parameter selects the interaction check mode.
   *  0 is "overlapping" or "touching".
   *  -1 will select all secondary polygons inside polygons from the primary.
   *  -2 will select all primary polygons enclosing polygons from the secondary.
   *  +1 will select all secondary polygons outside polygons from the primary.
   *
   *  Use set_include_touching(f) to specify whether to include or not include the touching
   *  case as interacting for mode 0.
   *
   *  In modes -2, -1 and +1, finish () needs to be called before the interactions
   *  can be used.
   *
   *  All modes require property IDs to differentiate both inputs into primary and secondary.
   *  Property IDs from 0 to the given last primary ID value are considered to belong to
   *  the primary region. Property IDs above the last primary ID are considered to belong to
   *  the secondary region.
   *  This last property ID must be specified in the last_primary_id parameter.
   *  The reported interactions will be (primary_id,secondary_id) even for outside mode.
   *  For outside mode, the primary_id is always last_primary_id. In outside mode, the
   *  interactions are pseudo-interactions as by definition outside polygons don't interact.
   */
  InteractionDetector (int mode = 0, property_type last_primary_id = 0);

  /**
   *  @brief Sets the "touching" flag
   *  
   *  If this flag is set, the interaction will include "touching" interactions in mode 0, i.e.
   *  touching shapes will be counted as interacting.
   *  In the other modes, this flag should be true (the default).
   */
  void set_include_touching (bool f) 
  {
    m_include_touching = f;
  }

  /**
   *  @brief Gets the "touching" flag
   */
  // [[ZH]] 功能：读取"相切"标志。
  // [[ZH]] 附带作用：本值也被当作 prefer_touch() 的返回值，从而影响 edge() 的 enter 实参。
  bool include_touching () const
  {
    return m_include_touching;
  }

  /**
   *  @brief Finish the collection
   *
   *  This method must be called in mode -1 and +1 in order to finish the collection.
   */
  // [[ZH]] 功能：★ 收尾计算 —— 在读取结果**之前**必须对 mode = -1/-2/+1 调用。
  // [[ZH]] 为什么：这些模式判定的是"包含/排除"关系，必须等整个扫描结束、
  // [[ZH]]       m_inside_n/m_inside_s 稳定后才能定论，无法在扫描过程中即时给出。
  // [[ZH]] 修改：向 m_interactions 写入最终的配对结果。
  void finish ();

  /**
   *  @brief Iterator delivering the interactions (begin iterator)
   *
   *  The iterator delivers pairs of property values. The lower value will be the first one of the pair.
   */
  // [[ZH]] 功能：结果迭代起始。遍历得到 (primary_id, secondary_id) 配对，
  // [[ZH]]       且**较小的 property 恒在第一位**。
  // [[ZH]] 坑：mode = -1/-2/+1 时若未先调 finish()，这里得到的结果不完整。
  iterator begin () const
  {
    return m_interactions.begin ();
  }

  /**
   *  @brief Iterator delivering the interactions (end iterator)
   */
  // [[ZH]] 功能：结果迭代终止（与 begin() 配对使用）。
  iterator end () const
  {
    return m_interactions.end ();
  }

  // [[ZH]] 功能：重置本评估器状态（清空"当前在谁内部"的集合与零计数）。
  virtual void reset ();
  // [[ZH]] 功能：按 property 总数 n 预分配 m_wcv_n/m_wcv_s。
  virtual void reserve (size_t n);
  // [[ZH]] 功能：更新该 property 的环绕数，并在 mode=0 时即时记录相互作用配对。
  virtual int edge (bool north, bool enter, property_type p);
  // [[ZH]] 功能：扫描线上/下两侧的"是否在内部"判定之差。
  virtual int compare_ns () const;
  // [[ZH]] 功能：两侧"当前在内部的多边形集合"是否都已清空。
  virtual bool is_reset () const { return m_inside_s.empty () && m_inside_n.empty (); }
  // [[ZH]] 功能：是否把相切算作相互作用 —— 同时决定 edge() 的 enter 实参取值。
  virtual bool prefer_touch () const { return m_include_touching; }

private:
  // [[ZH]] m_mode           ：上面说明的四种模式之一（0 / -1 / -2 / +1）。
  // [[ZH]] m_include_touching：是否把"相切"视为相互作用；也作为 prefer_touch() 返回值。
  // [[ZH]] m_last_primary_id：primary/secondary 的分界。property <= 它为 primary。
  // [[ZH]] m_wcv_n/m_wcv_s  ：每个 property 各自的环绕数（以 property 为下标）。
  // [[ZH]] m_inside_n/m_inside_s：当前处于内部的多边形集合（上/下侧）——
  // [[ZH]]                     用于在扫描过程中累积"谁在谁里面"，finish() 时汇总。
  // [[ZH]] m_interactions   ：★ 最终结果 —— 去重且有序的 property 配对集合。
  // [[ZH]] m_non_interactions：暂存"已确定不相互作用"的配对，用于在 finish() 时
  // [[ZH]]                     反推外部关系（尤其 +1 外部模式）。
  int m_mode;
  bool m_include_touching;
  property_type m_last_primary_id;
  std::vector <int> m_wcv_n, m_wcv_s;
  std::set <property_type> m_inside_n, m_inside_s;
  std::set<std::pair<property_type, property_type> > m_interactions;
  std::set<property_type> m_non_interactions;
};

/**
 *  @brief A generic inside operator
 *
 *  This incarnation of the evaluator class implements an "inside" function
 *  based on a generic operator.
 */
// [[ZH-BEGIN]]
// 功能：★ "通用融合"评估器 —— 用**可替换的内外判定函数 F** 实现融合运算的骨架。
//
//       它是理解所有融合类评估器的关键：把"什么算内部"抽象成一个函数对象 F，
//       本类只负责维护环绕数并计算"状态迁移量"。
//       改变 F 就得到不同的融合规则：
//         · SimpleMerge = GenericMerge<ParametrizedInsideFunc>（最常用，带 mode）
//         · dbRegionUtils 里还用自定义 F 做过"奇怪多边形"检测
//
// 【★ 本类的 wc 语义】此处 wc = **有向环绕数 (winding number)**，
//   即射线穿过多边形边界的有向次数。注意这与 MergeOp 的 wc
//   （"张开的 polygon 个数"）**完全不同**，切勿混淆。
//   因为是有向的，wc 可以为负 —— 这正是 ParametrizedInsideFunc 要同时处理
//   正负两种 mode 的原因。
//
// 参数：模板参数 F 是谓词，签名约定为 `bool operator()(int wc)`，
//       返回该环绕数是否代表"在内部"。
//
// ★ 实现范式（值得单独记住）：
//   每次 edge() 都做"**前/后**比较"：
//         t0 = F(wc)          // 改变之前是否在内部
//         wc += (enter ? +1 : -1)
//         t1 = F(wc)          // 改变之后是否在内部
//         返回 +1 (外→内) / -1 (内→外) / 0 (无变化)
//   即返回值是**状态迁移量**，不是布尔值。这正是基类文档所说的"累加"协议。
//
// 坑：本类**忽略 property 参数 p** —— 它把所有输入边合在一起算一个总环绕数。
//     若需要按输入分别计数（例如 BooleanOp 要区分 A/B），必须改用别的评估器。
// [[ZH-END]]
template <class F>
class DB_PUBLIC_TEMPLATE GenericMerge
  : public EdgeEvaluatorBase
{
public:
  /**
   *  @brief Constructor
   */
  // [[ZH]] 功能：用给定的内外判定函数 F 构造，环绕数初值置 0。
  // [[ZH]] 参数：function 内外判定谓词（会被拷贝保存到 m_function）。
  GenericMerge (const F &function) 
    : m_wc_n (0), m_wc_s (0), m_function (function)
  { }

  // [[ZH]] 功能：把上下两侧的环绕数都归零，回到确定初始状态。
  // [[ZH]] 修改：m_wc_n = m_wc_s = 0。
  virtual void reset ()
  {
    m_wc_n = m_wc_s = 0;
  }

  // [[ZH]] 功能：预分配 —— 本类只用两个 int 计状态，无容器可预留，故为空。
  virtual void reserve (size_t /*n*/)
  {
    // .. nothing yet ..
  }

  // [[ZH-BEGIN]]
  // 功能：★ 核心 —— 根据这条边更新对应侧的环绕数，并返回状态迁移量。
  //
  // 参数：north 选哪一侧（true = 上方 m_wc_n，false = 下方 m_wc_s）；
  //       enter true → 环绕数 +1，false → -1；
  //       p     来源标签（★ 本类忽略它）。
  // 返回：+1 = 由外变内；-1 = 由内变外；0 = 状态未变。
  //
  // 修改：选中一侧的环绕数（±1）。
  // 实现：见上面类注释里的"前/后比较"范式 —— 比较修改前后的 F(wc)。
  // [[ZH-END]]
  virtual int edge (bool north, bool enter, property_type /*p*/)
  {
    // [[ZH]] 按 north 选择要更新的那一侧计数器。
    int *wc = north ? &m_wc_n : &m_wc_s;
    // [[ZH]] t0 = 更新"之前"是否处于内部。
    bool t0 = m_function (*wc);
    // [[ZH]] 更新环绕数：enter 为真则 +1，否则 -1。
    if (enter) {
      ++*wc;
    } else {
      --*wc;
    }
    // [[ZH]] t1 = 更新"之后"是否处于内部。
    bool t1 = m_function (*wc);
    // [[ZH]] 返回状态迁移量：外→内 为 +1，内→外 为 -1，其余为 0。
    if (t1 && ! t0) {
      return 1;
    } else if (! t1 && t0) {
      return -1;
    } else {
      return 0;
    }
  }

  // [[ZH-BEGIN]]
  // 功能：比较"下方"与"上方"的内外状态之差，供引擎给合成的水平边定方向。
  // 返回：-1 = 下方在内而上方在外；+1 = 上方在内而下方在外；0 = 两侧一致。
  // 注意：语义等价于 result(south) - result(north) 的符号约定，
  //       与 MergeOp/BooleanOp 的 compare_ns 保持一致（见各自实现）。
  // [[ZH-END]]
  virtual int compare_ns () const
  {
    if (m_function (m_wc_s) && ! m_function (m_wc_n)) {
      // [[ZH]] 下方在内、上方在外 → -1。
      return -1;
    } else if (! m_function (m_wc_s) && m_function (m_wc_n)) {
      // [[ZH]] 上方在内、下方在外 → +1。
      return 1;
    } else {
      // [[ZH]] 两侧状态一致 → 无方向偏好。
      return 0;
    }
  }

  // [[ZH]] 功能：两侧环绕数是否都已回到 0（即无任何边界张开）。
  // [[ZH]] 用途：引擎据此识别"整段处理无变化"的区间，走 skip_n 优化路径。
  virtual bool is_reset () const
  {
    return (m_wc_n == 0 && m_wc_s == 0);
  }

private:
  // [[ZH]] m_wc_n / m_wc_s：扫描线**上方 / 下方**的有向环绕数（winding number）。
  // [[ZH]]                  可为负（取决于边的绕行方向）。
  // [[ZH]] m_function      ：内外判定谓词。决定"什么算内部"，是本类唯一的可替换策略。
  int m_wc_n, m_wc_s;
  F m_function;
};

/**
 *  @brief A helper class to implement the SimpleMerge operator
 */
// [[ZH-BEGIN]]
// 功能：把整数模式 (mode) 翻译成"判断某个环绕数 wc 是否算作内部"的谓词。
//       它是一个**函数对象**（functor），被 GenericMerge 当作模板参数 F 使用，
//       也被 EdgePolygonOp 用来解释多边形自身的环绕数。
//
//       一句话：本结构体就是全库"融合模式(mode)"语义的**唯一定义处**。
//       其他类（SimpleMerge、EdgePolygonOp、BooleanOp2）的 mode 参数含义都来自这里。
//
// ★ mode 语义表（wc = 环绕数 / wrap count）：
//
//     mode > 0  (即 mode = n):
//         返回 wc >= n
//         → "至少有 n 圈重叠才算内部"
//         例：n=1 即非零环绕规则；n=2 即"至少两个多边形重叠处才算"
//
//     mode < 0  (即 mode = -n):
//         返回 (wc <= -n) || (-wc <= -n)，等价于 **|wc| >= n**
//         → 与 mode = +n 的区别是**同时接受负环绕数**
//         为什么需要：环绕数可为负（取决于边的绕行方向）。
//         mode = -1（默认值）表示"不论正负，只要非零就算内部"，
//         即经典的 **非零环绕规则 (non-zero winding rule)**，最常用。
//
//     mode == 0:
//         返回 wc 为奇数（对负数取绝对值后再取模）
//         → **奇偶规则 (even-odd rule)**
//         即"射线穿过边界的次数为奇数 ⇒ 在内部"，可以正确处理自相交图形。
//
// 参数：wc 待判断的环绕数（可为负）。
// 返回：true = 该位置算作"在结果内部"。
//
// 坑：1) mode == 0 走的是**奇偶规则**，不是"wc >= 0"或"wc != 0" —— 容易被误读。
//     2) mode 的正负不对称是有意设计：+n 只看正环绕数，-n 看两侧。
//        所以 mode = 1 与 mode = -1 结果**不同**（前者只认正环绕）。
//     3) 对负数取模的行为在 C++ 中依赖实现约定，这里用 `wc < 0 ? (-wc) % 2 : wc % 2`
//        显式处理了符号，保证负数也得到正确的奇偶性。
// [[ZH-END]]
struct ParametrizedInsideFunc 
{
  // [[ZH]] 功能：用给定的整数模式构造谓词。mode 的取值语义见上面的语义表。
  ParametrizedInsideFunc (int mode)
    : m_mode (mode)
  {
    //  .. nothing yet ..
  }

  // [[ZH]] 功能：判断环绕数 wc 是否算作"内部"。见上面的 mode 语义表。
  inline bool operator() (int wc) const
  {
    if (m_mode > 0) {
      // [[ZH]] mode = +n：只认正环绕数，要求 wc >= n。
      return wc >= m_mode;
    } else if (m_mode < 0) {
      // [[ZH]] mode = -n：两边都认，等价于 |wc| >= n。
      // [[ZH]] mode = -1（默认）即"非零环绕规则"。
      return wc <= m_mode || -wc <= m_mode;
    } else {
      // [[ZH]] mode = 0：奇偶规则。对负数先取绝对值再判奇偶。
      return (wc < 0 ? ((-wc) % 2) : (wc % 2)) != 0;
    }
  }

public:
  // [[ZH]] m_mode：上面语义表里的模式值。构造时设定，之后不再改变。
  int m_mode;
};

/**
 *  @brief Simple merge operator
 *
 *  This incarnation of the evaluator class implements a simple 
 *  merge criterion for a set of edges. A result is generated if the wrap count (wc)
 *  of the edge set satisfies a condition given by "mode". Specifically:
 *    mode == 0: even-odd rule (wc = ... -3, -1, 1, 3, ...)
 *    mode == 1: wc >= 1
 *    mode == -1: wc >= 1 || wc <= -1
 *    mode == n: wc >= n
 *    mode == -n: wc >= n || wc <= -n
 */
// [[ZH]] 功能：最常用的融合评估器 —— GenericMerge + ParametrizedInsideFunc 的具名特化。
// [[ZH]]       等价于"用 mode 指定内外规则、把所有输入边合起来算一个总环绕数"。
// [[ZH]] 参数：模板已固定为 ParametrizedInsideFunc，无额外模板参数。
// [[ZH]] 用途：EdgeProcessor::simple_merge() 内部用的就是它。
// [[ZH]] 注意：本类**没有新增任何成员或方法**，纯粹是给模板取个好用的名字，
// [[ZH]]       因此行为完全由 GenericMerge + ParametrizedInsideFunc 决定。
class DB_PUBLIC SimpleMerge
  : public GenericMerge<ParametrizedInsideFunc>
{
public:
  /**
   *  @brief Constructor
   */
  // [[ZH]] 功能：构造。mode 决定内外规则（见 ParametrizedInsideFunc 的语义表）。
  // [[ZH]] 参数：mode 默认 -1 = **非零环绕规则**（最常用）。
  // [[ZH]]             0 = 奇偶规则（可正确处理自相交）；
  // [[ZH]]             +n = 至少 n 圈才算内部；-n = |wc| >= n。
  SimpleMerge (int mode = -1)
    : GenericMerge<ParametrizedInsideFunc> (ParametrizedInsideFunc (mode))
  { }
};

/**
 *  @brief Boolean operations
 *
 *  This incarnation of the evaluator class implements a boolean operation
 *  (AND, A NOT B, B NOT A, XOR, OR). The mode can be specified in the constructor.
 *  It relies on the properties being set in a certain way: bit 0 codes the layer (0 for A, 1 for B)
 *  while the other bits are used to distinguish the polygons. For each polygon, a non-zero
 *  wrap count rule is applied before the boolean operation's output is formed.
 */
class DB_PUBLIC BooleanOp 
  : public EdgeEvaluatorBase 
{
public:
  // [[ZH-BEGIN]]
  // 功能：五种布尔运算的枚举。★ 注意枚举值从 1 开始（不是 0），
  //       且这些数值会被直接当作 int mode 传给 boolean() 便捷函数。
  //          And   = 1   交集             A ∩ B
  //          ANotB = 2   A 减去 B         A \ B
  //          BNotA = 3   B 减去 A         B \ A
  //          Xor   = 4   对称差（异或）    A △ B
  //          Or    = 5   并集             A ∪ B
  //
  // ★ 输入属性(property)的约定 —— 用布尔运算前必须遵守，否则结果错误：
  //   评估器靠 `p % 2`（最低位）区分这条边属于哪个操作数：
  //        偶数 property → 操作数 A
  //        奇数 property → 操作数 B
  //   因此：
  //     · boolean() 的多边形版本会自动分配 A = 0,2,4,...  B = 1,3,5,...
  //     · 若你自己 insert 边并手工填 property，务必保持这个奇偶约定。
  //   高位（除 bit0 外）用于区分同一操作数内的不同多边形（各自独立算环绕数，
  //   从而让"同一个操作数内部的多个多边形"先各自归一化再参与布尔运算）。
  //
  // 提示：脚本里这些常量暴露为 ModeAnd / ModeOr / ModeXor / ModeANotB / ModeBNotA。
  // [[ZH-END]]
  enum BoolOp {
    And = 1, ANotB = 2, BNotA = 3, Xor = 4, Or = 5
  };
 
  /**
   *  @brief Constructor
   *
   *  @param mode The boolean operation that this object represents
   */
  // [[ZH]] 功能：用指定的布尔运算类型构造评估器。mode 取值见上面的 BoolOp 枚举。
  BooleanOp (BoolOp mode);

  virtual void reset ();
  virtual void reserve (size_t n);
  virtual int edge (bool north, bool enter, property_type p);
  virtual int compare_ns () const;
  virtual bool is_reset () const { return m_zeroes == m_wcv_n.size () + m_wcv_s.size (); }

protected:
  template <class InsideFunc> bool result (int wca, int wcb, const InsideFunc &inside_a, const InsideFunc &inside_b) const;
  template <class InsideFunc> int edge_impl (bool north, bool enter, property_type p, const InsideFunc &inside_a, const InsideFunc &inside_b);
  template <class InsideFunc> int compare_ns_impl (const InsideFunc &inside_a, const InsideFunc &inside_b) const;

private:
  int m_wc_na, m_wc_nb, m_wc_sa, m_wc_sb;
  std::vector <int> m_wcv_n, m_wcv_s;
  BoolOp m_mode;
  size_t m_zeroes;
};

/**
 *  @brief Edge vs. Polygon intersection
 *
 *  This operator detects edges inside or outside polygons.
 *  The polygon edges must be given with property 0, the other edges
 *  with properties 1 and higher.
 *
 *  The operator will deliver edges inside polygons or outside polygons.
 *  It can be configured to include edges on the border of the polygons
 *  as being considered "inside the polygon".
 */
class DB_PUBLIC EdgePolygonOp 
  : public db::EdgeEvaluatorBase
{
public:
  /**
   * @brief The operation mode
   */
  // [[ZH-BEGIN]]
  // 功能：选择"挑出哪些边"的模式。
  //         Inside  = 0  挑出位于多边形**内部**的边
  //         Outside = 1  挑出位于多边形**外部**的边
  //         Both    = 2  两者都挑，并用**标签(tag)**区分：
  //                      内部边 → tag #1
  //                      外部边 → tag #2
  //       标签通过 sink 的 put(edge, tag) 重载传出，sink 可据此分流。
  //
  // ★ 输入约定：多边形自身的边必须用 property = 0，被检查的那些边用 property >= 1。
  //              评估器靠这个区分"边界"与"待过滤对象"。
  //
  // 关于 include_touching：决定"恰好落在多边形边界上"的边算内部还是外部。
  // [[ZH-END]]
  enum mode_t {
    Inside = 0,    //  Selects inside edges
    Outside = 1,   //  Selects outside edges
    Both = 2       //  Selects both (inside -> tag #1, outside -> tag #2)
  };

  /**
   *  @brief Constructor
   *
   *  @param outside If true, the operator will deliver edges outside the polygon
   *  @param include_touching If true, edges on the polygon's border will be considered "inside" of polygons
   *  @param polygon_mode Determines how the polygon edges on property 0 are interpreted (see merge operators)
   */
  // [[ZH-BEGIN]]
  // 功能：构造 边-多边形 关系评估器。
  //
  // 参数：mode 选择 Inside / Outside / Both，含义见上面 mode_t 的注释。
  //       include_touching 为 true 时，"恰好落在多边形边界上"的边被视为**内部**。
  //                        它同时决定 prefer_touch() 的返回值，因此会影响环绕数判定。
  //       polygon_mode 如何解释 property=0 的多边形边 —— 即用哪种融合规则
  //                    （语义同 SimpleMerge 的 mode，见 ParametrizedInsideFunc 的语义表）。
  //                    默认 -1 = 非零环绕规则。可用 0 表示某些"退化/自相交"的多边形按奇偶规则解释。
  //
  // ⚠ 上游文档笔误提醒：上面英文 @param 写的是 "outside"（布尔），
  //   但实际形参是 `mode_t mode`（枚举）—— 属于重构后未更新的残留注释。
  //   请以函数签名为准。
  // [[ZH-END]]
  EdgePolygonOp (mode_t mode = Inside, bool include_touching = true, int polygon_mode = -1);

  virtual void reset ();
  virtual int select_edge (bool horizontal, property_type p);
  virtual int edge (bool north, bool enter, property_type p);
  virtual bool is_reset () const;
  virtual bool prefer_touch () const;
  virtual bool selects_edges () const;

private:
  mode_t m_mode;
  bool m_include_touching;
  db::ParametrizedInsideFunc m_function;
  int m_wcp_n, m_wcp_s;
};

/**
 *  @brief Boolean operations
 *
 *  This class implements a boolean operation similar to BooleanOp, but 
 *  in addition it allows one  to specify the merge mode for the two inputs.
 *  See "SimpleMergeOp" for the definition of the merge modes.
 *  This operator is especially useful to implement boolean operations
 *  with sized polygons which required a >0 interpretation.
 */
// [[ZH-BEGIN]]
// 功能：BooleanOp 的增强版 —— 允许为 A、B 两个操作数**分别**指定融合模式。
//
// 【与 BooleanOp 的唯一区别】
//   BooleanOp  用固定的 NonZeroInsideFunc（非零环绕规则）解释两侧的环绕数。
//   BooleanOp2 额外接收 wc_mode_a / wc_mode_b，改用 ParametrizedInsideFunc，
//               因此可以对 A 和 B 采用**不同**的内外规则。
//
// 参数：wc_mode_a / wc_mode_b 的语义与 SimpleMerge 的 mode 完全一致
//       （见 ParametrizedInsideFunc 的语义表）：
//         -1 = 非零环绕规则（默认）；0 = 奇偶规则；+n = 至少 n 圈；-n = |wc| >= n。
//
// 【典型用途（上游文档特别指出的场景）】
//   对**已经做过 sizing（尺寸缩放）的多边形**做布尔运算时，
//   缩放会在图形上产生自相交的重叠圈，此时必须用 mode > 0 的"重叠计数"解释，
//   否则自相交区域会被非零规则误判为内部。
//
// 实例参考：单元测试 TEST(27) 用 BooleanOp2(Xor, -1, -1) 把"盒子与其内缩副本"
//           的异或结果从 4 个角上的碎片合并为 1 个带孔多边形 ——
//           这是理解 wc_mode_a/b 作用的最佳示例。
// [[ZH-END]]
class DB_PUBLIC BooleanOp2
  : public BooleanOp
{
public:
  /**
   *  @brief Constructor
   *
   *  @param mode The boolean operation that this object represents
   */
  BooleanOp2 (BoolOp mode, int wc_mode_a, int wc_mode_b);

  virtual int edge (bool north, bool enter, property_type p);
  virtual int compare_ns () const;

private:
  int m_wc_mode_a, m_wc_mode_b;
};

/**
 *  @brief Merge operation
 *
 *  This incarnation of the evaluator class implements a merge operation
 *  which allows one  to distinguish polygons (through edge properties) and
 *  allows one to specify a overlap value. Default is 0 which means that the
 *  merge is equivalent to producing all polygins. A overlap value of 1 means
 *  that at least two polygons must overlap to produce a result.
 */
// [[ZH-BEGIN]]
// ⚠ 先看这里，避免和其他 wc 混淆：
//   本类中的 wc **不是环绕数(winding number)**，而是"**当前张开的（重叠的）多边形个数**"。
//   它由每个 property 各自的环绕数统计得来：
//       对某个 property p：  "该多边形张开"  ⟺  m_wcv[p] != 0
//       m_wc（本类用来判定的那个 wc）  =   当前 wcv 非零的 property 个数
//   所以 wc 的语义是"此处有几个多边形叠在一起"，与方向、绕行圈数无关。
//
// 判定条件（实现见 dbEdgeProcessor.cc 的 result_by_mode）：
//       result = (wc > min_wc)      ← 严格大于
//   代入可得：
//       min_overlap = 0  →  wc >= 1  →  "至少 1 个多边形"  ⇒ 输出全部多边形（普通融合）
//       min_overlap = 1  →  wc >= 2  →  "至少 2 个多边形重叠处才输出"
//       min_overlap = n  →  wc >= n+1
//   这与上面英文类文档的表述**一致**（"0 = all polygons，1 = at least two"），
//   已核对实现，不存在差一问题。但请注意"至少 n+1 个"这个实际效果，
//   因为 wc 是"个数"而条件是"严格大于"，初看容易数错。
//
// 与 SimpleMerge 的分工（两者名字很像，用途不同）：
//   SimpleMerge —— 把所有多边形**合在一起**算一个总环绕数（重叠可能相互抵消）
//   MergeOp     —— 每个多边形**先各自归一化**（只看它自己是否张开），
//                  再数"有几个叠在一起"，因此更适合"判定重叠数目"的场景
//
// 相关：便捷函数 EdgeProcessor::merge(...) 的 min_wc 参数就是本类的 min_overlap。
//       实现里它给第 i 个多边形分配 property = i（见 .cc），正好对应"按 property 计数"。
// [[ZH-END]]
class DB_PUBLIC MergeOp 
  : public EdgeEvaluatorBase 
{
public:
  /**
   *  @brief Constructor
   *
   *  @param min_overlap See class description
   */
  // [[ZH]] 功能：构造重叠数评估器。min_overlap 的含义见上面的说明
  // [[ZH]]       （实际判定为 wc > min_overlap，即"多于 min_overlap 个多边形重叠处才输出"）。
  MergeOp (unsigned int min_overlap = 0);

  // [[ZH]] 功能：清空所有计数，回到确定初始状态。
  // [[ZH]] 修改：m_wcv_n/m_wcv_s 全部置 0；m_wc_n/m_wc_s 置 0；m_zeroes 置为两侧槽位总数。
  virtual void reset ();
  // [[ZH]] 功能：按 property 总数 n 预分配 m_wcv_n/m_wcv_s（它们以 property 为下标）。
  // [[ZH]] 参数：n 需覆盖的 property 个数（= 最大 property + 1）。
  // [[ZH]] 为什么必须：edge() 里有 tl_assert(p < m_wcv_n.size())，未 reserve 就会断言失败。
  virtual void reserve (size_t n);
  // [[ZH-BEGIN]]
  // 功能：★ 核心 —— 更新"该 property 的多边形是否张开"，并重算重叠个数，返回状态迁移量。
  //
  // 参数：north 选上/下侧；enter ±1；p 边的 property（★ 本类**会用它**，按 property 分开计数）。
  // 返回：+1 = 重叠判定由假变真；-1 = 由真变假；0 = 无变化。
  //
  // 修改：m_wcv_n[p] 或 m_wcv_s[p]（±1）、m_wc_n/m_wc_s（张开个数）、m_zeroes。
  // 实现细节（见 .cc）：
  //   · inside_before/after 由"该 property 的 wcv 是否非零"决定，而非由 wcv 的数值决定；
  //   · 只有 when 该 property 的张开状态**发生变化**时才调整 m_wc（张开个数），
  //     这样 z 字形的重复穿越不会把同一个多边形数两次。
  //   · m_zeroes 同步维护，使 is_reset() 成为 O(1)。
  // [[ZH-END]]
  virtual int edge (bool north, bool enter, property_type p);
  // [[ZH]] 功能：比较下方与上方的重叠判定之差，供引擎给合成的水平边定方向。
  // [[ZH]] 实现：result_by_mode(m_wc_n, m_min_wc) - result_by_mode(m_wc_s, m_min_wc)。
  virtual int compare_ns () const;
  // [[ZH]] 功能：是否所有 property 槽位都已归零（即该处无任何多边形重叠）。
  // [[ZH]] 实现：比较 m_zeroes（零槽位计数）与两侧槽位总数之和 —— O(1)，无需遍历。
  virtual bool is_reset () const { return m_zeroes == m_wcv_n.size () + m_wcv_s.size (); }

private:
  // [[ZH]] m_wc_n / m_wc_s：★ 当前张开的（重叠的）**多边形个数**，非环绕数。见类注释。
  // [[ZH]]                 判定为 (m_wc > m_min_wc)，故 min_wc=0 表示"至少 1 个"。
  // [[ZH]] m_wcv_n/m_wcv_s：**每个 property 各自的**环绕数 —— 非零即表示该多边形"张开"。
  // [[ZH]]                  以 property 为下标，故必须先 reserve(n_props)，否则断言失败。
  // [[ZH]] m_min_wc        ：构造时给定的最小重叠阈值；实际判定是 wc > m_min_wc（严格大于）。
  // [[ZH]] m_zeroes        ：两侧 m_wcv 中仍为零的槽位数；用于 O(1) 判断 is_reset()。
  int m_wc_n, m_wc_s;
  std::vector <int> m_wcv_n, m_wcv_s;
  unsigned int m_min_wc;
  size_t m_zeroes;
};

/**
 *  @brief The basic edge processor
 *
 *  An edge processor takes a set of edges, processes them by removing intersections and 
 *  applying a custom operator for computing the output edge sets which then are delivered
 *  to an EdgeSink receiver object.
 */
// [[ZH-BEGIN]]
// ============================================================================
//  EdgeProcessor —— 公开门面（用户实际使用的类）
// ============================================================================
//
// 【心智模型】
//   把 EdgeProcessor 想象成一台"边的布尔运算机"：
//       [投入] 一堆边/多边形 + 一个评估器 + 一个输出 sink
//       [产出] 经"去相交 → 打断 → 扫描线求值"后，由 sink 收到的结果
//
//   典型三步用法（记住这个模式，80% 的场景都是它）：
//       1) 用 insert()/insert_sequence() 把输入边或整个多边形灌进去
//       2) 准备好 一个 EdgeEvaluatorBase（评估器）+ 一个 EdgeSink（输出端）
//       3) 调用 process(sink, evaluator)
//
//   若你只是做常见的布尔/合并/放大，**不必自己写评估器与 sink**：
//   直接用下面的 bulk 便捷函数（simple_merge / merge / size / boolean），
//   它们已经把"插入 + 选评估器 + 选 sink + 缝合多边形"全部打包好了。
//
// 【API 分组速览】
//   配置：  enable_progress / disable_progress / set_base_verbosity
//   投入：  insert(...)          —— 边、Polygon、SimplePolygon、PolygonRef（含带变换版本）
//           insert_sequence(...) —— 批量插入边序列
//           reserve / count / clear
//   处理：  process(sink, evaluator)        单输出
//           process(vector<pair<sink,evaluator>>)  ★ 一次扫描、多路输出（高效）
//           redo(...)                        复用已准备的边表重跑（跳过阶段 2、3）
//   便捷：  simple_merge(...)  按环绕数融合（mode 见 ParametrizedInsideFunc）
//           merge(...)         每个多边形独立归一化后融合（min_wc = 最小重叠数）
//           size(...)          尺寸放大/缩小（各向异性，dx/dy 可不同）
//           boolean(...)       两操作数的布尔运算（And/ANotB/BNotA/Xor/Or）
//   输出端：结果通过 EdgeSink 投递（见 EdgeSink 与 dbPolygonGenerators.h）
//
// 【★ 三个必须知道的行为特性】
//   1) **有状态、可复用**：边一旦 insert 就留在内部；process() 可以反复调用，
//      每次换不同的评估器/sink 得到不同结果。redo() 更进一步，跳过昂贵的
//      阶段 2、3（求交与打断），只重跑扫描线 —— 适合"同一批输入、多种输出"的场景。
//   2) **不是线程安全的**：所有状态都在对象内部。多线程请各线程各用各的实例。
//   3) 效率上，process() 处理的是**拷贝**出来的工作边表，代价主要在阶段 2（求交）。
//
// 【property 约定（做布尔运算时必须遵守）】
//   boolean() 对多边形的约定：操作数 A 用偶数 property（0,2,4,...），
//   操作数 B 用奇数（1,3,5,...）—— 即评估器靠 `p % 2` 区分 A/B。
//   若不按此约定填 property，BooleanOp 会把输入理解错。
//
// 【常见坑速查】
//   · 零点长的边在 insert 时被**静默丢弃**（不报错、不抛异常）——见 .cc 的 insert。
//   · 输入**不做任何合法性校验**，畸形多边形不会被拒绝，只会得到奇怪结果。
//   · 想要多边形结果时，记得 sink 要选 PolygonGenerator 一类（EdgeProcessor 自己
//     只输出边）；见 dbPolygonGenerators.h。
//   · 若输入量很大，务必用 reserve() 预留容量。
// [[ZH-END]]
class DB_PUBLIC EdgeProcessor
{
public:
  // [[ZH]] property_type = size_t：边的"来源标签"类型，详见文件头关于 property 的说明。
  typedef size_t property_type;

  /**
   *  @brief Default constructor
   *
   *  @param report_progress If true, a tl::Progress object will be created to report any progress (warning: this will impose a performance penalty)
   *  @param progress_text The description text of the progress object
   */
  // [[ZH]] 功能：构造一台空闲的边处理器（内部边表为空）。
  // [[ZH]] 参数：report_progress 是否上报进度（★ 会带来可观测的性能损失，批量运算时建议关闭）；
  // [[ZH]]       progress_text 进度条上显示的描述文字。
  EdgeProcessor (bool report_progress = false, const std::string &progress_desc = std::string ());

  /**
   *  @brief Destructor
   */
  ~EdgeProcessor ();

  /**
   *  @brief Enable progress reporting
   *
   *  @param progress_text The description text of the progress object
   */
  void enable_progress (const std::string &progress_desc = std::string ());

  /**
   *  @brief Disable progress reporting
   */
  void disable_progress ();

  /**
   *  @brief Base verbosity for timer reporting
   *
   *  The default value is 30. Basic timing will be reported for > base_verbosity, detailed timing
   *  for > base_verbosity + 10.
   */
  void set_base_verbosity (int bv);

  /**
   *  @brief Reserve space for at least n edges
   */
  void reserve (size_t n);

  /**
   *  @brief Reports the number of edges stored in the processor
   */
  size_t count () const;

  /**
   *  @brief Insert an edge 
   */
  void insert (const db::Edge &e, property_type p = 0);

  /**
   *  @brief Insert an edge with transformation
   */
  template <class Trans>
  void insert_with_trans (const db::Edge &e, const Trans &tr, property_type p = 0)
  {
    insert (tr * e, p);
  }

  /**
   *  @brief Insert a polygon
   */
  void insert (const db::Polygon &q, property_type p = 0)
  {
    for (db::Polygon::polygon_edge_iterator e = q.begin_edge (); ! e.at_end (); ++e) {
      insert (*e, p);
    }
  }

  /**
   *  @brief Insert a polygon with transformation
   */
  template <class Trans>
  void insert_with_trans (const db::Polygon &q, const Trans &tr, property_type p = 0)
  {
    for (db::Polygon::polygon_edge_iterator e = q.begin_edge (); ! e.at_end (); ++e) {
      insert (tr * *e, p);
    }
  }

  /**
   *  @brief Insert a simple polygon
   */
  void insert (const db::SimplePolygon &q, property_type p = 0)
  {
    for (db::SimplePolygon::polygon_edge_iterator e = q.begin_edge (); ! e.at_end (); ++e) {
      insert (*e, p);
    }
  }

  /**
   *  @brief Insert a simple polygon with transformation
   */
  template <class Trans>
  void insert_with_trans (const db::SimplePolygon &q, const Trans &tr, property_type p = 0)
  {
    for (db::SimplePolygon::polygon_edge_iterator e = q.begin_edge (); ! e.at_end (); ++e) {
      insert (tr * *e, p);
    }
  }

  /**
   *  @brief Insert a polygon reference
   */
  void insert (const db::PolygonRef &q, property_type p = 0)
  {
    for (db::PolygonRef::polygon_edge_iterator e = q.begin_edge (); ! e.at_end (); ++e) {
      insert (*e, p);
    }
  }

  /**
   *  @brief Insert a polygon reference with transformation
   */
  template <class Trans>
  void insert_with_trans (const db::PolygonRef &q, const Trans &tr, property_type p = 0)
  {
    for (db::PolygonRef::polygon_edge_iterator e = q.begin_edge (); ! e.at_end (); ++e) {
      insert (tr * *e, p);
    }
  }

  /**
   *  @brief Insert a sequence of edges
   *
   *  This method does not reserve for the number of elements required. This must
   *  be done explicitly for performance benefits.
   */
  template <class Iter>
  void insert_sequence (Iter from, Iter to, property_type p = 0)
  {
    for (Iter i = from; i != to; ++i) {
      insert (*i, p);
    }
  }

  /**
   *  @brief Insert a sequence of edges (iterator with at_end semantics)
   *
   *  This method does not reserve for the number of elements required. This must
   *  be done explicitly for performance benefits.
   */
  template <class Iter>
  void insert_sequence (Iter i, property_type p = 0)
  {
    for ( ; !i.at_end (); ++i) {
      insert (*i, p);
    }
  }

  /**
   *  @brief Clears all edges stored currently in this processor
   */
  void clear ();

  /**
   *  @brief Performs the actual processing
   *
   *  This method will use the edges stored so far and runs it through the
   *  scanline algorithm.
   */
  // [[ZH-BEGIN]]
  // 功能：★ 执行核心处理 —— 把目前 insert 进来的所有边跑一遍扫描线算法，
  //       结果按顺序投递给 sink `es`。
  //
  // 参数：es 结果接收端（EdgeSink 的子类实例）。
  //       op 评估器（决定每段边算不算结果）。
  //
  // 执行过程（即文件头所说的四阶段）：prep → intersections → split → production。
  //   ★ 前三个阶段（尤其"求交点"）通常占总耗时的绝大部分；
  //     若你要用同一批输入做多种运算，请优先考虑 redo()，它能跳过前三个阶段。
  //
  // 副作用：
  //   · 会修改本对象内部的工作边表（把边按交点打断、排序）。
  //     **原始 insert 进来的边顺序不再保留** —— 这是一次性的破坏性变换。
  //   · 会重置 sink 的停止标志、重置 op 的状态。
  //   · 注意 es 与 op 都是**引用**：它们的生命周期由调用者管理，且引擎会保留指针，
  //     所以不要让它们比本次调用更早析构。
  //
  // 坑：本函数**不检查** es/op 是否为空。传入空指针会崩溃。
  // [[ZH-END]]
  void process (db::EdgeSink &es, EdgeEvaluatorBase &op);

  /**
   *  @brief Performs the actual processing
   *
   *  This version allows giving multiple edge sinks and evaluators.
   *  Each evaluator is worked on separately and feeds the corresponding
   *  edge sink.
   */
  // [[ZH-BEGIN]]
  // 功能：★ 一次扫描、多路输出 —— 用同一批边同时喂给多个 (sink, evaluator) 对。
  //
  // 参数：gen 形如 [(sink1, eval1), (sink2, eval2), ...] 的列表。
  //
  // ★ 为什么这个重载很重要（性能原因）：
  //   求交点/打断边（阶段 2、3）是本算法最昂贵的部分，且**只取决于输入边**，
  //   与评估器无关。所以想同时算 AND 和 A-NOT-B 时：
  //       错误做法：调用两次 process()  → 昂贵的阶段 2、3 白跑两遍
  //       正确做法：一次 process(gen)    → 阶段 2、3 只跑一遍
  //   这在单元测试 TEST(9twobool) 中有完整示例（同时求 AND 与 A-NOT-B）。
  //
  // 实现：内部为每个 sink/evaluator 对维护一份独立的扫描线状态，
  //       单路与多路在 EdgeProcessorStates 中被统一处理。
  // [[ZH-END]]
  void process (const std::vector<std::pair<db::EdgeSink *, db::EdgeEvaluatorBase *> > &gen);

  /**
   *  @brief Performs the actual processing again
   *
   *  This method can be called after "process" was used and will re-run the
   *  scanline algorithm. This is somewhat more efficient as the initial
   *  sorting and edge clipping can be skipped.
   */
  // [[ZH-BEGIN]]
  // 功能：重跑处理。与 process() 结果等价，但**跳过阶段 2（求交）和阶段 3（打断）**，
  //       只重跑阶段 4（扫描线投递）。
  //
  // 前提：★ 必须先成功调用过一次 process()（或 redo()）。
  //       否则内部工作边表还是原始未打断的状态，结果不正确。
  //
  // 用途：同一批输入、多种输出时的高效写法：
  //       ep.process (sink1, eval1);   // 第一次：做完整工作（含求交）
  //       ep.redo    (sink2, eval2);   // 后续：复用已打断的边表，快得多
  //
  // 注意：反复 redo 时结果会被重复投递；若 sink 会累积内容，记得自行清空
  //       （EdgeContainer 的构造参数 clear 就是为此设计的）。
  // [[ZH-END]]
  void redo (db::EdgeSink &es, EdgeEvaluatorBase &op);

  /**
   *  @brief Performs the actual processing again
   *
   *  This method can be called after "process" was used and will re-run the
   *  scanline algorithm. This is somewhat more efficient as the initial
   *  sorting and edge clipping can be skipped.
   */
  // [[ZH]] 功能：redo 的多路输出版本，含义与 process(gen) 相同，但跳过阶段 2、3。
  // [[ZH]] 前提：同样必须先调用过一次 process()。适用于"同一批输入 + 多个后续运算"。
  void redo (const std::vector<std::pair<db::EdgeSink *, db::EdgeEvaluatorBase *> > &gen);

  /**
   *  @brief Merge the given polygons in a simple "non-zero wrapcount" fashion
   *
   *  The wrapcount is computed over all polygons, i.e. overlapping polygons may "cancel" if they
   *  have different orientation (since a polygon is oriented by construction that is not easy to achieve).
   *  The other merge operation provided for this purpose is "merge" which normalizes each polygon individually before
   *  merging them. "simple_merge" is somewhat faster and consumes less memory.
   *
   *  The result is presented as a set of edges forming closed contours. Hulls are oriented clockwise while
   *  holes are oriented counter-clockwise.
   *
   *  This is a convenience method that bundles filling of the edges, processing with
   *  a SimpleMerge operator and puts the result into an output vector.
   *
   *  @param in The input polygons
   *  @param out The output edges
   *  @param mode The merge mode (see SimpleMerge constructor)
   */
  // [[ZH-BEGIN]]
  // 功能：★ simple_merge（多边形 → **边**）—— 「插入 + 选评估器 + 选 sink + 处理」一次性打包。
  //       内部等价于：insert 所有输入多边形 → SimpleMerge(mode) → EdgeContainer → process。
  //
  // 参数：in   输入多边形列表；
  //       out  输出边列表（★ 会被清空后写入）；
  //       mode 融合模式，默认 -1 = 非零环绕规则（语义见 ParametrizedInsideFunc 的语义表）。
  //
  // ★ 与 merge() 的区别（上游文档特别说明，值得记住）：
  //   simple_merge : 把所有多边形**合在一起**算一个总环绕数。
  //                  → 反向绕行的多边形可能与其它多边形"相互抵消"。
  //                  → 更快、更省内存。
  //   merge        : 每个多边形**先各自归一化**（只看它自己是否张开）再融合。
  //                  → 结果更符合直觉，不会因绕行方向而抵消；代价是更慢/更费内存。
  //
  // 输出约定：结果是**闭合轮廓**的边集合，且方向有语义 ——
  //           外轮廓(hull) 顺时针，孔洞(hole) 逆时针。
  //           这个约定正是「边的右侧 = 内部」在后处理上的体现，
  //           也是 PolygonGenerator 能正确缝合出孔洞的依据。
  //
  // 坑：输出是**边**。想直接得到多边形，用下面返回 vector<Polygon> 的重载。
  // [[ZH-END]]
  void simple_merge (const std::vector<db::Polygon> &in, std::vector <db::Edge> &out, int mode = -1);

  /**
   *  @brief Merge the given polygons in a simple "non-zero wrapcount" fashion into polygons
   *
   *  The wrapcount is computed over all polygons, i.e. overlapping polygons may "cancel" if they
   *  have different orientation (since a polygon is oriented by construction that is not easy to achieve).
   *  The other merge operation provided for this purpose is "merge" which normalizes each polygon individually before
   *  merging them. "simple_merge" is somewhat faster and consumes less memory.
   *
   *  This method produces polygons and allows fine-tuning the parameters for that purpose.
   *
   *  This is a convenience method that bundles filling of the edges, processing with
   *  a SimpleMerge operator and puts the result into an output vector.
   *
   *  @param in The input polygons
   *  @param out The output polygons
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if touching corners should be resolved into less connected contours
   *  @param mode The merge mode (see SimpleMerge constructor)
   */
  // [[ZH-BEGIN]]
  // 功能：simple_merge 的**多边形输出**版本 —— 结果直接缝合成 db::Polygon。
  //
  // 相比输出边的版本，多了两个只对"多边形缝合"有意义的参数：
  //   resolve_holes  true（默认）= 把孔洞"并入"外轮廓，即把带孔多边形拆成
  //                  **简单（无孔）多边形**。false = 保留孔洞（结果为带孔多边形）。
  //   min_coherence  true（默认）= 把"仅角点相接"的轮廓拆成更少连接的独立轮廓，
  //                  避免生成在单点相连的退化多边形。
  //
  // 参数：in 输入多边形；out 输出多边形；
  //       resolve_holes / min_coherence 见上；mode 见 SimpleMerge 构造。
  //
  // 实现提示：内部用 PolygonGenerator（见 dbPolygonGenerators.h）做缝合。
  // 坑：这两个开关只影响**结果的表达方式**（是否带孔、是否在角点分开），
  //     不影响所表示的几何区域。
  // [[ZH-END]]
  void simple_merge (const std::vector<db::Polygon> &in, std::vector <db::Polygon> &out, bool resolve_holes = true, bool min_coherence = true, int mode = -1);

  /**
   *  @brief Merge the given edges in a simple "non-zero wrapcount" fashion
   *
   *  The edges provided must form valid closed contours. Contours oriented differently "cancel" each other. 
   *  Overlapping contours are merged when the orientation is the same.
   *
   *  The result is presented as a set of edges forming closed contours. Hulls are oriented clockwise while
   *  holes are oriented counter-clockwise.
   *
   *  This is a convenience method that bundles filling of the edges, processing with
   *  a SimpleMerge operator and puts the result into an output vector.
   *
   *  @param in The input edges
   *  @param out The output edges
   *  @param mode The merge mode (see SimpleMerge constructor)
   */
  // [[ZH]] 功能：simple_merge 的**边输入**版本（边 → 边）。
  // [[ZH]] ★ 前提：输入的边必须构成**合法的闭合轮廓**（本函数不做闭合性校验）。
  // [[ZH]] 绕行方向相反的两组轮廓会**相互抵消**；方向相同的重叠轮廓会被合并。
  // [[ZH]] 输出方向约定同上：外轮廓顺时针、孔洞逆时针。
  void simple_merge (const std::vector<db::Edge> &in, std::vector <db::Edge> &out, int mode = -1);

  /**
   *  @brief Merge the given edges in a simple "non-zero wrapcount" fashion into polygons
   *
   *  The edges provided must form valid closed contours. Contours oriented differently "cancel" each other. 
   *  Overlapping contours are merged when the orientation is the same.
   *
   *  This method produces polygons and allows fine-tuning the parameters for that purpose.
   *
   *  This is a convenience method that bundles filling of the edges, processing with
   *  a SimpleMerge operator and puts the result into an output vector.
   *
   *  @param in The input edges
   *  @param out The output polygons
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if touching corners should be resolved into less connected contours
   *  @param mode The merge mode (see SimpleMerge constructor)
   */
  // [[ZH]] 功能：simple_merge 的**边输入 + 多边形输出**版本（闭合轮廓直接缝成多边形）。
  // [[ZH]] 参数含义同上面两个重载（输入为边，可选缝合开关 + mode）。
  // [[ZH]] 前提：输入边必须构成合法闭合轮廓。
  void simple_merge (const std::vector<db::Edge> &in, std::vector <db::Polygon> &out, bool resolve_holes = true, bool min_coherence = true, int mode = -1);

  /**
   *  @brief Merge the given polygons 
   *
   *  In contrast to "simple_merge", this merge implementation considers each polygon individually before merging them.
   *  Thus self-overlaps are effectively removed before the output is computed and holes are correctly merged with the
   *  hull. In addition, this method allows one to select areas with a higher wrap count which allows one to compute overlaps
   *  of polygons on the same layer. Because this method merges the polygons before the overlap is computed, self-overlapping
   *  polygons do not contribute to higher wrap count areas.
   *
   *  The result is presented as a set of edges forming closed contours. Hulls are oriented clockwise while
   *  holes are oriented counter-clockwise.
   *
   *  This is a convenience method that bundles filling of the edges, processing with
   *  a Merge operator and puts the result into an output vector.
   *
   *  @param in The input polygons
   *  @param out The output edges
   *  @param min_wc The minimum wrap count for output (0: all polygons, 1: at least two overlapping)
   */
  // [[ZH-BEGIN]]
  // 功能：★ merge（多边形 → 边）—— 每个多边形**先各自归一化**再融合。
  //
  // ★ 与 simple_merge 的关键差别（上游文档要点）——
  //   本函数先对每个多边形**独立消除自重叠**，再做融合，因此：
  //     · 自相交的多边形不会因自身重叠而撑高环绕数；
  //     · 孔洞能与外轮廓**正确合并**（simple_merge 不保证）；
  //     · 可以借助 min_wc **选出重叠区域**（同一层多个多边形的公共部分）——
  //       这是 simple_merge 做不到的，因为自重叠会污染计数。
  //   代价：更慢、更占内存。
  //
  // 参数：in 输入多边形；out 输出边；
  //       min_wc 最小重叠数阈值 —— 输出"重叠数 **>** min_wc"的区域。
  //              ★ 0（默认）= 输出全部多边形；1 = 只输出至少两个多边形重叠处。
  //              注意判定是严格大于（见 MergeOp），故 min_wc = n 即"至少 n+1 个"。
  //
  // 实现提示：内部使用 MergeOp 评估器，它按 property 区分每个多边形
  //           （给第 i 个多边形分配 property = i）。
  // [[ZH-END]]
  void merge (const std::vector<db::Polygon> &in, std::vector <db::Edge> &out, unsigned int min_wc = 0);

  /**
   *  @brief Merge the given polygons 
   *
   *  In contrast to "simple_merge", this merge implementation considers each polygon individually before merging them.
   *  Thus self-overlaps are effectively removed before the output is computed and holes are correctly merged with the
   *  hull. In addition, this method allows one to select areas with a higher wrap count which allows one to compute overlaps
   *  of polygons on the same layer. Because this method merges the polygons before the overlap is computed, self-overlapping
   *  polygons do not contribute to higher wrap count areas.
   *
   *  This method produces polygons and allows one to fine-tune the parameters for that purpose.
   *
   *  This is a convenience method that bundles filling of the edges, processing with
   *  a Merge operator and puts the result into an output vector.
   *
   *  @param in The input polygons
   *  @param out The output polygons
   *  @param min_wc The minimum wrap count for output (0: all polygons, 1: at least two overlapping)
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if touching corners should be resolved into less connected contours
   */
  // [[ZH-BEGIN]]
  // 功能：★ merge 的**多边形输出**版本（每个多边形先归一化再融合，结果缝合成 Polygon）。
  //
  // 参数：in 输入多边形；out 输出多边形；
  //       min_wc 最小重叠阈值（同上面输出边的版本：输出重叠数 > min_wc 的区域）；
  //       resolve_holes / min_coherence 仅影响多边形缝合方式，含义同 simple_merge：
  //         resolve_holes = true  → 孔洞并入外轮廓，输出**简单（无孔）多边形**；
  //         min_coherence = true  → 把仅角点相接的轮廓拆成更少连接的独立轮廓。
  //
  // 用途提示：想"得到同一层上多个多边形的重叠区域"时，用
  //           merge (in, out, 1) —— 即 min_wc = 1（至少两个多边形重叠处）。
  // [[ZH-END]]
  void merge (const std::vector<db::Polygon> &in, std::vector <db::Polygon> &out, unsigned int min_wc = 0, bool resolve_holes = true, bool min_coherence = true);

  /**
   *  @brief Size the given polygons 
   *
   *  This method sizes a set of polygons. Before the sizing is applied, the polygons are merged. After that, sizing is applied 
   *  on the individual result polygons of the merge step. The result may contain overlapping contours, but no self-overlaps. 
   *
   *  dx and dy describe the sizing. A positive value indicates oversize (outwards) while a negative one describes undersize (inwards).
   *  The sizing applied can be chosen differently in x and y direction. In this case, the sign must be identical for both
   *  dx and dy.
   *
   *  The result is presented as a set of edges forming closed contours. Hulls are oriented clockwise while
   *  holes are oriented counter-clockwise.
   *
   *  This is a convenience method that bundles filling of the edges and processing them 
   *  and which puts the result into an output vector.
   *
   *  @param in The input polygons
   *  @param dx The sizing value in x direction
   *  @param dy The sizing value in y direction
   *  @param out The output edges
   *  @param mode The sizing mode (see db::Polygon for a description)
   */
  // [[ZH-BEGIN]]
  // 功能：★ size（尺寸放大/缩小，多边形 → 边）—— 把每个多边形向外扩张或向内收缩。
  //
  // 参数：in 输入多边形；dx / dy 各方向的缩放量（★ **各向异性**，可不同）；
  //       out 输出边；
  //       mode 尺寸处理模式（默认 2）—— 这是 db::Polygon::sized 的模式，
  //            **不是** SimpleMerge 的 mode，别搞混。它不在这里解释，
  //            而是直接透传给 db::SizingPolygonFilter（见 dbPolygonGenerators.h）。
  //
  // 语义：dx, dy 为正 → 向外扩张（膨胀）；为负 → 向内收缩（腐蚀）。
  //       ★ 收缩过多时图形可能**完全消失**（结果为空的边集），这是正常行为，不是错误。
  //
  // 实现提示：内部先给每个多边形分配 property = 下标*2（偶数 = 操作数 A），
  //           再以 BooleanOp::Or 的形式跑一次，从而把重叠的膨胀结果合并起来。
  //           —— 这就是为什么"多个多边形同时膨胀后重叠处不会重复"。
  // [[ZH-END]]
  void size (const std::vector<db::Polygon> &in, db::Coord dx, db::Coord dy, std::vector <db::Edge> &out, unsigned int mode = 2);

  /**
   *  @brief Size the given polygons into polygons
   *
   *  This method sizes a set of polygons. Before the sizing is applied, the polygons are merged. After that, sizing is applied 
   *  on the individual result polygons of the merge step. The result may contain overlapping polygons, but no self-overlapping ones. 
   *  Polygon overlap occurs if the polygons are close enough, so a positive sizing makes polygons overlap.
   *  
   *  dx and dy describe the sizing. A positive value indicates oversize (outwards) while a negative one describes undersize (inwards).
   *  The sizing applied can be chosen differently in x and y direction. In this case, the sign must be identical for both
   *  dx and dy.
   *
   *  This method produces polygons and allows one to fine-tune the parameters for that purpose.
   *
   *  This is a convenience method that bundles filling of the edges, processing with
   *  a SimpleMerge operator and puts the result into an output vector.
   *
   *  @param in The input polygons
   *  @param dx The sizing value in x direction
   *  @param dy The sizing value in y direction
   *  @param out The output polygons
   *  @param mode The sizing mode (see db::Polygon for a description)
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if touching corners should be resolved into less connected contours
   */
  // [[ZH]] 功能：size 的**多边形输出**版本 —— 膨胀/收缩后结果直接缝合成 Polygon。
  // [[ZH]] 参数：dx/dy 各向缩放量（可不同）；mode 为 db::Polygon::sized 的模式（默认 2）；
  // [[ZH]]       resolve_holes / min_coherence 含义同 simple_merge，仅影响缝合方式。
  // [[ZH]] 提示：收缩量过大时输出可能为空 —— 属正常现象（图形被完全蚀掉）。
  void size (const std::vector<db::Polygon> &in, db::Coord dx, db::Coord dy, std::vector <db::Polygon> &out, unsigned int mode = 2, bool resolve_holes = true, bool min_coherence = true);

  /**
   *  @brief Size the given polygons (isotropic)
   *
   *  This method is equivalent to calling the anisotropic version with identical dx and dy.
   *
   *  @param in The input polygons
   *  @param d The sizing value in x direction
   *  @param out The output edges
   *  @param mode The sizing mode (see db::Polygon for a description)
   */
  // [[ZH]] 功能：size 的**各向同性**版本（dx == dy == d）—— 对上面的各向异性版本的内联转发。
  void size (const std::vector<db::Polygon> &in, db::Coord d, std::vector <db::Edge> &out, unsigned int mode = 2)
  {
    size (in, d, d, out, mode);
  }

  /**
   *  @brief Size the given polygons into polygons (isotropic)
   *
   *  This method is equivalent to calling the anisotropic version with identical dx and dy.
   *
   *  @param in The input polygons
   *  @param d The sizing value in x direction
   *  @param out The output polygons
   *  @param mode The sizing mode (see db::Polygon for a description)
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if touching corners should be resolved into less connected contours
   */
  void size (const std::vector<db::Polygon> &in, db::Coord d, std::vector <db::Polygon> &out, unsigned int mode = 2, bool resolve_holes = true, bool min_coherence = true)
  {
    size (in, d, d, out, mode, resolve_holes, min_coherence);
  }

  /**
   *  @brief Boolean operation for a set of given polygons, creating edges
   *
   *  This method computes the result for the given boolean operation on two sets of polygons.
   *  The result is presented as a set of edges forming closed contours. Hulls are oriented clockwise while
   *  holes are oriented counter-clockwise.
   *
   *  This is a convenience method that bundles filling of the edges, processing with
   *  a Boolean operator and puts the result into an output vector.
   *
   *  @param a The input polygons (first operand)
   *  @param b The input polygons (second operand)
   *  @param out The output edges
   *  @param mode The boolean mode
   */
  // [[ZH-BEGIN]]
  // 功能：★ boolean（两个多边形集合的布尔运算，→ 边）。
  //       真正的"两个不同输入做 与/或/异或/差"的入口（size 只是它的特例）。
  //
  // 参数：a、b 两个输入多边形集合；out 输出边；
  //       mode 布尔运算类型 —— 取 BooleanOp::BoolOp 的值：
  //             1 = And（交集）   2 = ANotB（A 减 B）   3 = BNotA（B 减 A）
  //             4 = Xor（对称差）  5 = Or（并集）
  //             ★ 注意枚举从 1 开始；脚本里名为 ModeAnd/ModeOr/ModeXor/ModeANotB/ModeBNotA。
  //
  // ★ property 自动分配（调用者不用管）：本函数会
  //       把 a 的多边形分配为**偶数** property（0,2,4,...）→ 操作数 A
  //       把 b 的多边形分配为**奇数** property（1,3,5,...）→ 操作数 B
  //   因为 BooleanOp 靠 `p % 2` 区分 A/B。
  //   若你**手工** insert 边并自己填 property，必须遵守同样的奇偶约定。
  //
  // 输出约定同 simple_merge：闭合轮廓，外轮廓顺时针、孔洞逆时针。
  // [[ZH-END]]
  void boolean (const std::vector<db::Polygon> &a, const std::vector<db::Polygon> &b, std::vector <db::Edge> &out, int mode);

  /**
   *  @brief Boolean operation for a set of given polygons, creating polygons
   *
   *  This method computes the result for the given boolean operation on two sets of polygons.
   *  This method produces polygons on output and allows one to fine-tune the parameters for that purpose.
   *
   *  This is a convenience method that bundles filling of the edges, processing with
   *  a Boolean operator and puts the result into an output vector.
   *
   *  @param a The input polygons (first operand)
   *  @param b The input polygons (second operand)
   *  @param out The output polygons
   *  @param mode The boolean mode
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if touching corners should be resolved into less connected contours
   */
  // [[ZH]] 功能：boolean 的**多边形输出**版本（a、b 布尔运算后直接缝合成 Polygon）。
  // [[ZH]] 参数：mode 取 BooleanOp::BoolOp 值（1=And, 2=ANotB, 3=BNotA, 4=Xor, 5=Or）；
  // [[ZH]]       resolve_holes / min_coherence 含义同 simple_merge，仅影响缝合方式。
  // [[ZH]] property 的奇偶约定与上面输出边的版本完全相同（a 偶数 / b 奇数），自动分配。
  void boolean (const std::vector<db::Polygon> &a, const std::vector<db::Polygon> &b, std::vector <db::Polygon> &out, int mode, bool resolve_holes = true, bool min_coherence = true);

  /**
   *  @brief Boolean operation for a set of given edges, creating edges
   *
   *  This method computes the result for the given boolean operation on two sets of edges.
   *  The input edges must form closed contours where holes and hulls must be oriented differently. 
   *  The input edges are processed with a simple non-zero wrap count rule as a whole.
   *
   *  The result is presented as a set of edges forming closed contours. Hulls are oriented clockwise while
   *  holes are oriented counter-clockwise.
   *
   *  This is a convenience method that bundles filling of the edges, processing with
   *  a Boolean operator and puts the result into an output vector.
   *
   *  @param a The input edges (first operand)
   *  @param b The input edges (second operand)
   *  @param out The output edges
   *  @param mode The boolean mode
   */
  // [[ZH-BEGIN]]
  // 功能：boolean 的**边输入**版本（两组闭合轮廓做布尔运算 → 边）。
  //
  // 参数：a、b 两组边（★ 各自必须构成合法闭合轮廓）；out 输出边；
  //       mode 同 BooleanOp::BoolOp（1=And, 2=ANotB, 3=BNotA, 4=Xor, 5=Or）。
  //
  // ★ 本版本与多边形版本的 property 分配不同，需要留意：
  //       边的版本只把 a 整体分配为 property **0**、b 整体分配为 property **1**。
  //       即它只区分"属于 A 还是 B"，**不再按每个轮廓细分**。
  //       （多边形版本则是 a→0,2,4... / b→1,3,5...）
  //   这个差异意味着：若一组内部的多个轮廓发生自重叠，行为会与多边形版本不同。
  //
  // 前提：输入边必须闭合 —— 本函数不校验闭合性，非闭合输入会得到无意义结果。
  // [[ZH-END]]
  void boolean (const std::vector<db::Edge> &a, const std::vector<db::Edge> &b, std::vector <db::Edge> &out, int mode);

  /**
   *  @brief Boolean operation for a set of given edges, creating polygons
   *
   *  This method computes the result for the given boolean operation on two sets of edges.
   *  The input edges must form closed contours where holes and hulls must be oriented differently. 
   *  The input edges are processed with a simple non-zero wrap count rule as a whole.
   *
   *  This method produces polygons on output and allows one to fine-tune the parameters for that purpose.
   *
   *  This is a convenience method that bundles filling of the edges, processing with
   *  a Boolean operator and puts the result into an output vector.
   *
   *  @param a The input polygons (first operand)
   *  @param b The input polygons (second operand)
   *  @param out The output polygons
   *  @param mode The boolean mode
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if touching corners should be resolved into less connected contours
   */
  // [[ZH]] 功能：boolean 的**边输入 + 多边形输出**版本（两组闭合轮廓运算后缝成多边形）。
  // [[ZH]] 参数：a、b 为两组边（须闭合）；mode 同 BooleanOp::BoolOp；
  // [[ZH]]       resolve_holes / min_coherence 仅影响缝合方式。
  // [[ZH]] property 约定与上面边输入的版本相同：a → 0，b → 1（仅区分 A/B）。
  void boolean (const std::vector<db::Edge> &a, const std::vector<db::Edge> &b, std::vector <db::Polygon> &out, int mode, bool resolve_holes = true, bool min_coherence = true);

private:
  std::vector <WorkEdge> *mp_work_edges;
  std::vector <CutPoints> *mp_cpvector;
  bool m_report_progress;
  std::string m_progress_desc;
  int m_base_verbosity;

  static size_t count_edges (const db::Polygon &q) 
  {
    size_t n = q.hull ().size ();
    for (unsigned int h = 0; h < q.holes (); ++h) {
      n += q.hole (h).size ();
    }
    return n;
  }

  static size_t count_edges (const std::vector<db::Polygon> &v) 
  {
    size_t n = 0;
    for (std::vector<db::Polygon>::const_iterator p = v.begin (); p != v.end (); ++p) {
      n += count_edges (*p);
    }
    return n;
  }

  void redo_or_process (const std::vector<std::pair<db::EdgeSink *, db::EdgeEvaluatorBase *> > &gen, bool redo);
};

/**
 *  @brief An edge sink feeding into an EdgeProcessor
 */
// [[ZH-BEGIN]]
// 功能：适配器 —— 一个把收到的边**重新插入**到另一个 EdgeProcessor 的 EdgeSink。
//
// 【用途】把某次运算的输出接成另一次运算的输入。例如需要"先合并、再对合并结果
//   做布尔运算"时，可以不必先把边收集到 vector 再插入，而是直接串联两个处理器。
//
// 【原理】实现 EdgeSink 的投递接口，但 put()/crossing_edge() 的动作不是保存边，
//   而是调用目标处理器的 insert()。因此它是"流水线的一环"而非"终点"。
//
// 参数：构造函数接收目标 EdgeProcessor 的引用（须比本对象活得久）。
//
// 注意：转发时**不携带 tag**（put(edge,tag) 的 tag 只影响本对象的过滤/分流，
//       重新插入时会丢失）。若依赖 tag 传递语义，需另作处理。
// [[ZH-END]]
class DB_PUBLIC EdgesToEdgeProcessor
  : public EdgeSink
{
public:
  EdgesToEdgeProcessor (db::EdgeProcessor &ep, db::EdgeProcessor::property_type prop)
    : mp_ep (&ep), m_prop (prop)
  {
    //  .. nothing yet ..
  }

  virtual void put (const db::Edge &edge)
  {
    mp_ep->insert (edge, m_prop);
  }

  virtual void put (const db::Edge &edge, int /*tag*/)
  {
    mp_ep->insert (edge, m_prop);
  }

private:
  db::EdgeProcessor *mp_ep;
  db::EdgeProcessor::property_type m_prop;
};

}

#endif


