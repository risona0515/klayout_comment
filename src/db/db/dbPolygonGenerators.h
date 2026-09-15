
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



#ifndef HDR_dbPolygonGenerators
#define HDR_dbPolygonGenerators

#include "dbCommon.h"

#include "dbTypes.h"
#include "dbEdge.h"
#include "dbEdgeProcessor.h"
#include "dbPolygon.h"
#include "dbObjectWithProperties.h"

#include <vector>

namespace db
{

// [[ZH-BEGIN]]
// ============================================================================
//  dbPolygonGenerators.h —— 把 EdgeProcessor 输出的「边」缝合成「多边形」
// ============================================================================
//
// 【为什么需要这个文件 —— 补齐 EdgeProcessor 说明中最重要的一个缺口】
//   db::EdgeProcessor **只输出边**，不认识多边形（它把多边形打散成一圈边来处理）。
//   但使用者的直觉需求往往直接就是多边形，因此必须有一步"反向操作"：
//   把一堆零散的边重新**缝合**成闭合的多边形轮廓。
//   本文件就是这个环节：它们是实现了 EdgeSink 接口的接收端，
//   一邍收到边就当场把它们拼装成多边形。
//
// 【在流水线上的位置】
//       输入多边形
//          ↓ 拆成边
//       db::EdgeProcessor（布尔运算，输出边）
//          ↓ 投递边
//       PolygonGenerator / TrapezoidGenerator   ★ 本文件
//          ↓ 调用
//       PolygonSink / SimplePolygonSink（多边形的最终接收端）
//
//   直观理解：EdgeProcessor 是"剪断并重组"，本文件是"把碎线接回成闭合图形"。
//
// 【本文件的几个类】
//   PGPoint / PGPolyContour / PGContourList  缝制过程的内部数据结构
//   PolygonSink / SimplePolygonSink           多边形输出端抽象接口
//   PolygonGenerator                          ★最常用：把边缝成带孔多边形
//   TrapezoidGenerator                        把图形分解为水平梯形（内存友好）
//   PolygonContainer / SimplePolygonContainer 现成的输出端（写进 vector）
//   SizingPolygonFilter                       桥接层：把"尺寸过滤器"当作输出端
//
// 【为什么缝合不是 trivial 的（新手最容易低估的地方）】
//   难点在**孔洞**。一个带孔的区域，其边集合由"外轮廓"与"内孔洞"两组闭合曲线构成
//   （回想 dbEdge.h 的约定：外轮廓顺时针、孔洞逆时针）。
//   而 db::Polygon 通常要求是**简单多边形**（不自相交、无孔）。
//   因此必须把孔洞"拆开"并接到外轮廓上，这在几何上要引入来回往返的**缝线
//   (stitch line)** —— 这就是 `resolve_holes` 参数在做的事，
//   也是本文件里 `eliminate_hole()`、`join_contours()` 存在的原因。
//   只要看到"带孔多边形""缝线""自相切"这些词，都是指这个问题。
// [[ZH-END]]
class PGPolyContour;
class PGContourList;
struct PGPoint;
class PolygonSink;
class SimplePolygonSink;

/**
 *  @brief Forms polygons from a edge set
 *
 *  This class implements EdgeSink. It builds polygons from the edges delivered to it 
 *  and outputs the polygons to another receiver (PolygonSink). 
 *  The way how touching corners are resolved can be specified (minimum and maximum coherence).
 *  In addition, it can be specified if the resulting polygons contain holes are whether the 
 *  holes are attached to the hull contour by stich lines.
 */
// [[ZH-BEGIN]]
// 功能：★ 把 EdgeProcessor 投递的边**缝合**成多边形（最常用的输出端）。
//
// 用法：`PolygonGenerator gen (polygon_container); ep.process (gen, evaluator);`
//       其中 polygon_container 是 PolygonSink（例如 PolygonContainer）。
//       处理结束后从 container 取出多边形。
//
// 参数（构造）：psink / spsink 多边形输出端（持引用，须活得比本对象久）；
//              resolve_holes 是否把孔洞拆解后接到外轮廓（用缝线）；
//              min_coherence 是否把结果"拆得更散"（详见下面的参数说明）。
//
// 【正常工作流程（这是 EdgeSink 协议的实现，对比基类注释看）】
//   start()          初始化内部轮廓列表
//   begin_scanline(y) 记录当前 y
//   put(e) / crossing_edge(e)  把边加入当前轮廓的拼接
//   end_scanline(y)  在遇到闭合处时尝试**缝合轮廓**并产出多边形
//   flush()          收尾：处理剩余未闭合/需拆解孔洞的轮廓，向 psink 输出
//
// 坑：1) ★ 请不要在 start() 与 flush() 之间修改 resolve_holes / min_coherence /
//          open_contours —— 上游英文文档明确提醒（会造成状态不一致）。
//     2) 本类**不可拷贝**：拷构与赋值被私有化禁用（内部持指针与轮廓链表）。
//     3) 只实现 put(e) 而**忽略 tag**（put(e,tag) 是空函数）——
//        因此它不适用于需要标签区分的场景（那种场景请直接收集边再自行处理）。
// [[ZH-END]]
class DB_PUBLIC PolygonGenerator
  : public EdgeSink
{
public:
  // [[ZH]] open_map_type："开放边映射"的内部容器（按 x 排列的待接合点）。
  // [[ZH]] 用途：扫描过程中记录尚未接到闭合轮廓上的点，以便后续接续。
  typedef std::list <PGPoint> open_map_type;
  typedef open_map_type::iterator open_map_iterator_type;

  /**
   *  @brief Constructor
   *
   *  This constructor takes the polygon receiver (of which is keeps a reference).
   *  It allows one to specify how holes are resolved and how touching corners are resolved.
   *
   *  @param psink The polygon receiver
   *  @param resolve_holes true, if holes should be resolved into the hull using stich lines
   *  @param min_coherence true, if the resulting polygons should be minimized (less holes, more polygons)
   */
  PolygonGenerator (PolygonSink &psink, bool resolve_holes = true, bool min_coherence = true); 

  /**
   *  @brief Constructor
   *
   *  This constructor takes the simple polygon receiver (of which is keeps a reference).
   *  It allows one to specify how touching corners are resolved. Holes are always resolved since this is
   *  the only way to create a simple polygon.
   *
   *  @param spsink The simple polygon receiver
   *  @param resolve_holes true, if holes should be resolved into the hull using stich lines
   *  @param min_coherence true, if the resulting polygons should be minimized (less holes, more polygons)
   */
  PolygonGenerator (SimplePolygonSink &spsink, bool min_coherence = true); 

  /**
   *  @brief Destructor
   */
  ~PolygonGenerator ();

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void start ();

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void flush ();

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void begin_scanline (db::Coord y);

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void end_scanline (db::Coord y);

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void crossing_edge (const db::Edge &e);

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void skip_n (size_t n);

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void put (const db::Edge &e);

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void put (const db::Edge & /*e*/, int /*tag*/) { }

  /**
   *  @brief Sets the way how holes are resolved dynamically
   *
   *  This property should not be changed why polygons are created (between start and flush)
   */
  void resolve_holes (bool f) { m_resolve_holes = f; }

  /**
   *  @brief Enables open contours for hole resolution
   *
   *  With this property set to false (the default), holes are resolved with a single stitch
   *  line. This will create self-touching polygons finally. By setting this property to true,
   *  a different hole resolution strategy is chosen which resolves holes by inserting a new
   *  contour at the left of the hole.
   *
   *  Using is feature will result in a larger number but less complex polygons on output.
   */
  void open_contours (bool f) { m_open_contours = f; }

  /**
   *  @brief Sets the way how touching corners are resolved dynamically
   *
   *  This property should not be changed while polygons are created (between start and flush)
   */
  void min_coherence (bool f) { m_min_coherence = f; }

  /**
   *  @brief Disables or enable compression for polygon contours
   *
   *  If compression is disabled, no vertices will be dropped.
   */
  void enable_compression (bool enable) { m_compress = enable; }

  /**
   *  @brief Disables or enable compression for polygon contours
   *
   *  This method switches the global flag and is intended for regression test purposes only!
   */
  static void enable_compression_global (bool enable) { ms_compress = enable; }

private:
  // [[ZH]] ===== PolygonGenerator 的内部状态 =====
  // [[ZH]] 这些字段共同实现"边 → 轮廓 → 多边形"的拼接，且与扫描线进度绑定。
  // [[ZH]]
  // [[ZH]] mp_contours  ：当前正在拼接的轮廓集合（用 PGPolyContour 表示一条条待闭合的曲线）。
  // [[ZH]] m_open/m_open_pos："开放点"链表与遍历位置 —— 用于在当前扫描线上
  // [[ZH]]                    寻找可以接续的轮廓端点（按 x 顺序查找）。
  // [[ZH]] m_y          ：当前扫描线的 y（边都投递在该高度上）。
  // [[ZH]] mp_psink / mp_spsink：两个输出端（带孔多边形 / 简单多边形），
  // [[ZH]]                    构造时二选一被置位，另一个为空。
  // [[ZH]] m_resolve_holes  ：是否把孔洞拆解并接到外轮廓（对应 resolve_holes(bool)）。
  // [[ZH]] m_open_contours  ：孔洞拆解策略开关（见 open_contours(bool) 的说明）。
  // [[ZH]] m_min_coherence  ：是否把结果拆成更少重叠的独立轮廓。
  // [[ZH]] m_poly / m_spoly ：复用的输出缓冲，避免每个多边形都重新分配内存。
  // [[ZH]] ms_compress      ：**全局静态**压缩开关（仅用于回归测试，见下）。
  // [[ZH]] m_compress       ：本实例的压缩开关（是否丢弃共线冗余顶点）。
  PGContourList *mp_contours;
  open_map_type m_open;
  db::Coord m_y;
  open_map_iterator_type m_open_pos;
  PolygonSink *mp_psink;
  SimplePolygonSink *mp_spsink;
  bool m_resolve_holes;
  bool m_open_contours;
  bool m_min_coherence;
  db::Polygon m_poly;
  db::SimplePolygon m_spoly;
  static bool ms_compress;
  bool m_compress;

  // [[ZH]] join_contours(x)：在扫描线位置 x 处把可以接续的轮廓端点接合起来。
  // [[ZH]] produce_poly(c)：把一条已闭合的轮廓 c 转换为多边形并输出给 sink。
  // [[ZH]] eliminate_hole()：★ 处理孔洞 —— 将孔洞轮廓与外轮廓用缝线接到一起，
  // [[ZH]]                    使结果成为合法（简单）多边形。这是本类最复杂的一步。
  void join_contours (db::Coord x);
  void produce_poly (const PGPolyContour &c);
  void eliminate_hole ();

  // [[ZH]] 拷贝/赋值被**私有且不实现**（C++03 风格的"禁止拷贝"写法）——
  // [[ZH]] 因为本类内部持有指针与轮廓链表，默认浅拷贝会产生双释放/共享状态。
  PolygonGenerator &operator= (const PolygonGenerator &);
  PolygonGenerator (const PolygonGenerator &);
};

/**
 *  @brief Forms trapezoids from an edge set
 *
 *  This class implements EdgeSink. It builds simple polygons from the edges delivered to it
 *  and outputs the polygons to another receiver (PolygonSink or SimplePolygonSink). The
 *  polygons created form a horizontal trapezoid decomposition of the full polygon.
 */
// [[ZH-BEGIN]]
// 功能：★ 把边集合分解为**水平梯形 (trapezoid)**，而非缝成多边形。
//
// 【为什么要梯形分解 —— 它的实际价值】
//   · 结果永远不会出现"孔洞"或"自相切"，因为梯形是最简单的凸形（每个梯形是凸的）；
//   · 不需要缝线、不需要处理孔洞 —— 绕开了 PolygonGenerator 最复杂的部分；
//   · 适合内存敏感或下游只支持简单凸形的场景。
//   代价：一个复杂图形会产生**大量**梯形（数量远多于多边形）。
//
// 【实现思路】
//   利用扫描线天然产生的"水平切片"：在相邻两条扫描线之间，每条活跃边的
//   左/右配对就构成一个水平梯形（上底与下底在两条扫描线上）。
//   因此它只需维护"当前扫描线处每条边的左右配对关系"，
//   不需要像 PolygonGenerator 那样拼接任意方向的轮廓。
//
// 输出端：PolygonSink（产出带孔多边形 —— 但因为是梯形，实际都是简单多边形）
//         或 SimplePolygonSink（产出简单多边形）。
//
// 实例参考：单元测试 TEST(9twobool) 里用 TrapezoidGenerator + MergeOp 直接驱动
//           EdgeProcessor（那是测试集中少数不经 ShapeProcessor 的用法）。
// [[ZH-END]]
class DB_PUBLIC TrapezoidGenerator
  : public EdgeSink
{
public:
  // [[ZH]] edge_map_type：梯形生成器的核心容器 —— 一堆「左边, 右边」边对。
  // [[ZH]] 每对代表当前扫描带内的一个梯形的左右两侧。
  typedef std::vector <std::pair<db::Edge, db::Edge> > edge_map_type;
  typedef edge_map_type::iterator edge_map_type_iterator;

  /**
   *  @brief Constructor
   *
   *  This constructor takes the polygon receiver (of which is keeps a reference).
   *  The trapezoids will be delivered to this sink.
   *
   *  @param psink The polygon receiver
   */
  TrapezoidGenerator (PolygonSink &psink);

  /**
   *  @brief Constructor
   *
   *  This constructor takes the polygon receiver (of which is keeps a reference).
   *  The trapezoids will be delivered to this sink.
   *
   *  @param spsink The simple polygon receiver
   */
  TrapezoidGenerator (SimplePolygonSink &spsink);

  /**
   *  @brief Destructor
   */
  ~TrapezoidGenerator ();

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void start ();

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void flush ();

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void begin_scanline (db::Coord y);

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void end_scanline (db::Coord y);

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void crossing_edge (const db::Edge &e);

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void skip_n (size_t n);

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void put (const db::Edge &e);

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void put (const db::Edge & /*e*/, int /*tag*/) { }

private:
  db::Coord m_y;
  PolygonSink *mp_psink;
  SimplePolygonSink *mp_spsink;
  db::Polygon m_poly;
  db::SimplePolygon m_spoly;
  edge_map_type m_edges, m_new_edges;
  edge_map_type_iterator m_current_edge;
  std::vector<size_t> m_new_edge_refs;

  TrapezoidGenerator &operator= (const TrapezoidGenerator &);
  TrapezoidGenerator (const TrapezoidGenerator &);

  void make_trap (const db::Point (&pts)[4]);
};

/**
 *  @brief Declaration of the simple polygon sink interface
 */
// [[ZH-BEGIN]]
// 功能："简单多边形（无孔）"的输出端抽象接口 —— 多边形缝合流水线的**终点**。
//
// 与 db::EdgeSink 的层次关系（别搞混这两个接口）：
//     EdgeSink        ← EdgeProcessor 的输出端（收的是**边**）
//        ↑ 实现
//     PolygonGenerator / TrapezoidGenerator   ← 把边缝成多边形
//        ↓ 输出
//     PolygonSink / SimplePolygonSink         ★ 本类：收的是**多边形**
//
// 调用协议（由 PolygonGenerator/TrapezoidGenerator 驱动）：
//     start() → put(polygon) × N → flush()
//   · 两个发生器都会把 EdgeSink 的 start/flush 原样转发给本类；
//   · flush() 之后不再投递（发生器在末尾把剩余多边形全部冲出）。
//
// 坑：与 EdgeSink 一样，所有方法默认空实现，只需重写关心的方法。
// [[ZH-END]]
class DB_PUBLIC SimplePolygonSink
{
public:
  /**
   *  @brief Constructor
   */
  SimplePolygonSink () { }

  /**
   *  @brief Destructor
   */
  // [[ZH]] 虚析构：允许通过基类指针删除派生输出端（否则会漏调派生类析构）。
  virtual ~SimplePolygonSink () { }

  /**
   *  @brief Deliver a simple polygon
   *
   *  This method is called to deliver a new polygon
   */
  // [[ZH]] 功能：★ 收取一个简单（无孔）多边形。这是本接口唯一必须关心的回调。
  virtual void put (const db::SimplePolygon & /*polygon*/) { }

  /**
   *  @brief Start event
   *
   *  This method is called before the first polygon is delivered. 
   *  The PolygonGenerator will simply forward the EdgeSink's start method to the
   *  polygon sink.
   */
  virtual void start () { }

  /**
   *  @brief End event
   *
   *  This method is called after the last polygon was delivered. 
   *  The PolygonGenerator will deliver all remaining polygons and call flush then.
   */
  virtual void flush () { }
};

/**
 *  @brief A simple polygon receiver collecting the simple polygons in a vector
 *
 *  This class implements the SimplePolygonSink interface.
 *  Like EdgeContainer, this receiver collects the objects either in an external
 *  or an internal vector of polygons.
 */
// [[ZH-BEGIN]]
// 功能：简单（无孔）多边形的**收集器** —— 把收到的多边形存进 vector。
//       是本研究中最常用的"现成输出端"（与 EdgeContainer 在边侧的定位对应）。
//
// 两种存放方式（与 EdgeContainer 完全同构）：
//   SimplePolygonContainer(vec) → 结果直接写进你提供的 vec
//   SimplePolygonContainer()    → 结果存在内部，用 polygons() 取出
//
// 坑：★ clear 同样是"单次生效"：只在首次 start() 时清空一次（见 start() 的注释）。
//     这样才能与尺寸过滤器等多帧投递的处理器兼容。
//     若要重复使用同一容器做多次运算，请自行在两次之间清空。
// [[ZH-END]]
class DB_PUBLIC SimplePolygonContainer
  : public SimplePolygonSink
{
public:
  /**
   *  @brief Constructor specifying an external vector for storing the polygons
   */
  // [[ZH]] 功能：绑定到**外部**vector，结果直接追加进去。参数 clear 表示首次 start() 时是否先清空。
  SimplePolygonContainer (std::vector<db::SimplePolygon> &polygons, bool clear = false) 
    : SimplePolygonSink (), mp_polygons (&polygons), m_clear (clear) 
  { }

  /**
   *  @brief Constructor which tells the container to use the internal vector for storing the polygons
   */
  // [[ZH]] 功能：不提供外部容器，结果存进对象自带的内部 vector，用 polygons() 读取。
  SimplePolygonContainer () 
    : SimplePolygonSink (), mp_polygons (&m_polygons), m_clear (false) 
  { }

  /**
   *  @brief Start the sequence
   */
  // [[ZH]] 功能：首次 start() 时按需清空（single-shot），之后不再清空。
  virtual void start ()
  {
    if (m_clear) {
      mp_polygons->clear ();
      //  The single-shot scheme is a easy way to overcome problems with multiple start/flush brackets (i.e. on size filter)
      // [[ZH]] 用完立即关闭标志 —— 这就是 single-shot，防止多帧投递时抹掉前面的结果。
      m_clear = false;
    }
  }

  /**
   *  @brief The polygons collected so far (const version)
   */
  const std::vector<db::SimplePolygon> &polygons () const
  { 
    return *mp_polygons; 
  }

  /**
   *  @brief The polygons collected so far (non-const version)
   */
  std::vector<db::SimplePolygon> &polygons () 
  { 
    return *mp_polygons; 
  }

  /**
   *  @brief Implementation of the PolygonSink interface
   */
  virtual void put (const db::SimplePolygon &polygon) 
  {
    mp_polygons->push_back (polygon);
  }

private:
  std::vector<db::SimplePolygon> m_polygons;
  std::vector<db::SimplePolygon> *mp_polygons;
  bool m_clear;
};

/**
 *  @brief A simple polygon receiver collecting the simple polygons in a vector with common properties
 *
 *  This class implements the SimplePolygonSink interface.
 *  Like EdgeContainer, this receiver collects the objects either in an external
 *  or an internal vector of polygons.
 */
class DB_PUBLIC SimplePolygonContainerWithProperties
  : public SimplePolygonSink
{
public:
  /**
   *  @brief Constructor specifying an external vector for storing the polygons
   */
  SimplePolygonContainerWithProperties (std::vector<db::SimplePolygonWithProperties> &polygons, db::properties_id_type prop_id, bool clear = false)
    : SimplePolygonSink (), mp_polygons (&polygons), m_prop_id (prop_id), m_clear (clear)
  { }

  /**
   *  @brief Constructor which tells the container to use the internal vector for storing the polygons
   */
  SimplePolygonContainerWithProperties ()
    : SimplePolygonSink (), mp_polygons (&m_polygons), m_prop_id (0), m_clear (false)
  { }

  /**
   *  @brief Start the sequence
   */
  virtual void start ()
  {
    if (m_clear) {
      mp_polygons->clear ();
      //  The single-shot scheme is a easy way to overcome problems with multiple start/flush brackets (i.e. on size filter)
      m_clear = false;
    }
  }

  /**
   *  @brief The polygons collected so far (const version)
   */
  const std::vector<db::SimplePolygonWithProperties> &polygons () const
  {
    return *mp_polygons;
  }

  /**
   *  @brief The polygons collected so far (non-const version)
   */
  std::vector<db::SimplePolygonWithProperties> &polygons ()
  {
    return *mp_polygons;
  }

  /**
   *  @brief Implementation of the PolygonSink interface
   */
  virtual void put (const db::SimplePolygon &polygon)
  {
    mp_polygons->push_back (db::SimplePolygonWithProperties (polygon, m_prop_id));
  }

private:
  std::vector<db::SimplePolygonWithProperties> m_polygons;
  std::vector<db::SimplePolygonWithProperties> *mp_polygons;
  db::properties_id_type m_prop_id;
  bool m_clear;
};

/**
 *  @brief Declaration of the polygon sink interface
 */
// [[ZH]] 功能："（可带孔的）多边形"输出端抽象接口 —— 与 SimplePolygonSink 平行，
// [[ZH]]       区别是 put() 收的是 db::Polygon（可含孔洞）而非 SimplePolygon。
// [[ZH]] 选用原则：需要保留孔洞信息 → 用本接口；只需要简单多边形 → 用 SimplePolygonSink。
class DB_PUBLIC PolygonSink
{
public:
  /**
   *  @brief Constructor
   */
  PolygonSink () { }

  /**
   *  @brief Destructor
   */
  // [[ZH]] 虚析构：允许通过基类指针删除派生输出端。
  virtual ~PolygonSink () { }

  /**
   *  @brief Deliver a polygons
   *
   *  This method is called to deliver a new polygon
   */
  // [[ZH]] 功能：★ 收取一个多边形（可能带孔）。这是本接口唯一必须关心的回调。
  virtual void put (const db::Polygon &) { }

  /**
   *  @brief Start event
   *
   *  This method is called before the first polygon is delivered. 
   *  The PolygonGenerator will simply forward the EdgeSink's start method to the
   *  polygon sink.
   */
  virtual void start () { }

  /**
   *  @brief End event
   *
   *  This method is called after the last polygon was delivered. 
   *  The PolygonGenerator will deliver all remaining polygons and call flush then.
   */
  virtual void flush () { }
};

/**
 *  @brief A polygon receiver collecting the polygons in a vector
 *
 *  This class implements the PolygonSink interface.
 *  Like EdgeContainer, this receiver collects the objects either in an external
 *  or an internal vector of polygons.
 */
class DB_PUBLIC PolygonContainer
  : public PolygonSink
{
public:
  /**
   *  @brief Constructor specifying an external vector for storing the polygons
   */
  PolygonContainer (std::vector<db::Polygon> &polygons, bool clear = false) 
    : PolygonSink (), mp_polygons (&polygons), m_clear (clear)
  { }

  /**
   *  @brief Constructor which tells the container to use the internal vector for storing the polygons
   */
  PolygonContainer () 
    : PolygonSink (), mp_polygons (&m_polygons), m_clear (false) 
  { }

  /**
   *  @brief The polygons collected so far (const version)
   */
  const std::vector<db::Polygon> &polygons () const
  { 
    return *mp_polygons; 
  }

  /**
   *  @brief The polygons collected so far (non-const version)
   */
  std::vector<db::Polygon> &polygons () 
  { 
    return *mp_polygons; 
  }

  /**
   *  @brief Start the sequence
   */
  virtual void start ()
  {
    if (m_clear) {
      mp_polygons->clear ();
      //  The single-shot scheme is a easy way to overcome problems with multiple start/flush brackets (i.e. on size filter)
      m_clear = false;
    }
  }

  /**
   *  @brief Implementation of the PolygonSink interface
   */
  virtual void put (const db::Polygon &polygon) 
  {
    mp_polygons->push_back (polygon);
  }

private:
  std::vector<db::Polygon> m_polygons;
  std::vector<db::Polygon> *mp_polygons;
  bool m_clear;
};

/**
 *  @brief A polygon receiver collecting the polygons in a vector with a common properties ID
 *
 *  This class implements the PolygonSink interface.
 *  Like EdgeContainer, this receiver collects the objects either in an external
 *  or an internal vector of polygons.
 */
class DB_PUBLIC PolygonContainerWithProperties
  : public PolygonSink
{
public:
  /**
   *  @brief Constructor specifying an external vector for storing the polygons
   */
  PolygonContainerWithProperties (std::vector<db::PolygonWithProperties> &polygons, db::properties_id_type prop_id, bool clear = false)
    : PolygonSink (), mp_polygons (&polygons), m_prop_id (prop_id), m_clear (clear)
  { }

  /**
   *  @brief Constructor which tells the container to use the internal vector for storing the polygons
   */
  PolygonContainerWithProperties ()
    : PolygonSink (), mp_polygons (&m_polygons), m_prop_id (0), m_clear (false)
  { }

  /**
   *  @brief The polygons collected so far (const version)
   */
  const std::vector<db::PolygonWithProperties> &polygons () const
  {
    return *mp_polygons;
  }

  /**
   *  @brief The polygons collected so far (non-const version)
   */
  std::vector<db::PolygonWithProperties> &polygons ()
  {
    return *mp_polygons;
  }

  /**
   *  @brief Start the sequence
   */
  virtual void start ()
  {
    if (m_clear) {
      mp_polygons->clear ();
      //  The single-shot scheme is a easy way to overcome problems with multiple start/flush brackets (i.e. on size filter)
      m_clear = false;
    }
  }

  /**
   *  @brief Implementation of the PolygonSink interface
   */
  virtual void put (const db::Polygon &polygon)
  {
    mp_polygons->push_back (db::PolygonWithProperties (polygon, m_prop_id));
  }

private:
  std::vector<db::PolygonWithProperties> m_polygons;
  std::vector<db::PolygonWithProperties> *mp_polygons;
  db::properties_id_type m_prop_id;
  bool m_clear;
};

/**
 *  @brief A polygon filter that sizes the polygons 
 *
 *  This class implements the PolygonSink interface and delivers the sized polygons to an EdgeSink.
 */
// [[ZH-BEGIN]]
// 功能：桥接层 —— 把"尺寸缩放 (sizing)"接成流水线中的一环：
//       它以 PolygonSink 身份接收多边形，缩放后以 EdgeSink 形式发给下游。
//
// 【为什么需要它 —— 它解决了什么真实问题】
//   多边形缩放最不易做的是"多个图形膨胀后重叠处如何合并"。
//   本类可以把这一步步推给 EdgeProcessor：
//       收多边形 → insert 到内部 m_sizing_processor → 以指定 dx/dy/mode 跑 size → 发边
//   即它内部**持有一个 EdgeProcessor**（见私有成员 m_sizing_processor）
//   来做真正的几何运算。这也解释了为何本文件 include 了 dbEdgeProcessor.h。
//
// 产生多帧投递（start/flush 会被调用多次）——
//   这正是下游容器需要"single-shot 清空"机制的原因
//   （见 SimplePolygonContainer/PolygonContainer 的 start() 注释）。
//
// 参数（构造）：output 下游边接收端；dx/dy 各方向缩放量（可不同）；
//               mode 为 db::Polygon::sized 的模式，会透传给内部处理器。
// [[ZH-END]]
class DB_PUBLIC SizingPolygonFilter
  : public PolygonSink
{
public:
  /**
   *  @brief Constructor 
   */
  // [[ZH]] 功能：构造。绑定下游边接收端并记录缩放参数（真正运算在 put() 时进行）。
  SizingPolygonFilter (EdgeSink &output, Coord dx, Coord dy, unsigned int mode)
    : PolygonSink (), mp_output (&output), m_dx (dx), m_dy (dy), m_mode (mode)
  { }

  /**
   *  @brief Implementation of the PolygonSink interface
   */
  virtual void put (const db::Polygon &polygon);

private:
  EdgeProcessor m_sizing_processor;
  EdgeSink *mp_output;
  Coord m_dx, m_dy;
  unsigned int m_mode;
};

/**
 *  @brief A polygon sink feeding into an EdgeProcessor
 */
// [[ZH-BEGIN]]
// 功能：反向桥接 —— 把多边形**重新插入**到 EdgeProcessor（与 EdgesToEdgeProcessor 类似的思路）。
//
// 【用途】串联"非几何运算的前置步"与"边运算"。典型场景：
//   自己实现一个 PolygonSink 做几何预处理，然后直接把结果推回处理器，
//   而不必先收集到 vector 再手动循环 insert。
//
// 【★ 它解决了 property 分配这个琐事（这是它最实用的价值）】
//   回顾：boolean() 要求 A 用偶数 property、B 用奇数 property。
//   本类可以在每次 put() 后自动把 property **递增 prop_step**：
//         insert(polygon, m_prop);  m_prop += m_prop_step;
//   于是构造时给 (0, 2) 就能得到 0,2,4,...（操作数 A）；
//   给 (1, 2) 则得到 1,3,5,...（操作数 B）。
//   这样就不用自己维护计数器。
//
// 参数（构造）：ep 目标处理器（引用）；prop 起始 property；prop_step 每次自增量。
//
// 坑：prop 的最终值会随 put() 次数递增，若本对象被复用（再次投递一批新多边形），
//     property 不会自动重置 —— 需要重新构造或自行控制起始偏移。
// [[ZH-END]]
class DB_PUBLIC PolygonsToEdgeProcessor
  : public PolygonSink
{
public:
  // [[ZH]] 功能：构造。记录目标处理器、起始 property 与步长（步长语义见上方说明）。
  PolygonsToEdgeProcessor (db::EdgeProcessor &ep, db::EdgeProcessor::property_type prop, db::EdgeProcessor::property_type prop_step)
    : mp_ep (&ep), m_prop (prop), m_prop_step (prop_step)
  {
    //  .. nothing yet ..
  }

  // [[ZH]] 功能：插入多边形并把 property 自增 prop_step（★ 修改 m_prop）。
  virtual void put (const db::Polygon &polygon)
  {
    mp_ep->insert (polygon, m_prop);
    m_prop += m_prop_step;
  }

private:
  db::EdgeProcessor *mp_ep;
  db::EdgeProcessor::property_type m_prop, m_prop_step;
};

}

#endif


