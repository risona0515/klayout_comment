
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



#ifndef HDR_dbShapeProcessor
#define HDR_dbShapeProcessor

#include "dbCommon.h"

#include "dbTypes.h"
#include "dbEdge.h"
#include "dbPolygonGenerators.h"
#include "dbShapes.h"

#include <vector>

namespace db
{

class Layout;
class Cell;

/**
 *  @brief A polygon receiver creating shapes from the polygons inside a db::Shapes container
 *
 *  This class implements the PolygonSink interface.
 */
class DB_PUBLIC ShapeGenerator
  : public PolygonSink
{
public:
  /**
   *  @brief Constructor associating the receiver with a shapes container
   *
   *  See the description of the EdgeSink and PolygonSink interface for a explanation when
   *  the start method is called and when the shape container is cleared if "clear_shapes"
   *  is set.
   *
   *  @param shapes Where to store the shapes
   *  @param clear_shapes If true, the shapes container is cleared on the start event.
   *  @param prop_id The properties ID to assign to all the output shapes (or 0 if no property shall be assigned)
   */
  // [[ZH]] 功能：把结果多边形直接写入一个 db::Shapes 容器（版图图层上的图形集合）。
  // [[ZH]] 参数：shapes 目标容器（引用，须比本对象活得久）；
  // [[ZH]]       clear_shapes 首次 start() 时是否清空 shapes（single-shot，见 start()）；
  // [[ZH]]       prop_id 给写入的每个图形统一附加的属性 ID（0 = 不附加属性）。
  // [[ZH]] 理解要点：ShapeGenerator 是 PolygonSink，所以它必须接在
  // [[ZH]]           PolygonGenerator / TrapezoidGenerator **之后**（先缝成多边形再写入）。
  ShapeGenerator (db::Shapes &shapes, bool clear_shapes = false, db::properties_id_type prop_id = 0)
    : PolygonSink (), mp_shapes (&shapes), m_clear_shapes (clear_shapes), m_prop_id (prop_id)
  { }

  /**
   *  @brief Sets the properties ID to be used for the next polygon
   */
  // [[ZH]] 功能：设定接下来写入图形的属性 ID（★ 修改 m_prop_id）。
  // [[ZH]] 用途：在投递过程中动态切换属性，从而给不同批次的图形打不同标签。
  void set_prop_id (db::properties_id_type prop_id)
  {
    m_prop_id = prop_id;
  }

  /**
   *  @brief Implementation of the PolygonSink interface
   */
  // [[ZH]] 功能：★ 把收到的多边形插入目标 Shapes 容器。
  // [[ZH]] 修改：mp_shapes 指向的容器（追加一个图形）。
  // [[ZH]] 分支：prop_id 非 0 时用 PolygonWithProperties 携带属性，否则插入裸多边形。
  virtual void put (const db::Polygon &polygon) 
  {
    if (m_prop_id) {
      mp_shapes->insert (db::PolygonWithProperties (polygon, m_prop_id));
    } else {
      mp_shapes->insert (polygon);
    }
  }

  /**
   *  @brief Implementation of the PolygonSink interface
   */
  // [[ZH]] 功能：首次 start() 时按需清空目标容器（single-shot，用完即关标志）。
  // [[ZH]] 为何不能每次清空：与尺寸过滤器等多帧投递的处理器兼容，见行内英文注释。
  virtual void start () 
  { 
    if (m_clear_shapes) {
      mp_shapes->clear ();
      //  The single-shot scheme is a easy way to overcome problems with multiple start/flush brackets (i.e. on size filter)
      m_clear_shapes = false;
    }
  }

private:
  // [[ZH]] mp_shapes     ：目标图形容器（外部传入，不拥有）。
  // [[ZH]] m_clear_shapes：是否在首次 start() 清空（single-shot 标志）。
  // [[ZH]] m_prop_id     ：给写入图形附带的属性 ID；0 表示不附加。
  db::Shapes *mp_shapes;
  bool m_clear_shapes;
  db::properties_id_type m_prop_id;
};

/**
 *  @brief An edge receiver creating shapes from the edges inside a db::Shapes container
 *
 *  This class implements the EdgeSink interface.
 */
// [[ZH-BEGIN]]
// 功能：★ ShapeProcessor 的**输出端** —— 把结果**边**直接写回 db::Shapes 容器。
//
// 【为什么需要它 —— 补齐整个闭环】
//   ShapeProcessor 把 Shape 拆成边交给 EdgeProcessor；
//   但布尔运算的结果也是**边**，使用者却期望得到**图形**。
//   本类就是这么座桥：它实现 EdgeSink（收边），每次 put() 就把边插进 Shapes。
//
//   ★ 注意它直接插入的是 db::Edge（即结果是"边图形"而不是多边形）：
//     若希望结果是多边形，需要在中间接 PolygonGenerator 缝合成多边形，
//     再用 ShapeGenerator（PolygonSink）写入。两条路线的区别：
//        EdgeProcessor → EdgeShapeGenerator            → Shapes 里是**边**
//        EdgeProcessor → PolygonGenerator → ShapeGenerator → Shapes 里是**多边形**
//
// 【与本文件其它类的配合】
//   它与 ShapeGenerator 名字很像，区别是接收层次不同：
//     EdgeShapeGenerator : EdgeSink（收**边**，直接存边）
//     ShapeGenerator     : PolygonSink（收**多边形**，存多边形）
//
// 参数（构造）：shapes 目标容器；clear_shapes 首次 start() 是否清空；
//               tag 标签过滤（0 = 不限）；chained 链式下游（可选）。
//
// 坑：put(edge,tag) 的链式转发**不受标签过滤影响** —— 与 EdgeContainer 行为一致。
// [[ZH-END]]
class DB_PUBLIC EdgeShapeGenerator
  : public EdgeSink
{
public:
  /**
   *  @brief Constructor associating the receiver with a shapes container
   *
   *  See the description of the EdgeSink and PolygonSink interface for a explanation when
   *  the start method is called and when the shape container is cleared if "clear_shapes"
   *  is set.
   *
   *  @param clear_shapes If true, the shapes container is cleared on the start event.
   */
  EdgeShapeGenerator (db::Shapes &shapes, bool clear_shapes = false, int tag = 0, EdgeShapeGenerator *chained = 0)
    : EdgeSink (), mp_shapes (&shapes), m_clear_shapes (clear_shapes), m_tag (tag), mp_chained (chained)
  { }

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void put (const db::Edge &edge) 
  {
    mp_shapes->insert (edge);
    if (mp_chained) {
      mp_chained->put (edge);
    }
  }

  /**
   *  @brief Implementation of the EdgeSink interface
   */
  virtual void put (const db::Edge &edge, int tag)
  {
    if (m_tag == 0 || m_tag == tag) {
      mp_shapes->insert (edge);
    }
    if (mp_chained) {
      mp_chained->put (edge, tag);
    }
  }

  /**
   *  @brief Implementation of the PolygonSink interface
   */
  virtual void start () 
  { 
    if (m_clear_shapes) {
      mp_shapes->clear ();
      //  The single-shot scheme is a easy way to overcome problems with multiple start/flush brackets (i.e. on size filter)
      m_clear_shapes = false;
    }
    if (mp_chained) {
      mp_chained->start ();
    }
  }

private:
  // [[ZH]] mp_shapes     ：目标图形容器（外部传入，本类不拥有）。
  // [[ZH]] m_clear_shapes：是否在首次 start() 清空（single-shot 标志）。
  // [[ZH]] m_tag         ：只写入该标签的边；0 表示不限。
  // [[ZH]] mp_chained    ：可选的链式下游边接收端；0 表示无。
  db::Shapes *mp_shapes;
  bool m_clear_shapes;
  int m_tag;
  EdgeShapeGenerator *mp_chained;
};

/**
 *  @brief A processor for shape objects
 *
 *  Similar to the edge processor, this class deals with shape objects and shape containers 
 *  instead of polygons.
 */
// [[ZH-BEGIN]]
// ============================================================================
//  ShapeProcessor —— 面向「Shape」的边处理器（EdgeProcessor 的前端包装）
// ============================================================================
//
// 【与 EdgeProcessor 的关系（一句话）】
//   EdgeProcessor 只认「边」；ShapeProcessor 认「版图图形」——
//   它把 db::Shape（可能是 box / polygon / path / text...）拆成边，喂给内部的
//   EdgeProcessor，再把结果包装回去。因此它是**面向使用者的便利外观**。
//
//        db::Shape / db::Shapes（版图图层上的图形）
//              ↓ 拆成边               ShapeProcessor  ★ 本类
//        db::EdgeProcessor（布尔/融合/缩放引擎）
//              ↓ 输出
//        EdgeSink / PolygonSink
//
// 【★ 为什么需要它 —— 它额外做了 EdgeProcessor 不做的事】
//   1) **图形类型的归一化**：Shape 可以是 box / polygon / path / text 等，
//      ShapeProcessor 负责把它们统一转成边（path 要按宽度展开成轮廓，
//      box 要转成 4 条边）。见 insert(shape, ...) 的各重载。
//   2) **变换的应用**：insert 时可直接传入变换（db::Trans 等），
//      在拆边的同时完成坐标变换，避免用户先手工变换再插入。
//   3) **批量筛选**：可以按图形类型/属性过滤，只插入关心的图形
//      （见下面的 insert(..., ShapeSelector) 之类重载）。
//
// 【组成】
//   · 内部持有一个 EdgeProcessor（成员 m_processor）—— 所有几何运算都委托给它。
//     因此 EdgeProcessor 的性能特性（阶段 2 求交最贵、redo 可跳过等）在这里同样适用。
//   · 同文件还有 EdgeShapeGenerator（实现 EdgeSink），负责**反向**把结果边
//     写回 db::Shapes 容器 —— 即"输出端"。于是形成完整的闭环。
//
// 【与 Python/Ruby API 的对应】
//   脚本里的 RBA::ShapeProcessor / klayout.db.ShapeProcessor 就是本类，
//   其绑定文档明确提示 "See the EdgeProcessor for a description" ——
//   因为两者语义一致，只是输入类型不同。
//
// 参数（构造）：report_progress 是否上报进度（有性能代价）；
//               progress_desc 进度条文字。
// [[ZH-END]]
class DB_PUBLIC ShapeProcessor 
{
public:
  /**
   *  @brief Constructor
   *
   *  @param report_progress If true, a tl::Progress object will be created to report any progress (warning: this will impose a performance penalty)
   */
  // [[ZH]] 功能：构造。内部创建 EdgeProcessor 并转发进度设置；此时没有任何图形。
  // [[ZH]] 参数：report_progress 是否上报进度（★ 有性能损失，批量运算建议关闭）。
  ShapeProcessor (bool report_progress = false, const std::string &progress_desc = std::string ());

  /**
   *  @brief Clear the shapes stored currently
   */
  // [[ZH]] 功能：清空已插入的图形（转发给内部 EdgeProcessor::clear）。
  // [[ZH]] 用途：复用同一个对象处理下一批数据时调用。
  void clear ();

  /**
   *  @brief Reserve the number of edges
   */
  // [[ZH]] 功能：预分配容量。参数 n 是**预估的边数**（注意不是图形数）——
  // [[ZH]]       一个多边形会展开出与顶点数相同的边数，所以 n 通常远大于图形数。
  void reserve (size_t n);

  /**
   *  @brief Reports the number of edges stored in the processor
   */
  // [[ZH]] 功能：返回内部已存储的**边**数量（不是图形数量）。
  // [[ZH]] 坑：因为是把图形拆成边后计数，所以它通常比插入的图形数大得多；
  // [[ZH]]     并且零点长的边会在插入时被丢弃（继承 EdgeProcessor 的行为）。
  size_t count () const;

  /**
   *  @brief Sets the base verbosity of the processor (see EdgeProcessor::set_base_verbosity for details)
   */
  // [[ZH]] 功能：设置耗时日志的详细程度阈值（直接转发给内部 EdgeProcessor，含义见彼处）。
  void set_base_verbosity (int bv)
  {
    m_processor.set_base_verbosity (bv);
  }

  /**
   *  @brief Enable progress
   *
   *  @param progress_text The description text of the progress object
   */
  // [[ZH]] 功能：启用进度上报（转发给内部 EdgeProcessor）。
  void enable_progress (const std::string &progress_desc = std::string ())
  {
    m_processor.enable_progress (progress_desc);
  }

  /**
   *  @brief Disable progress
   */
  // [[ZH]] 功能：关闭进度上报（转发给内部 EdgeProcessor）。
  void disable_progress ()
  {
    m_processor.disable_progress ();
  }

  /**
   *  @brief Insert a shape without transformation
   */
  // [[ZH]] 功能：插入一个图形（无坐标变换）。等价于用单位变换调用带变换的版本。
  // [[ZH]] 参数：shape 待插入图形；p 来源标签（见 EdgeProcessor 文件头的 property 说明）。
  void insert (const db::Shape &shape, db::EdgeProcessor::property_type p)
  {
    insert (shape, db::UnitTrans (), p);
  }

  /**
   *  @brief Insert a native shape
   */
  template <class S>
  void insert_native (const S &shape, db::EdgeProcessor::property_type p)
  {
    m_processor.insert (shape, p);
  }

  /**
   *  @brief Insert a shape
   */
  template <class T>
  void insert (const db::Shape &shape, const T &trans, db::EdgeProcessor::property_type p)
  {
    if (shape.is_polygon ()) {

      for (db::Shape::polygon_edge_iterator e = shape.begin_edge (); ! e.at_end (); ++e) {
        m_processor.insert ((*e).transform (trans), p);
      }

    } else if (shape.is_path ()) {

      db::Polygon poly;
      shape.polygon (poly);
      for (db::Polygon::polygon_edge_iterator e = poly.begin_edge (); ! e.at_end (); ++e) {
        m_processor.insert ((*e).transform (trans), p);
      }

    } else if (shape.is_box ()) {

      db::Box b (shape.box ());
      m_processor.insert (db::Edge (b.lower_left (), b.upper_left ()).transform (trans), p);
      m_processor.insert (db::Edge (b.upper_left (), b.upper_right ()).transform (trans), p);
      m_processor.insert (db::Edge (b.upper_right (), b.lower_right ()).transform (trans), p);
      m_processor.insert (db::Edge (b.lower_right (), b.lower_left ()).transform (trans), p);

    }
  }

  /**
   *  @brief Count the edges for a shape
   */
  size_t count_edges (const db::Shape &shape) const;

  /**
   *  @brief Insert a sequence of shapes
   *
   *  This method does not reserve for the number of elements required. This must
   *  be done explicitly for performance benefits.
   */
  template <class Iter>
  void insert_sequence (Iter from, Iter to, db::EdgeProcessor::property_type p = 0)
  {
    for (Iter i = from; i != to; ++i) {
      insert (*i, p);
    }
  }

  /**
   *  @brief Insert a sequence of shapes (iterator with at_end semantics)
   *
   *  This method does not reserve for the number of elements required. This must
   *  be done explicitly for performance benefits.
   */
  template <class Iter>
  void insert_sequence (Iter i, db::EdgeProcessor::property_type p = 0)
  {
    for ( ; !i.at_end (); ++i) {
      insert (*i, p);
    }
  }

  /**
   *  @brief Process the given edges 
   *
   *  This method uses the given sink as target and the given evaluator for defining
   *  the method to use
   */
  void process (db::EdgeSink &es, EdgeEvaluatorBase &op);

  /**
   *  @brief Merge the given shapes
   *
   *  See the EdgeProcessor for a description of the merge method. This implementation takes shapes
   *  rather than polygons for input and produces a polygon set.
   *
   *  @param in The set of shapes to merge
   *  @param trans A corresponding set of transformations to apply on the shapes
   *  @param out The result (a polygon vector)
   *  @param min_wc The minimum wrap count for output (0: all polygons, 1: at least two overlapping)
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void merge (const std::vector<db::Shape> &in, const std::vector<db::CplxTrans> &trans,
              std::vector <db::Polygon> &out, unsigned int min_wc = 0, bool resolve_holes = true, bool min_coherence = true);

  /**
   *  @brief Merge the given shapes
   *
   *  This is equivalent to the previous method except that no transformations can be specified.
   *
   *  @param in The set of shapes to merge
   *  @param out The result (a polygon vector)
   *  @param min_wc The minimum wrap count for output (0: all polygons, 1: at least two overlapping)
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void merge (const std::vector<db::Shape> &in, std::vector <db::Polygon> &out, unsigned int min_wc = 0, bool resolve_holes = true, bool min_coherence = true)
  {
    merge (in, std::vector<db::CplxTrans> (), out, min_wc, resolve_holes, min_coherence);
  }

  /**
   *  @brief Merge the given shapes into an edge set
   *
   *  This is basically a "or" operation on a single layer creating edges.
   *
   *  @param in The set of shapes to merge
   *  @param trans A corresponding set of transformations to apply on the shapes
   *  @param out The result (an edge vector)
   *  @param min_wc The minimum wrap count for output (0: all polygons, 1: at least two overlapping)
   */
  void merge (const std::vector<db::Shape> &in, const std::vector<db::CplxTrans> &trans,
              std::vector <db::Edge> &out, unsigned int min_wc = 0);

  /**
   *  @brief Merge the given shapes into an edge set
   *
   *  This is basically a "or" operation on a single layer creating edges.
   *  This is equivalent to the previous method except that no transformations can be specified.
   *
   *  @param in The set of shapes to merge
   *  @param out The result (an edge vector)
   *  @param min_wc The minimum wrap count for output (0: all polygons, 1: at least two overlapping)
   */
  void merge (const std::vector<db::Shape> &in, std::vector<db::Edge> &out, unsigned int min_wc = 0)
  {
    merge (in, std::vector<db::CplxTrans> (), out, min_wc);
  }

  /**
   *  @brief Merge the given shapes from a layout to a shape container
   *
   *  This is basically a "or" operation on a single layer.
   *  The input can be taken from a layout and the result is delivered to a 
   *  shape container.
   *
   *  @param layout_in The layout from which to take the input
   *  @param cell_in The cell from which to take the input
   *  @param layer_in The layer from which to take the input
   *  @param out Where to store the results 
   *  @param with_sub_hierarchy Take shapes from the cell and subcells
   *  @param min_wc The minimum wrap count for output (0: all polygons, 1: at least two overlapping)
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void merge (const db::Layout &layout_in, const db::Cell &cell_in, unsigned int layer_in, 
              db::Shapes &out, bool with_sub_hierarchy, unsigned int min_wc = 0, bool resolve_holes = true, bool min_coherence = true)
  {
    std::vector<unsigned int> layers_in;
    layers_in.push_back (layer_in);
    merge (layout_in, cell_in, layers_in, out, with_sub_hierarchy, min_wc, resolve_holes, min_coherence);
  }

  /**
   *  @brief Merge the given shapes from a layout to a shape container
   *
   *  This is basically a "or" operation on a single layer.
   *  The input can be taken from a layout and the result is delivered to a 
   *  shape container.
   *
   *  @param layout_in The layout from which to take the input
   *  @param cell_in The cell from which to take the input
   *  @param layers_in The layers from which to take the input
   *  @param out Where to store the results 
   *  @param with_sub_hierarchy Take shapes from the cell and subcells
   *  @param min_wc The minimum wrap count for output (0: all polygons, 1: at least two overlapping)
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void merge (const db::Layout &layout_in, const db::Cell &cell_in, const std::vector<unsigned int> &layers_in, 
              db::Shapes &out, bool with_sub_hierarchy, unsigned int min_wc = 0, bool resolve_holes = true, bool min_coherence = true);

  /**
   *  @brief Boolean operation on two given shape sets into a polygon set
   *
   *  @param in_a The set of shapes to use for input A
   *  @param trans_a A set of transformations to apply before the shapes are used
   *  @param in_b The set of shapes to use for input A
   *  @param trans_b A set of transformations to apply before the shapes are used
   *  @param mode The boolean operation
   *  @param out The result (an polygon vector)
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void boolean (const std::vector<db::Shape> &in_a, const std::vector<db::CplxTrans> &trans_a,
                const std::vector<db::Shape> &in_b, const std::vector<db::CplxTrans> &trans_b,
                int mode, std::vector <db::Polygon> &out, bool resolve_holes = true, bool min_coherence = true);

  /**
   *  @brief Boolean operation on two given shape sets into a polygon set
   *
   *  @param in_a The set of shapes to use for input A
   *  @param in_b The set of shapes to use for input A
   *  @param mode The boolean operation
   *  @param out The result (an polygon vector)
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void boolean (const std::vector<db::Shape> &in_a, const std::vector<db::Shape> &in_b, 
                int mode, std::vector <db::Polygon> &out, bool resolve_holes = true, bool min_coherence = true)
  {
    boolean (in_a, std::vector<db::CplxTrans> (), in_b, std::vector<db::CplxTrans> (), mode, out, resolve_holes, min_coherence);
  }

  /**
   *  @brief Boolean operation on two given shape sets into an edge set
   *
   *  @param in_a The set of shapes to use for input A
   *  @param trans_a A set of transformations to apply before the shapes are used
   *  @param in_b The set of shapes to use for input A
   *  @param trans_b A set of transformations to apply before the shapes are used
   *  @param mode The boolean operation
   *  @param out The result (an edge vector)
   */
  void boolean (const std::vector<db::Shape> &in_a, const std::vector<db::CplxTrans> &trans_a,
                const std::vector<db::Shape> &in_b, const std::vector<db::CplxTrans> &trans_b,
                int mode, std::vector <db::Edge> &out);

  /**
   *  @brief Boolean operation on two given shape sets into an edge set
   *
   *  @param in_a The set of shapes to use for input A
   *  @param in_b The set of shapes to use for input A
   *  @param mode The boolean operation
   *  @param out The result (an edge vector)
   */
  void boolean (const std::vector<db::Shape> &in_a, const std::vector<db::Shape> &in_b, 
                int mode, std::vector <db::Edge> &out)
  {
    boolean (in_a, std::vector<db::CplxTrans> (), in_b, std::vector<db::CplxTrans> (), mode, out);
  }

  /**
   *  @brief Boolean operation on the given shapes from a layout to a shape container
   *
   *  The input can be taken from separate layouts and the result is delivered to a 
   *  shape container.
   *
   *  @param layout_in_a The layout from which to take the input for source A
   *  @param cell_in_a The cell from which to take the input for source A
   *  @param layer_in_a The layer from which to take the input for source A
   *  @param layout_in_b The layout from which to take the input for source B
   *  @param cell_in_b The cell from which to take the input for source B
   *  @param layer_in_b The layer from which to take the input for source B
   *  @param out Where to store the results 
   *  @param mode The boolean operation to apply
   *  @param with_sub_hierarchy Take shapes from the cell and subcells
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void boolean (const db::Layout &layout_in_a, const db::Cell &cell_in_a, unsigned int layer_in_a, 
                const db::Layout &layout_in_b, const db::Cell &cell_in_b, unsigned int layer_in_b, 
                db::Shapes &out, int mode, bool with_sub_hierarchy = false, bool resolve_holes = true, bool min_coherence = true)
  {
    std::vector<unsigned int> layers_in_a;
    layers_in_a.push_back (layer_in_a);
    std::vector<unsigned int> layers_in_b;
    layers_in_b.push_back (layer_in_b);
    boolean (layout_in_a, cell_in_a, layers_in_a, layout_in_b, cell_in_b, layers_in_b, out, mode, with_sub_hierarchy, resolve_holes, min_coherence);
  }

  /**
   *  @brief Boolean operation on the given shapes from a layout to a shape container
   *
   *  The input can be taken from separate layouts and the result is delivered to a 
   *  shape container.
   *
   *  @param layout_in_a The layout from which to take the input for source A
   *  @param cell_in_a The cell from which to take the input for source A
   *  @param layers_in_a The layer from which to take the input for source A
   *  @param layout_in_b The layout from which to take the input for source B
   *  @param cell_in_b The cell from which to take the input for source B
   *  @param layers_in_b The layer from which to take the input for source B
   *  @param out Where to store the results 
   *  @param mode The boolean operation to apply
   *  @param with_sub_hierarchy Take shapes from the cell and subcells
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void boolean (const db::Layout &layout_in_a, const db::Cell &cell_in_a, const std::vector<unsigned int> &layers_in_a, 
                const db::Layout &layout_in_b, const db::Cell &cell_in_b, const std::vector<unsigned int> &layers_in_b, 
                db::Shapes &out, int mode, bool with_sub_hierarchy = false, bool resolve_holes = true, bool min_coherence = true);

  /**
   *  @brief Size the given shapes into an polygon set
   *
   *  This is equivalent to the previous method except that no transformations can be specified.
   *
   *  @param in The set of shapes to size
   *  @param trans A set of transformations to apply before the shapes are used
   *  @param dx The sizing to apply (x-direction)
   *  @param dy The sizing to apply (y-direction)
   *  @param out The result (an edge vector)
   *  @param mode The sizing mode (see db::Polygon for a description)
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void size (const std::vector<db::Shape> &in, const std::vector<db::CplxTrans> &trans,
             db::Coord dx, db::Coord dy, std::vector <db::Polygon> &out, unsigned int mode = 2, bool resolve_holes = true, bool min_coherence = true);

  /**
   *  @brief Size the given shapes into an polygon set
   *
   *  This is equivalent to the previous method except that no transformations can be specified.
   *
   *  @param in The set of shapes to size
   *  @param trans A set of transformations to apply before the shapes are used
   *  @param d The sizing to apply 
   *  @param out The result (an edge vector)
   *  @param mode The sizing mode (see db::Polygon for a description)
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void size (const std::vector<db::Shape> &in, const std::vector<db::CplxTrans> &trans,
             db::Coord d, std::vector <db::Polygon> &out, unsigned int mode = 2, bool resolve_holes = true, bool min_coherence = true)
  {
    size (in, trans, d, d, out, mode, resolve_holes, min_coherence);
  }

  /**
   *  @brief Size the given shapes into an polygon set
   *
   *  This is equivalent to the previous method except that no transformations can be specified.
   *
   *  @param in The set of shapes to size
   *  @param d The sizing to apply 
   *  @param out The result (an edge vector)
   *  @param mode The sizing mode (see db::Polygon for a description)
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void size (const std::vector<db::Shape> &in, 
             db::Coord d, std::vector <db::Polygon> &out, unsigned int mode = 2, bool resolve_holes = true, bool min_coherence = true)
  {
    size (in, std::vector<db::CplxTrans> (), d, out, mode, resolve_holes, min_coherence);
  }

  /**
   *  @brief Size the given shapes into an polygon set
   *
   *  This is equivalent to the previous method except that no transformations can be specified.
   *
   *  @param in The set of shapes to size
   *  @param dx The sizing to apply (x-direction)
   *  @param dy The sizing to apply (y-direction)
   *  @param out The result (an edge vector)
   *  @param mode The sizing mode (see db::Polygon for a description)
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void size (const std::vector<db::Shape> &in, 
             db::Coord dx, db::Coord dy, std::vector <db::Polygon> &out, unsigned int mode = 2, bool resolve_holes = true, bool min_coherence = true)
  {
    size (in, std::vector<db::CplxTrans> (), dx, dy, out, mode, resolve_holes, min_coherence);
  }

  /**
   *  @brief Size the given shapes into an edge set
   *
   *  This is equivalent to the previous method except that no transformations can be specified.
   *
   *  @param in The set of shapes to size
   *  @param trans A set of transformations to apply before the shapes are used
   *  @param dx The sizing to apply (x-direction)
   *  @param dy The sizing to apply (y-direction)
   *  @param out The result (an edge vector)
   *  @param mode The sizing mode (see db::Polygon for a description)
   */
  void size (const std::vector<db::Shape> &in, const std::vector<db::CplxTrans> &trans,
             db::Coord dx, db::Coord dy, std::vector <db::Edge> &out, unsigned int mode = 2);

  /**
   *  @brief Size the given shapes into an edge set
   *
   *  This is equivalent to the previous method except that no transformations can be specified.
   *
   *  @param in The set of shapes to size
   *  @param trans A set of transformations to apply before the shapes are used
   *  @param d The sizing to apply 
   *  @param out The result (an edge vector)
   *  @param mode The sizing mode (see db::Polygon for a description)
   */
  void size (const std::vector<db::Shape> &in, const std::vector<db::CplxTrans> &trans,
             db::Coord d, std::vector <db::Edge> &out, unsigned int mode = 2)
  {
    size (in, trans, d, d, out, mode);
  }

  /**
   *  @brief Size the given shapes into an edge set
   *
   *  This is equivalent to the previous method except that no transformations can be specified.
   *
   *  @param in The set of shapes to size
   *  @param d The sizing to apply 
   *  @param out The result (an edge vector)
   *  @param mode The sizing mode (see db::Polygon for a description)
   */
  void size (const std::vector<db::Shape> &in, 
             db::Coord d, std::vector <db::Edge> &out, unsigned int mode = 2)
  {
    size (in, std::vector<db::CplxTrans> (), d, out, mode);
  }

  /**
   *  @brief Size the given shapes into an edge set
   *
   *  This is equivalent to the previous method except that no transformations can be specified.
   *
   *  @param in The set of shapes to size
   *  @param dx The sizing to apply (x-direction)
   *  @param dy The sizing to apply (y-direction)
   *  @param out The result (an edge vector)
   *  @param mode The sizing mode (see db::Polygon for a description)
   */
  void size (const std::vector<db::Shape> &in, 
             db::Coord dx, db::Coord dy, std::vector <db::Edge> &out, unsigned int mode = 2)
  {
    size (in, std::vector<db::CplxTrans> (), dx, dy, out, mode);
  }

  /**
   *  @brief Size operation on the given shapes from a layout to a shape container
   *
   *  The input can be taken from a layout and the result is delivered to a 
   *  shape container.
   *
   *  @param layout_in The layout from which to take the input for source A
   *  @param cell_in The cell from which to take the input for source A
   *  @param layer_in The layer from which to take the input for source A
   *  @param out Where to store the results 
   *  @param d The sizing to apply
   *  @param mode The sizing mode (see db::Polygon for a description)
   *  @param with_sub_hierarchy Take shapes from the cell and subcells
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void size (const db::Layout &layout_in, const db::Cell &cell_in, unsigned int layer_in, 
             db::Shapes &out, db::Coord d, unsigned int mode = 2, bool with_sub_hierarchy = false, bool resolve_holes = true, bool min_coherence = true)
  {
    size (layout_in, cell_in, layer_in, out, d, d, mode, with_sub_hierarchy, resolve_holes, min_coherence);
  }
             
  /**
   *  @brief Size operation on the given shapes from a layout to a shape container
   *
   *  The input can be taken from a layout and the result is delivered to a 
   *  shape container.
   *
   *  @param layout_in The layout from which to take the input for source A
   *  @param cell_in The cell from which to take the input for source A
   *  @param layers_in The layers from which to take the input for source A
   *  @param out Where to store the results 
   *  @param d The sizing to apply
   *  @param mode The sizing mode (see db::Polygon for a description)
   *  @param with_sub_hierarchy Take shapes from the cell and subcells
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void size (const db::Layout &layout_in, const db::Cell &cell_in, const std::vector<unsigned int> &layers_in, 
             db::Shapes &out, db::Coord d, unsigned int mode = 2, bool with_sub_hierarchy = false, bool resolve_holes = true, bool min_coherence = true)
  {
    size (layout_in, cell_in, layers_in, out, d, d, mode, with_sub_hierarchy, resolve_holes, min_coherence);
  }
             
  /**
   *  @brief Size operation on the given shapes from a layout to a shape container
   *
   *  The input can be taken from a layout and the result is delivered to a 
   *  shape container.
   *
   *  @param layout_in The layout from which to take the input for source A
   *  @param cell_in The cell from which to take the input for source A
   *  @param layer_in The layer from which to take the input for source A
   *  @param out Where to store the results 
   *  @param dx The sizing to apply (x-direction)
   *  @param dy The sizing to apply (y-direction)
   *  @param mode The sizing mode (see db::Polygon for a description)
   *  @param with_sub_hierarchy Take shapes from the cell and subcells
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void size (const db::Layout &layout_in, const db::Cell &cell_in, unsigned int layer_in, 
             db::Shapes &out, db::Coord dx, db::Coord dy, unsigned int mode = 2, bool with_sub_hierarchy = false, bool resolve_holes = true, bool min_coherence = true)
  {
    std::vector<unsigned int> layers_in;
    layers_in.push_back (layer_in);
    size (layout_in, cell_in, layers_in, out, dx, dy, mode, with_sub_hierarchy, resolve_holes, min_coherence);
  }

  /**
   *  @brief Size operation on the given shapes from a layout to a shape container
   *
   *  The input can be taken from a layout and the result is delivered to a 
   *  shape container.
   *
   *  @param layout_in The layout from which to take the input for source A
   *  @param cell_in The cell from which to take the input for source A
   *  @param layers_in The layers from which to take the input for source A
   *  @param out Where to store the results 
   *  @param dx The sizing to apply (x-direction)
   *  @param dy The sizing to apply (y-direction)
   *  @param mode The sizing mode (see db::Polygon for a description)
   *  @param with_sub_hierarchy Take shapes from the cell and subcells
   *  @param resolve_holes true, if holes should be resolved into the hull
   *  @param min_coherence true, if minimum polygons should be created for touching corners
   */
  void size (const db::Layout &layout_in, const db::Cell &cell_in, const std::vector<unsigned int> &layers_in, 
             db::Shapes &out, db::Coord dx, db::Coord dy, unsigned int mode = 2, bool with_sub_hierarchy = false, bool resolve_holes = true, bool min_coherence = true);

private:
  EdgeProcessor m_processor;

  void collect_shapes_hier (const db::CplxTrans &tr, const db::Layout &layout, const db::Cell &cell, unsigned int layer, int hier_levels, size_t &pn, size_t pdelta);
  size_t count_edges_hier (const db::Layout &layout, const db::Cell &cell, unsigned int layer, std::map<std::pair<db::cell_index_type, int>, size_t> &cache, int hier_levels) const;
};

}

#endif


