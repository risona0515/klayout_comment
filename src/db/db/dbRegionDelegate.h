
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


#ifndef HDR_dbRegionDelegate
#define HDR_dbRegionDelegate

#include "dbCommon.h"

#include "dbPolygon.h"
#include "dbEdges.h"
#include "dbTexts.h"
#include "dbEdgePairs.h"
#include "dbEdgePairRelations.h"
#include "dbShapeCollection.h"
#include "dbGenericShapeIterator.h"
#include "dbRegionLocalOperations.h"
#include "dbHash.h"
#include "dbLayoutToNetlistEnums.h"

#include <list>
#include <set>
#include <unordered_set>

namespace db {

class RecursiveShapeIterator;
class EdgeFilterBase;
class EdgesDelegate;
class EdgePairsDelegate;
class CompoundRegionOperationNode;
class LayoutToNetlist;
class Net;

/**
 *  @brief A base class for polygon filters
 */
// [[ZH-BEGIN]]
// 功能：★ **多边形过滤器**的抽象基类 —— `Region::filtered(...)` 用它挑选多边形。
//
// 【它解决什么问题】
//   常需要“从区域里挑出符合条件的一部分多边形”，例如：
//     · 属性 net 等于 "VDD" 的多边形；
//     · 面积大于某阈值的多边形；
//     · 位于某框内/外的多边形。
//   若为每种条件都加一个 Region 方法会爆炸，因此改成“**过滤器对象**”模式：
//       region.filtered (my_filter)   → 返回只含被选中多边形的 Region
//   你只需实现本类，即可复用整套集合运算基础设施。
//
// 【★ 需要实现的方法（全是纯虚函数）】
//   selected (polygon, prop_id)       —— ★ 核心：该多边形是否选中
//   selected (polygon_ref, prop_id)   —— 同上，但输入是“引用+变换”形态
//   selected_set (set_of_polygons)    —— 同上，但一次看**一整组**（可看整体属性）
//   selected_set (set_of_refs)        —— 同上
//   vars ()                           —— 与“单元变体 (variants)”相关的变换归约器
//   requires_raw_input ()             —— ★ 是否需要**未合并**的输入
//   wants_variants ()                 —— 是否想生成变体
//
// 【★★ 三个容易忽略但重要的点】
//   1) **有 4 个 selected 重载，不是 1 个**：
//      因为多边形的存在形态不同（平铺值 / 引用+变换 / 成组、带整体属性），
//      过滤逻辑在每种形态下取数据的方式不同。
//      ★ 通常你只需真正实现其中一个、其余转发或返回同一判断，
//        但**必须都实现**（纯虚）。
//   2) **prop_id 参数是给“属性过滤”用的**：
//      过滤器可能只关心几何、也可能按属性挑；属性值本身在仓库里
//      （见 dbPropertiesRepository.h），这里只给 ID。
//   3) **requires_raw_input () 会改变“喂给你的数据”**：
//      ★ 返回 true 时，库会传**未合并 (non-merged)** 的多边形给你；
//        返回 false 则通常传已合并的。
//      含义：若你的判断依赖“原始多边形边界”（例如想看到重叠），
//        就要返回 true；若只关心最终覆盖区域，用 false（更快）。
//      —— 这是本类唯一会影响**性能与语义**的开关，别忽略。
//
// 【与 Region 的配合】
//   用户看到的入口是 db::Region::filtered()；本类的派生类由库内置提供
//   （如按属性/按面积过滤），也可由脚本自定义。
// [[ZH-END]]
class DB_PUBLIC PolygonFilterBase
{
public:
  // [[ZH]] shape_type = db::Polygon —— 本过滤器处理的对象类型。
  typedef db::Polygon shape_type;

  /**
   *  @brief Constructor
   */
  // [[ZH]] 功能：构造（无状态基类，派生类自行保存过滤条件）。
  PolygonFilterBase () { }

  // [[ZH]] 虚析构：允许通过基类指针删除派生过滤器。
  virtual ~PolygonFilterBase () { }

  /**
   *  @brief Filters the polygon
   *  If this method returns true, the polygon is kept. Otherwise it's discarded.
   */
  virtual bool selected (const db::Polygon &polygon, db::properties_id_type prop_id) const = 0;

  /**
   *  @brief Filters the polygon reference
   *  If this method returns true, the polygon is kept. Otherwise it's discarded.
   */
  virtual bool selected (const db::PolygonRef &polygon, db::properties_id_type prop_id) const = 0;

  /**
   *  @brief Filters the set of polygons (taking the overall properties)
   *  If this method returns true, the polygon is kept. Otherwise it's discarded.
   */
  virtual bool selected_set (const std::unordered_set<db::PolygonWithProperties> &polygons) const = 0;

  /**
   *  @brief Filters the set of polygon references (taking the overall properties)
   *  If this method returns true, the polygon is kept. Otherwise it's discarded.
   */
  virtual bool selected_set (const std::unordered_set<db::PolygonRefWithProperties> &polygons) const = 0;

  /**
   *  @brief Returns the transformation reducer for building cell variants
   *  This method may return 0. In this case, not cell variants are built.
   */
  virtual const TransformationReducer *vars () const = 0;

  /**
   *  @brief Returns true, if the filter wants raw (not merged) input
   */
  virtual bool requires_raw_input () const = 0;

  /**
   *  @brief Returns true, if the filter wants to build variants
   *  If not true, the filter accepts shape propagation as variant resolution.
   */
  virtual bool wants_variants () const = 0;
};

typedef shape_collection_processor<db::Polygon, db::Polygon> PolygonProcessorBase;
typedef shape_collection_processor<db::Polygon, db::Edge> PolygonToEdgeProcessorBase;
typedef shape_collection_processor<db::Polygon, db::EdgePair> PolygonToEdgePairProcessorBase;

/**
 *  @brief The region iterator delegate
 */
typedef db::generic_shape_iterator_delegate_base <db::Polygon> RegionIteratorDelegate;

/**
 *  @brief The delegate for the actual region implementation
 */
// [[ZH-BEGIN]]
// 功能：★★ **Region 的委托接口** —— 这个抽象类定义了“一个区域实现必须能做什么”。
//
// 【★ 它在架构中的位置（与 dbShapeCollection.h / dbRegion.h 呼应）】
//       db::Region                     ← 门面（用户用的对象，很小）
//          └─ mp_delegate → RegionDelegate   ★ 本类：抽象接口
//                 ├─ db::FlatRegion        平铺实现
//                 ├─ db::DeepRegion        层次化实现（不展开）
//                 ├─ db::EmptyRegion       空实现（惰性）
//                 ├─ db::MutableRegion     可修改包装
//                 ├─ db::AsIfFlatRegion    按需展开但表现为平铺
//                 └─ db::OriginalLayerRegion  直接指向版图某层
//   ★ 因此：**看本类的方法列表 = 看“一个 Region 能做什么”的完整能力清单**。
//     Region 门面上的每个操作，最终都转发到这里某个虚函数。
//
// 【★ 从继承链看它的身份】
//   public db::ShapeCollectionDelegateBase（见 dbShapeCollection.h）
//     → 因此它有 tl::UniqueId 提供的唯一标识 m_data_id，
//       用于判断“两个 Region 是否共享同一实现”（写时复制的依据）。
//   ★ 关键点：`clone ()` 是纯虚 —— 每个实现必须能克隆自己，
//     这正是“修改操作返回新对象”得以实现的基础
//     （门面在需要独占时才 clone 一份再改，避免影响共享者）。
//
// 【★ 为什么规模如此大（几百行纯虚函数）】
//   因为 Region 的能力面很宽：布尔运算、sizing、选择/过滤、面积周长、
//   与其它集合的互转（转边/转 EdgePairs）、遍历、字符串化……
//   把它们抽象成接口后，**每个实现只需专注自己擅长的部分**：
//     · FlatRegion：什么都能算，但数据是展开的（内存大）；
//     · DeepRegion：只实现能在层次形式下做的事（内存小），
//       遇到必须展开的操作就报错或先展开。
//   ★ 实用提示：因此“某个操作在 Deep 下失败”往往不是 bug，
//     而是**该实现不支持该操作**（需要先平铺）。
// [[ZH-END]]
class DB_PUBLIC RegionDelegate
  : public db::ShapeCollectionDelegateBase
{
public:
  typedef db::Coord coord_type;
  typedef db::coord_traits<db::Coord> coord_traits;
  typedef db::Polygon polygon_type;
  typedef db::Vector vector_type;
  typedef db::Point point_type;
  typedef db::Box box_type;
  typedef coord_traits::distance_type distance_type;
  typedef coord_traits::perimeter_type perimeter_type;
  typedef coord_traits::area_type area_type;

  RegionDelegate ();
  virtual ~RegionDelegate ();

  RegionDelegate (const RegionDelegate &other);
  RegionDelegate &operator= (const RegionDelegate &other);

  virtual RegionDelegate *clone () const = 0;

  RegionDelegate *remove_properties (bool remove = true)
  {
    ShapeCollectionDelegateBase::remove_properties (remove);
    return this;
  }

  void set_base_verbosity (int vb);
  int base_verbosity () const
  {
    return m_base_verbosity;
  }

  void enable_progress (const std::string &progress_desc);
  void disable_progress ();

  void set_min_coherence (bool f);
  bool min_coherence () const
  {
    return m_merge_min_coherence;
  }

  void set_merged_semantics (bool f);
  bool merged_semantics () const
  {
    return m_merged_semantics;
  }

  void set_join_properties_on_merge (bool f);
  bool join_properties_on_merge () const
  {
    return m_join_properties_on_merge;
  }

  void set_strict_handling (bool f);
  bool strict_handling () const
  {
    return m_strict_handling;
  }

  virtual std::string to_string (size_t nmax) const = 0;

  virtual RegionIteratorDelegate *begin () const = 0;
  virtual RegionIteratorDelegate *begin_merged () const = 0;
  virtual RegionIteratorDelegate *begin_unmerged () const = 0;

  virtual std::pair<db::RecursiveShapeIterator, db::ICplxTrans> begin_iter () const = 0;
  virtual std::pair<db::RecursiveShapeIterator, db::ICplxTrans> begin_merged_iter () const = 0;
  virtual std::pair<db::RecursiveShapeIterator, db::ICplxTrans> begin_unmerged_iter () const = 0;

  virtual bool empty () const = 0;
  virtual bool is_box () const = 0;
  virtual bool is_merged () const = 0;
  virtual size_t hier_count () const = 0;
  virtual size_t count () const = 0;

  virtual area_type area (const db::Box &box) const = 0;
  virtual perimeter_type perimeter (const db::Box &box) const = 0;
  virtual Box bbox () const = 0;

  virtual EdgePairsDelegate *cop_to_edge_pairs (db::CompoundRegionOperationNode &node, db::PropertyConstraint prop_constraint) = 0;
  virtual RegionDelegate *cop_to_region (db::CompoundRegionOperationNode &node, db::PropertyConstraint prop_constraint) = 0;
  virtual EdgesDelegate *cop_to_edges (db::CompoundRegionOperationNode &node, db::PropertyConstraint prop_constraint) = 0;

  virtual EdgePairsDelegate *width_check (db::Coord d, const RegionCheckOptions &options) const = 0;
  virtual EdgePairsDelegate *space_check (db::Coord d, const RegionCheckOptions &options) const = 0;
  virtual EdgePairsDelegate *isolated_check (db::Coord d, const RegionCheckOptions &options) const = 0;
  virtual EdgePairsDelegate *notch_check (db::Coord d, const RegionCheckOptions &options) const = 0;
  virtual EdgePairsDelegate *enclosing_check (const Region &other, db::Coord d, const RegionCheckOptions &options) const = 0;
  virtual EdgePairsDelegate *overlap_check (const Region &other, db::Coord d, const RegionCheckOptions &options) const = 0;
  virtual EdgePairsDelegate *separation_check (const Region &other, db::Coord d, const RegionCheckOptions &options) const = 0;
  virtual EdgePairsDelegate *inside_check (const Region &other, db::Coord d, const RegionCheckOptions &options) const = 0;
  virtual EdgePairsDelegate *grid_check (db::Coord gx, db::Coord gy) const = 0;
  virtual EdgePairsDelegate *angle_check (double min, double max, bool inverse) const = 0;

  virtual RegionDelegate *snapped_in_place (db::Coord gx, db::Coord gy) = 0;
  virtual RegionDelegate *snapped (db::Coord gx, db::Coord gy) = 0;
  virtual RegionDelegate *scaled_and_snapped_in_place (db::Coord gx, db::Coord mx, db::Coord dx, db::Coord gy, db::Coord my, db::Coord dy) = 0;
  virtual RegionDelegate *scaled_and_snapped (db::Coord gx, db::Coord mx, db::Coord dx, db::Coord gy, db::Coord my, db::Coord dy) = 0;

  virtual EdgesDelegate *edges (const EdgeFilterBase *filter, const db::PolygonToEdgeProcessorBase *proc) const = 0;
  virtual RegionDelegate *filter_in_place (const PolygonFilterBase &filter) = 0;
  virtual RegionDelegate *filtered (const PolygonFilterBase &filter) const = 0;
  virtual std::pair<RegionDelegate *, RegionDelegate *> filtered_pair (const PolygonFilterBase &filter) const = 0;
  virtual RegionDelegate *process_in_place (const PolygonProcessorBase &filter) = 0;
  virtual RegionDelegate *processed (const PolygonProcessorBase &filter) const = 0;
  virtual EdgesDelegate *processed_to_edges (const PolygonToEdgeProcessorBase &filter) const = 0;
  virtual EdgePairsDelegate *processed_to_edge_pairs (const PolygonToEdgePairProcessorBase &filter) const = 0;

  virtual RegionDelegate *merged_in_place () = 0;
  virtual RegionDelegate *merged_in_place (bool min_coherence, unsigned int min_wc, bool join_properties_on_merge) = 0;
  virtual RegionDelegate *merged () const = 0;
  virtual RegionDelegate *merged (bool min_coherence, unsigned int min_wc, bool join_properties_on_merge) const = 0;

  virtual RegionDelegate *sized (coord_type d, unsigned int mode) const = 0;
  virtual RegionDelegate *sized (coord_type dx, coord_type dy, unsigned int mode) const = 0;
  virtual RegionDelegate *sized_inside (const Region &inside, bool outside, coord_type d, int steps, unsigned int mode) const = 0;
  virtual RegionDelegate *sized_inside (const Region &inside, bool outside, coord_type dx, coord_type dy, int steps, unsigned int mode) const = 0;

  virtual RegionDelegate *and_with (const Region &other, PropertyConstraint prop_constraint) const = 0;
  virtual RegionDelegate *not_with (const Region &other, PropertyConstraint prop_constraint) const = 0;
  virtual RegionDelegate *xor_with (const Region &other, PropertyConstraint prop_constraint) const = 0;
  virtual RegionDelegate *or_with (const Region &other, PropertyConstraint prop_constraint) const = 0;
  virtual RegionDelegate *add_in_place (const Region &other) = 0;
  virtual RegionDelegate *add (const Region &other) const = 0;
  virtual std::pair<RegionDelegate *, RegionDelegate *> andnot_with (const Region &other, PropertyConstraint prop_constraint) const = 0;

  virtual RegionDelegate *peel (double complexity_factor) const = 0;

  virtual RegionDelegate *selected_outside (const Region &other) const = 0;
  virtual RegionDelegate *selected_not_outside (const Region &other) const = 0;
  virtual std::pair<RegionDelegate *, RegionDelegate *> selected_outside_pair (const Region &other) const = 0;
  virtual RegionDelegate *selected_inside (const Region &other) const = 0;
  virtual RegionDelegate *selected_not_inside (const Region &other) const = 0;
  virtual std::pair<RegionDelegate *, RegionDelegate *> selected_inside_pair (const Region &other) const = 0;
  virtual RegionDelegate *selected_enclosing (const Region &other, size_t min_count, size_t max_count) const = 0;
  virtual RegionDelegate *selected_not_enclosing (const Region &other, size_t min_count, size_t max_count) const = 0;
  virtual std::pair<RegionDelegate *, RegionDelegate *> selected_enclosing_pair (const Region &other, size_t min_count, size_t max_count) const = 0;
  virtual RegionDelegate *selected_interacting (const Region &other, size_t min_count, size_t max_count) const = 0;
  virtual RegionDelegate *selected_not_interacting (const Region &other, size_t min_count, size_t max_count) const = 0;
  virtual std::pair<RegionDelegate *, RegionDelegate *> selected_interacting_pair (const Region &other, size_t min_count, size_t max_count) const = 0;
  virtual RegionDelegate *selected_interacting (const Edges &other, size_t min_count, size_t max_count) const = 0;
  virtual RegionDelegate *selected_not_interacting (const Edges &other, size_t min_count, size_t max_count) const = 0;
  virtual std::pair<RegionDelegate *, RegionDelegate *> selected_interacting_pair (const Edges &other, size_t min_count, size_t max_count) const = 0;
  virtual RegionDelegate *selected_interacting (const Texts &other, size_t min_count, size_t max_count) const = 0;
  virtual RegionDelegate *selected_not_interacting (const Texts &other, size_t min_count, size_t max_count) const = 0;
  virtual std::pair<RegionDelegate *, RegionDelegate *> selected_interacting_pair (const Texts &other, size_t min_count, size_t max_count) const = 0;
  virtual RegionDelegate *selected_overlapping (const Region &other, size_t min_count, size_t max_count) const = 0;
  virtual RegionDelegate *selected_not_overlapping (const Region &other, size_t min_count, size_t max_count) const = 0;
  virtual std::pair<RegionDelegate *, RegionDelegate *> selected_overlapping_pair (const Region &other, size_t min_count, size_t max_count) const = 0;
  virtual RegionDelegate *pull_inside (const Region &other) const = 0;
  virtual RegionDelegate *pull_interacting (const Region &other) const = 0;
  virtual EdgesDelegate *pull_interacting (const Edges &other) const = 0;
  virtual RegionDelegate *pull_overlapping (const Region &other) const = 0;
  virtual TextsDelegate *pull_interacting (const Texts &other) const = 0;
  virtual RegionDelegate *in (const Region &other, bool invert) const = 0;
  virtual std::pair<RegionDelegate *, RegionDelegate *> in_and_out (const Region &other) const = 0;

  virtual const db::Polygon *nth (size_t n) const = 0;
  virtual db::properties_id_type nth_prop_id (size_t n) const = 0;
  virtual bool has_valid_polygons () const = 0;
  virtual bool has_valid_merged_polygons () const = 0;

  virtual const db::RecursiveShapeIterator *iter () const = 0;

  virtual void apply_property_translator (const db::PropertiesTranslator &pt) = 0;

  virtual bool equals (const Region &other) const = 0;
  virtual bool less (const Region &other) const = 0;

  virtual void insert_into (Layout *layout, db::cell_index_type into_cell, unsigned int into_layer) const = 0;

  virtual RegionDelegate *nets (LayoutToNetlist *l2n, NetPropertyMode prop_mode, const tl::Variant &net_prop_name, const std::vector<const db::Net *> *net_filter) const = 0;

  const std::string &progress_desc () const
  {
    return m_progress_desc;
  }

  bool report_progress () const
  {
    return m_report_progress;
  }

protected:
  virtual void merged_semantics_changed () { }
  virtual void min_coherence_changed () { }
  virtual void join_properties_on_merge_changed () { }

private:
  bool m_merged_semantics;
  bool m_join_properties_on_merge;
  bool m_strict_handling;
  bool m_merge_min_coherence;
  bool m_report_progress;
  std::string m_progress_desc;
  int m_base_verbosity;
};

}

#endif

