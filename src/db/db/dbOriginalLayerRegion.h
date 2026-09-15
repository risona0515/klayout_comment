
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


#ifndef HDR_dbOriginalLayerRegion
#define HDR_dbOriginalLayerRegion

#include "dbCommon.h"

#include "dbAsIfFlatRegion.h"

namespace db {

class EdgesDelegate;
class RegionDelegate;
class DeepShapeStore;

/**
 *  @brief An original layerregion based on a RecursiveShapeIterator
 */
// [[ZH-BEGIN]]
// 功能：★ **直接基于版图某一层（非拥有数据）的区域实现**。
//
// 【上面英文注释的意思】
//   "An original layer region based on a RecursiveShapeIterator"
//   即：这个 Region **不存多边形** ——
//   它只是对**版图中某一层的图形**的一个“视图”，通过
//   db::RecursiveShapeIterator（见 dbRecursiveShapeIterator.h）**按需遍历**。
//
// 【★★ 核心价值：零拷贝】
//   假设你想对“M1 层”做布尔运算。有两种做法：
//     · 先把 M1 层的图形全部读进来构造一个 Region（拷贝 / 展开，内存大）；
//     · 用 OriginalLayerRegion **直接引用那一层**（不拷贝，几乎不占内存）。
//   后者显然更优 —— 尤其在“只想快速查询/统计某个已有层”时。
//
// 【★ 代价与限制（必须知道）】
//   1) 它是**只读视图**：不能插入新多边形（数据归属 Layout，不归它）。
//      需要修改时应先展开/拷贝成 FlatRegion。
//   2) 它的有效性**依附于源 Layout**：
//      若那个 Layout 或那一层被修改/重建，本对象可能失效或结果不一致。
//   3) 由于数据是“按需遍历”，某些需要随机访问的操作会较慢或不可用。
//
// 【与“代理模式”的类比】
//   它很像数据库里的**视图 (view)**：看起来像一个 Region，
//   但底层是引用而非拷贝。这与 db::PolygonRef（引用不拥有，见 dbPolygon.h）
//   是同一设计思想在不同层次上的体现。
//
// 继承：public AsIfFlatRegion —— 因此它对外表现为“平铺区域”（按需展开）。
// [[ZH-END]]
class DB_PUBLIC OriginalLayerRegion
  : public AsIfFlatRegion
{
public:
  OriginalLayerRegion ();
  OriginalLayerRegion (const OriginalLayerRegion &other);
  OriginalLayerRegion (const RecursiveShapeIterator &si, bool is_merged = false, bool min_coh = false);
  OriginalLayerRegion (const RecursiveShapeIterator &si, const db::ICplxTrans &trans, bool merged_semantics, bool is_merged = false, bool min_coh = false);
  virtual ~OriginalLayerRegion ();

  RegionDelegate *clone () const;

  virtual RegionIteratorDelegate *begin () const;
  virtual RegionIteratorDelegate *begin_merged () const;
  virtual RegionIteratorDelegate *begin_unmerged () const;

  virtual std::pair<db::RecursiveShapeIterator, db::ICplxTrans> begin_iter () const;
  virtual std::pair<db::RecursiveShapeIterator, db::ICplxTrans> begin_merged_iter () const;
  virtual std::pair<db::RecursiveShapeIterator, db::ICplxTrans> begin_unmerged_iter () const;

  virtual RegionDelegate *merged () const;
  virtual RegionDelegate *merged (bool min_coherence, unsigned int min_wc, bool join_properties_on_merge) const;

  bool merged_polygons_available () const;

  virtual bool empty () const;

  virtual bool is_merged () const;
  virtual size_t count () const;
  virtual size_t hier_count () const;

  virtual const db::Polygon *nth (size_t n) const;
  virtual db::properties_id_type nth_prop_id (size_t) const;
  virtual bool has_valid_polygons () const;
  virtual bool has_valid_merged_polygons () const;

  virtual const db::RecursiveShapeIterator *iter () const;
  virtual void apply_property_translator (const db::PropertiesTranslator &pt);

  virtual bool equals (const Region &other) const;
  virtual bool less (const Region &other) const;

  virtual void insert_into (Layout *layout, db::cell_index_type into_cell, unsigned int into_layer) const;

protected:
  virtual void merged_semantics_changed ();
  virtual void join_properties_on_merge_changed ();
  virtual void min_coherence_changed ();

private:
  OriginalLayerRegion &operator= (const OriginalLayerRegion &other);

  bool m_is_merged;
  mutable bool m_is_merged_min_coherence;
  mutable db::Shapes m_merged_polygons;
  mutable bool m_merged_polygons_valid;
  mutable bool m_merged_polygons_min_coherence;
  mutable db::RecursiveShapeIterator m_iter;
  db::ICplxTrans m_iter_trans;

  void init ();
  void ensure_merged_polygons_valid () const;
  bool merged_polygons_valid () const;
};

}

#endif

