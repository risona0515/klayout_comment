
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


#ifndef HDR_dbFlatRegion
#define HDR_dbFlatRegion

#include "dbCommon.h"

#include "dbMutableRegion.h"
#include "dbShapes.h"
#include "dbShapes2.h"
#include "tlCopyOnWrite.h"

namespace db {

/**
 *  @brief An iterator delegate for the flat region
 */
typedef generic_shapes_iterator_delegate<db::Polygon> FlatRegionIterator;

/**
 *  @brief A flat, polygon-set delegate
 */
// [[ZH-BEGIN]]
// 功能：★ **平铺（flat）区域实现** —— 把多边形**实际存在本地**并展开成一个大集合。
//
// 【它是什么】
//   这是 Region 最"直接"的实现：内部就是一个多边形集合（可能来自 merge/布尔运算的结果）。
//   上面提到它继承 MutableRegion → AsIfFlatRegion → RegionDelegate，
//   因此它支持全部 Region 操作（布尔、sizing、插入、遍历……）。
//
// 【★★ 与 DeepRegion 的取舍（决定何时用哪个）】
//     FlatRegion ：数据**已展开**——
//                  优点是几乎所有操作都能直接做；
//                  代价是**内存占用大**（层次被展平，重复图形各存一份）。
//     DeepRegion ：数据**保留层次**——
//                  优点是内存小；代价是部分操作做不了（需先平铺）。
//   ★ 实用影响：
//     · 对**小数据 / 需要任意运算** → Flat 更省心；
//     · 对**超大层次版图** → 尽量用 Deep 能做的操作（如面积、布尔），
//       避免触发展开（否则内存可能爆）。
//
// 【典型来源】
//   很多操作的结果天然是 flat，例如 `region.merge()`、布尔运算的输出、
//   以及从多边形列表构造的 Region。
//
// 提示：本文件开头还有一个 FlatRegionIterator（迭代器委托），
//   负责把内部存储的多边形逐个交出。
// [[ZH-END]]
class DB_PUBLIC FlatRegion
  : public MutableRegion
{
public:
  typedef db::Polygon value_type;
  typedef db::layer<db::Polygon, db::unstable_layer_tag> polygon_layer_type;
  typedef polygon_layer_type::iterator polygon_iterator_type;
  typedef db::layer<db::PolygonWithProperties, db::unstable_layer_tag> polygon_layer_wp_type;
  typedef polygon_layer_wp_type::iterator polygon_iterator_wp_type;

  FlatRegion (double area_ratio = 0.0, size_t max_vertex_count = 0);
  FlatRegion (const db::Shapes &polygons, bool is_merged = false, double area_ratio = 0.0, size_t max_vertex_count = 0, bool min_coh = false);
  FlatRegion (const db::Shapes &polygons, const db::ICplxTrans &trans, bool merged_semantics, bool is_merged = false, double area_ratio = 0.0, size_t max_vertex_count = 0, bool min_coh = false);
  FlatRegion (bool is_merged, double area_ratio = 0.0, size_t max_vertex_count = 0, bool min_coh = false);

  FlatRegion (const FlatRegion &other);

  virtual ~FlatRegion ();

  RegionDelegate *clone () const
  {
    return new FlatRegion (*this);
  }

  virtual void reserve (size_t);

  virtual RegionIteratorDelegate *begin () const;
  virtual RegionIteratorDelegate *begin_merged () const;
  virtual RegionIteratorDelegate *begin_unmerged () const;

  virtual std::pair<db::RecursiveShapeIterator, db::ICplxTrans> begin_iter () const;
  virtual std::pair<db::RecursiveShapeIterator, db::ICplxTrans> begin_merged_iter () const;
  virtual std::pair<db::RecursiveShapeIterator, db::ICplxTrans> begin_unmerged_iter () const;

  virtual bool empty () const;
  virtual size_t count () const;
  virtual size_t hier_count () const;
  virtual bool is_merged () const;

  virtual void insert_into (Layout *layout, db::cell_index_type into_cell, unsigned int into_layer) const;

  virtual RegionDelegate *merged_in_place ();
  virtual RegionDelegate *merged_in_place (bool min_coherence, unsigned int min_wc, bool join_properties_on_merge);
  virtual RegionDelegate *merged () const;
  virtual RegionDelegate *merged (bool min_coherence, unsigned int min_wc, bool join_properties_on_merge) const;

  bool merged_polygons_available () const;

  virtual RegionDelegate *process_in_place (const PolygonProcessorBase &filter);
  virtual RegionDelegate *filter_in_place (const PolygonFilterBase &filter);

  virtual RegionDelegate *add_in_place (const Region &other);
  virtual RegionDelegate *add (const Region &other) const;

  virtual const db::Polygon *nth (size_t n) const;
  virtual db::properties_id_type nth_prop_id (size_t) const;
  virtual bool has_valid_polygons () const;
  virtual bool has_valid_merged_polygons () const;

  virtual const db::RecursiveShapeIterator *iter () const;
  virtual void apply_property_translator (const db::PropertiesTranslator &pt);

  void do_insert (const db::Polygon &polygon, db::properties_id_type prop_id);

  void do_transform (const db::Trans &t)
  {
    transform_generic (t);
  }

  void do_transform (const db::ICplxTrans &t)
  {
    transform_generic (t);
  }

  virtual void do_transform (const db::IMatrix2d &t)
  {
    transform_generic (t);
  }

  virtual void do_transform (const db::IMatrix3d &t)
  {
    transform_generic (t);
  }

  void flatten () { }

  db::Shapes &raw_polygons () { return *mp_polygons; }
  const db::Shapes &raw_polygons () const { return *mp_polygons; }

protected:
  virtual void merged_semantics_changed ();
  virtual void join_properties_on_merge_changed ();
  virtual void min_coherence_changed ();
  virtual Box compute_bbox () const;
  void invalidate_cache ();
  void set_is_merged (bool m);
  bool merged_polygons_valid () const;

private:
  friend class AsIfFlatRegion;
  friend class Region;

  FlatRegion &operator= (const FlatRegion &other);

  mutable bool m_is_merged;
  mutable bool m_is_merged_min_coherence;
  mutable tl::copy_on_write_ptr<db::Shapes> mp_polygons;
  mutable tl::copy_on_write_ptr<db::Shapes> mp_merged_polygons;
  mutable bool m_merged_polygons_valid;
  mutable bool m_merged_polygons_min_coherence;
  double m_area_ratio;
  size_t m_max_vertex_count;

  void init ();
  void ensure_merged_polygons_valid () const;
  void ensure_unmerged_polygons_valid () const;

  void set_area_ratio (double ar)
  {
    m_area_ratio = ar;
  }

  void set_max_vertex_count (size_t mc)
  {
    m_max_vertex_count = mc;
  }

  template <class Trans>
  void transform_generic (const Trans &trans)
  {
    if (! trans.is_unity ()) {
      db::Shapes &polygons = *mp_polygons;
      for (polygon_iterator_type p = polygons.get_layer<db::Polygon, db::unstable_layer_tag> ().begin (); p != polygons.get_layer<db::Polygon, db::unstable_layer_tag> ().end (); ++p) {
        polygons.get_layer<db::Polygon, db::unstable_layer_tag> ().replace (p, p->transformed (trans));
      }
      for (polygon_iterator_wp_type p = polygons.get_layer<db::PolygonWithProperties, db::unstable_layer_tag> ().begin (); p != polygons.get_layer<db::PolygonWithProperties, db::unstable_layer_tag> ().end (); ++p) {
        polygons.get_layer<db::PolygonWithProperties, db::unstable_layer_tag> ().replace (p, p->transformed (trans));
      }
      invalidate_cache ();
    }
  }
};

}

#endif

