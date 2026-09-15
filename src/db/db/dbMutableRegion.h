
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


#ifndef HDR_dbMutableRegion
#define HDR_dbMutableRegion

#include "dbCommon.h"

#include "dbAsIfFlatRegion.h"

#include <set>

namespace db {

/**
 *  @brief An interface representing mutable regions
 *
 *  Mutable regions offer insert, transform, flatten and other manipulation functions.
 */
// [[ZH-BEGIN]]
// 功能：★ **可修改区域的接口** —— 在 AsIfFlatRegion 之上增加“写”能力。
//
// 【上面英文注释已点明】
//   "Mutable regions offer insert, transform, flatten and other manipulation functions."
//   即：它额外提供 **插入 / 变换 / 平铺 (flatten) / 其它就地修改** 操作。
//
// 【★ 在继承链中的位置（理解整族的钥匙）】
//     RegionDelegate          抽象接口（能做什么）
//       └─ AsIfFlatRegion     默认的“平铺”行为（只读视角）
//            └─ MutableRegion ★ 本类：增加写能力
//                 ├─ FlatRegion   写的就是本地展开数据
//                 └─ DeepRegion   写的是层次结构
//   ★ 所以要区分三个层次的能力：
//     · RegionDelegate：抽象契约；
//     · AsIfFlatRegion：**能当作平铺来读**（即使底数据是层次也按需展开）；
//     · MutableRegion ：**能改**。
//
// 【★ “可修改”与“写时复制”的关系（重要）】
//   注意：拿到本接口**并不等于**可以随意改共享数据。
//   门面（Region）在需要修改时会先确保自己**独占**底层实现（可能要 clone），
//   否则改一个会影响到共享同一实现的其他 Region。
//   ★ 这就是为什么外部看到的很多修改操作**返回新对象**（见 dbRegionDelegate.h 的 clone 说明）。
//
// 【本文件很小（仅 109 行）的原因】
//   它主要是一个**接口声明**（声明 insert/transform/flatten 等），
//   具体实现在 FlatRegion / DeepRegion 里。
// [[ZH-END]]
class DB_PUBLIC MutableRegion
  : public AsIfFlatRegion
{
public:
  MutableRegion ();
  MutableRegion (const MutableRegion &other);
  virtual ~MutableRegion ();

  virtual void do_insert (const db::Polygon &polygon, db::properties_id_type prop_id) = 0;

  void transform (const db::UnitTrans &) { }
  void transform (const db::Disp &t) { do_transform (db::Trans (t)); }
  void transform (const db::Trans &t) { do_transform (t); }
  void transform (const db::ICplxTrans &t) { do_transform (t); }
  void transform (const db::IMatrix2d &t) { do_transform (t); }
  void transform (const db::IMatrix3d &t) { do_transform (t); }

  virtual void do_transform (const db::Trans &t) = 0;
  virtual void do_transform (const db::ICplxTrans &t) = 0;
  virtual void do_transform (const db::IMatrix2d &t) = 0;
  virtual void do_transform (const db::IMatrix3d &t) = 0;

  virtual void flatten () = 0;

  virtual void reserve (size_t n) = 0;

  void insert (const db::Polygon &polygon) { do_insert (polygon, 0); }
  void insert (const db::PolygonWithProperties &polygon) { do_insert (polygon, polygon.properties_id ()); }
  void insert (const db::Box &box);
  void insert (const db::BoxWithProperties &box);
  void insert (const db::Path &path);
  void insert (const db::PathWithProperties &path);
  void insert (const db::SimplePolygon &polygon);
  void insert (const db::SimplePolygonWithProperties &polygon);

  void insert (const db::Shape &shape);

  template <class T>
  void insert (const db::Shape &shape, const T &trans)
  {
    if (shape.is_polygon () || shape.is_path () || shape.is_box ()) {
      db::Polygon poly;
      shape.polygon (poly);
      poly.transform (trans);
      do_insert (poly, shape.prop_id ());
    }
  }

  template <class Iter>
  void insert (const Iter &b, const Iter &e)
  {
    reserve (count () + (e - b));
    for (Iter i = b; i != e; ++i) {
      insert (*i);
    }
  }

  template <class Iter>
  void insert_seq (const Iter &seq)
  {
    for (Iter i = seq; ! i.at_end (); ++i) {
      insert (*i);
    }
  }
};

}

#endif

