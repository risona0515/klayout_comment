
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

#ifndef _HDR_dbShapeCollection
#define _HDR_dbShapeCollection

#include "dbCommon.h"
#include "dbDeepShapeStore.h"
#include "tlUniqueId.h"
#include "tlVariant.h"
#include "gsiObject.h"

namespace db
{

// [[ZH-BEGIN]]
// ============================================================================
//  dbShapeCollection.h —— “图形集合”的**委托（delegate）基础设施**
// ============================================================================
//
// 【★ 这个文件为什么值得单独读（它是理解 dbRegion/Edges/EdgePairs/Texts 的钥匙）】
//   库里的高层几何集合（Region、Edges、EdgePairs、Texts...）都采用了同一种架构：
//       **门面 + 委托 + 多个具体实现**
//   而本文件正是这套架构的**地基**（那两个 DelegateBase 基类）。
//   看懂本文件，再看 dbRegion.h 那一大家子 `db*Region*` 就会豁然开朗。
//
// 【★★ 委托模式在这里长什么样（务必理解）】
//       db::Region            ← 门面：用户用的小对象（只持有一个指针）
//          └─ mp_delegate → db::RegionDelegate   ← 抽象委托接口（本文件的思想来源）
//                 ├─ db::FlatRegion        平铺实现（数据真的在这）
//                 ├─ db::DeepRegion        层次化实现（保留层次，不展开）
//                 ├─ db::EmptyRegion       空实现（惰性、零数据）
//                 ├─ db::MutableRegion     可修改包装
//                 └─ ...（同构的还有 AsIfFlat / OriginalLayer 等）
//
//   为什么要这么设计 —— 因为**同一个几何集合可以用完全不同的方式实现**：
//     · 平铺 (Flat)：把层次展开成一个大集合，操作简单、内存大；
//     · 深层 (Deep)：保留层次结构，内存小、但操作复杂；
//     · 空 (Empty)：表示“还没有数据”，避免过早分配；
//     · 可修改 (Mutable)：写时复制/惰性求值的包装。
//   而**用户代码不应该关心用的是哪一种** —— 因此用一个门面（Region）
//   + 一个抽象接口（Delegate）把差异藏起来。
//   ★ 这正是“策略模式/桥接模式”的典型用法：换实现不改调用方。
//
// 【两个 DelegateBase 的分工】
//   ShapeCollectionDelegateBase  —— 一般图形集合委托的基类，
//       继承 tl::UniqueId 从而有一个**唯一标识**（m_data_id）。
//       这个 id 的作用：判断两个门面是否共享同一个底层实现
//       （用于写时复制、缓存键、判等优化），避免误改共享数据。
//   DeepShapeCollectionDelegateBase —— **深层（层次化）**委托的基类，
//       多持有一个 db::DeepLayer（指向深层数据结构）。
//       只有 Deep 系实现才用它。
//
// 【★ 一个容易踩的坑：门面共享同一委托】
//   因为门面小且可以拷贝，**两个 Region 可能指向同一个委托**。
//   因此修改前通常需要“脱离共享”（copy-on-write）——
//   也正因如此，修改操作常常返回**新对象**而不是就地改。
//   这也是本库大量“xxx() 返回新对象”而非“xxx() 就地修改”的原因。
//
// 【属性和仓库】注意文件开头的前向声明（PropertiesTranslator /
//   PropertiesRepository）—— 委托接口需要它们来支持“复制时迁移属性”
//   （见 dbPropertiesRepository.h 的说明）。
// [[ZH-END]]
class PropertiesTranslator;
class PropertiesRepository;

/**
 *  @brief A base class for the deep collection delegates
 */
// [[ZH]] 功能：**深层（层次化）委托**的基类 —— 供 Deep 系实现继承（见文件头说明）。
// [[ZH]] 与基类的差别：多持有一个 db::DeepLayer（指向深层数据结构）。
// [[ZH]] 参数/修改：apply_property_translator 用于在复制时把属性 ID 翻译到目标仓库
// [[ZH]]                 （见 dbPropertiesRepository.h 的 PropertiesTranslator）。
// [[ZH]] 设计点：set_deep_layer 是 protected virtual —— 子类可介入“设置深层层”的动作，
// [[ZH]]       这样 Deep 系实现能在层变更时维护自己的不变量。
class DB_PUBLIC DeepShapeCollectionDelegateBase
{
public:
  DeepShapeCollectionDelegateBase ();
  DeepShapeCollectionDelegateBase (const DeepShapeCollectionDelegateBase &other);

  DeepShapeCollectionDelegateBase &operator= (const DeepShapeCollectionDelegateBase &other);

  const db::DeepLayer &deep_layer () const
  {
    return m_deep_layer;
  }

  db::DeepLayer &deep_layer ()
  {
    return m_deep_layer;
  }

  void apply_property_translator (const db::PropertiesTranslator &pt);

protected:
  virtual void set_deep_layer (const db::DeepLayer &dl)
  {
    m_deep_layer = dl;
  }

private:
  db::DeepLayer m_deep_layer;
};

/**
 *  @brief A base class for the shape collection delegates
 */
// [[ZH-BEGIN]]
// 功能：★ **图形集合委托的基类** —— 所有集合实现的抽象接口（见文件头说明）。
//
// ★ 关键点：它继承 **tl::UniqueId**，因此每个委托实例有一个唯一标识 m_data_id。
//   这个 id 的用处：
//     · 判断两个门面对象是否**共享同一个底层实现**
//       （用于写时复制、缓存键、快速判等）；
//     · 避免误改共享数据（两个 Region 指向同一委托时，改一个会影响另一个）。
//   ★ 这就是为什么库中很多修改操作**返回新对象**而不是就地改：
//     为了不与其它门面共享的数据发生冲突（copy-on-write）。
//
// 构造：初始化 m_data_id = tl::id_of (this)，即“本对象的唯一 id”。
//
// 提示：具体能力（区域布尔运算 / 边运算 / …）由派生类定义，本基类只负责
//   “有个稳定身份”与“共同接口”。
// [[ZH-END]]
class DB_PUBLIC ShapeCollectionDelegateBase
  : public tl::UniqueId
{
public:
  ShapeCollectionDelegateBase ()
    : m_data_id (tl::id_of (this))
  {
    //  .. nothing yet ..
  }

  virtual ~ShapeCollectionDelegateBase () { }

  virtual DeepShapeCollectionDelegateBase *deep () { return 0; }

  virtual void apply_property_translator (const db::PropertiesTranslator & /*pt*/) = 0;

  void remove_properties (bool remove = true)
  {
    if (remove) {
      apply_property_translator (db::PropertiesTranslator::make_remove_all ());
    }
  }

  tl::id_type data_id () const
  {
    return m_data_id;
  }

private:
  friend class ShapeCollection;

  tl::id_type m_data_id;

  //  used for conversion to deep
  void set_data_id (tl::id_type data_id)
  {
    m_data_id = data_id;
  }
};

/**
 *  @brief A base class for the shape collections such as Region, Edges, EdgePairs etc.
 */
class DB_PUBLIC ShapeCollection
  : public gsi::ObjectBase
{
public:
  ShapeCollection () { }
  virtual ~ShapeCollection () { }

  virtual ShapeCollectionDelegateBase *get_delegate () const = 0;

  /**
   *  @brief Converts the shape collection to a deep one using the specified layer
   */
  virtual void convert_to_deep (const db::DeepLayer &layer) = 0;

  /**
   *  @brief Applies a PropertyTranslator
   *
   *  This method will translate the property IDs according to the given property translator.
   *
   *  Note that the property translator needs to be built from the PropertiesRepository
   *  delivered by "properties_repository".
   */
  void apply_property_translator (const db::PropertiesTranslator &pt);

protected:
  template <class Delegate>
  Delegate *copy_data_id (Delegate *dlg)
  {
    dlg->set_data_id (get_delegate ()->data_id ());
    return dlg;
  }
};

}

#endif
