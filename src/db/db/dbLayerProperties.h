
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


#ifndef HDR_dbLayerProperties
#define HDR_dbLayerProperties

#include "dbCommon.h"

#include "tlString.h"
#include "tlTypeTraits.h"

#include <string>

namespace db
{

/**
 *  @brief A layer property
 *
 *  The layer properties are basically to be used for storing of layer name and 
 *  layer/datatype information.
 *
 *  A special use case is for the target of a layer mapping specification.
 *  In this case, the layer properties can make use of the relative
 *  layer/datatype specifications.
 */
// [[ZH-BEGIN]]
// 功能：★ **层的标识与名称** —— “这一层是什么”（层号/数据类型/名字）的载体。
//
// 【它解决什么问题】
//   版图里一层由 GDS 的 (layer, datatype) 这一对整数标识；
//   但人需要读得懂的名字（如 "M1"、"poly"）。
//   LayerProperties 就是把这两者绑在一起的类型：
//       layer       —— 层号（整数，GDS 的 layer）
//       datatype    —— 数据类型（整数，GDS 的 datatype）
//       name        —— 人类可读的名字（可为空）
//       (还可能带一个 layer offset，见下面 LayerOffset)
//   ★ 注意：它**只是描述**，不含图形本身 —— 内容在 db::Shapes 里（见 dbShapes.h）。
//
// 【★★ 一个关键概念：相对层规格（relative specifications）—— 容易忽略】
//   上面英文注释特别点出了“目标(target)用途”下的相对规格。
//   含义：在**层映射 (layer mapping)** 场景中，目标层可以写成
//       “相对于源层偏移 +1”这种形式（如 `*+1`），而不是绝对层号。
//   ★ 为什么需要：写 DRC/LVS 的层映射规则时，希望“所有层的目标层都等于源层+1”，
//     而不必逐层硬编码。
//   实用影响：解析字符串时可能需要 `as_target = true` 才允许相对写法，
//     因此“同一字符串按源解析与按目标解析可能不同”。
//
// 【与 db::Layout 的关系】
//   Layout 持有一张层表（layer → LayerProperties）。
//   ★ 重要：**图形是用 layer index 索引的，不是用层号**。
//     所以典型流程是：用层号/层名查出 layer index，再用 index 取该层的 Shapes。
//     （见 dbLayout.h 的说明与 dbShapes.h 的说明。）
//
// 【本文件其余内容】
//   LayerPropertiesLessThan          —— 用于排序/作为 map 键的“逻辑比较”
//   LayerOffset                      —— 层的偏移量（用于层映射时做整体平移）
//   LayerPropertiesWithOffset        —— 把 LayerProperties 与偏移组合起来
//   末尾的 extractor 特化              —— 注册文本解析（同 dbPoint.cc 的模式）
// [[ZH-END]]
struct DB_PUBLIC LayerProperties
{
  /**
   *  @brief Default constructor
   */
  LayerProperties ();
  
  /**
   *  @brief Constructor with layer and datatype
   */
  LayerProperties (int l, int d);
  
  /**
   *  @brief Constructor with name
   */
  LayerProperties (const std::string &n);
  
  /**
   *  @brief Constructor with layer and datatype and name
   */
  LayerProperties (int l, int d, const std::string &n);

  /**
   *  @brief Returns true, if the layer specification is not a null specification
   *
   *  A null specification is one created by the default constructor. It does not have
   *  a layer, datatype or name assigned.
   */
  bool is_null () const;
  
  /**
   *  @brief Return true, if the layer is specified by name only
   */
  bool is_named () const;

  /**
   *  @brief Convert to a string
   */
  std::string to_string (bool as_target = false) const;

  /**
   *  @brief Extract from a tl::Extractor
   *
   *  With "with_relative" true, the extractor allows giving
   *  relative layer/datatype specifications in the format "*+1" or "*-100".
   *  "*" for layer or datatype is for "don't care" (on input) or "leave as is"
   *  (for output).
   *
   *  Returns true, if a layer was read successfully.
   */
  bool read (tl::Extractor &ex, bool as_target = false);

  /**
   *  @brief "Logical" equality
   *
   *  This currently reflects only equality of layers and datatypes, name is of second order
   *  and used only if no layer or datatype is given
   */
  bool log_equal (const LayerProperties &b) const;

  /**
   *  @brief "Logical" less operator
   *
   *  This currently reflects only order of layer and datatype, name is of second order
   */
  bool log_less (const LayerProperties &b) const;

  /**
   *  @brief Exact equality
   */
  bool operator== (const LayerProperties &b) const;

  /**
   *  @brief Exact inequality
   */
  bool operator!= (const LayerProperties &b) const;

  /**
   *  @brief Exact less operator
   */
  bool operator< (const LayerProperties &b) const;

  std::string name;
  int layer;
  int datatype;
};

/**
 *  @brief "Logical less" functor for LayerProperties
 */
struct LPLogicalLessFunc 
{
  bool operator () (const LayerProperties &a, const LayerProperties &b) const
  {
    return a.log_less (b);
  }
};

/**
 *  @brief A layer offset 
 *
 *  This struct defines a layer offset which can be "added" to a LayerProperties object
 *  If the layer offset is defined with a name, any occurrence of '*' in the string
 *  is replaced with the original name. This way, applying "*_A" with "+" yields a 
 *  postfix "_A" to the original layer name (if it is named).
 */
struct DB_PUBLIC LayerOffset
{
  /**
   *  @brief Default constructor
   */
  LayerOffset ();
  
  /**
   *  @brief Constructor with layer and datatype
   */
  LayerOffset (int l, int d);
  
  /**
   *  @brief Constructor with name
   */
  LayerOffset (const std::string &n);
  
  /**
   *  @brief Constructor with layer and datatype and name
   */
  LayerOffset (int l, int d, const std::string &n);
  
  /**
   *  @brief Return true, if the layer is specified by name only
   */
  bool is_named () const;

  /**
   *  @brief Convert to a string
   */
  std::string to_string () const;

  /**
   *  @brief Extract from a tl::Extractor
   */
  void read (tl::Extractor &ex);

  /**
   *  @brief Exact equality
   */
  bool operator== (const LayerOffset &b) const;

  /**
   *  @brief Exact inequality
   */
  bool operator!= (const LayerOffset &b) const;

  /**
   *  @brief Exact less operator
   */
  bool operator< (const LayerOffset &b) const;

  /**
   *  @brief Apply to a LayerProperties object
   */
  LayerProperties apply (const LayerProperties &props) const;

  std::string name;
  int layer;
  int datatype;
};

/**
 *  @brief Apply a LayerOffset to a LayerProperties object
 */
inline LayerProperties operator+ (const LayerProperties &props, const LayerOffset &offset)
{
  return offset.apply (props);
}

/**
 *  @brief Apply a LayerOffset to a LayerProperties itself
 */
inline LayerProperties &operator+= (LayerProperties &props, const LayerOffset &offset)
{
  props = offset.apply (props);
  return props;
}

}

/**
 *  @brief Special extractors for LayerProperties and LayerOffset
 */
namespace tl
{
  template<> DB_PUBLIC void extractor_impl<db::LayerProperties> (tl::Extractor &ex, db::LayerProperties &p);
  template<> DB_PUBLIC void extractor_impl<db::LayerOffset> (tl::Extractor &ex, db::LayerOffset &p);

  template<> DB_PUBLIC bool test_extractor_impl<db::LayerProperties> (tl::Extractor &ex, db::LayerProperties &p);
  template<> DB_PUBLIC bool test_extractor_impl<db::LayerOffset> (tl::Extractor &ex, db::LayerOffset &p);
} // namespace tl

#endif

