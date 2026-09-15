
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


#ifndef HDR_dbWriter
#define HDR_dbWriter

#include "dbCommon.h"

#include "tlException.h"
#include "dbSaveLayoutOptions.h"

namespace tl 
{
  class OutputStream;
  class OutputStreamBase;
}

namespace db
{

class Layout;

/**
 *  @brief The generic writer base class
 */
// [[ZH-BEGIN]]
// ============================================================================
//  dbWriter.h —— 写出/导出（与 dbReader.h 完全对称）
// ============================================================================
//
// 【本文件的三个类（与 Reader 一一对应）】
//   WriterBase  —— 各格式 writer 的抽象基类（规定“必须能写出”）
//   Writer      —— ★ **面向使用者的统一入口**：你指定格式与目标，它写出去
//   （还有 WriterException 等异常类型）
//
// 【★★ 与 Reader 的关键不对称：写出**必须指定格式**】
//   读取时可以**探测**（Reader 构造时自动识别格式）；
//   但写出时**无法探测** —— 必须由你告知要写哪种格式，因为同一份数据
//   可以用 GDS2 / OASIS / DXF ... 任一格式写，内容与能力都不同。
//   ★ 因此 Writer 的构造/写入通常需要显式格式名或选择对象。
//   实用影响：
//     · 写 OASIS 可保留 PCell 参数与更多信息；
//     · 写 GDS2 则可能**丢失** OASIS 才有的特性（如部分属性、PCell 参数）。
//     因此“存成哪种格式”是一个有后果的选择（见 dbStream.h 的 supports_context）。
//
// 【★ 写出选项（options）同样重要】
//   由 db::SaveLayoutOptions（本文件包含的头）承载，例如：
//     格式相关的版本/参数、层映射、是否压缩/是否写出文本图形等。
//   ★ 与读取一样：**同一份 Layout 用不同选项写，结果文件不同**。
//     所以“导出的文件别人打不开/丢东西”时，先检查选项。
//
// 【is_valid / 是否可用】
//   注意本文件有一个“对该格式是否真的有可用 writer”的查询 ——
//   因为并非所有格式都支持写出（有些只读）。
//   ★ 写代码时应先查它，而不是假定一定能写（否则会拿到空/异常）。
//
// 【典型使用流程】
//       db::SaveLayoutOptions opt;        // 配置选项
//       opt.set_format (\"OASIS\");         // 指定格式（写出时必须指定）
//       tl::OutputStream os (\"out.oas\"); // 打开输出流
//       db::Writer w (os, opt);           // 构造 writer
//       w.write (layout);                 // 写出
// [[ZH-END]]
class DB_PUBLIC WriterBase
{
public:
  /**
   *  @brief Constructor
   */
  // [[ZH]] 功能：WriterBase 构造 —— 各格式 writer 的基类实例化入口。
  WriterBase () { }

  /**
   *  @brief Destructor
   */
  virtual ~WriterBase () { }

  /**
   *  @brief Actually write the layout
   *  The layout is non-const since the writer may modify the meta information of the layout.
   */
  virtual void write (db::Layout &layout, tl::OutputStream &stream, const db::SaveLayoutOptions &options) = 0;
};

/**
 *  @brief A generic stream format writer
 */
class DB_PUBLIC Writer
{
public:
  typedef std::vector<MetaInfo> meta_info;
  typedef meta_info::const_iterator meta_info_iterator;

  /**
   *  @brief The constructor
   */
  Writer (const SaveLayoutOptions &options);

  /**
   *  @brief The destructor
   */
  ~Writer ();

  /**
   *  @brief The generic write method
   *  The layout is non-const since the writer may modify the meta information of the layout.
   */
  void write (db::Layout &layout, tl::OutputStream &stream);

  /**
   *  @brief True, if for this format a valid writer is provided
   */
  bool is_valid () const
  {
    return mp_writer != 0;
  }

private:
  WriterBase *mp_writer;
  db::SaveLayoutOptions m_options;
};

}

#endif

