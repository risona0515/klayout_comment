
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



#ifndef HDR_dbReader
#define HDR_dbReader

#include "dbCommon.h"

#include "tlException.h"
#include "tlInternational.h"
#include "tlString.h"

#include "tlStream.h"
#include "dbLoadLayoutOptions.h"

#include <vector>

namespace db
{

class Layout;
class ReaderBase;

/**
 *  @brief Generic base class of reader exceptions
 */
class DB_PUBLIC ReaderException
  : public tl::Exception 
{
public:
  ReaderException (const std::string &msg)
    : tl::Exception (msg)
  { }
};

/**
 *  @brief A class representing the "unknown format" reader error
 *
 *  The purpose of this class is supply the header bytes of the
 *  data stream for analysis.
 */
class DB_PUBLIC ReaderUnknownFormatException
  : public ReaderException
{
public:
  ReaderUnknownFormatException (const std::string &msg, const std::string &data, bool has_more)
    : ReaderException (msg), m_data (data), m_has_more (has_more)
  { }

  const std::string &data () const
  {
    return m_data;
  }

  bool has_more () const
  {
    return m_has_more;
  }

private:
  std::string m_data;
  bool m_has_more;
};

/**
 *  @brief Joins layer names into a single, combined layer
 *  @param s The first layer name and output
 *  @param n The name to add
 */
DB_PUBLIC void
join_layer_names (std::string &s, const std::string &n);

/**
 *  @brief The generic reader base class
 */
// [[ZH-BEGIN]]
// ============================================================================
//  dbReader.h —— 读入版图（回答“文件怎么变成 Layout”）
// ============================================================================
//
// 【本文件的三层结构（由下到上理解）】
//   1) **异常类**（文件开头）
//        ReaderException              读取出错的基类异常
//        ReaderUnknownFormatException ★ “无法识别格式”专用异常
//      ★ 后者尤其值得注意：它让调用方能**区分“格式不认识”与“文件内容损坏”**。
//        这在“按扩展名猜格式”或“遍历多种格式尝试读取”时是必需的。
//
//   2) **ReaderBase** —— 各格式 reader 的抽象基类（此注释所在处）
//        规定“一个 reader 必须能做什么”（读入、报错、给进度…）。
//        具体实现由各格式插件提供（见 dbStream.h 的 StreamFormatDeclaration）。
//
//   3) **Reader** —— ★ **面向使用者的统一入口**（本文件主体）
//        你给它一个流/文件名，它自己去已注册的格式里挑一个能处理的。
//        ★ 这意味着**你不必知道文件是什么格式** —— 由内容探测决定。
//
// 【★★ 使用 Reader 的典型流程与两个必知行为】
//   典型：
//       tl::InputStream s ("x.gds");   // 1. 先打开流
//       db::Reader r (s);              // 2. 构造 → 探测格式（可能抛异常）
//       db::Layout ly;                 // 3. 目标 Layout
//       r.read (ly);                   // 4. 读入
//
//   ★ 行为一：**构造时就会探测格式，探测失败即抛异常**。
//     上游英文注释原文：
//       "If no valid format can be detected, the constructor will throw an exception.
//        The stream must be opened already in order to allow format detection."
//     → 所以：**流必须先打开**才能构造 Reader；
//     → 并且应把构造放在 try/catch 里，或先用探测接口判断。
//
//   ★ 行为二：read() **只往 Layout 里“插入”对象**，不做额外处理。
//     原文："This will not do much on the layout object beside inserting the objects."
//     → 即：read 之后 Layout 只包含**读到的原始内容**；
//     → 若需要“规范化”（乱序层、缺失 bbox 等），要自己再做。
//     → 这也解释了为什么导入后常常还要再跑一次合并/清理。
//
// 【★ 选项（options）—— 读取不是“无参数”的操作】
//   本文件还管理各格式的**读取选项**，例如：
//     · 层映射（layer map）：把文件里的层对应到逻辑层；
//     · 是否创建 PCell（而非展开成静态几何）等。
//   选项通过 db::LoadLayoutOptions 一类的对象传入，并可按格式分别设置。
//   ★ 实用影响：**同一个文件用不同选项读入，结果可以不同**。
//     因此“读进来的东西不符合预期”时，先检查选项，而不是怀疑文件。
//
// 【与写出的对称性】
//   写出见 dbWriter.h：结构完全对称（Writer 探测格式 → 写 Layout）。
//   格式的注册见 dbStream.h。
// [[ZH-END]]
class DB_PUBLIC ReaderBase
{
public:
  // [[ZH]] 功能：ReaderBase 构造 —— 各格式 reader 的基类实例化入口。
  ReaderBase ();
  virtual ~ReaderBase ();

  virtual const db::LayerMap &read (db::Layout &layout, const db::LoadLayoutOptions &options) = 0;
  virtual const db::LayerMap &read (db::Layout &layout) = 0;
  virtual const char *format () const = 0;

  /**
   *  @brief Sets a flag indicating that warnings shall be treated as errors
   *  If this flag is set, warnings will be handled as errors.
   */
  void set_warnings_as_errors (bool f);

  /**
   *  @brief Gets a flag indicating that warnings shall be treated as errors
   */
  bool warnings_as_errors () const
  {
    return m_warnings_as_errors;
  }

  /**
   *  @brief Gets the warning level
   */
  int warn_level () const
  {
    return m_warn_level;
  }

  /**
   *  @brief Returns true (once) if this is the first warning
   */
  bool first_warning ();

  /**
   *  @brief Returns a value indicating whether to compress the given warning
   *
   *  The return value is either -1 (do not skip), 0 (first warning not to be shown), 1 (warning not shown(.
   */
  int compress_warning (const std::string &msg);

  /**
   *  @brief Sets the expected database unit
   *
   *  With this value set, the reader can check if the present database unit is
   *  compatible with the expected one and either take actions to scale the layouts
   *  or to reject the file.
   *
   *  Setting the value to 0 resets the expected DBU and will disable all checks
   *  or scaling.
   */
  void set_expected_dbu (double dbu);

  /**
   *  @brief Gets the expected database unit
   */
  double expected_dbu () const
  {
    return m_expected_dbu;
  }

  /**
   *  @brief Checks the given DBU against the expected one
   *
   *  This method will raise an exception if the database units do not match.
   */
  void check_dbu (double dbu) const;

protected:
  virtual void init (const db::LoadLayoutOptions &options);

private:
  bool m_warnings_as_errors;
  int m_warn_level;
  std::string m_last_warning;
  int m_warn_count_for_same_message;
  bool m_first_warning;
  double m_expected_dbu;
};

/**
 *  @brief The generic stream reader 
 *
 *  This reader is supposed to fork to one of the specific readers
 *  depending on the format detected.
 */
class DB_PUBLIC Reader
{
public: 
  /**
   *  @brief Construct a reader object
   *
   *  If no valid format can be detected, the constructor will throw 
   *  an exception. The stream must be opened already in order to allow
   *  format detection.
   *
   *  @param s The stream object from which to read stream data from
   */
  Reader (tl::InputStream &s);

  /**  
   *  @brief Destructor
   */
  ~Reader ();

  /** 
   *  @brief The basic read method 
   *
   *  This method will read the stream data and translate this to
   *  insert calls into the layout object. This will not do much
   *  on the layout object beside inserting the objects.
   *  It can be passed options with a layer map which tells which
   *  OASIS layer(s) to read on which logical layers.
   *  In addition, a flag can be passed that tells whether to create 
   *  new layers. The returned map will contain all layers, the passed
   *  ones and the newly created ones.
   *
   *  @param layout The layout object to write to
   *  @param options The LayerMap object
   */
  const db::LayerMap &read (db::Layout &layout, const db::LoadLayoutOptions &options);

  /** 
   *  @brief The basic read method (without mapping)
   *
   *  This method will read the stream data and translate this to
   *  insert calls into the layout object. This will not do much
   *  on the layout object beside inserting the objects.
   *  This version will read all input layers and return a map
   *  which tells which OASIS layer has been read into which logical
   *  layer.
   *
   *  @param layout The layout object to write to
   *  @return The LayerMap object
   */
  const db::LayerMap &read (db::Layout &layout);

  /**
   *  @brief Returns a format describing the file format found
   */
  const char *format () const
  {
    return mp_actual_reader->format ();
  }

  /**
   *  @brief Sets a flag indicating that warnings shall be treated as errors
   *  If this flag is set, warnings will be handled as errors.
   */
  void set_warnings_as_errors (bool f)
  {
    mp_actual_reader->set_warnings_as_errors (f);
  }

  /**
   *  @brief Gets a flag indicating that warnings shall be treated as errors
   */
  bool warnings_as_errors () const
  {
    return mp_actual_reader->warnings_as_errors ();
  }

  /**
   *  @brief Sets the expected database unit (see ReaderBase)
   */
  void set_expected_dbu (double dbu)
  {
    return mp_actual_reader->set_expected_dbu (dbu);
  }

  /**
   *  @brief Gets the expected database unit
   */
  double expected_dbu () const
  {
    return mp_actual_reader->expected_dbu ();
  }

private:
  ReaderBase *mp_actual_reader;
  tl::InputStream &m_stream;
};

}

#endif

