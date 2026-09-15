
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



#ifndef HDR_dbStream
#define HDR_dbStream

#include "dbCommon.h"

#include "dbSaveLayoutOptions.h"
#include "dbLoadLayoutOptions.h"

#include "tlClassRegistry.h"
#include "tlXMLParser.h"
#include "tlXMLWriter.h"

#include <string>
#include <vector>

namespace tl
{
  class InputStream;
}

namespace db
{

class ReaderBase;
class WriterBase;

// [[ZH-BEGIN]]
// 功能：★ **文件格式的注册声明** —— 这是整个 I/O 体系的“插件点”。
//
// 【★★ 本类的角色：库与具体格式之间的接口】
//   本库支持 GDS2 / OASIS / DXF / CIF / LEF-DEF / MAG / Gerber ... 多种格式。
//   但 db 核心**不应硬编码**任何一种 —— 否则加一个格式就要改核心。
//   于是设计成本类：**每种格式提供一个 StreamFormatDeclaration 的派生类**，
//   向库注册“我是谁、我怎么读、我怎么写”。
//   ★ 具体格式的实现在插件目录里，例如：
//       src/plugins/streamers/gds2/   → GDS2 的 reader/writer
//       src/plugins/streamers/oasis/  → OASIS 的 reader/writer
//       src/plugins/streamers/lefdef/ → LEF/DEF
//   而 db::Reader / db::Writer（见 dbReader.h / dbWriter.h）则是**面向使用者的
//   统一入口**：你给文件名，它们去已注册的格式里挑一个能处理的。
//
// 【★★ 六个纯虚函数 = 一个格式必须回答的六个问题】
//   format_name ()       —— 格式的短名（如 "GDS2"），用于按名指定格式。
//   format_desc ()       —— 可读描述，用于界面/错误信息。
//   create_reader ()     —— 造一个能读该格式的 reader，**不支持则返回 0**。
//   can_read ()          —— 是否支持读（有些格式只写不读）。
//   create_writer ()     —— 造一个 writer；**不支持则返回 0**。
//   can_write ()         —— 是否支持写。
//   supports_context ()  —— ★ 是否能携带“上下文”（PCell 参数、库引用等）。
//                          含义见下。
//   reader_options_xml_element () —— 提供“读取选项”在技术 XML 里的表示。
//
// 【★ supports_context 是什么意思（容易被忽略但很重要）】
//   有些格式（如 OASIS、原生 XML）不止存几何，还能存**单元的参数化信息**
//   （PCell 参数）与**跨库引用**。这类“超出几何本身的信息”就是 context。
//   ★ 实用影响：
//     · 若格式不支持 context，则 PCell 会被**展开成静态几何**（参数信息丢失）；
//     · 因此“存成哪种格式”会影响**能否保留参数化单元**。
//     这是 GDS2 与 OASIS 的一个重要差别来源。
//
// 【★ 由于全是纯虚函数，本类必须被继承】
//   你**不能直接实例化** StreamFormatDeclaration —— 它只是一份契约。
//   加新格式的步骤就是：写一个派生类实现这六个函数，然后注册它。
//
// 【本文件其余内容】
//   ReaderOptionsXMLElement / 相关辅助 —— 把各格式的读取选项接入技术的 XML 持久化
//   db::ReaderBase / WriterBase            —— reader/writer 的具体基类（在其它头文件）
// [[ZH-END]]
/**
 *  @brief A stream format declaration
 */
class DB_PUBLIC StreamFormatDeclaration 
{
public:
  /**
   *  @brief Constructor
   */
  StreamFormatDeclaration () { }

  /**
   *  @brief Destructor
   */
  virtual ~StreamFormatDeclaration () { }

  /**
   *  @brief Obtain the format name
   */
  virtual std::string format_name () const = 0;

  /**
   *  @brief Obtain the format description
   */
  virtual std::string format_desc () const = 0;

  /**
   *  @brief Obtain the (long) format description
   */
  virtual std::string format_title () const = 0;

  /**
   *  @brief Obtain the file dialog format contribution
   */
  virtual std::string file_format () const = 0;

  /**
   *  @brief Auto-detect this format from the stream
   */
  virtual bool detect (tl::InputStream &s) const = 0;

  /**
   *  @brief Create the reader
   */
  virtual ReaderBase *create_reader (tl::InputStream &s) const = 0;

  /**
   *  @brief Returns true, when the format supports import (has a reader)
   */
  virtual bool can_read () const = 0;

  /**
   *  @brief Create the reader
   */
  virtual WriterBase *create_writer () const = 0;

  /**
   *  @brief Returns true, when the format supports export (has a writer)
   */
  virtual bool can_write () const = 0;

  /**
   *  @brief Returns true, when the format supports a context (e.g. PCell parameters, Library references)
   */
  virtual bool supports_context () const = 0;

  /**
   *  @brief Delivers the XMLElement object that represents the reader options within a technology XML tree
   *
   *  This method is supposed to return an instance ReaderOptionsXMLElement<RO> where RO is the
   *  specific reader options type. The return value can be 0 to indicate there is no specific reader
   *  option.
   *
   *  The returned XMLElement is destroyed by the caller and needs to be a new object.
   */
  virtual tl::XMLElementBase *xml_reader_options_element () const
  {
    return 0;
  }

  /**
   *  @brief Delivers the XMLElement object that represents this writer options within a technology XML tree
   *
   *  This method is supposed to return an instance WriterOptionsXMLElement<WO> where WO is the
   *  specific writer options type. The return value can be 0 to indicate there is no specific writer
   *  option.
   *
   *  The returned XMLElement is destroyed by the caller and needs to be a new object.
   */
  virtual tl::XMLElementBase *xml_writer_options_element () const
  {
    return 0;
  }

  /**
   *  @brief Returns a string for the file dialogs that describes all formats
   */
  static std::string all_formats_string ();
};

/**
 *  @brief A helper class for the XML serialization of the stream options (custom read adaptor)
 *
 *  OPT is a reader or writer options class and HOST is the host class. For example, OPT
 *  can be db::GDS2ReaderOptions and HOST then is db::LoadLayoutOptions.
 */
template <class OPT, class HOST>
class StreamOptionsReadAdaptor
{
public:
  typedef tl::pass_by_ref_tag tag;

  StreamOptionsReadAdaptor ()
    : mp_options (0), m_done (false)
  {
    // .. nothing yet ..
  }

  const OPT &operator () () const
  {
    return mp_options->template get_options<OPT> ();
  }

  bool at_end () const
  {
    return m_done;
  }

  void start (const HOST &options)
  {
    mp_options = &options;
    m_done = false;
  }

  void next ()
  {
    mp_options = 0;
    m_done = true;
  }

private:
  const HOST *mp_options;
  bool m_done;
};

/**
 *  @brief A helper class for the XML serialization of the stream option (custom write adaptor)
 *
 *  See StreamOptionsReadAdaptor for details.
 */
template <class OPT, class HOST>
class StreamOptionsWriteAdaptor
{
public:
  StreamOptionsWriteAdaptor ()
  {
    // .. nothing yet ..
  }

  void operator () (HOST &options, tl::XMLReaderState &reader) const
  {
    std::unique_ptr<OPT> opt (new OPT ());

    tl::XMLObjTag<OPT> tag;
    *opt = *reader.back (tag);

    options.set_options (opt.release ());
  }
};

/**
 *  @brief A XMLElement specialization for stream options
 */
template <class OPT, class HOST>
class StreamOptionsXMLElement
  : public tl::XMLElement<OPT, HOST, StreamOptionsReadAdaptor<OPT, HOST>, StreamOptionsWriteAdaptor<OPT, HOST> >
{
public:
  StreamOptionsXMLElement (const std::string &element_name, const tl::XMLElementList &children)
    : tl::XMLElement<OPT, HOST, StreamOptionsReadAdaptor<OPT, HOST>, StreamOptionsWriteAdaptor<OPT, HOST> > (StreamOptionsReadAdaptor<OPT, HOST> (), StreamOptionsWriteAdaptor<OPT, HOST> (), element_name, children)
  {
    //  .. nothing yet ..
  }

  StreamOptionsXMLElement (const StreamOptionsXMLElement &d)
    : tl::XMLElement<OPT, HOST, StreamOptionsReadAdaptor<OPT, HOST>, StreamOptionsWriteAdaptor<OPT, HOST> > (d)
  {
    //  .. nothing yet ..
  }
};

/**
 *  @brief A custom XMLElement for the serialization of reader options
 *
 *  StreamReaderPluginDeclaration::xml_element can return such an element to
 *  insert a custom XML element into the XML tree which represents the
 *  reader options.
 */
template <class OPT>
class ReaderOptionsXMLElement
  : public StreamOptionsXMLElement<OPT, db::LoadLayoutOptions>
{
public:
  ReaderOptionsXMLElement (const std::string &element_name, const tl::XMLElementList &children)
    : StreamOptionsXMLElement<OPT, db::LoadLayoutOptions> (element_name, children)
  {
    //  .. nothing yet ..
  }

  ReaderOptionsXMLElement (const ReaderOptionsXMLElement &d)
    : StreamOptionsXMLElement<OPT, db::LoadLayoutOptions> (d)
  {
    //  .. nothing yet ..
  }

  virtual tl::XMLElementBase *clone () const
  {
    return new ReaderOptionsXMLElement (*this);
  }
};

/**
 *  @brief A custom XMLElement for the serialization of writer options
 *
 *  StreamWriterPluginDeclaration::xml_element can return such an element to
 *  insert a custom XML element into the XML tree which represents the
 *  writer options.
 */
template <class OPT>
class WriterOptionsXMLElement
  : public StreamOptionsXMLElement<OPT, db::SaveLayoutOptions>
{
public:
  WriterOptionsXMLElement (const std::string &element_name, const tl::XMLElementList &children)
    : StreamOptionsXMLElement<OPT, db::SaveLayoutOptions> (element_name, children)
  {
    //  .. nothing yet ..
  }

  WriterOptionsXMLElement (const WriterOptionsXMLElement &d)
    : StreamOptionsXMLElement<OPT, db::SaveLayoutOptions> (d)
  {
    //  .. nothing yet ..
  }

  virtual tl::XMLElementBase *clone () const
  {
    return new WriterOptionsXMLElement (*this);
  }
};

/**
 *  @brief Returns the XMLElement list that can represent a db::LoadLayoutOptions object
 */
DB_PUBLIC tl::XMLElementList load_options_xml_element_list ();

/**
 *  @brief Returns the XMLElement list that can represent a db::SaveLayoutOptions object
 */
DB_PUBLIC tl::XMLElementList save_options_xml_element_list ();

}

#endif


