
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


#ifndef HDR_dbLibraryDescriptor
#define HDR_dbLibraryDescriptor

#include "dbCommon.h"
#include "gsiObject.h"
#include "dbLayout.h"
#include "tlTypeTraits.h"
#include "tlObject.h"

#include <string>
#include <set>

namespace db
{

class Layout;

/**
 *  @brief A library
 *
 *  A library is basically a wrapper around a layout object.
 *  A library is additionally associated with an id, a name and a description.
 *  A library must provide a layout. This class does not specify how the layout 
 *  is provided. To do so, this class must be reimplemented.
 */
// [[ZH-BEGIN]]
// 功能：★ **库（library）** —— 对 Layout 的包装，并额外带上 id / 名字 / 描述。
//
// 【它解决什么问题（为什么不能直接用 Layout）】
//   一个 KLayout 会话里可能同时打开**很多个版图文件**，还有内置库、
//   技术库等。光有 Layout 不够，因为：
//     · 需要一个**稳定标识**（id）来引用它；
//     · 需要**名字与描述**用于界面显示与检索；
//     · 需要支持“**惰性加载/重新加载**”（库可能很大，不必立刻读进来）；
//     · 需要允许“库”的**内容来源**各不相同（文件 / 内存 / 数据库 / 远程...）。
//   本类因此把"“提供 Layout”这一动作抽象成**虚函数**，由子类实现。
//
// 【★★ 核心设计：它是抽象基类，必须被继承】
//   上面英文注释明确写着：
//     “A library must provide a layout. This class does not specify how the
//      layout is provided. To do so, this class must be reimplemented.”
//   ★ 即：本类**不能直接拿来用** —— 它只规定“库应该能提供 Layout”这个契约。
//     具体怎么提供（从文件读？从内存构造？）由派生类决定。
//   典型派生类：db::LayoutLibrary（直接持有 Layout，最常用）、
//               以及从文件惰性加载的实现（见 dbLibraryManager / dbLibraryProxy）。
//
// 【★ 继承两个基类各为什么】
//     gsi::ObjectBase —— 让它能被脚本（Ruby/Python）当作对象使用；
//     tl::Object      —— 让它能参与**共享/弱指针**的引用计数管理
//                        （见 tlObject.h：含 keep_object / release_object）。
//   ★ 后者尤其重要：库可能被很多地方引用，且需要“在使用中不被销毁”，
//     引用计数正是为此。
//
// 【与 db::LibraryManager 的关系】
//   由 LibraryManager 统一登记与管理所有 Library（按 id / 名字索引）。
//   本类负责“一个库是什么”，Manager 负责“有哪些库”。
//
// 【本文件其余内容】
//   LayoutLibrary —— 直接持有一个 Layout 的具体实现（最常用）。
// [[ZH-END]]
class DB_PUBLIC Library
  : public gsi::ObjectBase, public tl::Object
{
public:
  /**
   *  @brief The constructor
   */
  Library ();

  /**
   *  @brief Copy constructor
   */
  Library (const Library &);

  /**
   *  @brief The destructor
   */
  virtual ~Library ();

  /**
   *  @brief Called to reload the library
   *
   *  If the library is a file-based one, this method can be reimplemented to reload
   *  the file. This method must not change the name of the library, but return a new
   *  name in case it has changed.
   *
   *  @return The new name of the library
   */
  virtual std::string reload ()
  {
    return get_name ();
  }

  /**
   *  @brief The layout object
   *
   *  This method must be reimplemented by some derived class to actually provide the
   *  layout or the derived class fills the layout.
   */
  virtual db::Layout &layout () 
  {
    return m_layout;
  }

  /**
   *  @brief Get the const layout
   *
   *  This version uses the non-const, virtual implementation to provide a const
   *  layout accessor.
   */
  const db::Layout &layout () const
  {
    return (const_cast<Library *> (this))->layout ();
  }

  /**
   *  @brief Getter for the name property
   */
  const std::string &get_name () const
  {
    return m_name;
  }

  /**
   *  @brief Setter for the name property
   */
  void set_name (const std::string &name) 
  {
    m_name = name;
  }

  /**
   *  @brief Gets the technology name this library is associated with
   *
   *  If this attribute is non-empty, the library is selected only when the given technology is
   *  used for the layout.
   */
  const std::set<std::string> &get_technologies () const
  {
    return m_technologies;
  }

  /**
   *  @brief Gets a value indicating whether this library is associated with the given technology
   */
  bool is_for_technology (const std::string &name) const;

  /**
   *  @brief Gets a value indicating whether the library is associated with any technology
   */
  bool for_technologies () const;

  /**
   *  @brief Sets the technology names this library is associated with
   *
   *  This will reset the list of technologies to this set.
   */
  void set_technologies (const std::set<std::string> &t);

  /**
   *  @brief Sets the technology name this library is associated with
   *
   *  This will reset the list of technologies to this one.
   *  If the given technology string is empty, the list of technologies will be cleared.
   */
  void set_technology (const std::string &t);

  /**
   *  @brief Clears the list of technologies this library is associated with
   */
  void clear_technologies ();

  /**
   *  @brief Additionally associate the library with the given technology
   */
  void add_technology (const std::string &tech);

  /**
   *  @brief Getter for the description property
   */
  const std::string &get_description () const
  {
    return m_description;
  }

  /**
   *  @brief Setter for the description property
   */
  void set_description (const std::string &description) 
  {
    m_description = description;
  }

  /**
   *  @brief Sets a value indicating whether the library produces replicas
   *
   *  If this value is true (the default), layout written will include the
   *  actual layout of a library cell (replica). With this, it is possible
   *  to regenerate the layout without actually having the library at the
   *  cost of additional bytes in the file.
   *
   *  Setting this flag to false avoids this replication, but a layout
   *  cannot be regenerated without having this library.
   */
  void set_replicate (bool f);

  /**
   *  @brief Gets a value indicating whether the library produces replicas
   */
  bool replicate () const
  {
    return m_replicate;
  }

  /**
   *  @brief Getter for the library Id property
   */
  lib_id_type get_id () const
  {
    return m_id;
  }

  /**
   *  @brief Setter for the library Id property
   */
  void set_id (lib_id_type id) 
  {
    m_id = id;
  }

  /**
   *  @brief Register a LibraryProxy in the given layout
   */
  void register_proxy (db::LibraryProxy *lib_proxy, db::Layout *layout);
  
  /**
   *  @brief Unregister the Library proxy
   */
  void unregister_proxy (db::LibraryProxy *lib_proxy, db::Layout *layout);

  /**
   *  @brief Retires a LibraryProxy in the given layout
   *
   *  A proxy becomes entirely retired if the refcount is equal to the
   *  retired count. This feature is used to decide whether a proxy
   *  is actually used or only present as a shadow object for the transaction
   *  management.
   */
  void retire_proxy (db::LibraryProxy *lib_proxy);

  /**
   *  @brief Unretires the Library proxy
   */
  void unretire_proxy (db::LibraryProxy *lib_proxy);

  /**
   *  @brief Gets a value indicating whether a proxy is entirely retired
   */
  bool is_retired (const cell_index_type library_cell_index) const;

  /**
   *  @brief Refreshes the library on all clients
   *
   *  This will refresh PCells, retire cells (turn them into "cold proxies") and reload layouts.
   */
  void refresh ();

  /**
   *  @brief Refreshes the library on all clients without restoring proxies
   *
   *  This method is intended to be used internally for bulk refreshes.
   */
  void refresh_without_restore ();

  /**
   *  @brief Renames the library
   *
   *  Unlike "set_name", this method will take care of properly re-registering the library
   *  under the new name.
   */
  void rename (const std::string &name);

  /**
   *  @brief Remap the library proxies to a different library
   *
   *  After remapping, "other" can replace "this".
   *  When calling with "other=this", a pointer to the original
   *  layout needs to be supplied, because in that case, the
   *  layout of "this" is already replaced.
   */
  void remap_to (db::Library *other, Layout *original_layout = 0);

  /**
   *  @brief This event is fired if proxies get retired on unretired
   */
  tl::Event retired_state_changed_event;

private:
  std::string m_name;
  std::string m_description;
  std::set<std::string> m_technologies;
  lib_id_type m_id;
  db::Layout m_layout;
  std::map<db::Layout *, int> m_referrers;
  std::map<db::cell_index_type, int> m_refcount, m_retired_count;
  bool m_replicate;

  // no copying.
  Library &operator=(const Library &);
};

}

#endif


