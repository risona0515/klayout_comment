
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


#ifndef HDR_dbObject
#define HDR_dbObject

#include "dbCommon.h"

#include "dbManager.h"

namespace db
{

/**
 *  @brief The base class of a database object
 *
 *  The main purpose of the db::Object class is to provide
 *  transaction and recall functionality. Basically this is 
 *  an implementation of the strategy pattern that is used
 *  to implement the recall functionality.
 */
// [[ZH-BEGIN]]
// 功能：★ **数据库对象体系的共同基类** —— 让对象能接入事务与“回忆(recall)”机制。
//
// 【它解决什么问题】
//   版图数据是**可编辑**的：用户改一个图形，系统要能撤销、也要能重做。
//   要做到这一点，每个对象必须能回答：“我在哪个管理器下？我的修改怎么记录成 Op？”
//   本基类就是提供这个共同接口（以及与 Manager 的关联关系）。
//
// 【★ 上面英文注释提到的 "recall functionality" 是什么】
//   “回忆(recall)”指**重新执行一个已记录的操作**（即 redo 的核心动作）。
//   英文注释说本类“是策略模式的实现，用来实现 recall 功能”——意思是：
//   具体的“怎么回忆”由**派生类**决定（策略），基类只给出统一入口。
//   ★ 这解释了为何本类几乎只有“管理器关联”相关的代码：
//     真正的编辑逻辑在各派生类（Cell / Shapes / Shape / Instance / Layout ...）。
//
// 【★ 与 db::Manager 的关系（两个类分工不同，别混）】
//     db::Object   ← 本文件：**被管理的对象**（“我属于哪个 Manager”）。
//     db::Manager  ← dbManager.h：**管理者**，负责发对象 ID、记事务、管 undo/redo。
//   两者是**多对一**：一个 Manager 可管很多 Object，
//   而一个对象要么挂在某个 Manager 下，要么不挂（manager() 返回 0）。
//
// 【★ 关键行为：manager() 可能返回 0】
//   英文注释明确写了 "Returns 0 if not attached to a manager."
//   ★ 实用影响：调用 manager() 后**必须判空**再使用。什么时候会是 0？
//       · 对象还未被加入任何 Layout；
//       · 或该 Manager 被禁用了（见 dbManager.h 的 enabled 开关）；
//       · 或对象刚构造出来还没挂接。
//     库内大量代码先 `if (manager ())` 再决定是否构造 Op ——
//     这正是“禁用 Manager 时能提速”的机制。
//
// 【拷贝语义（一个细节）】
//   拷贝构造会**一并拷贝与 Manager 的挂接关系**（英文注释说明了）。
//   即拷贝出的对象仍挂接在同一个 Manager 下；析构则自动脱钩。
//   ★ 因此对象的生命周期与挂接关系是自动维护的，不用手工管理。
// [[ZH-END]]

class DB_PUBLIC Object
{
public:
  /**
   *  @brief Default ctor
   *
   *  Attach the object to a manager if required
   */
  // [[ZH]] 功能：构造并可选择挂接到一个 Manager。
  // [[ZH]] 参数：manager 目标管理器；0（默认）= 不挂接（即不可撤销，见 manager() 说明）。
  Object (db::Manager *manager = 0);

  /**
   *  @brief Destructor
   *
   *  Detaches the object from the manager
   */
  // [[ZH]] 功能：析构时**自动从 Manager 脱钩**（并释放对象 ID）。
  // [[ZH]] 虚析构：允许通过基类指针删除派生对象（库中普遍如此）。
  virtual ~Object ();

  /** 
   *  @brief Copy constructor
   *
   *  The copy constructor copies the attachment to
   *  a manager object. 
   */
  // [[ZH]] 功能：拷贝构造 —— ★ **同时拷贝与 Manager 的挂接关系**。
  // [[ZH]] 即拷贝出的对象仍挂接在同一个 Manager 下（见文件头说明）。
  Object (const Object &);

  /**
   *  @brief Manager object retrieval
   *
   *  Obtain the pointer to the management object this
   *  object is attached to. Returns 0 if not attached to a manager.
   */
  db::Manager *manager () const
  {
    return mp_manager;
  }

  /**
   *  @brief Attach to a different manager or detach
   *
   *  Changes the attachment of the object to any
   *  manager. Pass 0 to detach it from any manager.
   */
  void manager (db::Manager *p_manager);

  /** 
   *  @brief The undo 'strategy'
   *
   *  For a detailed description of this method see db::Manager::queue.
   *  This method has been declared to be no-throwing since it is
   *  assumed that once the operation is successfully queued it can be undone
   *  in every case.
   */
  virtual void undo (db::Op * /*op*/) 
  { }

  /** 
   *  @brief The redo 'strategy'
   *
   *  For a detailed description of this method see db::Manager::queue.
   *  This method has been declared to be no-throwing since it is
   *  assumed that once the operation is successfully queued it can be redone
   *  in every case.
   */
  virtual void redo (db::Op * /*op*/)
  { }

  /**
   *  @brief The id getter method
   */
  db::Manager::ident_t id () const 
  {
    return m_id;
  }

  /**
   *  @brief A convenience function to determine if we are transacting
   */
  bool transacting () const
  {
    return manager () && manager ()->transacting ();
  }

  /**
   *  @brief A convenience function to determine if we are in an undo or redo replay operation
   */
  bool replaying () const
  {
    return manager () && manager ()->replaying ();
  }

  /**
   *  @brief Begins a transaction (same as calling manager ()->transaction (), but safe against null manager ())
   */
  void transaction (const std::string &description, db::Manager::transaction_id_t join_with = 0)
  {
    if (manager ()) {
      manager ()->transaction (description, join_with);
    }
  }

  /**
   *  @brief Ends a transaction (same as calling manager ()->commit (), but safe against null manager ())
   */
  void commit ()
  {
    if (manager ()) {
      manager ()->commit ();
    }
  }

private:
  db::Manager::ident_t m_id;
  db::Manager *mp_manager;

  //  The assignment operator is private in order to
  //  force a specific treatment of the manager object attachment
  Object &operator= (const Object &);
};

} // namespace db

#endif

