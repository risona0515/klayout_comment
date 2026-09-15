
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


#ifndef HDR_dbManager
#define HDR_dbManager

#include "dbCommon.h"

#include "tlTypeTraits.h"

#include <vector>
#include <list>
#include <string>

namespace db
{

class Object;

/**
 *  @brief An atomic operation.
 * 
 *  See Manager::queue for a detailed description.
 */
// [[ZH-BEGIN]]
// 功能：**一个原子编辑操作** —— 撤销/重做体系里的最小单位。
//
// 【在撤销体系里的位置】
//   db::Manager 维护事务栈，而每个事务里存的就是一串 **Op**。
//   撤销一个事务 = 逆序执行每个 Op 的 undo；重做 = 顺序执行 redo。
//   所以：想支持撤销的编辑动作，就得实现一个 Op 子类。
//
// 【它只有一个状态位（设计极简）】
//   m_done 记录“这个操作当前是否已生效”。
//   初始化时就已生效 → m_done = true（默认）；
//   若你是“先构造后执行”的写法，传 false。
//   Manager 会用它来判断能不能撤销/重做（防止重复逆执行）。
//
// 【私有的 set_done 与 friend Manager】
//   set_done 是私有的，只有 Manager 是友元可以改 ——
//   ★ 即：**生效状态由 Manager 维护，Op 自己不能随意改**。
//     这是刻意的权限划分：否则自定义 Op 可能把状态搞乱。
//
// 【★ 与 db::Layout 事务协议的关系（容易混）】
//   Layout::start_changes()/end_changes() 负责的是**缓存失效**；
//   Manager 的事务负责的是**撤销记录**。两者是不同层次的事，但常一起出现。
// [[ZH-END]]

class DB_PUBLIC Op
{
private:
  // [[ZH]] 与 Manager 的友元关系：允许 Manager 修改 m_done（见上方说明）。
  friend class Manager;

  // [[ZH]] m_done：该操作当前是否已生效。由 Manager 维护，不由 Op 自身修改。
  bool m_done;

  // [[ZH]] 功能：设置生效标志。**私有**，且只有 Manager 能调 →
  // [[ZH]]       保证“生效状态”不被自定义 Op 随意篡改。
  void set_done (bool d)
  {
    m_done = d;
  }

public:
  // [[ZH]] 功能：构造。参数 done 表示“是否已生效”——
  // [[ZH]]       true（默认）= 构造时操作就已执行过；
  // [[ZH]]       false = 先构造 Op，稍后再执行其动作。
  Op (bool done = true) : m_done (done)
  { }

  // [[ZH]] 虚析构：允许通过基类指针删除自定义 Op（否则会漏调派生类析构）。
  virtual ~Op () 
  { }

  // [[ZH]] 功能：查询该操作当前是否已生效（供 Manager 决定能否撤销/重做）。
  bool is_done () const
  {
    return m_done;
  }
};

/**
 *  @brief The database object manager
 *
 *  The functionality of the database object manager is to
 *  manage absolute object references through object id's
 *  and to provide transaction management.
 */
// [[ZH-BEGIN]]
// ============================================================================
//  db::Manager —— ★ 数据库对象的管理器（**撤销/重做** + 对象 ID 表）
// ============================================================================
//
// 【它做两件事（上面英文注释已点出，展开说明）】
//
//   ① **对象 ID 表** —— “通过 ID 管理绝对对象引用”
//      为什么需要：库中的图形/实例/单元对象在容器里会移动（增删、重排），
//      因此**指针会失效**。而 ID 不会。
//      于是：对象向 Manager 申请一个 ID（next_id），
//      需要时用 ID 换回对象（object_by_id）。
//      ★ 这就是库中“句柄/引用”能长期持有的底层机制 ——
//        db::Object 基类就是靠它实现稳定标识的。
//      ★ 代价：为了能“用 ID 换回对象”，Manager 必须维护映射并在对象销毁时回收
//        （release_object），因此它是有状态、有开销的。（下面 enabled 开关的意义）
//
//   ② **事务与撤销/重做**
//      典型使用流程（与 db::Layout 的修改配合）：
//           transaction_id_t t = mgr.transaction ("Move shape");  // 开事务
//           ... 在这里把修改包装成 Op 并 mgr.queue (op) 入队 ...
//           mgr.commit ();        // 提交（此时该事务可被 undo）
//      // 或 mgr.cancel () 丢弃；
//      // 之后可 mgr.undo () / mgr.redo () 来回回放。
//
// 【★ 与 db::Layout 事务协议的区别（最易混的一点）】
//   Layout::start_changes()/end_changes()  → 解决**缓存一致性**（包围盒、Region 展开等）；
//   Manager::transaction()/commit()       → 解决**撤销记录**。
//   两者**不同层次**、可同时使用，且**不能相互替代**：
//   只调 Layout 事务 → 修改生效但不可撤销；
//   只调 Manager 事务 → 可撤销但缓存可能不一致。
//
// 【★ enabled 开关与性能权衡】
//   构造函数可传 enabled = false。关闭后 ID 表/事务管理的开销消失，
//   但也就没有撤销能力。
//   ★ 实用场景：批处理/脚本一次性大改动时关掉它可显著提速。
//   is_enabled() 可查询当前状态；库内很多地方会先查它以避免无效开销。
//
// 【undo/redo 的可用性查询】
//   available_undo() / available_redo() 返回 (是否可用, 描述文字)，
//   供 GUI 显示“撤销 移动图形”这类菜单项。
// [[ZH-END]]

class DB_PUBLIC Manager
{
public:
  // [[ZH]] ident_t：对象 ID 类型（尺寸无关的整数标识）。
  // [[ZH]] 注意：ID 不是指针，也不能自行构造 —— 必须向 Manager 申请。
  typedef size_t ident_t;
  // [[ZH]] transaction_id_t：事务 ID。0 通常表示“无事务”或“无效”。
  typedef size_t transaction_id_t;

  /**
   *  @brief Default constructor
   */
  // [[ZH]] 功能：构造管理器。参数 enabled 为 false 时关闭 ID 表与事务管理：
  // [[ZH]]       更快、更省内存，但**不支持撤销**。批处理场景建议关闭。
  Manager (bool enabled = true);
  
  /**
   *  @brief Destructor
   */
  // [[ZH]] 功能：析构。注意它持有对象 ID 映射，销毁时不需要清理那些对象
  // [[ZH]]       （对象归其容器所有，Manager 只是登记它们的 ID）。
  ~Manager ();

  /**
   *  @brief Gets a value indicating whether the manager is enabled
   */
  // [[ZH]] 功能：查询管理器是否启用（即是否支持撤销/对象 ID 登记）。
  // [[ZH]] 用途：库内大量代码先查它，禁用时跳过 Op 构造等开销。
  bool is_enabled () const
  {
    return m_enabled;
  }

  /**
   *  @brief Release an object with the given id.
   *
   *  This will free this id and recycle it upon the
   *  next request. After this call, the id is no longer valid.
   */
  // [[ZH]] 功能：释放一个对象 ID，使其可被后续 next_id 复用。
  // [[ZH]] 参数：id 待释放的 ID。
  // [[ZH]] ★ 调用后该 ID **立即失效** —— 任何还持有它的引用都必须停止使用。
  // [[ZH]] 何时调用：对象销毁时必须调，否则 ID 泄漏（且 ID 表持续增长）。
  void release_object (ident_t id);

  /**
   *  @brief Request the next available id
   *
   *  Request the next id available and associate with an object
   */
  // [[ZH]] 功能：为一个对象申请一个**新的、稳定的** ID，并建立 对象↔ID 映射。
  // [[ZH]] 参数：obj 目标对象（基类指针，须比 ID 活得久）。
  // [[ZH]] 返回：新的 ident_t。
  // [[ZH]] ★ 这就是库中“句柄能长期持有”的机制来源（见类注释）。
  // [[ZH]] 对应释放：release_object（成对使用，否则泄漏）。
  ident_t next_id (db::Object *obj);

  /**
   *  @brief Retrieve the object pointer for a given id
   *
   *  For the given id, retrieve the object pointer.
   *  Returns 0 if the id is not a valid one.
   */  
  // [[ZH]] 功能：用 ID 换回对象指针。
  // [[ZH]] 返回：对象指针；★ **ID 无效时返回 0**（不是抛异常）。
  // [[ZH]] 坑：因此**必须判空** —— 若对象已销毁而 ID 未释放（或已释放），
  // [[ZH]]     这里会安静地返回 0；直接用会空指针崩溃。
  db::Object *object_by_id (ident_t id);

  /** 
   *  @brief Begin a transaction
   *
   *  This call will open a new transaction. A transaction consists
   *  of a set of operations issued with the 'queue' method.
   *  A transaction is closed with the 'commit' method.
   *
   *  The transaction can be joined with a previous transaction. To do so, pass the
   *  previous transaction id to the "join_with" parameter. If the transaction specified
   *  with "join_with" is not the previous transaction, it is not joined.
   */
  // [[ZH-BEGIN]]
  // 功能：★ **开始一个事务** —— 撤销/重做体系的操作边界。
  //
  // 参数：description 事务描述（会显示在“撤销 移动图形”这类菜单里）；
  //       join_with 要**并入**的前一个事务 ID（0 = 不并入）。
  // 返回：新事务的 ID（后续 queue / commit / cancel 都用它）。
  //
  // 【典型用法（三步）】
  //     transaction_id_t t = mgr.transaction ("Move shapes");  // ① 开
  //     mgr.queue (new MyOp (...));                            // ② 填入 Op
  //     mgr.commit ();                                         // ③ 提交
  //
  // 【★ join_with 的用途与限制（上面英文注释特意说明了）】
  //   把多个小事务**合并成一个撤销单元**（用户按一次 Ctrl+Z 就整体回退）。
  //   例：拖动过程中每一步都开事务，但都 join 到拖动开始时的事务 →
  //        撤销时整段拖动一次消失，而不是一步一撤。
  //   ★ 限制：“若 join_with 指定的不是**前一个**事务，则不会并入” ——
  //     即只能向**紧邻的前一个**事务合并，不能跨事务乱并。
  //
  // 坑：并行/交错的事务不支持 —— 事务是栈式的（后开先关）。
  // [[ZH-END]]
  transaction_id_t transaction (const std::string &description, transaction_id_t join_with = 0);

  /**
   *  @brief Returns the last transaction id
   *
   *  This method can be used to identify the current transaction by id.
   */
  // [[ZH]] 功能：返回**最近一个**（即当前）事务的 ID。
  // [[ZH]] 用途：需要判断“我是否还在同一个事务里”或做条件性 join 时用。
  transaction_id_t last_transaction_id () const;

  /**
   *  @brief Gets the id of the next transaction to undo
   */
  // [[ZH]] 功能：返回“**下一个将被撤销**”的事务 ID（即撤销栈顶）。
  // [[ZH]] 用途：与 transaction_id_for_redo 一起，用于把撤销/重做
  // [[ZH]]       与具体对象状态绑定（例如撤销后需要重新定位相关对象）。
  transaction_id_t transaction_id_for_undo () const;

  /**
   *  @brief Gets the id of the next transaction to redo
   */
  // [[ZH]] 功能：返回“**下一个将被重做**”的事务 ID（即重做栈顶）。
  transaction_id_t transaction_id_for_redo () const;

  /**
   *  @brief Close a transaction successfully.
   */
  void commit ();

  /**
   *  @brief Cancels a transaction
   *
   *  If called instead of commit, this method will undo all operations of the pending
   *  transaction.
   */
  void cancel ();

  /**
   *  @brief Undo the current transaction
   *
   *  The current transaction is undone with this method.
   *  The 'available_undo' method can be used to determine whether
   *  there are transactions to undo.
   */
  void undo ();

  /**
   *  @brief Redo the next available transaction
   *
   *  The next transaction is redone with this method.
   *  The 'available_redo' method can be used to determine whether
   *  there are transactions to undo.
   */
  void redo ();

  /**
   *  @brief Determine available 'undo' transactions
   *
   *  @return The first parameter is true, if there is a transaction
   *          to undo. The second is the description of the transaction
   *          if there is one available.
   */
  std::pair<bool, std::string> available_undo () const;

  /**
   *  @brief Determine available 'redo' transactions
   *
   *  @return The first parameter is true, if there is a transaction
   *          to redo. The second is the description of the transaction
   *          if there is one available.
   */
  std::pair<bool, std::string> available_redo () const;

  /**
   *  @brief Gets the number of available undo items
   */
  int available_undo_items ();

  /**
   *  @brief Gets the number of available redo items
   */
  int available_redo_items ();

  /**
   *  @brief Gets an item from the list
   *
   *  @param delta A positive value or 0 for the nth redo item, A negative value for the nth undo item
   *
   *  A delta of "0" will give you the next redo item, a delta of "1" the second next one.
   *  A delta of "-1" will give you the first undo item.
   *  delta must be less than "available_redo_items" and larger or equal than "-available_undo_items".
   *
   *  @return The description of the transaction
   */
  std::string undo_or_redo_item (int delta) const;

  /**
   *  @brief Queue a operation for undo
   *
   *  With this method a atomic undoable operation can be registered.
   *  The operation is an object derived from db::Op. 
   *  This object's pointer is passed to the 'undo' method of the
   *  object in charge once a undo operation is requested.
   *  The same object is passed also to the 'redo' method to redo
   *  to operation. 
   *  The operation also holds the state: initially the operation
   *  signals "done" which means that the operation defined by the
   *  db::Op object was performed. Upon 'undo' the state changes to
   *  'undone' which signals that the operation was undone. Upon 'redo'
   *  then the state again changes to 'done'.
   *  If the 'op' object is passed in 'undone' state to the queue method,
   *  it will be brought into done state by issueing a 'redo'. This 
   *  way the operation can be implemented fully implicitly through
   *  db::Object's undo and redo methods.
   *  The db::Op object will be owned by the manager and there will
   *  be no copying for the object. Therefore the db::Op object is
   *  a good place for storing pointers to objects created in the
   *  process of the operation for example.
   */ 
  void queue (db::Object *object, db::Op *op);

  /**
   *  @brief Get the last queued db::Op object
   *
   *  This method allows one to fetch and modify the last queued operation for the given object in order
   *  to allow some optimisation, i.e. joining two ops. It can be modified but must not
   *  be deleted. The returned object is guaranteed to be inside same transaction.
   *
   *  @param object The object for which to look for a queued operation.
   *  @return See above. 0 if no operation is queued for this transaction.
   */
  db::Op *last_queued (db::Object *object);

  /** 
   *  @brief Clear all transactions 
   */
  void clear ();

  /**
   *  @brief Query if we are within a transaction
   */
  bool transacting () const
  {
    return m_opened;
  }

  /**
   *  @brief Query if we are within a undo/redo operation
   */
  bool replaying () const
  {
    return m_replay;
  }

private:
  std::vector<db::Object *> m_id_table;
  std::vector<ident_t> m_unused_ids;

  typedef std::pair<db::Manager::ident_t, db::Op *> operation_t;
  typedef std::list<operation_t> operations_t;
  typedef std::pair<operations_t, std::string> transaction_t;
  typedef std::list<transaction_t> transactions_t;

  transactions_t m_transactions;
  transactions_t::iterator m_current;
  bool m_opened;
  bool m_replay;
  bool m_enabled;

  void erase_transactions (transactions_t::iterator from, transactions_t::iterator to);
};

/**
 *  @brief A transaction controller utility class
 *
 *  This object controls a transaction through it's lifetime. On construction, the 
 *  transaction is started, on destruction, the transaction is committed.
 *
 *  "cancel" can be used to cancel the operation. This will undo all operations collected
 *  so far and delete the transaction.
 *
 *  "close" temporarily disable the collection of operations.
 *  "open" will enable operation collection again and continue
 *  collection at the point when it was stopped with "close".
 */

class DB_PUBLIC Transaction
{
public:
  Transaction (db::Manager *manager, const std::string &desc)
    : mp_manager (manager), m_transaction_id (0), m_description (desc)
  {
    if (mp_manager) {
      m_transaction_id = mp_manager->transaction (desc);
    }
  }

  Transaction (db::Manager *manager, const std::string &desc, db::Manager::transaction_id_t join_with)
    : mp_manager (manager), m_transaction_id (0), m_description (desc)
  {
    if (mp_manager) {
      m_transaction_id = mp_manager->transaction (desc, join_with);
    }
  }

  ~Transaction ()
  {
    if (mp_manager) {
      if (mp_manager->transacting ()) {
        mp_manager->commit ();
      }
      mp_manager = 0;
    }
  }

  void cancel ()
  {
    if (mp_manager) {
      open ();
      mp_manager->cancel ();
      mp_manager = 0;
    }
  }

  void close ()
  {
    if (mp_manager->transacting ()) {
      mp_manager->commit ();
    }
  }

  void open ()
  {
    if (mp_manager && ! mp_manager->transacting ()) {
      mp_manager->transaction (m_description, m_transaction_id);
    }
  }

  bool is_empty () const
  {
    return ! mp_manager || mp_manager->last_queued (0) == 0;
  }

  db::Manager::transaction_id_t id () const
  {
    return m_transaction_id;
  }

private:
  db::Manager *mp_manager;
  db::Manager::transaction_id_t m_transaction_id;
  std::string m_description;

  //  no copying.
  Transaction (const Transaction &);
  Transaction &operator= (const Transaction &);
};

} // namespace db

#endif

