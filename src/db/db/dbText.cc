
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


#include "dbText.h"
#include "tlThreads.h"

namespace db
{

// [[ZH-BEGIN]]
// ============================================================================
//  dbText.cc —— 文本图形的实现（含**字符串去重**机制）
// ============================================================================
//
// 【本文件有两块内容，都很重要】
//
//   ① **文本的对齐与格式**（文件开头那几个小函数）
//      文本有水平/垂直对齐方式，它们需要与字符编码互转，
//      而这些字符编码会出现在**文本格式**里（因此不能随意改）：
//          水平对齐： 'c' 居中 / 'l' 左 / 'r' 右
//          垂直对齐： 'c' 居中 / 't' 顶 / 'b' 底
//      ★ 注意水平与垂直都用了 'c'，但分属两套枚举，互不冲突。
//
//   ② **字符串仓库 + 引用计数**（StringRepository / StringRef）★ 本文件的主体
//      解决的问题：版图里文本字串大量重复（如几万个 "VDD"）。
//      若每个 text 各存一份字串，内存浪费极大。
//
//      【设计：三层结构】
//        StringRepository   —— 全局**单例**，记录所有存活引用的集合
//        StringRef          —— 指向字串的引用，**带引用计数**
//        StringRepository::create_string_ref() 造引用并登记到仓库
//
//      【★ 生命周期：两边都会释放，谁是第一个】
//        · StringRef::remove_ref() 把计数减到 0 时 → **自己 delete 自己**；
//        · StringRef 析构时 → 主动去仓库 **注销 (unregister_ref)**；
//        · 仓库析构时 → 反过来把**还活着的**引用全部 delete。
//        这三条共同保证：无论“引用先死”还是“仓库先死”，都不会泄漏。
//      ★ 这是本文件最值得学习的地方 —— 一个自洽的双向清理协议。
//
//      【★ 线程安全】
//        仓库操作与引用计数各自有**独立的互斥锁**（s_repository_lock / s_ref_lock）。
//        因此文本的创建/销毁是线程安全的 —— 在版图并行处理中很重要。
//        注意两把锁是分开的：不要假设“仓库锁能保护引用计数”。
//
// 【与未命名对象设计的关系（实用影响）】
//   因为字串**本身不在 text 对象里**，所以：
//    · 拷贝一个 text 很便宜（只拷引用）；
//    · 但修改字串会影响所有引用者（共享而非拷贝）。
//   这与 db::Shape 的“句柄语义”是同一种思路：小对象 + 共享数据。
//
// 【本文件其余部分】
//   text<C>::to_string (dbu)    ★ 文本的文本表示格式
//   namespace tl 下的四个特化   注册“如何解析一个文本”（同 dbPoint.cc 的模式）
// [[ZH-END]]

// [[ZH]] 功能：把水平对齐枚举转成**单字符编码**（用于文本格式）。
// [[ZH]] 返回：'c' 居中 / 'l' 左 / 'r' 右 / 0 表示“无对齐”。
// [[ZH]] 坑：返回 0 而不是某个字符 —— 调用方需处理“无对齐”这一情形。
// [[ZH]] 注意：这些编码会写进文本格式，因此**不能随意改动**（影响兼容性）。
static char halign2code (db::HAlign ha)
{
  if (ha == db::HAlignCenter) {
    return 'c';
  } else if (ha == db::HAlignLeft) {
    return 'l';
  } else if (ha == db::HAlignRight) {
    return 'r';
  } else {
    return 0;
  }
}

// [[ZH]] 功能：halign2code 的反向操作 —— 从文本里读一个字符并转成水平对齐。
// [[ZH]] 参数：ex 文本解析器（会消费一个字符）。
// [[ZH]] 返回：对应的枚举；**无匹配时返回 NoHAlign**（而不是报错）。
// [[ZH]] 相比 halign2code 的不对称之处：单向有“无对齐”的表示（0），
// [[ZH]] 反向则用 NoHAlign 枚举成员。
static db::HAlign extract_halign (tl::Extractor &ex)
{
  if (ex.test ("c")) {
    return db::HAlignCenter;
  } else if (ex.test ("l")) {
    return db::HAlignLeft;
  } else if (ex.test ("r")) {
    return db::HAlignRight;
  } else {
    return db::NoHAlign;
  }
}

// [[ZH]] 功能：把垂直对齐枚举转成单字符编码。
// [[ZH]] 返回：'c' 居中 / 't' 顶 / 'b' 底 / 0 表示“无”。
// [[ZH]] 注意：与水平对齐**共用字符 'c'**（均指居中），但分属不同枚举，互不影响。
static char valign2code (db::VAlign va)
{
  if (va == db::VAlignCenter) {
    return 'c';
  } else if (va == db::VAlignBottom) {
    return 'b';
  } else if (va == db::VAlignTop) {
    return 't';
  } else {
    return 0;
  }
}

// [[ZH]] 功能：从文本读一个字符并转成垂直对齐；无匹配时返回 NoVAlign。
// [[ZH]] 注意：这里对 't'/'b' 的判定顺序与 valign2code 的枚举顺序不同，
// [[ZH]]       但结果是等价的（只是查表顺序差异，不影响语义）。
static db::VAlign extract_valign (tl::Extractor &ex)
{
  if (ex.test ("c")) {
    return db::VAlignCenter;
  } else if (ex.test ("t")) {
    return db::VAlignTop;
  } else if (ex.test ("b")) {
    return db::VAlignBottom;
  } else {
    return db::NoVAlign;
  }
}

// ----------------------------------------------------------------------------
//  StringRepository implementation

static StringRepository s_repository;
static StringRepository *sp_repository = 0;
static tl::Mutex s_repository_lock;

// [[ZH-BEGIN]]
// 功能：字符串仓库的**单例访问器** —— 返回全局唯一的仓库指针（未创建时为 0）。
//
// ★ 注意：它**不主动创建**仓库，只返回 sp_repository 当时的值。
//   仓库的建立靠下面那个静态成员 `s_repository`（构造时就登记自己），
//   因此“单例”是**靠一个全局对象的构造/析构自动完成**的，而不是靠首次调用时 new。
//   ★ 这种手法的好处：线程安全由 C++ 的静态初始化保证，无需额外同步；
//     且程序退出时**自动析构**，从而触发“清理还活着的引用”。
//   代价：依赖“静态对象存在且已构造”—— 因此早期/晚期静态初始化阶段需谨慎。
//
// 返回：仓库指针；若仓库尚未构造或已析构则为 0。
// ★ 调用方**必须**判空（StringRef 析构里就是这么做的）。
// [[ZH-END]]
StringRepository *
StringRepository::instance ()
{
  return sp_repository;
}

StringRepository::StringRepository ()
{
  sp_repository = this;
}

StringRepository::~StringRepository ()
{
  if (sp_repository == this) {
    sp_repository = 0;
  }

  for (std::set<StringRef *>::const_iterator s = m_string_refs.begin (); s != m_string_refs.end (); ++s) {
    delete *s;
  }
}

// [[ZH]] 功能：创建一个新的字符串引用并**登记**到仓库（加锁保护）。
// [[ZH]] 返回：新引用的指针（const，调用方只读使用）。
// [[ZH]] 修改：向 m_string_refs 插入新引用（仓库开始“记着”它）。
// [[ZH]] 注意：归还所有权给调用方 —— 但仓库会在析构时兜底删除还活着的引用。
const StringRef *
StringRepository::create_string_ref ()
{
  tl::MutexLocker locker (&s_repository_lock);
  StringRef *ref = new StringRef ();
  m_string_refs.insert (ref);
  return ref;
}

// [[ZH]] 功能：从仓库注销一个引用（加锁保护）—— 由 StringRef 析构时调用。
// [[ZH]] 修改：从 m_string_refs 移除该引用。
// [[ZH]] 坑：这里判了 `! m_string_refs.empty()` 而非直接 erase ——
// [[ZH]]     语义上无差别（空集 erase 也无害），更像防御性写法。
void
StringRepository::unregister_ref (StringRef *ref)
{
  tl::MutexLocker locker (&s_repository_lock);
  if (! m_string_refs.empty ()) {
    m_string_refs.erase (ref);
  }
}

// ----------------------------------------------------------------------------
//  StringRef implementation

static tl::Mutex s_ref_lock;

StringRef::~StringRef ()
{
  if (StringRepository::instance ()) {
    StringRepository::instance ()->unregister_ref (this);
  }
}

// [[ZH]] 功能：**引用计数加一**（加锁）。
// [[ZH]] 用途：新建立一个指向同一字串的引用时调用。
// [[ZH]] 对应的减一/释放见 remove_ref。
void
StringRef::add_ref ()
{
  tl::MutexLocker locker (&s_ref_lock);
  ++m_ref_count;
}

// [[ZH-BEGIN]]
// 功能：**引用计数减一**；减到 0 时**自己 delete 自己**（加锁）。
//
// ★★ 这是本文件生命周期协议的**第一半**，也是全库少见的“自我销毁”模式：
//     若最后一个引用被释放，这个 StringRef 就随之消失。
//   但要注意它“自己 delete 自己”会**触发析构**，而析构里又会去仓库注销
//   （见上面的 StringRef::~StringRef）—— 两步合起来才完成完整清理。
//
// ★ 一个必须警惕的后果：
//   因为“减到 0 就自杀”，所以只要曾调用过 remove_ref，**就不能再碰这个对象**。
//   这是经典的引用计数陷阱（使用后仍访问 → 悬空）。这里的做法是
//   约定 StringRef 的持有者必须严格成对操作（add_ref / remove_ref）。
//
// ★ 第二半（相对的另一半）：仓库析构时会把**还活着**的引用全部 delete。
//   两者共存的意义：
//     · 正常情况下由引用计数决定何时释放（及时回收）；
//     · 若仓库先于引用销毁（程序退出），则由仓库兜底，避免泄漏。
// [[ZH-END]]
void
StringRef::remove_ref ()
{
  tl::MutexLocker locker (&s_ref_lock);
  --m_ref_count;
  if (m_ref_count == 0) {
    delete this;
  }
}

// ----------------------------------------------------------------------------
//  text implementation

// [[ZH-BEGIN]]
// 功能：把文本图形转为字符串（其文本表示，用于调试打印与文本图形格式）。
//
// 参数：dbu 坐标→微米的换算因子（0 或 1 时输出原始 DBU，其余按微米格式化）。
// 返回：形如 `("字串",变换)` 的表示，例如 ("VDD",r0 0,0)。
//
// 实现要点（看下面一行就够）：
//   · 字串用 **tl::to_quoted_string** 包引号 —— 因此字串内部的引号/特殊字符
//     会被转义，**不会**破坏整个表示的解析（这点与简单的 to_string 不同）。
//   · 变换部分交由 m_trans.to_string(dbu) 处理（复用变换自己的格式）。
//   · 因此括号与逗号是分隔符，字串里的逗号由引号保护 ——
//     与 tl 下那个解析特化正好配套。
//
// 坑：输出格式与解析特化**必须成对演进而不能单改一边**，
//     否则往返（写→读）会失效。
// [[ZH-END]]
template <class C>
std::string text<C>::to_string (double dbu) const
{
  std::string s = std::string ("(") + tl::to_quoted_string (string ()) + "," + m_trans.to_string (dbu) + ")";

  if (size () > 0) {
    s += " s=";
    s += tl::to_string (size ());
  }

  if (font () >= 0) {
    s += " f=";
    s += tl::to_string (int (font ()));
  }

  char c;
  c = halign2code (halign ());
  if (c) {
    s += " ha=";
    s += c;
  }
  c = valign2code (valign ());
  if (c) {
    s += " va=";
    s += c;
  }

  return s;
}

// [[ZH]] 功能：显式实例化 text 模板（同 dbPoint.cc 说明的机制）。
// [[ZH]] 提示：本文件里真正有逻辑的是上面的 StringRepository/StringRef 与 to_string，
// [[ZH]]       这两行只是告诉编译器“用 Coord 与 DCoord 各生成一份代码”。
template class text<Coord>;
template class text<DCoord>;

}

namespace tl
{

// [[ZH-BEGIN]]
// 功能：为 tl::Extractor 注册“如何解析一个文本图形”（同 dbPoint/dbEdge 的模式）。
//
// 【文本的文本格式（与上面 to_string 正好互逆）】
//   形如：("hello",r0 10,20,1)
//          ↑字串（带引号，因此内部符支不会歧义）
//                    ↑变换（位置/方向）
//                         ↑文本大小
//   对齐信息以 `ha=` / `va=` 后缀出现（与上面 to_string 的输出对应），
//   取值就是 'c'/'l'/'r' 与 'c'/'t'/'b'（见文件开头那四个辅助函数）。
//
// ★ 为什么文本的解析比其他几何类型复杂：
//   · 它含**字串**（要处理引号与转义）；
//   · 它含**变换+大小+对齐**多个可选部分，因此需要逐个 test 并允许缺省。
//   这也是为什么本文件要把 to_string 写得那么繁琐 —— 两者必须严格互逆。
//
// 出错文案：“Expected a text specification”。
// 提供的特化：extractor_impl / test_extractor_impl，Coord 与 DCoord 各一份。
// [[ZH-END]]
template<> DB_PUBLIC void extractor_impl (tl::Extractor &ex, db::Text &p)
{
  if (! test_extractor_impl (ex, p)) {
    ex.error (tl::to_string (tr ("Expected a text specification")));
  }
}

template<> DB_PUBLIC void extractor_impl (tl::Extractor &ex, db::DText &p)
{
  if (! test_extractor_impl (ex, p)) {
    ex.error (tl::to_string (tr ("Expected a text specification")));
  }
}


template<class C> bool _test_extractor_impl (tl::Extractor &ex, db::text<C> &t)
{
  if (ex.test ("(")) {

    std::string s;
    ex.read_word_or_quoted (s);
    t.string (s);

    ex.expect (",");

    typename db::text<C>::trans_type tt;
    ex.read (tt);
    t.trans (tt);

    ex.expect (")");

    if (ex.test ("s=")) {
      C size = 0;
      ex.read (size);
      t.size (size);
    }

    if (ex.test ("f=")) {
      int font = -1;
      ex.read (font);
      t.font (db::Font (font));
    }

    if (ex.test ("ha=")) {
      db::HAlign ha = db::extract_halign (ex);
      t.halign (ha);
    }

    if (ex.test ("va=")) {
      db::VAlign va = db::extract_valign (ex);
      t.valign (va);
    }

    return true;

  } else {
    return false;
  }
}

template<> DB_PUBLIC bool test_extractor_impl (tl::Extractor &ex, db::Text &p)
{
  return _test_extractor_impl (ex, p);
}

template<> DB_PUBLIC bool test_extractor_impl (tl::Extractor &ex, db::DText &p)
{
  return _test_extractor_impl (ex, p);
}

}

