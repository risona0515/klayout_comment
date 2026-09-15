
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

#if !defined(HDR_tlDefs_h)
# define HDR_tlDefs_h

// [[ZH-BEGIN]]
// ============================================================================
//  tlDefs.h —— 动态库符号可见性宏（全库的 DLL/共享库导出基础设施）
// ============================================================================
//
// 【它解决什么问题】
//   本库被拆成**多个共享库**（klayout_tl、klayout_db ...），模块之间互相引用。
//   而不同平台导出符号的方式不同：
//     · Windows 必须在**编译库时**标 dllexport、**使用库时**标 dllimport，
//       写错或漏标会链接失败；
//     · GCC/Clang 默认所有符号可见，需要用 visibility 属性才做到
//       “默认隐藏、显式导出”（好处是符号表小、加载快、可避免命名冲突）。
//   本文件用一组宏吸收这些差异，让上层代码**只写一个宏名**即可跨平台。
//
// 【★ 六个宏的命名逻辑（记住这个规律就够）】
//     INSIDE  = 我正在**实现**这个库（编译库本身）
//     OUTSIDE = 我正在**使用**这个库（编译依赖它的模块）
//     PUBLIC          → 需要对外导出（跨库可见）
//     PUBLIC_TEMPLATE → 同上，但用于**模板**（Windows 对模板导出有特殊处理，
//                       因此单独一个宏而不是复用 PUBLIC）
//     LOCAL           → 只在本库内部可见（不导出，减少符号表）
//   ★ 所以代码里写 `TL_PUBLIC` 之类时，其展开结果取决于“我在库内还是库外”。
//     这就是为什么同一个宏在不同的翻译单元里展开不同，却都能正确工作。
//
// 【★ 与其它模块的关系（同样的模式重复出现）】
//   每个模块都有自己的这一套同构文件与专属前缀：
//     db/dbCommon.h  → DB_PUBLIC / DB_PUBLIC_TEMPLATE / DB_LOCAL
//     tl/tlCommon.h  → TL_PUBLIC / ...
//     rdb/rdbCommon.h→ RDB_PUBLIC / ...
//   而它们都**转发到本文件的 DEF_* 宏** —— 因此本文件是全库可见性的最终开关。
//
// 【为什么值得知道这一点（实用价值）】
//   阅读代码时看到 TL_PUBLIC / DB_PUBLIC，就知道：
//     · 这个符号**是跨库 API 的一部分**（改动它可能影响 ABI 兼容性）；
//     · 而标 LOCAL 的只是内部实现，可以放心重构。
//   这是判断“哪些接口是稳定的”的一条重要线索。
//
// 【本文件没有运行时逻辑】它是纯宏定义，不含任何函数或类型。
// [[ZH-END]]
//  templates provided for building the external symbol
//  declarations per library

# if defined _WIN32 || defined __CYGWIN__

#   define DEF_INSIDE_PUBLIC __declspec(dllexport)
#   define DEF_INSIDE_LOCAL
#   define DEF_INSIDE_PUBLIC_TEMPLATE

#   define DEF_OUTSIDE_PUBLIC __declspec(dllimport)
#   define DEF_OUTSIDE_LOCAL
#   define DEF_OUTSIDE_PUBLIC_TEMPLATE

# else

#   if __GNUC__ >= 4 || defined(__clang__)
#     define DEF_INSIDE_PUBLIC __attribute__ ((visibility ("default")))
#     define DEF_INSIDE_PUBLIC_TEMPLATE __attribute__ ((visibility ("default")))
#     define DEF_INSIDE_LOCAL  __attribute__ ((visibility ("hidden")))
#   else
#     define DEF_INSIDE_PUBLIC
#     define DEF_INSIDE_PUBLIC_TEMPLATE
#     define DEF_INSIDE_LOCAL
#   endif

#   define DEF_OUTSIDE_PUBLIC DEF_INSIDE_PUBLIC
#   define DEF_OUTSIDE_PUBLIC_TEMPLATE DEF_INSIDE_PUBLIC_TEMPLATE
#   define DEF_OUTSIDE_LOCAL DEF_INSIDE_LOCAL

# endif

#endif
