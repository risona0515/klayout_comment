#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zh_annotate.py -- tooling for the Chinese/English bilingual source annotation
of the KLayout library.

[[ZH]] 双语注释工具

Why this exists / 为什么需要这个工具
------------------------------------
KLayout is an actively maintained upstream project.  Editing its sources in
place makes it painful to merge future upstream changes.  To keep that pain
bounded, every annotation added by this effort follows two hard rules:

  KLayout 是一个持续活跃的上游项目。直接就地修改源码会让日后合并上游更新变得
  非常痛苦。为把痛苦控制在可接受范围内，本次注释工作遵守两条硬规则：

  1. ADDITIVE ONLY / 只增行
     No existing line is ever modified, deleted or reordered.  Annotations are
     inserted as brand-new lines that always carry the marker `[[ZH]]`.
     绝不修改、删除或重排任何既有行。注释一律作为新行插入，且必定带有
     `[[ZH]]` 标记。

  2. MACHINE STRIPPABLE / 可机器剥离
     `strip` removes every annotated line and must reproduce the pristine
     upstream file byte for byte.  `verify` proves that this is the case.
     `strip` 会移除所有注释行，且必须逐字节还原出未改动的上游文件。
     `verify` 用来证明这一点。

Marker syntax / 标记语法
------------------------
    // [[ZH]] 单行说明                  -- one-line annotation
    // [[ZH-BEGIN]]                     -- start of an annotation block
    // 功能：...
    // 参数：...
    // [[ZH-END]]                       -- end of an annotation block
    // [[ZH]] 修改：...                  -- a state-change note in a data flow

Only line comments (`//`) are used, never Doxygen blocks (`/** */`).  This is
deliberate: it leaves the existing English Doxygen documentation semantically
untouched and avoids duplicate-parameter warnings.
只使用行注释（`//`），绝不使用 Doxygen 块（`/** */`）。这是刻意的：既有的英文
Doxygen 文档语义因此完全不变，也不会产生重复参数警告。

Usage / 用法
------------
    python3 scripts/zh_annotate/zh_annotate.py verify <file> [<file> ...]
    python3 scripts/zh_annotate/zh_annotate.py strip  <file> [<file> ...]
    python3 scripts/zh_annotate/zh_annotate.py stats  <file> [<file> ...]

The `verify` command compares against the pristine file taken from git
(`git show HEAD:<file>`), so it must be run from inside the repository.
`verify` 命令会与取自 git 的原始文件（`git show HEAD:<file>`）比对，因此必须在
仓库内运行。

Exit codes / 退出码
-------------------
    0  all checks passed
    1  at least one check failed
    2  usage error
"""

import os
import re
import subprocess
import sys

# Markers identifying annotated lines.
# 用于标识注释行的标记。
MARKER = "[[ZH]]"
BLOCK_BEGIN = "[[ZH-BEGIN]]"
BLOCK_END = "[[ZH-END]]"

# The three markers we recognise at the start of a line comment.
# 我们认可三种位于行注释起始处的标记。
START_RE = re.compile(r"^[ \t]*//[ \t]*\[\[ZH(-BEGIN|-END)?\]\][ \t]*")
BEGIN_RE = re.compile(r"^[ \t]*//[ \t]*\[\[ZH-BEGIN\]\][ \t]*$")
END_RE = re.compile(r"^[ \t]*//[ \t]*\[\[ZH-END\]\][ \t]*$")

# Files touched by this effort contain Chinese text, so they must be valid
# UTF-8.
# 本次注释的文件含中文，因此必须是合法 UTF-8。
ENCODING = "utf-8"


class CheckFailure(Exception):
    """Raised when a verification check fails. / 校验失败时抛出。"""


def read_lines(path):
    """Read a file as UTF-8 and return its lines (keeping line endings).

    以 UTF-8 读取文件并返回其行列表（保留行尾符）。
    """
    with open(path, "rb") as fp:
        raw = fp.read()
    try:
        text = raw.decode(ENCODING)
    except UnicodeDecodeError as exc:
        raise CheckFailure(
            "%s: not valid UTF-8 / 不是合法的 UTF-8: %s" % (path, exc)
        )
    # splitlines(keepends=True) preserves the exact line terminators so that
    # round-tripping is byte exact.
    # splitlines(keepends=True) 会保留精确的行尾符，从而保证逐字节往返一致。
    return text.splitlines(keepends=True)


def build_removal_mask(path, lines):
    """Return a bool list marking which lines are annotations to be removed.

    返回一个布尔列表，标记哪些行是需要移除的注释行。

    Also validates marker well-formedness and block pairing as a side effect,
    raising CheckFailure on any violation.
    同时校验标记格式与块配对，发现违规即抛出 CheckFailure。
    """
    mask = []
    in_block = False
    block_start = 0
    n_marked = 0
    n_blocks = 0

    for lineno, line in enumerate(lines, 1):
        if in_block:
            # Inside a block every line belongs to the annotation, regardless
            # of its content.
            # 块内所有行都属于注释，无论内容如何。
            mask.append(True)
            if END_RE.match(line):
                in_block = False
            continue

        if BEGIN_RE.match(line):
            in_block = True
            block_start = lineno
            n_blocks += 1
            n_marked += 1
            mask.append(True)
            continue

        if END_RE.match(line):
            raise CheckFailure(
                "%s:%d: [[ZH-END]] without a matching [[ZH-BEGIN]] / "
                "块结束标记没有对应的开始标记" % (path, lineno)
            )

        if START_RE.match(line):
            n_marked += 1
            mask.append(True)
            continue

        # An ordinary line must not carry a marker token in a trailing comment,
        # otherwise stripping would leave a dangling fragment behind.  We check
        # for the token anywhere in the line to catch misplaced markers.
        # 普通行不应在尾随注释里带有标记记号，否则剥离后会留下残缺片段。
        # 这里在整行范围内查找记号，以捕获位置写错的标记。
        if MARKER in line or BLOCK_BEGIN in line or BLOCK_END in line:
            raise CheckFailure(
                "%s:%d: marker token found in a position that is not the start "
                "of a line comment / 标记记号不在行注释起始处: %r"
                % (path, lineno, line.rstrip("\n"))
            )

        mask.append(False)

    if in_block:
        raise CheckFailure(
            "%s:%d: unclosed [[ZH-BEGIN]] block / 有未闭合的注释块"
            % (path, block_start)
        )

    return mask, n_marked, n_blocks


def strip_annotations(mask, lines):
    """Remove the lines flagged in `mask`. / 移除 `mask` 中标记的行。"""
    return [ln for ln, drop in zip(lines, mask) if not drop]


def pristine_lines(path):
    """Fetch the pristine version of `path` from git.

    从 git 中取出 `path` 的原始版本（HEAD 版本）。

    Raises CheckFailure if the file is untracked or git is unavailable.
    若文件未被 git 跟踪或 git 不可用则抛出 CheckFailure。
    """
    repo_root = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        capture_output=True, text=True,
    )
    if repo_root.returncode != 0:
        raise CheckFailure("not inside a git repository / 不在 git 仓库内")

    root = repo_root.stdout.strip()
    abspath = os.path.abspath(path)
    if not abspath.startswith(root + os.sep):
        raise CheckFailure("%s: outside the repository / 不在仓库内" % path)

    relpath = os.path.relpath(abspath, root)
    show = subprocess.run(
        ["git", "show", "HEAD:%s" % relpath],
        capture_output=True, cwd=root,
    )
    if show.returncode != 0:
        raise CheckFailure(
            "%s: not tracked by git at HEAD / 未被 git 跟踪: %s"
            % (path, show.stderr.decode("utf-8", "replace").strip())
        )

    try:
        text = show.stdout.decode(ENCODING)
    except UnicodeDecodeError as exc:
        raise CheckFailure(
            "%s: pristine file is not valid UTF-8 / 原始文件不是合法 UTF-8: %s"
            % (path, exc)
        )
    return text.splitlines(keepends=True)


def is_untracked(exc):
    """True if the failure means 'file is new and has no HEAD version'.

    当失败原因是「文件是新增的、HEAD 中不存在」时返回 True。
    """
    return "not tracked by git at HEAD" in str(exc)


def verify(path):
    """Verify that `path` is a purely additive annotation of its HEAD version.

    校验 `path` 相对其 HEAD 版本是「纯增量注释」。

    Returns (annotation line count, block count, added line count).
    返回 (注释行数, 注释块数, 新增行数)。
    """
    if not os.path.isfile(path):
        raise CheckFailure("%s: no such file / 文件不存在" % path)

    lines = read_lines(path)
    mask, n_marked, n_blocks = build_removal_mask(path, lines)
    stripped = strip_annotations(mask, lines)

    # A file that git does not track yet (e.g. a newly added one) has no
    # pristine counterpart; in that case we only check marker well-formedness.
    # 对于 git 尚未跟踪的新文件没有原始版本可比，此时只校验标记本身。
    try:
        original = pristine_lines(path)
    except CheckFailure as exc:
        if is_untracked(exc):
            return n_marked, n_blocks, len(lines)
        raise

    # THE central guarantee: removing annotations reproduces the upstream file
    # exactly.  Any mismatch means an existing line was touched.
    # 核心保证：移除注释后必须与上游文件完全一致。任何不一致都说明改动到了既有行。
    if stripped != original:
        detail = first_difference(stripped, original)
        raise CheckFailure(
            "%s: stripping [[ZH]] lines does NOT reproduce HEAD "
            "(a non-annotation line was modified, added or removed) / "
            "剥离注释行后无法还原 HEAD 版本（说明有非注释行被修改/新增/删除）\n"
            "    %s" % (path, detail)
        )

    return n_marked, n_blocks, len(lines) - len(original)


def first_difference(got, want):
    """Describe the first differing line between two line lists.

    描述两个行列表中第一处差异，便于定位问题。
    """
    n = max(len(got), len(want))
    for i in range(n):
        g = got[i] if i < len(got) else "<missing>"
        w = want[i] if i < len(want) else "<missing>"
        if g != w:
            return ("first mismatch at stripped line %d:\n"
                    "      got : %r\n"
                    "      want: %r" % (i + 1, g, w))
    return "no difference found / 未发现差异"


def cmd_verify(paths):
    """Implement the `verify` subcommand. / 实现 `verify` 子命令。"""
    ok = True
    total_annotations = 0
    total_blocks = 0
    total_added = 0
    for path in paths:
        try:
            n_marked, n_blocks, n_added = verify(path)
        except CheckFailure as exc:
            print("FAIL  %s" % exc)
            ok = False
            continue
        total_annotations += n_marked
        total_blocks += n_blocks
        total_added += n_added
        print("ok    %-60s  %5d annotation lines, %4d blocks, %+5d lines"
              % (path, n_marked, n_blocks, n_added))

    if ok:
        print("\nAll %d file(s) verified: purely additive, fully strippable."
              % len(paths))
        print("全部 %d 个文件校验通过：纯增量注释，可完全剥离还原。" % len(paths))
        print("Total: %d annotation lines, %d blocks, %+d lines."
              % (total_annotations, total_blocks, total_added))
        return 0

    print("\nVerification FAILED / 校验失败.")
    return 1


def cmd_strip(paths):
    """Implement the `strip` subcommand. / 实现 `strip` 子命令。"""
    for path in paths:
        lines = read_lines(path)
        # Validate first: never strip a file with malformed markers, because a
        # missing [[ZH-END]] would otherwise delete real code.
        # 先校验：标记有问题的文件绝不剥离，否则缺失 [[ZH-END]] 会删掉真实代码。
        mask, _, _ = build_removal_mask(path, lines)
        with open(path, "w", encoding=ENCODING, newline="") as fp:
            fp.write("".join(strip_annotations(mask, lines)))
        print("stripped %s" % path)
    return 0


def cmd_stats(paths):
    """Implement the `stats` subcommand. / 实现 `stats` 子命令。"""
    for path in paths:
        try:
            n_marked, n_blocks, n_added = verify(path)
        except CheckFailure as exc:
            print("FAIL  %s" % exc)
            continue
        print("%-60s  %5d annotation lines, %4d blocks, %+5d lines"
              % (path, n_marked, n_blocks, n_added))
    return 0


COMMANDS = {
    "verify": cmd_verify,
    "strip": cmd_strip,
    "stats": cmd_stats,
}


def main(argv):
    """Entry point. / 程序入口。"""
    if len(argv) < 3 or argv[1] not in COMMANDS:
        sys.stderr.write(
            "usage: %s {verify|strip|stats} <file> [<file> ...]\n" % argv[0]
        )
        sys.stderr.write(
            "       verify -- prove that annotations are purely additive\n"
            "       strip  -- remove all [[ZH]] annotation lines in place\n"
            "       stats  -- report annotation counts per file\n"
        )
        return 2
    return COMMANDS[argv[1]](argv[2:])


if __name__ == "__main__":
    sys.exit(main(sys.argv))
