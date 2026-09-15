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

# Optional explicit override for the pristine base ref, e.g. ZH_BASE=v0.30.12
# 可选的显式基准 ref 覆盖，例如 ZH_BASE=v0.30.12
ENV_BASE_REF = os.environ.get("ZH_BASE", "").strip()

# Candidate names for the upstream branch we branched off from.
# 我们从中分叉出来的上游分支的候选名字。
UPSTREAM_CANDIDATES = ("master", "main", "origin/master", "origin/main")


def base_ref(root):
    """Resolve the pristine base ref used for comparison.

    解析用于比对的「原始基准」ref。

    WHY NOT HEAD: we commit our own annotations as we go, so HEAD moves and
    eventually contains the annotations themselves.  Comparing against HEAD
    would then demand that stripping remove *fewer* lines than we added.
    The correct baseline is the commit the annotation branch started from --
    i.e. the merge base with the upstream branch.

    为什么不直接用 HEAD：我们会不断提交自己的注释，HEAD 会随之前移并最终包含
    注释本身。此时以 HEAD 为基准会要求"剥离掉的行数少于实际新增"，判断必然失败。
    正确的基准是注释分支的起点，即与上游分支的 merge base。

    Resolution order / 解析顺序:
      1. $ZH_BASE if set                  / 若设置了 $ZH_BASE
      2. merge-base(HEAD, upstream)       / 与上游分支的合并基点
      3. HEAD (degenerate fallback)       / 退化回退到 HEAD
    """
    if ENV_BASE_REF:
        return ENV_BASE_REF
    for candidate in UPSTREAM_CANDIDATES:
        r = subprocess.run(
            ["git", "merge-base", "HEAD", candidate],
            capture_output=True, text=True, cwd=root,
        )
        if r.returncode == 0 and r.stdout.strip():
            return r.stdout.strip()
    return "HEAD"


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


def check_insertion_context(path, lines, mask):
    """Check that no annotation line lands where a `//` comment changes meaning.

    检查没有注释行落在「加一个 // 会改变语义」的位置。

    Stripping annotations back to the pristine file proves we did not *edit* any
    code.  It does NOT by itself prove that inserting a `//` line is harmless in
    every context.  Three contexts make it harmful:

      1. after a line-continuation backslash -- line splicing (phase 2) happens
         BEFORE comment removal (phase 3), so the annotation can end up inside
         the joined logical line and comment out the rest of a macro;
      2. inside a raw string literal R"(...)" -- the text becomes string content;
      3. inside a block comment -- harmless to the compiler, but the annotation
         becomes invisible to a reader, which defeats its purpose.

    剥离注释能证明我们没有**修改**任何代码，但不能证明在每个位置上插入一行
    `//` 都是无害的。有三种上下文会让它有害：
      1. 紧跟在续行反斜杠之后 —— 行拼接（第 2 阶段）先于注释移除（第 3 阶段），
         注释可能被拼进同一条逻辑行，从而把宏的其余部分注释掉；
      2. 落在原始字符串 R"(...)" 内部 —— 注释会变成字符串内容；
      3. 落在块注释内部 —— 对编译器无害，但读者看不到，失去注释的意义。

    This is a heuristic lexer, not a full C++ parser: it is deliberately
    conservative and may under-report exotic constructs, never over-report.
    这里是一个启发式词法扫描，不是完整的 C++ 解析器：刻意保守，
    可能漏报冷门写法，但不会误报。
    """
    problems = []
    in_block_comment = False
    in_raw_string = False
    prev_code = None

    for lineno, (line, is_ann) in enumerate(zip(lines, mask), 1):
        if is_ann:
            if prev_code is not None and prev_code.rstrip("\n").endswith("\\"):
                problems.append(
                    "%s:%d: annotation follows a line-continuation backslash "
                    "(it may be spliced into the preceding macro) / "
                    "注释紧跟在续行反斜杠之后（可能被拼进上一行的宏）"
                    % (path, lineno))
            if in_raw_string:
                problems.append(
                    "%s:%d: annotation inserted inside a raw string literal / "
                    "注释落在原始字符串内部" % (path, lineno))
            if in_block_comment:
                problems.append(
                    "%s:%d: annotation inserted inside a block comment / "
                    "注释落在块注释内部" % (path, lineno))
            continue

        prev_code = line

        # Strip an obvious trailing line comment so a '//' inside it does not
        # confuse the state tracking.  Quotes are left alone on purpose: getting
        # this wrong only causes under-reporting, which is the safe direction.
        # 去掉明显的行尾注释，避免其中的 // 干扰状态跟踪。
        # 刻意不处理引号：判断错只会漏报，属于安全方向。
        code = line.split("//")[0] if '"' not in line.split("//")[0] else line

        # Toggle raw-string state.  Only the real raw-string syntax `R"(` (and the
        # `R"delim(` form) opens one.  Matching a bare `R"` is wrong: prose such as
        # `"R" is the type ...` in a doc comment contains it and would latch
        # in_raw_string on for the rest of the file, producing false positives.
        # 切换原始字符串状态。只有真正的原始字符串语法 `R"(`（以及 `R"delim(` 形式）
        # 才会开启它。只匹配 `R"` 是错的：文档注释里的 `"R" is the type ...` 这类文字
        # 含有它，会把 in_raw_string 一直误置为真，导致后续全是误报。
        if not in_block_comment and 'R"' in code:
            rpos = code.find('R"')
            # The char after R"( or R"<delim>( must be '(' eventually; accept the
            # common `R"(` and `R"xxx(` shapes.
            rest = code[rpos + 2:]
            if rest.startswith("(") or ("(" in rest.split(")")[0]
                                        and rest[:rest.find("(")].replace("-", "").isalnum()):
                in_raw_string = not in_raw_string

        # Track block comments, ignoring one-line /* ... */ pairs.
        # 跟踪块注释，忽略单行 /* ... */ 对。
        if not in_raw_string:
            if "/*" in code and "*/" not in code:
                in_block_comment = True
            elif "*/" in code:
                in_block_comment = False

    return problems


def pristine_lines(root, relpath, ref):
    """Fetch the pristine version of a file from git.

    从 git 中取出某个文件的原始版本。

    `ref` is the base ref resolved by base_ref() (the annotation branch point),
    not HEAD.  `ref` 是由 base_ref() 解析出的基准 ref（注释分支的起点），不是 HEAD。

    Raises CheckFailure if the file is untracked at `ref`.
    若文件在该 ref 上未被跟踪则抛出 CheckFailure。
    """
    show = subprocess.run(
        ["git", "show", "%s:%s" % (ref, relpath)],
        capture_output=True, cwd=root,
    )
    if show.returncode != 0:
        raise CheckFailure(
            "%s: not tracked by git at %s / 在该基准 ref 上未被跟踪: %s"
            % (relpath, ref, show.stderr.decode("utf-8", "replace").strip())
        )

    try:
        text = show.stdout.decode(ENCODING)
    except UnicodeDecodeError as exc:
        raise CheckFailure(
            "%s: pristine file is not valid UTF-8 / 原始文件不是合法 UTF-8: %s"
            % (relpath, exc)
        )
    return text.splitlines(keepends=True)


def is_untracked(exc):
    """True if the failure means 'file is new and has no base version'.

    当失败原因是「文件是新增的、基准 ref 中不存在」时返回 True。
    """
    return "not tracked by git at" in str(exc)


def repo_root():
    """Return the absolute path of the repository root.

    返回仓库根目录的绝对路径。
    """
    r = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        capture_output=True, text=True,
    )
    if r.returncode != 0:
        raise CheckFailure("not inside a git repository / 不在 git 仓库内")
    return r.stdout.strip()


def verify(path, root=None, ref=None):
    """Verify that `path` is a purely additive annotation of its base version.

    校验 `path` 相对其「基准版本」是纯增量注释。

    Returns (annotation line count, block count, added line count).
    返回 (注释行数, 注释块数, 新增行数)。
    """
    if not os.path.isfile(path):
        raise CheckFailure("%s: no such file / 文件不存在" % path)

    own_root = root is None
    if own_root:
        root = repo_root()
    if ref is None:
        ref = base_ref(root)

    abspath = os.path.abspath(path)
    if not abspath.startswith(root + os.sep):
        raise CheckFailure("%s: outside the repository / 不在仓库内" % path)
    relpath = os.path.relpath(abspath, root)

    lines = read_lines(path)
    mask, n_marked, n_blocks = build_removal_mask(path, lines)
    stripped = strip_annotations(mask, lines)

    # Guard the orthogonal risk: stripping proves we edited nothing, but not that
    # a `//` line is harmless where we put it.  See check_insertion_context.
    # 检查正交风险：剥离能证明没改代码，但不能证明插入的 // 在位置上无害。
    context_problems = check_insertion_context(path, lines, mask)
    if context_problems:
        raise CheckFailure("\n".join(context_problems))

    try:
        original = pristine_lines(root, relpath, ref)
    except CheckFailure as exc:
        # A brand-new file has no base counterpart to compare against.
        # 新增文件没有基准版本可比对。
        if is_untracked(exc):
            return n_marked, n_blocks, len(lines)
        raise

    # THE central guarantee: removing annotations reproduces the upstream file
    # exactly.  Any mismatch means an existing line was touched.
    # 核心保证：移除注释后必须与上游文件完全一致。任何不一致都说明改动到了既有行。
    if stripped != original:
        detail = first_difference(stripped, original)
        raise CheckFailure(
            "%s: stripping [[ZH]] lines does NOT reproduce the base version "
            "(%s) -- a non-annotation line was modified, added or removed / "
            "剥离注释行后无法还原基准版本（%s）—— 说明有非注释行被修改/新增/删除\n"
            "    %s" % (path, ref, ref, detail)
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
            # The most common mistakes all amount to "a line was added or
            # changed that does not carry a marker".  Name the likely ones.
            # 最常见的错误都属于同一类：「新增或改动了不带标记的行」。
            # 下面直接点出最可能的几种。
            hint = ""
            if not g.strip() or not w.strip():
                hint = (
                    "\n      HINT: this looks like an unmarked blank line. "
                    "Never leave a blank line outside a "
                    "[[ZH-BEGIN]]/[[ZH-END]] block, and never add one right "
                    "after [[ZH-END]] -- reuse the existing blank line or put "
                    "the separation inside the block."
                    "\n      提示：这看起来是未标记的空行。不要在 "
                    "[[ZH-BEGIN]]/[[ZH-END]] 块之外留空行，"
                    "尤其不要紧跟在 [[ZH-END]] 之后加空行；"
                    "应复用已有的空行，或把空行放进块内。")
            elif w.strip().startswith(("/**", "*", "*/")) or \
                    g.strip().startswith(("/**", "*", "*/")):
                hint = (
                    "\n      HINT: this looks like an added or edited doc "
                    "comment block. EVERY added line must be marked: turn the "
                    "new block into '// [[ZH-BEGIN]] ... // [[ZH-END]]' lines, "
                    "and never rewrite an existing '/** ... */' block -- add "
                    "beside it instead."
                    "\n      提示：这看起来是新增或改写了文档注释块。"
                    "新增的每一行都必须带标记：把新块写成 "
                    "'// [[ZH-BEGIN]] ... // [[ZH-END]]' 形式；"
                    "而原有的 '/** ... */' 块绝不可改写，只能在旁边新增。")
            elif g.strip().endswith(";") or w.strip().endswith(";"):
                hint = (
                    "\n      HINT: this looks like a code line was swallowed "
                    "into a comment or merged with an adjacent line -- commonly "
                    "caused by a literal backslash-n typed into the annotation "
                    "text instead of a real line break."
                    "\n      提示：这看起来是一行代码被吞进注释、或与相邻行合并了 ——"
                    "常见原因是注释文本里写了字面的 \\n 而不是真正的换行。")
            return ("first mismatch at stripped line %d:\n"
                    "      got : %r\n"
                    "      want: %r%s" % (i + 1, g, w, hint))
    return "no difference found / 未发现差异"


def cmd_verify(paths):
    """Implement the `verify` subcommand. / 实现 `verify` 子命令。"""
    try:
        root = repo_root()
        ref = base_ref(root)
    except CheckFailure as exc:
        print("FAIL  %s" % exc)
        return 1

    print("Base ref for comparison / 比对基准: %s" % ref)
    print("(the annotation branch point, not HEAD -- HEAD contains our "
          "annotations)\n")

    ok = True
    total_annotations = 0
    total_blocks = 0
    total_added = 0
    for path in paths:
        try:
            n_marked, n_blocks, n_added = verify(path, root, ref)
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
    try:
        root = repo_root()
        ref = base_ref(root)
    except CheckFailure as exc:
        print("FAIL  %s" % exc)
        return 1

    for path in paths:
        try:
            n_marked, n_blocks, n_added = verify(path, root, ref)
        except CheckFailure as exc:
            print("FAIL  %s" % exc)
            continue
        print("%-60s  %5d annotation lines, %4d blocks, %+5d lines"
              % (path, n_marked, n_blocks, n_added))
    return 0


def cmd_autofix(paths):
    """Implement the `autofix` subcommand.

    实现 `autofix` 子命令。

    Removes the stray blank lines that are the single most common mistake when
    inserting an annotation block by hand -- a blank line added just before
    [[ZH-BEGIN]] or just after [[ZH-END]] where the original had none.  Those
    lines carry no marker, so stripping leaves them behind and byte-exactness
    breaks.

    移除手工插入注释块时最常见的多余空行 —— 在原文没有空行的地方，
    于 [[ZH-BEGIN]] 之前或 [[ZH-END]] 之后多加的空行。这类行不带标记，
    因此剥离后会残留，破坏逐字节还原。

    The fix is *verified*: a candidate edit is kept only if it makes `verify`
    pass.  That makes it safe to run on a file without understanding the exact
    mismatch -- if no candidate helps, nothing is written.

    修复是**经过校验的**：只有让 `verify` 通过的候选改动才会被保留。
    因此在不必看懂具体差异的情况下也可以安全运行 —— 若无候选有效，则不写入。
    """
    root = repo_root()
    ref = base_ref(root)
    for path in paths:
        if not os.path.isfile(path):
            print("FAIL  %s: no such file" % path)
            continue

        # Already fine?  If verify passes there is nothing to do.
        try:
            verify(path, root, ref)
            print("ok    %s (already verified, unchanged)" % path)
            continue
        except CheckFailure:
            pass

        original = read_lines(path)
        fixed = _try_blank_line_fixes(path, original, root, ref)
        if fixed is None:
            print("HINT  %s: verify fails, but no blank-line fix applies -- "
                  "inspect manually / 校验失败，但空行修复不适用，请手工检查"
                  % path)
            continue

        with open(path, "w", encoding=ENCODING, newline="") as fp:
            fp.write("".join(fixed))
        n_marked, n_blocks, n_added = verify(path, root, ref)
        print("FIXED %s (%d ann.lines, %d blocks, %+d lines)"
              % (path, n_marked, n_blocks, n_added))
    return 0


def _verifies_against_pristine(kept_lines, pristine, path):
    """True if stripping markers from `kept_lines` reproduces `pristine`.

    若从 `kept_lines` 剥离标记后能还原 `pristine`，则返回 True。

    IMPORTANT: this must NOT go through the file-based `verify`, because that
    compares against the git version of the path it is given -- so a temporary
    file would appear "untracked" and verify would report success for ANY content
    (it assumes a new file has nothing to compare against).  An earlier version of
    autofix wrote candidates to a temp file and therefore accepted the first
    candidate unconditionally, which made it silently wrong.

    重要：这里**不能**走基于文件的 `verify`，因为它比对的是“给定路径的 git 版本”——
    临时文件会被视为“未跟踪”，而 verify 对未跟踪文件会直接返回成功
    （它假定新文件没有比对对象）。早先版本的 autofix 把候选写到临时文件，
    因此会无条件接受第一个候选，导致**静默错误**。

    Comparing the lines directly removes that whole failure mode.
    直接比较行内容可以彻底消除这一失效模式。
    """
    try:
        mask, _, _ = build_removal_mask(path, kept_lines)
    except CheckFailure:
        return False
    return strip_annotations(mask, kept_lines) == pristine


def _try_blank_line_fixes(path, lines, root, ref):
    """Repair missing/extra BLANK lines by aligning against the pristine file.

    通过与原始文件对齐，修复**空行**的缺失或多余。

    Method / 方法：
      1. strip the markers to get what upstream would see;
      2. align that against the pristine lines with difflib;
      3. keep only the differences that consist of BLANK lines -- every such
         "insert" means we must add a blank line at that point, every such
         "delete" means we must remove one;
      4. map the position back into the annotated file and apply the edit;
      5. accept the result only if it reproduces the pristine file exactly.

    1. 剥离标记，得到“上游会看到的”内容；
    2. 用 difflib 与原始行对齐；
    3. **只保留由空行构成**的差异 —— 每个 "insert" 表示该处需补一个空行，
       每个 "delete" 表示该处需删一个空行；
    4. 把位置映射回带注释的文件并施加改动；
    5. 仅当结果能精确还原原始文件时才接受。

    This is deliberately conservative: any difference involving a non-blank line
    (i.e. real code) makes it give up and say "inspect manually", rather than
    guessing.  This replaces an earlier candidate-guessing approach that was
    unreliable (it verified a temporary file, which git sees as untracked, so
    verify accepted anything).
    本函数刻意保守：只要差异涉及**非空行**（即真实代码），就直接放弃并提示
    “请手工检查”，而不是猜测。它取代了早期那种“枚举候选”的做法 ——
    那种做法不可靠（它校验的是临时文件，而 git 视其为未跟踪，导致 verify 一律通过）。
    """
    pristine = pristine_lines(root, os.path.relpath(os.path.abspath(path), root), ref)

    try:
        mask, _, _ = build_removal_mask(path, lines)
    except CheckFailure:
        return None
    stripped = strip_annotations(mask, lines)

    # Map each stripped index -> index in the annotated file.
    # 建立「剥离后下标 -> 带注释文件下标」的映射。
    stripped_to_file = [i for i, drop in enumerate(mask) if not drop]

    import difflib
    sm = difflib.SequenceMatcher(a=stripped, b=pristine, autojunk=False)

    # Collect edits as (stripped_position, kind).  kind is "add" or "del".
    # 收集编辑点：(剥离后位置, 类型)。
    edits = []
    for tag, i1, i2, j1, j2 in sm.get_opcodes():
        if tag == "equal":
            continue
        a_lines = stripped[i1:i2]
        b_lines = pristine[j1:j2]
        # Only blank-line differences are repairable.
        # 只有“空行”差异是可修复的。
        if any(t.strip() for t in a_lines) or any(t.strip() for t in b_lines):
            return None
        if j2 - j1 > i2 - i1:
            edits.append((i1, "add", (j2 - j1) - (i2 - i1)))
        else:
            edits.append((i1, "del", (i2 - i1) - (j2 - j1)))

    if not edits:
        return None

    # Apply from the end so earlier positions stay valid.
    # 从后往前施加，避免前面位置失效。
    out = list(lines)
    for pos, kind, count in reversed(edits):
        if pos < len(stripped_to_file):
            file_pos = stripped_to_file[pos]
        else:
            file_pos = len(out)
        if kind == "add":
            out[file_pos:file_pos] = ["\n"] * count
        else:
            removed = 0
            i = file_pos
            while i < len(out) and removed < count:
                if out[i].strip() == "":
                    del out[i]
                    removed += 1
                else:
                    i += 1
            if removed < count:
                return None

    if _verifies_against_pristine(out, pristine, path):
        return out
    return None


COMMANDS = {
    "verify": cmd_verify,
    "strip": cmd_strip,
    "stats": cmd_stats,
    "autofix": cmd_autofix,
}


def main(argv):
    """Entry point. / 程序入口。"""
    if len(argv) < 3 or argv[1] not in COMMANDS:
        sys.stderr.write(
            "usage: %s {verify|strip|stats|autofix} <file> [<file> ...]\n" % argv[0]
        )
        sys.stderr.write(
            "       verify  -- prove that annotations are purely additive\n"
            "       strip   -- remove all [[ZH]] annotation lines in place\n"
            "       stats   -- report annotation counts per file\n"
            "       autofix -- remove stray blank lines beside marker blocks,\n"
            "                  keeping the change only if verify then passes\n"
        )
        return 2
    return COMMANDS[argv[1]](argv[2:])


if __name__ == "__main__":
    sys.exit(main(sys.argv))
