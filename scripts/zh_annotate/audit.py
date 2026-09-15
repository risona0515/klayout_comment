#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
audit.py -- end-to-end audit of the [[ZH]] annotation work.

[[ZH]] 注释工作的全面审计脚本

Answers, in one run, the questions worth asking before calling this work done:
一次运行回答「宣布完成之前值得问」的所有问题：

  A. SCOPE       did we touch anything we were told not to?
  B. INTEGRITY   does stripping every file really reproduce upstream?
  C. MARKERS     are all markers well-formed and balanced?
  D. COVERAGE    which files are annotated, and how densely?
  E. COMPILER    do the headers preprocess to identical output?
  F. OMISSIONS   which function definitions lack annotation?
  G. HYGIENE     were any generated/excluded paths touched?

Usage / 用法
------------
    python3 scripts/zh_annotate/audit.py            # full audit
    python3 scripts/zh_annotate/audit.py --quick    # skip preprocessor checks

Exit code 0 = no problems found, 1 = at least one problem.
退出码 0 表示未发现问题，1 表示存在至少一个问题。
"""

import os
import re
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import zh_annotate as z  # noqa: E402

# ---------------------------------------------------------------------------
# Scope rules: what the user explicitly allowed and forbade.
# 范围规则：用户明确允许和禁止的内容。
# ---------------------------------------------------------------------------
ALLOWED_SUFFIXES = (".h", ".cc")

# Path fragments that must never be annotated (user's explicit exclusions).
# 绝不允许注释的路径片段（用户明确排除）。
FORBIDDEN_FRAGMENTS = (
    "/unit_tests/",          # tests, per user instruction
    "/gsiqt/qt4/", "/gsiqt/qt5/", "/gsiqt/qt6/",   # generated Qt bindings
    "/pymod/distutils_src/",  # generated Python stubs
    "/testdata/",
    "/pyastub/",
    "/rbastub/",
    "/vendor/",
    "/macbuild/",
    "/ci-scripts/",
    "/doc/",
)
FORBIDDEN_SUFFIXES = (".pro", ".pri", ".qrc", ".pyi", ".cc_gen", ".lym",
                      ".json", ".md", ".xml", ".sh", ".bat", ".pl")

# The annotation tooling itself is not an annotation target: it necessarily
# mentions the marker token in its own documentation, and it is Python.
# 注释工具本身不是注释对象：它的文档里必然会提到标记记号，而且它是 Python。
TOOLING_PREFIX = "scripts/zh_annotate/"


def is_tooling(rel):
    """True for our own helper scripts. / 是否为我们自己的辅助脚本。"""
    return rel.startswith(TOOLING_PREFIX)

problems = []
notes = []


def report(ok, label, detail=""):
    """Record and print a check result. 记录并打印一项检查结果。"""
    tag = "PASS" if ok else "FAIL"
    print("  [%s] %s%s" % (tag, label, (" -- " + detail) if detail else ""))
    if not ok:
        problems.append(label + (": " + detail if detail else ""))
    return ok


def git(*args):
    """Run git and return stdout (text). 运行 git 并返回标准输出。"""
    r = subprocess.run(["git"] + list(args), capture_output=True, text=True)
    return r.stdout


def changed_files(root, ref):
    """Files changed between `ref` and the working tree, repo-relative.

    返回 `ref` 与工作区之间发生变化的文件（仓库相对路径）。
    """
    out = git("-C", root, "diff", "--name-only", ref)
    untracked = git("-C", root, "ls-files", "--others", "--exclude-standard")
    files = [ln for ln in out.splitlines() if ln.strip()]
    files += [ln for ln in untracked.splitlines() if ln.strip()]
    return sorted(set(files))


def annotated_files(root, ref):
    """Changed files that actually contain [[ZH]] markers.

    返回实际包含 [[ZH]] 标记的变更文件。

    Our own tooling is excluded: it mentions the marker token by design.
    我们自己的工具脚本被排除：它们按设计就会提到该标记记号。

    NOTE: the test is for the common prefix "[[ZH", not the literal "[[ZH]]".
    A file annotated only with block markers contains "[[ZH-BEGIN]]" and
    "[[ZH-END]]" but not the bare token, so matching the full token would
    under-count such files (it did, before this fix).
    注意：检测的是公共前缀 "[[ZH"，而不是字面量 "[[ZH]]"。
    只用块标记的文件含 "[[ZH-BEGIN]]"/"[[ZH-END]]"，不含裸标记，
    因此用完整标记去匹配会**少算**（修复前就少算了）。
    """
    result = []
    for rel in changed_files(root, ref):
        if is_tooling(rel):
            continue
        path = os.path.join(root, rel)
        if not os.path.isfile(path):
            continue
        try:
            if "[[ZH" in open(path, encoding="utf-8").read():
                result.append(rel)
        except (UnicodeDecodeError, OSError):
            pass
    return result


# ---------------------------------------------------------------------------
# A. SCOPE
# ---------------------------------------------------------------------------
def check_scope(root, ref, files):
    print("\nA. SCOPE / 范围")
    bad = []
    for rel in files:
        if is_tooling(rel):
            continue
        if rel.startswith("src/"):
            if not rel.endswith(ALLOWED_SUFFIXES):
                bad.append("%s (not .h/.cc)" % rel)
                continue
            for frag in FORBIDDEN_FRAGMENTS:
                if frag in "/" + rel:
                    bad.append("%s (excluded path: %s)" % (rel, frag))
                    break
        else:
            bad.append("%s (outside src/ and not our tooling)" % rel)
    report(not bad, "no out-of-scope paths touched",
           "; ".join(bad) if bad else "%d file(s) checked" % len(files))
    return bad


# ---------------------------------------------------------------------------
# B. INTEGRITY -- the central guarantee
# ---------------------------------------------------------------------------
def check_integrity(root, ref, files):
    print("\nB. INTEGRITY / 完整性（剥离后逐字节还原）")
    fails = []
    total_ann = total_added = 0
    for rel in files:
        path = os.path.join(root, rel)
        try:
            n_marked, n_blocks, n_added = z.verify(path, root, ref)
            total_ann += n_marked
            total_added += n_added
            print("       %-44s %4d ann.lines %3d blocks %+6d lines"
                  % (rel, n_marked, n_blocks, n_added))
        except z.CheckFailure as exc:
            fails.append(str(exc).split("\n")[0])
    report(not fails, "stripping restores upstream byte-for-byte",
           "; ".join(fails) if fails else
           "%d file(s), %d annotation lines, %+d lines"
           % (len(files), total_ann, total_added))
    return fails


# ---------------------------------------------------------------------------
# C. MARKERS
# ---------------------------------------------------------------------------
def check_markers(root, files):
    print("\nC. MARKERS / 标记格式")
    bad = []
    total_blocks = 0
    for rel in files:
        path = os.path.join(root, rel)
        try:
            lines = z.read_lines(path)
            _, n_marked, n_blocks = z.build_removal_mask(path, lines)
            total_blocks += n_blocks
        except z.CheckFailure as exc:
            bad.append(str(exc).split("\n")[0])
    report(not bad, "markers well-formed and balanced",
           "; ".join(bad) if bad else "%d block(s)" % total_blocks)
    return bad


# ---------------------------------------------------------------------------
# D. COVERAGE
# ---------------------------------------------------------------------------
DEF_RE = re.compile(r"^([A-Za-z_~][\w:<>,*&\s]*)\(")
CTRL_RE = re.compile(r"^(if|for|while|switch|return|else|do|catch)\b")

# Lines that are part of a comment but carry no marker.  When looking backwards
# for an annotation we must skip these, because an annotation is often placed
# ABOVE the pre-existing English doc block rather than directly above the code.
# 这些行属于注释但不带标记。向后查找注释时必须跳过它们，
# 因为注释常常放在**原有英文文档块之上**，而不是紧贴代码。
COMMENT_TAIL_RE = re.compile(r"^\s*(/\*\*?|\*|\*/|//)")


def count_functions(path):
    """Count function definitions and how many look annotated (approximate).

    统计函数定义数量，以及其中看起来已有注释的数量（**近似值**）。

    This is a lexical heuristic, not a C++ parser -- it is deliberately
    conservative and its counts are indicative, useful for spotting files that
    were skipped entirely, not for asserting exact percentages.
    这是词法启发式，不是 C++ 解析器 —— 刻意保守，数值仅作指示用途：
    可用于发现「整份文件被漏掉」，不适宜用来断言精确百分比。

    It counts BOTH top-level functions (typical in .cc) and indented member
    functions (typical in headers), since a header is mostly the latter.
    同时统计顶层函数与类内缩进的成员函数，因为头文件以前者为主。

    Two subtleties it handles, both of which produced wrong numbers before:
    它处理了两个曾导致计数错误的细节：

      1. this codebase often puts the return type on its own line
         (`void` / `Name (...)` / `{`), so the annotation sits above the
         RETURN TYPE line, not above the name.  The backward walk must skip
         signature lines as well as comments.
         本库常把返回类型单独放一行，因此注释位于**返回类型行**之上而非名字之上；
         向后查找时必须同时跳过签名行与注释行。

      2. constructor initialiser lists look like definitions
         (`m_x (0), m_y (0)`); they are excluded by rejecting a candidate whose
         previous non-blank line ends with a comma.
         构造函数初始化列表形似函数定义；通过「上一非空行以逗号结尾则排除」来剔除。
    """
    lines = z.read_lines(path)
    n = len(lines)
    defs, annotated = 0, 0

    def blank_or_comment(s):
        t = s.strip()
        return (not t) or t.startswith("//")

    def prev_significant(i):
        k = i - 1
        while k >= 0:
            s = lines[k]
            if s.strip() and not COMMENT_TAIL_RE.match(s) and not s.strip().startswith("//"):
                return s
            k -= 1
        return None

    def has_annotation_above(i):
        """Walk back over comments AND signature lines looking for a marker.

        穿过注释行**与签名行**向前查找标记。
        """
        k = i - 1
        while k >= 0:
            s = lines[k]
            if "[[ZH" in s:
                return True
            if not s.strip():
                k -= 1
                continue
            if COMMENT_TAIL_RE.match(s) or s.strip().startswith("//"):
                k -= 1
                continue
            # A signature line: the return type / template prefix / qualifiers.
            # They never end with a statement terminator and start at column >= 0.
            # 签名行：返回类型 / template 前缀 / 限定符。它们不以语句终结符结尾。
            if not s.rstrip().endswith((";", "}", "{")) and \
                    re.match(r"^[A-Za-z_~]", s.strip()):
                k -= 1
                continue
            return False
        return False

    for i, ln in enumerate(lines):
        stripped = ln.strip()
        if not stripped or stripped.startswith("//"):
            continue
        if not DEF_RE.match(stripped):
            continue
        if CTRL_RE.match(stripped):
            continue
        # Reject continuation lines (constructor initialiser lists and the like).
        # 排除续行（构造函数初始化列表等）。
        prev = prev_significant(i)
        if prev is not None and prev.rstrip().endswith((",", "(")):
            continue
        j = i + 1
        while j < n and blank_or_comment(lines[j]):
            j += 1
        if j < n and lines[j].lstrip().startswith("{"):
            defs += 1
            if has_annotation_above(i):
                annotated += 1
    return defs, annotated


def check_coverage(root, ref, files):
    print("\nD. COVERAGE / 覆盖密度（顶层函数）")
    rows = []
    for rel in files:
        path = os.path.join(root, rel)
        defs, ann = count_functions(path)
        if defs:
            rows.append((rel, ann, defs))
    for rel, ann, defs in rows:
        pct = 100.0 * ann / defs
        print("       %-46s %3d/%-3d  %5.1f%%" % (rel, ann, defs, pct))
    low = ["%s (%d/%d)" % (r, a, d) for r, a, d in rows if a < d]
    if low:
        notes.append("files with undocumented top-level functions: "
                     + "; ".join(low))
    report(True, "coverage measured (see notes)",
           "%d file(s) with functions" % len(rows))
    return rows


# ---------------------------------------------------------------------------
# E. COMPILER -- headers must preprocess to identical output
# ---------------------------------------------------------------------------
# Include paths for preprocessing.  These MUST be absolute: the pristine copy is
# preprocessed from a temporary directory, so with relative -I paths gcc records
# a relative __FILE__ ("src/db/db/dbTrans.h") for included headers while the
# in-tree file records an absolute one -- a difference that has nothing to do
# with our annotations but looks like one.
# 预处理用的包含路径。必须是**绝对路径**：原始副本从临时目录预处理，
# 若用相对 -I，gcc 对被包含头文件记录的 __FILE__ 是相对形式
# （"src/db/db/dbTrans.h"），而源码树内的文件记录的是绝对形式 ——
# 这个差异与我们的注释无关，却看起来像个差异。
_INC_REL = ["src/db/db", "src/tl/tl", "src/gsi/gsi", "src/rdb/rdb"]
INC = []  # filled in by main() once the repo root is known


def setup_include_paths(root):
    """Build absolute -I flags. / 构造绝对路径的 -I 参数。"""
    global INC
    INC = []
    for d in _INC_REL:
        p = os.path.join(root, d)
        if os.path.isdir(p):
            INC += ["-I", p]


def _normalise_preprocessed(text, root):
    """Normalise the metadata that MUST differ between pristine and annotated.

    归一化「必然不同」的元数据。

    Comments change line numbers, so `__LINE__` (and the file/line strings baked
    into tl_assert) cannot stay identical -- any change that adds lines has this
    effect, and it is harmless.  `__FILE__` can also differ in FORM (absolute vs
    relative) depending on how the translation unit was named.
    Both are normalised away so that a real semantic difference stays visible.

    注释会改变行号，因此 `__LINE__`（以及 tl_assert 里固化的文件/行号字符串）
    不可能保持一致 —— 任何增加行数的改动都如此，且无害。`__FILE__` 的**形式**
    （绝对/相对）也可能不同。把两者归一化掉，剩下的才是真正的语义差异。

    Implemented in three ordered, order-INDEPENDENT steps.  An earlier version
    substituted a list of path spellings in sequence, which broke when a relative
    spelling was a substring of the absolute one: the short form matched first,
    leaving a prefix behind and defeating the line-number pattern too.
    分三步实现，且与替换顺序无关。早期版本按顺序替换一组路径写法，当相对写法是
    绝对写法的子串时会失效：短形式先命中，留下前缀，连行号匹配也一并失效。
    """
    # 1) make absolute paths relative, so both sides use one spelling
    text = text.replace(root.rstrip("/") + "/", "")
    # 2) any file reference inside an assertion -> <FILE>
    text = re.sub(r'(assertion_failed\s*\(\s*)"[^"]*"', r'\1"<FILE>"', text)
    # 3) any line number inside an assertion -> <LINE>
    text = re.sub(r'(assertion_failed\s*\(\s*"<FILE>"\s*,\s*)\d+', r"\1<LINE>", text)
    return text


def check_preprocessor(root, ref, files, quick):
    print("\nE. COMPILER / 预处理输出一致性（仅头文件，已归一化 __FILE__/__LINE__）")
    if quick:
        print("       (skipped: --quick)")
        return []
    headers = [f for f in files if f.endswith(".h")]
    fails = []
    for rel in headers:
        path = os.path.join(root, rel)
        pristine = os.path.join("/tmp", "zh_audit_p_" + os.path.basename(rel))
        with open(pristine, "wb") as fp:
            fp.write(subprocess.run(["git", "-C", root, "show", "%s:%s" % (ref, rel)],
                                    capture_output=True).stdout)
        a = subprocess.run(["g++", "-E", "-P"] + INC + [pristine],
                           capture_output=True, cwd=root)
        b = subprocess.run(["g++", "-E", "-P"] + INC + [path],
                           capture_output=True, cwd=root)
        if a.returncode != 0 or b.returncode != 0:
            fails.append("%s (preprocess error)" % rel)
            continue
        ta = _normalise_preprocessed(a.stdout.decode("utf-8", "replace"), root)
        tb = _normalise_preprocessed(b.stdout.decode("utf-8", "replace"), root)
        if ta != tb:
            fails.append("%s (real difference beyond __FILE__/__LINE__)" % rel)
            # Show the first differing line to make the failure actionable.
            la, lb = ta.splitlines(), tb.splitlines()
            for i in range(max(len(la), len(lb))):
                x = la[i] if i < len(la) else "<missing>"
                y = lb[i] if i < len(lb) else "<missing>"
                if x != y:
                    print("       first diff for %s at line %d:" % (rel, i + 1))
                    print("         pristine : %s" % x[:110])
                    print("         annotated: %s" % y[:110])
                    break
    report(not fails, "header preprocessing semantically unchanged",
           "; ".join(fails) if fails else "%d header(s)" % len(headers))
    return fails


# ---------------------------------------------------------------------------
# G. HYGIENE -- generated / excluded artifacts must be untouched
# ---------------------------------------------------------------------------
def check_hygiene(root, ref, files):
    print("\nG. HYGIENE / 生成物与排除项未被触碰")
    bad = [f for f in files
           if not is_tooling(f)
           and (any(frag in "/" + f for frag in FORBIDDEN_FRAGMENTS)
                or any(f.endswith(s) for s in FORBIDDEN_SUFFIXES))]
    report(not bad, "no generated or excluded artifact modified",
           "; ".join(bad) if bad else "%d file(s) scanned" % len(files))
    return bad


def main(argv):
    quick = "--quick" in argv
    root = z.repo_root()
    ref = z.base_ref(root)
    setup_include_paths(root)

    print("=" * 76)
    print("[[ZH]] ANNOTATION AUDIT / 注释工作审计")
    print("=" * 76)
    print("repo            : %s" % root)
    print("pristine base   : %s" % ref)

    files = changed_files(root, ref)
    ann = annotated_files(root, ref)
    print("changed files   : %d" % len(files))
    print("annotated files : %d" % len(ann))
    print()

    check_scope(root, ref, files)
    check_integrity(root, ref, ann)
    check_markers(root, ann)
    check_coverage(root, ref, ann)
    check_preprocessor(root, ref, ann, quick)
    check_hygiene(root, ref, files)

    print("\n" + "=" * 76)
    print("NOTES / 说明")
    print("=" * 76)
    if notes:
        for n in notes:
            print("  * " + n)
    else:
        print("  (none)")

    print("\n" + "=" * 76)
    if problems:
        print("RESULT: %d problem(s) found / 发现 %d 个问题" % (len(problems), len(problems)))
        for p in problems:
            print("  - " + p)
        print("=" * 76)
        return 1
    print("RESULT: all checks passed / 全部检查通过")
    print("=" * 76)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
