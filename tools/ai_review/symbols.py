"""Map changed lines to enclosing Python symbols via AST."""

from __future__ import annotations

import ast
from dataclasses import dataclass
from pathlib import Path


@dataclass
class SymbolSpan:
    file: str
    qualname: str
    kind: str  # function | async_function | class | module
    start_line: int
    end_line: int
    source: str
    clipped: bool = False

    @property
    def line_range(self) -> list[int]:
        return [self.start_line, self.end_line]


# A clipped excerpt keeps this many leading lines (signature + docstring head)
# before spending the rest of the budget on the changed lines themselves.
_HEAD_CONTEXT_LINES = 12
_CHANGE_CONTEXT_LINES = 4


def extract_changed_symbols(
    repo: Path,
    file_path: str,
    changed_lines: set[int],
    *,
    max_excerpt_lines: int = 80,
) -> list[SymbolSpan]:
    path = repo / file_path
    if not file_path.endswith(".py") or not path.is_file():
        return []
    if not changed_lines:
        return []

    try:
        source = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return []

    try:
        tree = ast.parse(source)
    except SyntaxError:
        # Fall back to whole-file excerpt when AST fails.
        lines = source.splitlines()
        excerpt = _excerpt_lines(lines, 1, min(len(lines), max_excerpt_lines), max_excerpt_lines)
        return [
            SymbolSpan(
                file=file_path,
                qualname="<syntax_error_module>",
                kind="module",
                start_line=1,
                end_line=min(len(lines), max_excerpt_lines),
                source=excerpt,
                clipped=len(lines) > max_excerpt_lines,
            )
        ]

    spans = _collect_spans(file_path, source, tree, max_excerpt_lines)
    hit: list[SymbolSpan] = []
    for span in spans:
        if any(span.start_line <= ln <= span.end_line for ln in changed_lines):
            hit.append(span)

    if hit:
        # Prefer innermost symbols: drop a class if a method inside was also hit.
        innermost = []
        for span in sorted(hit, key=lambda s: (s.end_line - s.start_line, s.start_line)):
            if any(
                other is not span
                and other.start_line >= span.start_line
                and other.end_line <= span.end_line
                and other.qualname != span.qualname
                for other in hit
            ):
                # Keep classes only when no nested hit? Prefer nested.
                if span.kind == "class" and any(
                    o.start_line >= span.start_line and o.end_line <= span.end_line and o.kind != "class"
                    for o in hit
                    if o is not span
                ):
                    continue
            innermost.append(span)
        # Deduplicate by qualname.
        seen: set[str] = set()
        out: list[SymbolSpan] = []
        lines = source.splitlines()
        for span in innermost:
            if span.qualname in seen:
                continue
            seen.add(span.qualname)
            # A long symbol clipped to its head can hide every changed line, so
            # re-excerpt around the diff instead.
            span.source = _excerpt_around_changes(
                lines, span.start_line, span.end_line, changed_lines, max_excerpt_lines
            )
            span.clipped = _is_clipped(span.start_line, span.end_line, max_excerpt_lines)
            out.append(span)
        return out

    # Module-level change outside defs.
    lines = source.splitlines()
    first = min(changed_lines)
    last = max(changed_lines)
    start = max(1, first - 5)
    end = min(len(lines), last + 5)
    excerpt = _excerpt_around_changes(
        lines, start, end, changed_lines, max_excerpt_lines
    )
    return [
        SymbolSpan(
            file=file_path,
            qualname="<module>",
            kind="module",
            start_line=start,
            end_line=end,
            source=excerpt,
            clipped=_is_clipped(start, end, max_excerpt_lines),
        )
    ]


def load_file_symbols(repo: Path, file_path: str, *, max_excerpt_lines: int = 80) -> list[SymbolSpan]:
    path = repo / file_path
    if not file_path.endswith(".py") or not path.is_file():
        return []
    try:
        source = path.read_text(encoding="utf-8", errors="replace")
        tree = ast.parse(source)
    except (OSError, SyntaxError):
        return []
    return _collect_spans(file_path, source, tree, max_excerpt_lines)


def _collect_spans(
    file_path: str,
    source: str,
    tree: ast.AST,
    max_excerpt_lines: int,
) -> list[SymbolSpan]:
    lines = source.splitlines()
    spans: list[SymbolSpan] = []

    class Visitor(ast.NodeVisitor):
        def __init__(self) -> None:
            self.stack: list[str] = []

        def visit_ClassDef(self, node: ast.ClassDef) -> None:
            self._add(node, "class")
            self.stack.append(node.name)
            self.generic_visit(node)
            self.stack.pop()

        def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
            self._add(node, "function")
            self.stack.append(node.name)
            self.generic_visit(node)
            self.stack.pop()

        def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
            self._add(node, "async_function")
            self.stack.append(node.name)
            self.generic_visit(node)
            self.stack.pop()

        def _add(self, node: ast.AST, kind: str) -> None:
            name = getattr(node, "name", "?")
            qual = ".".join([*self.stack, name])
            start = getattr(node, "lineno", 1)
            end = getattr(node, "end_lineno", start) or start
            excerpt = _excerpt_lines(lines, start, end, max_excerpt_lines)
            spans.append(
                SymbolSpan(
                    file=file_path,
                    qualname=qual,
                    kind=kind,
                    start_line=start,
                    end_line=end,
                    source=excerpt,
                    clipped=_is_clipped(start, end, max_excerpt_lines),
                )
            )

    Visitor().visit(tree)
    return spans


def _excerpt_lines(lines: list[str], start: int, end: int, max_lines: int) -> str:
    if end < start:
        start, end = end, start
    length = end - start + 1
    if length <= max_lines:
        chunk = lines[start - 1 : end]
        body = "\n".join(chunk)
        return body
    # Keep head of symbol; note truncation.
    chunk = lines[start - 1 : start - 1 + max_lines]
    return "\n".join(chunk) + f"\n# ... truncated ({length - max_lines} more lines)"


def _is_clipped(start: int, end: int, max_lines: int) -> bool:
    return (abs(end - start) + 1) > max_lines


def _excerpt_around_changes(
    lines: list[str],
    start: int,
    end: int,
    changed_lines: set[int],
    max_lines: int,
) -> str:
    """Excerpt a symbol, prioritizing the lines the diff actually touched.

    Under the budget the whole symbol is returned. Over it, every changed line
    inside the symbol is always kept (even if that alone exceeds
    ``max_lines``). Remaining budget fills a short head (signature) then
    widening context around the changes. Gaps carry absolute line numbers so a
    reviewer can locate the omitted region.
    """
    if end < start:
        start, end = end, start
    if not _is_clipped(start, end, max_lines):
        return "\n".join(lines[start - 1 : end])

    inside = sorted(ln for ln in changed_lines if start <= ln <= end)
    if not inside:
        return _excerpt_lines(lines, start, end, max_lines)

    # Changed lines are mandatory — never drop them for the line budget.
    keep: set[int] = set(inside)
    head_len = min(_HEAD_CONTEXT_LINES, max(1, max_lines // 4))
    for line_no in range(start, min(end, start + head_len - 1) + 1):
        keep.add(line_no)

    budget = max_lines - len(keep)
    for radius in range(1, _CHANGE_CONTEXT_LINES + 1):
        if budget <= 0:
            break
        for line_no in inside:
            for candidate in (line_no - radius, line_no + radius):
                if budget <= 0:
                    break
                if start <= candidate <= end and candidate not in keep:
                    keep.add(candidate)
                    budget -= 1

    out: list[str] = []
    previous: int | None = None
    for line_no in sorted(keep):
        if previous is not None and line_no != previous + 1:
            out.append(f"# ... skipped lines {previous + 1}-{line_no - 1} ...")
        out.append(lines[line_no - 1])
        previous = line_no
    if previous is not None and previous < end:
        out.append(f"# ... skipped lines {previous + 1}-{end} ...")
    return "\n".join(out)
