"""Same-file dependency slicing for changed symbols (MVP, no twin cache)."""

from __future__ import annotations

import ast
import re
from dataclasses import dataclass
from pathlib import Path

from . import MAX_RELATED_SYMBOLS
from .symbols import SymbolSpan, load_file_symbols


@dataclass
class RelatedSymbol:
    span: SymbolSpan
    relation: str  # calls | called_by | same_module | keyword_hit


_CALL_NAME = re.compile(r"^[A-Za-z_][\w\.]*$")
_KEYWORD_HIT = re.compile(
    r"\b(Lock|RLock|Thread|Queue|shared_state|FrameRing|lease|generation|"
    r"create_subscription|create_publisher|create_timer|rknn|RKNN|emergency_stop)\b",
    re.I,
)


def slice_related(
    repo: Path,
    changed: list[SymbolSpan],
    *,
    max_related: int = MAX_RELATED_SYMBOLS,
    max_excerpt_lines: int = 80,
) -> list[RelatedSymbol]:
    if not changed:
        return []

    by_file: dict[str, list[SymbolSpan]] = {}
    for span in changed:
        by_file.setdefault(span.file, []).append(span)

    related: list[RelatedSymbol] = []
    seen: set[tuple[str, str, str]] = set()

    for file_path, file_changed in by_file.items():
        all_spans = load_file_symbols(repo, file_path, max_excerpt_lines=max_excerpt_lines)
        if not all_spans:
            continue

        source_path = repo / file_path
        try:
            source = source_path.read_text(encoding="utf-8", errors="replace")
            tree = ast.parse(source)
        except (OSError, SyntaxError):
            for span in all_spans[:3]:
                key = (span.file, span.qualname, "same_module")
                if key not in seen and span.qualname not in {c.qualname for c in file_changed}:
                    seen.add(key)
                    related.append(RelatedSymbol(span=span, relation="same_module"))
            continue

        call_map = _build_call_map(tree)  # caller_qual -> set(callee simple names)
        changed_names = {c.qualname.split(".")[-1] for c in file_changed}
        changed_quals = {c.qualname for c in file_changed}

        for span in file_changed:
            simple = span.qualname.split(".")[-1]
            callees = call_map.get(span.qualname, set()) | call_map.get(simple, set())
            for other in all_spans:
                if other.qualname in changed_quals:
                    continue
                other_simple = other.qualname.split(".")[-1]
                # calls: changed calls other
                if other_simple in callees or other.qualname in callees:
                    key = (other.file, other.qualname, "calls")
                    if key not in seen:
                        seen.add(key)
                        related.append(RelatedSymbol(span=other, relation="calls"))
                # called_by: other calls changed
                other_callees = call_map.get(other.qualname, set()) | call_map.get(other_simple, set())
                if simple in other_callees or span.qualname in other_callees:
                    key = (other.file, other.qualname, "called_by")
                    if key not in seen:
                        seen.add(key)
                        related.append(RelatedSymbol(span=other, relation="called_by"))

        # Keyword hits in same file near concurrency/ROS/etc.
        for other in all_spans:
            if other.qualname in changed_quals:
                continue
            if _KEYWORD_HIT.search(other.source):
                key = (other.file, other.qualname, "keyword_hit")
                if key not in seen:
                    seen.add(key)
                    related.append(RelatedSymbol(span=other, relation="keyword_hit"))

        # Import neighbors: modules imported by this file (path guess only).
        for imp in _imported_local_modules(tree, file_path):
            for neigh in load_file_symbols(repo, imp, max_excerpt_lines=max_excerpt_lines)[:5]:
                if neigh.qualname.split(".")[-1] in changed_names:
                    key = (neigh.file, neigh.qualname, "same_module")
                    if key not in seen:
                        seen.add(key)
                        related.append(RelatedSymbol(span=neigh, relation="same_module"))

    # Prefer calls/called_by over keyword_hit.
    order = {"calls": 0, "called_by": 1, "same_module": 2, "keyword_hit": 3}
    related.sort(key=lambda r: (order.get(r.relation, 9), r.span.file, r.span.qualname))
    return related[:max_related]


def _build_call_map(tree: ast.AST) -> dict[str, set[str]]:
    """Map qualified/simple function names to called simple names."""
    result: dict[str, set[str]] = {}

    class Visitor(ast.NodeVisitor):
        def __init__(self) -> None:
            self.stack: list[str] = []

        def visit_ClassDef(self, node: ast.ClassDef) -> None:
            self.stack.append(node.name)
            self.generic_visit(node)
            self.stack.pop()

        def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
            self._visit_fn(node)

        def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
            self._visit_fn(node)

        def _visit_fn(self, node: ast.AST) -> None:
            name = getattr(node, "name", "?")
            qual = ".".join([*self.stack, name])
            calls: set[str] = set()
            for sub in ast.walk(node):
                if isinstance(sub, ast.Call):
                    calls.update(_call_names(sub.func))
            result[qual] = calls
            result[name] = result.get(name, set()) | calls
            self.stack.append(name)
            # Do not generic_visit nested with wrong stack twice — walk body with stack.
            for child in ast.iter_child_nodes(node):
                if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                    self.visit(child)
            self.stack.pop()

    Visitor().visit(tree)
    return result


def _call_names(func: ast.AST) -> set[str]:
    if isinstance(func, ast.Name):
        return {func.id}
    if isinstance(func, ast.Attribute):
        names = {func.attr}
        # Also keep dotted form when simple.
        parts: list[str] = [func.attr]
        cur = func.value
        while isinstance(cur, ast.Attribute):
            parts.append(cur.attr)
            cur = cur.value
        if isinstance(cur, ast.Name):
            parts.append(cur.id)
            dotted = ".".join(reversed(parts))
            if _CALL_NAME.match(dotted):
                names.add(dotted)
        return names
    return set()


def _imported_local_modules(tree: ast.AST, current_file: str) -> list[str]:
    """Best-effort resolve relative package imports under cat_follow/."""
    out: list[str] = []
    parts = current_file.replace("\\", "/").split("/")
    if "cat_follow" not in parts:
        return out
    pkg_index = parts.index("cat_follow")
    pkg_root = "/".join(parts[: pkg_index + 1])

    for node in tree.body if isinstance(tree, ast.Module) else []:
        if isinstance(node, ast.ImportFrom) and node.module:
            mod = node.module
            if node.level and node.level > 0:
                # Relative import: resolve from current package dir.
                base = parts[: -(node.level)]
                if node.module:
                    candidate = "/".join([*base, *node.module.split(".")]) + ".py"
                else:
                    continue
            elif mod.startswith("cat_follow"):
                candidate = mod.replace(".", "/") + ".py"
            else:
                # Same top package local imports like `from control import fsm`
                candidate = f"{pkg_root}/{mod.replace('.', '/')}.py"
            out.append(candidate.replace("\\", "/"))
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.startswith("cat_follow"):
                    out.append(alias.name.replace(".", "/") + ".py")
    return out
