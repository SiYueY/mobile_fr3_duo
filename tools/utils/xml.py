"""Canonical XML formatter for MuJoCo Menagerie-style files.

Rules: two-space indentation, double quotes, self-closing empty elements,
comments preserved. `--check` mode exits non-zero when formatting differs.
"""

from __future__ import annotations

import argparse
import io
import sys
import xml.etree.ElementTree as ET


class CommentedTreeBuilder(ET.TreeBuilder):
    """TreeBuilder that preserves XML comments."""

    def comment(self, data: str) -> None:  # noqa: D102
        self.start(ET.Comment, {})
        self.data(data)
        self.end(ET.Comment)


def parse(source: str) -> ET.Element:
    parser = ET.XMLParser(target=CommentedTreeBuilder())
    return ET.parse(io.StringIO(source), parser=parser).getroot()


def _escape(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def _serialize(elem: ET.Element, level: int, lines: list[str]) -> None:
    indent = "  " * level
    if elem.tag is ET.Comment:
        lines.append(f"{indent}<!--{elem.text}-->")
        return

    attrs = "".join(f' {k}="{v}"' for k, v in elem.attrib.items())
    children = list(elem)
    text = (elem.text or "").strip()

    if not children and not text:
        lines.append(f"{indent}<{elem.tag}{attrs}/>")
        return
    if not children:
        lines.append(f"{indent}<{elem.tag}{attrs}>{_escape(text)}</{elem.tag}>")
        return

    lines.append(f"{indent}<{elem.tag}{attrs}>")
    if text:
        lines.append(f"{indent}  {_escape(text)}")
    for child in children:
        _serialize(child, level + 1, lines)
    lines.append(f"{indent}</{elem.tag}>")


def format_element(root: ET.Element) -> str:
    lines: list[str] = ['<?xml version="1.0" encoding="utf-8"?>']
    _serialize(root, 0, lines)
    return "\n".join(lines) + "\n"


def format_xml(source: str) -> str:
    return format_element(parse(source))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("files", nargs="+", help="XML/URDF files to format")
    ap.add_argument("--check", action="store_true", help="verify formatting only")
    ap.add_argument("-i", "--inplace", action="store_true", help="rewrite files in place")
    args = ap.parse_args()

    failed = False
    for path in args.files:
        with open(path, encoding="utf-8") as fh:
            source = fh.read()
        formatted = format_xml(source)
        if source != formatted:
            failed = True
            print(f"would reformat: {path}")
            if args.inplace and not args.check:
                with open(path, "w", encoding="utf-8") as fh:
                    fh.write(formatted)
    return 1 if (failed and args.check) else 0


if __name__ == "__main__":
    sys.exit(main())
