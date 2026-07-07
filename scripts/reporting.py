"""Small report-generation helpers shared by research scripts."""

from __future__ import annotations

import base64
from dataclasses import dataclass, field
from html import escape
import json
from pathlib import Path
from typing import Any, Mapping, Sequence


def image_to_base64(path: Path) -> str:
    """Return base64 text for embedding a local image in self-contained HTML."""
    return base64.b64encode(path.read_bytes()).decode("ascii")


def image_data_uri(path: Path, mime: str = "image/png") -> str:
    """Return a data URI for a local image."""
    return f"data:{mime};base64,{image_to_base64(path)}"


def html_attrs(attrs: Mapping[str, Any] | None = None) -> str:
    """Render safe HTML attributes."""
    if not attrs:
        return ""
    parts = []
    for key, value in attrs.items():
        if value is None:
            continue
        parts.append(f' {escape(str(key), quote=True)}="{escape(str(value), quote=True)}"')
    return "".join(parts)


@dataclass(frozen=True)
class HtmlCell:
    """A table cell with optional attributes and controlled raw HTML content."""
    body: Any
    attrs: Mapping[str, Any] = field(default_factory=dict)
    raw: bool = False


def html_cell(body: Any, attrs: Mapping[str, Any] | None = None, raw: bool = False) -> HtmlCell:
    return HtmlCell(body=body, attrs=attrs or {}, raw=raw)


def _cell_body(cell: Any) -> str:
    if isinstance(cell, HtmlCell):
        return str(cell.body) if cell.raw else escape(str(cell.body))
    return escape(str(cell))


def _render_cell(tag: str, cell: Any) -> str:
    attrs = cell.attrs if isinstance(cell, HtmlCell) else None
    return f"<{tag}{html_attrs(attrs)}>{_cell_body(cell)}</{tag}>"


def html_table(headers: Sequence[Any], rows: Sequence[Sequence[Any]]) -> str:
    """Render a compact HTML table with escaped cells by default."""
    head = "<tr>" + "".join(_render_cell("th", h) for h in headers) + "</tr>"
    body = "".join(
        "<tr>" + "".join(_render_cell("td", c) for c in row) + "</tr>"
        for row in rows
    )
    return f"<table>{head}{body}</table>"


def embedded_png_figure(base64_png: str, alt: str, attrs: Mapping[str, Any] | None = None) -> str:
    """Render an embedded PNG figure from base64 text."""
    img_attrs = {
        "src": f"data:image/png;base64,{base64_png}",
        "alt": alt,
        "style": "max-width:100%",
        **(attrs or {}),
    }
    return f"<figure><img{html_attrs(img_attrs)}/></figure>"


def report_section(title: str, body: str, level: int = 2, attrs: Mapping[str, Any] | None = None) -> str:
    """Render a report section with escaped heading text and raw body HTML."""
    if level < 1 or level > 6:
        raise ValueError("section heading level must be between 1 and 6")
    return f"<h{level}{html_attrs(attrs)}>{escape(title)}</h{level}>\n{body.strip()}"


def callout_box(body: str, class_name: str, attrs: Mapping[str, Any] | None = None) -> str:
    """Render a reusable callout block for abstracts, key findings, or tool notes."""
    merged_attrs = {"class": class_name, **(attrs or {})}
    return f"<div{html_attrs(merged_attrs)}>{body.strip()}</div>"


def meta_paragraph(body: str, attrs: Mapping[str, Any] | None = None) -> str:
    """Render muted report metadata text."""
    merged_attrs = {"class": "meta", **(attrs or {})}
    return f"<p{html_attrs(merged_attrs)}>{body.strip()}</p>"


@dataclass(frozen=True)
class ReportDocument:
    """Reusable self-contained HTML report shell."""
    title: str
    styles: str
    lang: str = "zh-CN"

    def render(self, body: str) -> str:
        styles = self.styles.strip("\n")
        body_html = body.strip("\n")
        return (
            "<!DOCTYPE html>\n"
            f"<html{html_attrs({'lang': self.lang})}><head><meta charset=\"utf-8\">\n"
            f"<title>{escape(self.title)}</title>\n"
            f"<style>\n{styles}\n</style></head><body>\n\n"
            f"{body_html}\n"
            "</body></html>"
        )

    def write(self, path: Path, body: str) -> None:
        path.write_text(self.render(body), encoding="utf-8")


def write_json(path: Path, data: Any) -> None:
    """Write stable UTF-8 JSON for report sidecar metrics."""
    path.write_text(
        json.dumps(data, ensure_ascii=False, indent=1, default=float),
        encoding="utf-8",
    )
