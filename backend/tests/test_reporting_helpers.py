from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "reporting.py"


def load_reporting():
    spec = importlib.util.spec_from_file_location("reporting", SCRIPT)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_report_document_renders_escaped_shell() -> None:
    reporting = load_reporting()
    doc = reporting.ReportDocument(
        title='A <unsafe> "title"',
        styles="body { color: #111; }",
        lang="zh-CN",
    )

    html = doc.render("<h1>正文</h1>")

    assert html.startswith("<!DOCTYPE html>\n")
    assert '<html lang="zh-CN">' in html
    assert "<title>A &lt;unsafe&gt; &quot;title&quot;</title>" in html
    assert "<style>\nbody { color: #111; }\n</style></head><body>" in html
    assert "<h1>正文</h1>" in html
    assert html.endswith("</body></html>")


def test_report_document_write_uses_utf8(tmp_path: Path) -> None:
    reporting = load_reporting()
    doc = reporting.ReportDocument(title="报告", styles="body { }")
    out = tmp_path / "report.html"

    doc.write(out, "<p>中文内容</p>")

    assert "中文内容" in out.read_text(encoding="utf-8")


def test_report_section_escapes_heading_and_preserves_body() -> None:
    reporting = load_reporting()

    html = reporting.report_section('1 <unsafe> "标题"', "<p><b>raw</b></p>", level=3)

    assert html == '<h3>1 &lt;unsafe&gt; &quot;标题&quot;</h3>\n<p><b>raw</b></p>'


def test_report_section_rejects_invalid_heading_level() -> None:
    reporting = load_reporting()

    try:
        reporting.report_section("bad", "<p>x</p>", level=7)
    except ValueError as exc:
        assert "between 1 and 6" in str(exc)
    else:
        raise AssertionError("expected invalid heading level to fail")


def test_html_table_escapes_cells_and_allows_explicit_raw_html() -> None:
    reporting = load_reporting()

    html = reporting.html_table(
        [reporting.html_cell("等级", {"style": "width:20%"})],
        [
            [
                "<unsafe>",
                reporting.html_cell("<b>C2</b>", {"style": "color:#ff9f1c"}, raw=True),
            ],
        ],
    )

    assert html == (
        '<table><tr><th style="width:20%">等级</th></tr>'
        '<tr><td>&lt;unsafe&gt;</td><td style="color:#ff9f1c"><b>C2</b></td></tr></table>'
    )


def test_callout_and_meta_helpers_merge_safe_attributes() -> None:
    reporting = load_reporting()

    callout = reporting.callout_box("<b>重点</b>", "kbox", {"data-kind": 'A "quote"'})
    meta = reporting.meta_paragraph("复现：<code>cmd</code>", {"id": "run-meta"})

    assert callout == '<div class="kbox" data-kind="A &quot;quote&quot;"><b>重点</b></div>'
    assert meta == '<p class="meta" id="run-meta">复现：<code>cmd</code></p>'
