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
