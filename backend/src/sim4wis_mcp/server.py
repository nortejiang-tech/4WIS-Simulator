"""sim4wis-mcp — the agent facade over the study layer (stdio MCP server).

Thin client, no business logic, no ``sim4wis`` import: every capability is an
HTTP call against ``SIM4WIS_BACKEND_HTTP`` (default http://127.0.0.1:8010),
per docs/agent_interface_design.md §10/§13. If the backend is not reachable
this process starts one (``<venv python> -m sim4wis.main``) and, on exit,
stops it — but only the backend it started, never a user's own.

The agent-facing acceptance flow is: describe_capabilities → run_study
(dry_run) → run_study → read_report. Large payloads stay on disk; tools
return summaries and paths, never full traces, except get_trace which is
explicitly downsampled.
"""

from __future__ import annotations

import argparse
import asyncio
import atexit
import http.client
import json
import os
import subprocess
import sys
import time
import urllib.parse
from pathlib import Path
from typing import Any

from mcp.server import Server
from mcp.types import (
    CallToolRequestParams,
    CallToolResult,
    ListToolsResult,
    TextContent,
    Tool,
)
import mcp.server.stdio

from sim4wis_mcp import __version__

#: Backend base URL. The frontend dev server already uses this convention.
BASE = os.environ.get("SIM4WIS_BACKEND_HTTP", "http://127.0.0.1:8010")

REPO_ROOT = Path(__file__).resolve().parents[3]

#: How long to wait for a backend we started ourselves to come up [s].
_START_TIMEOUT_S = 30.0
#: Study job poll ceiling [s]. Studies are minutes; this is a guard, not a limit.
_JOB_TIMEOUT_S = 1800.0

_started_by_us: subprocess.Popen | None = None


# ---------------------------------------------------------------------------
# HTTP — the whole facade is this plus the tool descriptions
# ---------------------------------------------------------------------------


def _request(
    method: str,
    path: str,
    body: dict[str, Any] | None = None,
    timeout_s: float = 120.0,
) -> tuple[int, Any]:
    """One JSON request. Returns (status, parsed body — dict, list, or str)."""
    parsed = urllib.parse.urlparse(BASE)
    conn = http.client.HTTPConnection(
        parsed.hostname or "127.0.0.1", parsed.port or 8010, timeout=timeout_s
    )
    try:
        headers = {"Content-Type": "application/json"} if body is not None else {}
        conn.request(method, path, body=json.dumps(body) if body is not None else None,
                     headers=headers)
        resp = conn.getresponse()
        raw = resp.read()
        status = resp.status
    finally:
        conn.close()
    if not raw:
        return status, {}
    try:
        return status, json.loads(raw)
    except json.JSONDecodeError:
        return status, raw.decode("utf-8", "replace")


def _backend_up() -> bool:
    try:
        _request("GET", "/api/version", timeout_s=1.0)
        return True
    except OSError:
        return False


def _stop_owned_backend() -> None:
    global _started_by_us
    if _started_by_us is not None and _started_by_us.poll() is None:
        _started_by_us.terminate()
    _started_by_us = None


def _ensure_backend() -> None:
    """Start the backend if it is not up; remember we own it if we did."""
    global _started_by_us
    if _backend_up():
        return
    _started_by_us = subprocess.Popen(
        [sys.executable, "-m", "sim4wis.main"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    atexit.register(_stop_owned_backend)
    deadline = time.time() + _START_TIMEOUT_S
    while time.time() < deadline:
        if _backend_up():
            return
        if _started_by_us.poll() is not None:
            raise RuntimeError("sim4wis backend exited during startup")
        time.sleep(0.5)
    raise RuntimeError(
        "sim4wis backend did not come up within "
        f"{_START_TIMEOUT_S:.0f}s on {BASE}; start it manually (python -m sim4wis.main)"
    )


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------


def _describe() -> dict[str, Any]:
    """The agent's first stop: models, metrics, strategies, targets, notes."""
    _, caps = _request("GET", "/api/study/capabilities")
    try:
        _, version = _request("GET", "/api/version", timeout_s=2.0)
        caps["backend_version"] = version.get("version") if isinstance(version, dict) else None
    except OSError:
        caps["backend_version"] = None
    caps["mcp_facade"] = {"version": __version__, "base": BASE}
    return caps


def _list_studies() -> dict[str, Any]:
    _, data = _request("GET", "/api/study")
    return data


def _get_study(study_id: str) -> dict[str, Any]:
    status, data = _request("GET", f"/api/study/{urllib.parse.quote(study_id)}")
    if status == 404:
        raise RuntimeError(f"unknown study: {study_id}")
    return data


def _list_runs() -> dict[str, Any]:
    _, data = _request("GET", "/api/runs")
    return data


def _get_trace(run_id: str, channels: str, max_points: int = 200) -> dict[str, Any]:
    query = urllib.parse.urlencode({"channels": channels, "max_points": max_points})
    status, data = _request("GET", f"/api/study/trace/{urllib.parse.quote(run_id)}?{query}")
    if status == 404:
        raise RuntimeError(f"unknown run: {run_id}")
    return data


def _dry_run(spec: dict[str, Any]) -> dict[str, Any]:
    status, data = _request("POST", "/api/study/dry-run", spec)
    if status >= 400:
        raise RuntimeError("spec refused: " + json.dumps(data, ensure_ascii=False)[:500])
    return data


async def _run_study(spec: dict[str, Any], dry_run: bool = False) -> dict[str, Any]:
    """Submit a StudySpec; optionally only estimate it. Returns the summary.

    dry_run=True costs nothing and validates the spec; a real run returns the
    job id, polls it, and comes back with the study id, the result summary and
    the report path.
    """
    if dry_run:
        return {"dry_run": _dry_run(spec)}
    status, data = _request("POST", "/api/study/run", spec)
    if status >= 400:
        raise RuntimeError("spec refused: " + json.dumps(data, ensure_ascii=False)[:500])
    job_id = data["job_id"]
    deadline = time.time() + _JOB_TIMEOUT_S
    while True:
        await asyncio.sleep(1.0)
        _, job = _request("GET", f"/api/study/jobs/{job_id}")
        if job.get("status") in ("done", "error"):
            break
        if time.time() > deadline:
            raise RuntimeError(f"study job {job_id} exceeded {_JOB_TIMEOUT_S:.0f}s")
    if job.get("status") == "error":
        return {"job_id": job_id, "status": "error", "error": job.get("error")}
    study_id = job.get("study_id")
    result = job.get("result") or {}
    return {
        "job_id": job_id,
        "study_id": study_id,
        "report_path": result.get("report_path"),
        "result": result,
    }


def _read_report(study_id: str) -> dict[str, Any]:
    """Where the study's report is, and whether the backend serves it.

    Returns the local report path (the agent reads it itself) plus whether
    the backend would serve it — never the HTML inline.
    """
    study = _get_study(study_id)
    # load(study_id) returns the result summary itself — report_path is top-level.
    path = study.get("report_path")
    status, _ = _request("GET", f"/api/study/{urllib.parse.quote(study_id)}/report",
                         timeout_s=30.0)
    return {"study_id": study_id, "report_path": path, "served": status == 200}


def _verify_golden() -> dict[str, Any]:
    """Run the golden regression gate in a subprocess and report the tail."""
    proc = subprocess.run(
        [sys.executable, str(REPO_ROOT / "scripts" / "check_golden_experiments.py")],
        capture_output=True, text=True, timeout=_JOB_TIMEOUT_S,
    )
    text = (proc.stdout + proc.stderr).strip()
    return {"exit_code": proc.returncode, "tail": text[-2000:]}


TOOL_DESCRIPTIONS: list[Tool] = [
    Tool(name="describe_capabilities",
         description="Models × capability notes, metric registry, strategies, steer kinds, target sets, channels — everything needed to write a valid spec. The agent's first stop.",
         inputSchema={"type": "object", "properties": {}}),
    Tool(name="run_study",
         description="Submit a StudySpec (JSON object). dry_run=true only validates and estimates. A real run blocks until done and returns the result summary plus report_path.",
         inputSchema={"type": "object",
                      "properties": {"spec": {"type": "object", "description": "StudySpec as JSON, same schema as POST /api/study/run"},
                                     "dry_run": {"type": "boolean", "default": False}},
                      "required": ["spec"]}),
    Tool(name="get_study",
         description="Fetch a stored study by id (result summary, provenance, verdicts).",
         inputSchema={"type": "object",
                      "properties": {"study_id": {"type": "string"}},
                      "required": ["study_id"]}),
    Tool(name="list_studies",
         description="List stored studies.",
         inputSchema={"type": "object", "properties": {}}),
    Tool(name="list_runs",
         description="List run metadata, newest first — includes runs made from the GUI.",
         inputSchema={"type": "object", "properties": {}}),
    Tool(name="get_trace",
         description="Downsampled channels for one run. channels is required (comma-separated); max_points caps the sample count.",
         inputSchema={"type": "object",
                      "properties": {"run_id": {"type": "string"},
                                     "channels": {"type": "string", "description": "comma-separated channel names"},
                                     "max_points": {"type": "integer", "default": 200}},
                      "required": ["run_id", "channels"]}),
    Tool(name="read_report",
         description="Return the local report path for a study (and whether the backend serves it). The agent reads the file itself.",
         inputSchema={"type": "object",
                      "properties": {"study_id": {"type": "string"}},
                      "required": ["study_id"]}),
    Tool(name="verify_golden",
         description="Run the golden regression gate (scripts/check_golden_experiments.py) and report exit code + output tail.",
         inputSchema={"type": "object", "properties": {}}),
]


async def _list_tools(ctx: Any, params: Any) -> ListToolsResult:
    return ListToolsResult(tools=TOOL_DESCRIPTIONS)


async def _call_tool(ctx: Any, params: CallToolRequestParams) -> CallToolResult:
    name = params.name
    args = dict(params.arguments or {})
    try:
        _ensure_backend()
        if name == "describe_capabilities":
            result = _describe()
        elif name == "run_study":
            result = await _run_study(args["spec"], dry_run=bool(args.get("dry_run")))
        elif name == "get_study":
            result = _get_study(str(args["study_id"]))
        elif name == "list_studies":
            result = _list_studies()
        elif name == "list_runs":
            result = _list_runs()
        elif name == "get_trace":
            result = _get_trace(str(args["run_id"]), str(args["channels"]),
                                int(args.get("max_points", 200)))
        elif name == "read_report":
            result = _read_report(str(args["study_id"]))
        elif name == "verify_golden":
            result = _verify_golden()
        else:
            raise RuntimeError(f"unknown tool: {name}")
        text = json.dumps(result, ensure_ascii=False, indent=1, default=str)
        return CallToolResult(content=[TextContent(type="text", text=text)], is_error=False)
    except Exception as exc:  # noqa: BLE001 - the agent sees the failure, not a crash
        return CallToolResult(
            content=[TextContent(type="text", text=f"error: {type(exc).__name__}: {exc}")],
            is_error=True,
        )


server = Server(
    "sim4wis-mcp",
    version=__version__,
    description="4WIS Simulator study layer — run studies, read reports, check goldens.",
    on_list_tools=_list_tools,
    on_call_tool=_call_tool,
)


async def _stdio() -> None:
    async with mcp.server.stdio.stdio_server() as (read, write):
        await server.run(read, write, server.create_initialization_options())


def main() -> int:
    parser = argparse.ArgumentParser(prog="sim4wis-mcp")
    parser.add_argument("--base", default=None,
                        help="backend base URL (default: $SIM4WIS_BACKEND_HTTP or http://127.0.0.1:8010)")
    args = parser.parse_args()
    global BASE
    if args.base:
        BASE = args.base
    _ensure_backend()
    asyncio.run(_stdio())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
