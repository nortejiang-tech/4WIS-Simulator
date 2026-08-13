"""Tests — the sim4wis-mcp facade (S3).

The acceptance bar for the facade is the agent flow from
docs/optimization_direction_2026-08-13.md §3.4: an agent completes
"new spec → dry-run → run → read report" using only MCP tools. These tests
run that flow against a real backend on an isolated port and data dir, and
exercise the error path through the tool handler itself.
"""

from __future__ import annotations

import asyncio
import threading
import time

import sys

import pytest
import uvicorn

from mcp import ClientSession
from mcp.client.stdio import StdioServerParameters, stdio_client
from mcp.types import CallToolRequestParams

from sim4wis import main as sim_main
from sim4wis_mcp import server as mcp_server

@pytest.fixture()
def backend(monkeypatch, tmp_path):
    """A real backend on a dynamically chosen port, throwaway run/study dirs."""
    monkeypatch.setenv("SIM4WIS_RUNS_DIR", str(tmp_path / "runs"))
    monkeypatch.setenv("SIM4WIS_STUDIES_DIR", str(tmp_path / "studies"))
    # port=0 lets the OS pick a free port — never collide with anything the
    # user happens to be running on this machine.
    config = uvicorn.Config(sim_main.app, host="127.0.0.1", port=0,
                            log_level="warning")
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    deadline = time.time() + 20.0
    while time.time() < deadline and not server.started:
        time.sleep(0.05)
    if not server.started:
        raise RuntimeError("test backend did not start")
    port = server.servers[0].sockets[0].getsockname()[1]
    monkeypatch.setattr(mcp_server, "BASE", f"http://127.0.0.1:{port}")
    deadline = time.time() + 20.0
    while time.time() < deadline:
        if mcp_server._backend_up():
            break
        time.sleep(0.1)
    else:
        raise RuntimeError("test backend did not come up")
    yield
    server.should_exit = True
    thread.join(timeout=10)


def _test_base() -> str:
    return mcp_server.BASE


def _spec() -> dict:
    """The smallest valid study: a front step held to settle, one speed cell."""
    return {
        "study": "mcp_agent_flow",
        "question": "稳态横摆角速度与解析解是否一致？",
        "model": "simplified_dynamic",
        "baseline": {
            "name": "step_1deg",
            "strategy": "ideal_ackermann",
            "maneuver": {"steps": [
                {"duration": 2.0, "speed_kmh": 60,
                 "steer": {"kind": "constant", "amplitude": 0.0}},
                {"duration": 4.0, "speed_kmh": 60,
                 "steer": {"kind": "step", "amplitude": 1.0,
                           "unit": "front_deg", "t_step": 0.5}},
            ]},
        },
        "sweep": {"speed_kmh": {"values": [60],
                                "bind": "maneuver.steps.0.speed_kmh"}},
        "metrics": ["yaw_rate_peak_dps", "yaw_gain_dps"],
        "criteria": [{"metric": "yaw_rate_peak_dps", "must": "> 1", "at": "all"}],
        "report": {"template": "sweep"},
    }


def test_describe_is_the_first_stop(backend):
    caps = mcp_server._describe()
    assert caps["models"], caps.keys()
    assert caps["metrics"]
    assert caps["backend_version"]
    assert caps["mcp_facade"]["base"] == _test_base()


def test_the_agent_flow_spec_dry_run_run_report(backend):
    """S3 acceptance: new spec → dry-run → run → read report, tools only."""
    spec = _spec()

    # dry-run: zero cost, validates and estimates
    dry = mcp_server._dry_run(spec)
    assert dry["ok"], dry.get("problems")
    assert dry["runs"] == 1

    # run: blocks until done, returns the summary and the report path
    out = asyncio.run(mcp_server._run_study(spec))
    assert out.get("study_id"), out
    assert out["result"]["rows"], out["result"]
    verdict = out["result"]["verdicts"][0]
    assert verdict["passed"] is True, verdict

    # read the report: a path the agent can open, served by the backend
    rep = mcp_server._read_report(out["study_id"])
    assert rep["report_path"], rep
    assert rep["served"]

    # and the study is retrievable by id with provenance attached
    stored = mcp_server._get_study(out["study_id"])
    assert stored["provenance"]["git_sha"]


def test_get_trace_is_downsampled_and_listed_runs_include_the_run(backend):
    spec = _spec()
    out = asyncio.run(mcp_server._run_study(spec))
    run_id = out["result"]["rows"][0]["run_id"]

    trace = mcp_server._get_trace(run_id, "ay,yaw_rate", max_points=50)
    assert trace["run_id"] == run_id
    assert trace["n_samples"] <= 50

    runs = mcp_server._list_runs()
    assert any(run_id == r.get("run_id") for r in runs.get("runs", []))


def test_the_tool_handler_surfaces_refusals_as_errors(backend):
    """A refused spec comes back as an is_error result, not a crash."""
    bad = _spec()
    bad["model"] = "no_such_model"
    result = asyncio.run(mcp_server._call_tool(
        None,
        CallToolRequestParams(name="run_study",
                              arguments={"spec": bad, "dry_run": True}),
    ))
    assert result.is_error is True
    assert "error:" in result.content[0].text

def test_the_stdio_transport_serves_tools_end_to_end(backend):
    """A real MCP client over stdio: initialize, list tools, call one.

    This is the transport an agent host actually uses — the facade must speak
    the protocol, not just hold the functions.
    """

    async def run() -> None:
        params = StdioServerParameters(
            command=sys.executable,
            args=["-m", "sim4wis_mcp.server", "--base", mcp_server.BASE],
        )
        async with stdio_client(params) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                tools = await session.list_tools()
                names = {t.name for t in tools.tools}
                assert "run_study" in names and "read_report" in names, names
                res = await session.call_tool("describe_capabilities", {})
                assert "models" in res.content[0].text

    asyncio.run(run())
