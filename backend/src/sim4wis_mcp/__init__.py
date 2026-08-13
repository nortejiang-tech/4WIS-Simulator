"""sim4wis-mcp — the agent facade over the study layer.

A thin stdio MCP server with no `sim4wis` import: every capability is an HTTP
call against `SIM4WIS_BACKEND_HTTP` (default http://127.0.0.1:8010), so the
facade and the backend can evolve independently — see
docs/agent_interface_design.md §10 and §13.
"""

__version__ = "0.1.0"
