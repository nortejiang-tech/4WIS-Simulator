"""FastAPI application entry point."""

from __future__ import annotations

import logging
from contextlib import asynccontextmanager

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from sim4wis.api import rest, ws
from sim4wis.core.simulator import get_simulator

logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Start/stop the application-wide Simulator alongside the HTTP server."""
    # Plugin discovery — log failures, never abort startup
    try:
        from sim4wis.controller.plugins.loader import discover_and_register
        rep = discover_and_register()
        logger.info("Plugin discovery: %d loaded, %d failed",
                    sum(1 for r in rep if r.ok), sum(1 for r in rep if not r.ok))
    except Exception:
        logger.exception("Plugin discovery raised — continuing without plugins")
    sim = get_simulator()
    await sim.start()
    logger.info("sim4wis backend ready")
    try:
        yield
    finally:
        await sim.stop()
        logger.info("sim4wis backend shut down")


def create_app() -> FastAPI:
    app = FastAPI(
        title="4WIS Simulator",
        description="4-Wheel Independent Steering Simulator backend.",
        version="0.3.0",
        lifespan=lifespan,
    )

    # Dev-mode CORS — Vite dev server runs on :5173 and proxies to us.
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[
            "http://localhost:5173",
            "http://127.0.0.1:5173",
        ],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    app.include_router(rest.router)
    app.include_router(ws.router)

    @app.get("/health", tags=["meta"])
    async def health() -> dict[str, str]:
        return {"status": "ok"}

    # Serve the built frontend (production / portable build) at the root, so the
    # whole app is a single process on a single port — no Vite, no proxy. The
    # API/WS routes above are registered first and take precedence. In the dev
    # tree (no dist) this is skipped and you use the Vite dev server instead.
    from sim4wis.paths import dist_dir
    _dist = dist_dir()
    if _dist is not None:
        from fastapi.staticfiles import StaticFiles
        app.mount("/", StaticFiles(directory=str(_dist), html=True), name="frontend")
        logger.info("Serving frontend from %s", _dist)

    return app


app = create_app()


def run() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s — %(message)s",
    )
    # Port 8010 (NOT 8000 — that's reserved for the local LLM inference server).
    uvicorn.run("sim4wis.main:app", host="127.0.0.1", port=8010, reload=False)


if __name__ == "__main__":
    run()
