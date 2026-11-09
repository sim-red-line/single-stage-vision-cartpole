"""Command-line entry point for the Raspberry Pi service."""

from __future__ import annotations

import argparse
from pathlib import Path

import uvicorn
from dotenv import load_dotenv

from .config import load_config, project_root
from .runtime import CartPoleRuntime
from .web_server import create_app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Vision cart-pole controller and dashboard")
    parser.add_argument("--config", type=Path, default=None, help="optional YAML overlay")
    parser.add_argument("--host", default=None, help="dashboard bind address")
    parser.add_argument("--port", type=int, default=None, help="dashboard port")
    return parser.parse_args()


def main() -> None:
    load_dotenv(project_root() / ".env")
    args = parse_args()
    config = load_config(args.config)
    bind_host = args.host or str(config["web"]["host"])
    if bind_host not in {"127.0.0.1", "localhost", "::1"}:
        config["web"]["host"] = bind_host
        config["web"]["require_operator_token"] = True
    runtime = CartPoleRuntime(config)
    app = create_app(runtime)
    uvicorn.run(
        app,
        host=bind_host,
        port=args.port or int(config["web"]["port"]),
        log_level="info",
        # MJPEG clients must not keep the serial owner alive indefinitely.
        timeout_graceful_shutdown=3,
    )


if __name__ == "__main__":
    main()

