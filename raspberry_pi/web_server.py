"""FastAPI dashboard and operator API."""

from __future__ import annotations

import asyncio
import os
import secrets
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field

from .models import MarkerRole
from .runtime import CartPoleRuntime


class ROIRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    name: str = Field(pattern=r"^[A-Za-z0-9_-]{1,32}$")
    role: MarkerRole
    x: int
    y: int
    width: int = Field(ge=4)
    height: int = Field(ge=4)
    required: bool = True


class MarkerRangesRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    ranges: list[dict[str, list[int]]]
    min_area_px: float = Field(gt=0)
    max_area_px: float = Field(gt=0)


class CoordinateRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    rail_left_px: float
    rail_right_px: float
    rail_length_mm: float = Field(gt=0)
    camera_x_sign: int = Field(ge=-1, le=1)
    pivot_y_px: float | None = None
    vertical_offset_rad: float = 0.0
    vertical_offset_slope_rad_per_mm: float = Field(default=0.0, allow_inf_nan=False)
    vertical_offset_reference_mm: float = Field(default=0.0, allow_inf_nan=False)
    pendulum_length_mm: float | None = Field(default=None, gt=0)
    homography: list[list[float]] | None = None


class SaveProfileRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    name: str = Field(pattern=r"^[A-Za-z0-9_-]{1,40}$")
    coordinates: CoordinateRequest
    notes: str = Field(default="", max_length=500)


class NotesRequest(BaseModel):
    notes: str = Field(default="", max_length=1000)


class SwingUpParametersRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, strict=True)

    energy_gain: float | None = Field(default=None, ge=0.1, le=64.0)
    energy_target_margin: float | None = Field(default=None, ge=0.0, le=0.05)
    energy_rate_lag_s: float | None = Field(default=None, ge=0.0, le=0.06)
    kick_acceleration_mm_s2: float | None = Field(default=None, ge=0.0)
    kick_duration_s: float | None = Field(default=None, ge=0.0, le=1.0)
    max_duration_s: float | None = Field(default=None, ge=0.5, le=30.0)
    cart_position_gain_s2: float | None = Field(default=None, ge=0.0, le=5.0)
    cart_velocity_gain_s: float | None = Field(default=None, ge=0.0, le=5.0)


def _raise_http(exc: Exception) -> None:
    if isinstance(exc, KeyError):
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    if isinstance(exc, (ValueError, RuntimeError, ConnectionError)):
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    raise exc


def create_app(runtime: CartPoleRuntime, *, manage_runtime: bool = True) -> FastAPI:
    web_config = runtime.config.get("web", {})
    host = str(web_config.get("host", "127.0.0.1"))
    loopback = host in {"127.0.0.1", "localhost", "::1"}
    require_token = bool(web_config.get("require_operator_token", False)) or not loopback
    token_env = str(web_config.get("operator_token_env", "CARTPOLE_OPERATOR_TOKEN"))
    operator_token = os.environ.get(token_env, "")
    if require_token and len(operator_token) < 20:
        raise RuntimeError(
            f"{token_env} must contain at least 20 characters when the dashboard is remotely bound"
        )

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        if manage_runtime:
            runtime.start()
        yield
        if manage_runtime:
            runtime.close()

    app = FastAPI(
        title="Vision Cart-Pole Control",
        version="0.1.0",
        lifespan=lifespan,
        docs_url="/api/docs",
        redoc_url=None,
    )

    @app.middleware("http")
    async def operator_authentication(request: Request, call_next):
        mutating = request.method in {"POST", "PUT", "PATCH", "DELETE"}
        if require_token and mutating and request.url.path.startswith("/api/"):
            supplied = request.headers.get("X-Operator-Token", "")
            if not secrets.compare_digest(supplied, operator_token):
                return JSONResponse(
                    status_code=401,
                    content={"detail": "a valid operator token is required"},
                )
        return await call_next(request)

    @app.get("/api/auth/info")
    async def auth_info() -> dict[str, Any]:
        return {"operator_token_required": require_token}

    @app.get("/api/status")
    async def status() -> dict[str, Any]:
        return runtime.snapshot()

    @app.get("/api/recent")
    async def recent(limit: int = 600) -> dict[str, Any]:
        return {"samples": runtime.recent_series(limit)}

    @app.get("/api/video.mjpg")
    async def video() -> StreamingResponse:
        async def frames():
            last_identity: int | None = None
            while True:
                jpeg = runtime.latest_jpeg()
                if jpeg is not None and id(jpeg) != last_identity:
                    last_identity = id(jpeg)
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                await asyncio.sleep(0.025)

        return StreamingResponse(
            frames(), media_type="multipart/x-mixed-replace; boundary=frame"
        )

    @app.get("/api/calibration/mask/{marker_name}.jpg")
    async def marker_mask(marker_name: str) -> Response:
        jpeg = runtime.latest_mask_jpeg(marker_name)
        if jpeg is None:
            raise HTTPException(status_code=404, detail="marker mask is not available")
        return Response(jpeg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.get("/api/calibration/profiles")
    async def profiles() -> dict[str, Any]:
        return {
            "profiles": runtime.profile_store.list_profiles(),
            "active": runtime.profile.name if runtime.profile else None,
            "staged_markers": runtime.staged_markers(),
        }

    @app.post("/api/calibration/profiles/{name}/load")
    async def load_profile(name: str) -> dict[str, Any]:
        try:
            runtime.load_profile(name)
            return {"ok": True, "profile": name, "live_verification_required": True}
        except Exception as exc:
            _raise_http(exc)
        return {}

    @app.post("/api/calibration/sample-roi")
    async def sample_roi(request: ROIRequest) -> dict[str, Any]:
        try:
            marker = runtime.sample_marker(
                request.name,
                request.role,
                (request.x, request.y, request.width, request.height),
                request.required,
            )
            return {"ok": True, "marker": marker}
        except Exception as exc:
            _raise_http(exc)
        return {}

    @app.put("/api/calibration/markers/{name}")
    async def set_marker(name: str, request: MarkerRangesRequest) -> dict[str, Any]:
        try:
            marker = runtime.set_marker_ranges(
                name, request.ranges, request.min_area_px, request.max_area_px
            )
            return {"ok": True, "marker": marker}
        except Exception as exc:
            _raise_http(exc)
        return {}

    @app.post("/api/calibration/save")
    async def save_profile(request: SaveProfileRequest) -> dict[str, Any]:
        if request.coordinates.camera_x_sign not in {-1, 1}:
            raise HTTPException(status_code=422, detail="camera_x_sign must be -1 or 1")
        try:
            return runtime.save_profile(
                request.name, request.coordinates.model_dump(), request.notes
            )
        except Exception as exc:
            _raise_http(exc)
        return {}

    @app.post("/api/control/home")
    async def home() -> dict[str, Any]:
        try:
            runtime.home()
            return {"ok": True}
        except Exception as exc:
            _raise_http(exc)
        return {}

    @app.post("/api/control/arm")
    async def arm() -> dict[str, Any]:
        ok = runtime.arm()
        if not ok:
            raise HTTPException(status_code=409, detail=runtime.safety.reason)
        return {"ok": True}

    @app.post("/api/control/start")
    async def start_control() -> dict[str, Any]:
        try:
            ok = runtime.begin_control()
        except Exception as exc:
            _raise_http(exc)
        if not ok:
            raise HTTPException(status_code=409, detail=runtime.safety.reason)
        return {"ok": True}

    @app.post("/api/control/stop")
    async def stop_control() -> dict[str, Any]:
        runtime.operator_stop()
        return {"ok": True}

    @app.put("/api/control/swing-up/parameters")
    async def swing_up_parameters(request: SwingUpParametersRequest) -> dict[str, Any]:
        try:
            settings = runtime.configure_swing_up(request.model_dump(exclude_none=True))
            return {"ok": True, "swing_up": settings}
        except Exception as exc:
            _raise_http(exc)
        return {}

    @app.post("/api/control/estop")
    async def estop() -> dict[str, Any]:
        runtime.estop()
        return {"ok": True}

    @app.post("/api/control/clear-estop")
    async def clear_estop() -> dict[str, Any]:
        try:
            ok = runtime.clear_estop()
            if not ok:
                raise RuntimeError(runtime.safety.reason)
            return {"ok": True, "homing_required": True}
        except Exception as exc:
            _raise_http(exc)
        return {}

    @app.post("/api/control/clear-fault")
    async def clear_fault() -> dict[str, Any]:
        if not runtime.clear_fault():
            raise HTTPException(status_code=409, detail=runtime.safety.reason)
        return {"ok": True}

    @app.post("/api/experiments/start")
    async def experiment_start(request: NotesRequest) -> dict[str, Any]:
        try:
            return runtime.start_experiment(request.notes)
        except Exception as exc:
            _raise_http(exc)
        return {}

    @app.post("/api/experiments/finish")
    async def experiment_finish() -> dict[str, Any]:
        try:
            return runtime.finish_experiment()
        except Exception as exc:
            _raise_http(exc)
        return {}

    @app.get("/api/experiments")
    async def experiments(limit: int = 50) -> dict[str, Any]:
        summaries = runtime.experiment_summaries(limit=max(1, min(limit, 200)))
        compact = [
            {key: value for key, value in item.items() if key != "representative_series"}
            for item in summaries
        ]
        return {"experiments": compact}

    @app.get("/api/experiments/{experiment_id}")
    async def experiment(experiment_id: str) -> dict[str, Any]:
        try:
            return runtime.get_summary(experiment_id)
        except FileNotFoundError as exc:
            raise HTTPException(status_code=404, detail="experiment not found") from exc
        except Exception as exc:
            _raise_http(exc)
        return {}

    @app.websocket("/ws")
    async def websocket_status(socket: WebSocket) -> None:
        await socket.accept()
        try:
            while True:
                await socket.send_json(runtime.snapshot())
                await asyncio.sleep(0.10)
        except (WebSocketDisconnect, RuntimeError):
            return

    dashboard_dir = runtime.root / "dashboard"

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(dashboard_dir / "index.html")

    app.mount("/static", StaticFiles(directory=dashboard_dir), name="static")
    return app

