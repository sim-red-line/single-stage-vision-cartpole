import pytest

from raspberry_pi.config import load_config
from raspberry_pi.runtime import CartPoleRuntime
from raspberry_pi.web_server import create_app


def test_remote_binding_requires_operator_token(tmp_path, monkeypatch):
    config = load_config()
    config["web"]["host"] = "0.0.0.0"
    config["web"]["require_operator_token"] = True
    monkeypatch.delenv("CARTPOLE_OPERATOR_TOKEN", raising=False)
    runtime = CartPoleRuntime(config, root=tmp_path)
    try:
        with pytest.raises(RuntimeError, match="at least 20 characters"):
            create_app(runtime, manage_runtime=False)
    finally:
        runtime.close()

