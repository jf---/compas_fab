"""Pytest fixtures that run Python inside a live Rhino 8 via the ``rhinocode`` CLI.

These drive the ACTUAL Grasshopper kernel — a component is instantiated in a real
``GH_Document`` and solved by Grasshopper's solver, not by a monkeypatch. The
suite is opt-in: the repo's root ``conftest.pytest_ignore_collect`` drops any
path containing ``rhino`` from the default collection, so ``pytest -n auto`` never
runs it. Invoke it explicitly (``pytest tests/rhino``) on a machine with Rhino 8
open; the fixture fails loudly if no instance is found — it never skips.

See `docs/developer/rhino_harness.md` for the transport (async submit, filesystem
back-channel, extension-based language detection).
"""

import json
import subprocess
import tempfile
import time
import uuid
from pathlib import Path

import pytest

_RHINOCODE = "/Applications/Rhino 8.app/Contents/Resources/bin/rhinocode"
# World-writable dir both Rhino's sandboxed Python and the test process can reach.
_SHARED = Path("/Users/Shared")
# Placeholder the driver template uses for its result path; the fixture fills it.
_RESULT_TOKEN = "__RESULT_PATH__"


def _running_instance() -> str:
    """Return the id of a running Rhino instance, or fail loudly."""
    listing = subprocess.run([_RHINOCODE, "list"], capture_output=True, text=True, timeout=30).stdout
    for line in listing.splitlines():
        for token in line.split():
            if token.startswith("rhinocode_remotepipe_"):
                return token
    raise RuntimeError("No running Rhino 8 instance found (`rhinocode list` was empty). Open Rhino 8 to run the live-kernel suite.")


@pytest.fixture(scope="session")
def rhino_instance() -> str:
    """The live Rhino instance id all driver scripts are submitted to."""
    return _running_instance()


@pytest.fixture()
def rhino_run(rhino_instance):
    """Return a callable that runs a driver in live Rhino and returns its JSON result.

    The driver template must write JSON to the path substituted for
    ``__RESULT_PATH__``. Submission is asynchronous (``rhinocode script`` returns
    on submit), so the caller bounded-waits for the result file.
    """

    def run(driver_template: str, timeout: float = 40.0) -> dict:
        result_path = _SHARED / "rh_{}.json".format(uuid.uuid4().hex)
        result_path.unlink(missing_ok=True)
        source = driver_template.replace(_RESULT_TOKEN, str(result_path))
        with tempfile.NamedTemporaryFile("w", suffix=".py", delete=False) as handle:
            handle.write(source)
            driver_path = handle.name
        try:
            subprocess.run([_RHINOCODE, "-r", rhino_instance, "script", driver_path], timeout=timeout, capture_output=True)
            deadline = time.time() + timeout
            while not result_path.exists():
                if time.time() > deadline:
                    raise TimeoutError("Rhino produced no result within {}s (driver: {}).".format(timeout, driver_path))
                time.sleep(0.1)
            return json.loads(result_path.read_text())
        finally:
            Path(driver_path).unlink(missing_ok=True)
            result_path.unlink(missing_ok=True)

    return run
