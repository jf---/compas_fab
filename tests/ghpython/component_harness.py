from pathlib import Path
import sys
from types import ModuleType
from types import SimpleNamespace
from typing import Dict
from typing import Tuple

from pytest import MonkeyPatch

COMPONENTS = Path(__file__).parents[2] / "src" / "compas_fab" / "ghpython" / "components_cpython"


class FakeParameter:
    def __init__(self, name: str, connected: bool) -> None:
        self.Name = name
        self.SourceCount = int(connected)
        self.PersistentDataCount = 0


class FakeHost:
    def __init__(self, connections: Dict[str, bool]) -> None:
        parameters = [FakeParameter(name, connected) for name, connected in connections.items()]
        self.Params = SimpleNamespace(Input=parameters)


def load_component(
    monkeypatch: MonkeyPatch,
    directory: str,
    class_name: str,
    connections: Dict[str, bool],
) -> Tuple[object, FakeHost]:
    host = FakeHost(connections)
    grasshopper = ModuleType("Grasshopper")
    grasshopper.Kernel = SimpleNamespace(GH_ScriptInstance=object)
    monkeypatch.setitem(sys.modules, "Grasshopper", grasshopper)
    monkeypatch.setitem(sys.modules, "Rhino", ModuleType("Rhino"))
    monkeypatch.setitem(sys.modules, "System", ModuleType("System"))
    source = COMPONENTS / directory / "code.py"
    namespace = {"__name__": "test_" + directory, "__file__": str(source)}
    exec(compile(source.read_text(encoding="utf-8"), str(source), "exec"), namespace)
    namespace["ghenv"] = SimpleNamespace(Component=host)
    return namespace[class_name](), host
