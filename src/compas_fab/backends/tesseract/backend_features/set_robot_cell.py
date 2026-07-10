"""Install a COMPAS robot cell against an exact Tesseract artifact."""

from __future__ import annotations

from typing import TYPE_CHECKING
from typing import Optional
from xml.etree import ElementTree

from compas_fab.backends.interfaces import SetRobotCell
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState

from ..cell_state_validation import require_matching_cell_state
from ..client import TesseractClient
from ..errors import UnknownTesseractOptionError
from ..structural_validation import validate_robot_cell_structure


class TesseractSetRobotCell(SetRobotCell):
    """Validate COMPAS semantics against the native environment."""

    if TYPE_CHECKING:
        client: TesseractClient

    def set_robot_cell(
        self,
        robot_cell: RobotCell,
        robot_cell_state: Optional[RobotCellState] = None,
        options: Optional[dict[str, object]] = None,
    ) -> None:
        """Install an isolated COMPAS cell projection.

        Args:
            robot_cell: COMPAS projection of the exact artifact.
            robot_cell_state: Optional initial state retained as a copy.
            options: Must be empty; no backend options exist for this boundary.

        Raises:
            UnknownTesseractOptionError: Options were supplied.
            RobotArtifactMismatchError: Group joint order differs from Tesseract.
        """
        if options:
            raise UnknownTesseractOptionError("set_robot_cell does not accept Tesseract options: {}.".format(", ".join(sorted(options))))
        client: TesseractClient = self.client
        native_robot = client.environment.robot
        artifact_root = ElementTree.fromstring(client.artifact.srdf)
        artifact_groups = _artifact_group_chains(artifact_root)
        validate_robot_cell_structure(native_robot, robot_cell, artifact_groups)
        if robot_cell_state is not None:
            require_matching_cell_state(
                robot_cell,
                robot_cell_state,
                "stored native scene",
            )

        client._store_robot_cell_projection(robot_cell, robot_cell_state)


def _artifact_group_chains(
    artifact_root: ElementTree.Element,
) -> dict[str, Optional[tuple[str, str]]]:
    groups: dict[str, Optional[tuple[str, str]]] = {}
    for group in artifact_root.findall("group"):
        name = group.attrib.get("name")
        if not name:
            continue
        chain = group.find("chain")
        if chain is None:
            groups[name] = None
            continue
        base = chain.attrib.get("base_link")
        tip = chain.attrib.get("tip_link")
        groups[name] = (base, tip) if base and tip else None
    return groups
