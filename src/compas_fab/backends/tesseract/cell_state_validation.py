"""Named validation of COMPAS cell state at native boundaries."""

from compas_fab.robots import RobotCell
from compas_fab.robots import RobotCellState

from .errors import TesseractCellStateMismatchError


def require_matching_cell_state(
    robot_cell: RobotCell,
    state: RobotCellState,
    operation: str,
) -> None:
    """Wrap COMPAS cell-state mismatch diagnostics in a backend error."""
    try:
        robot_cell.assert_cell_state_match(state)
    except ValueError as error:
        raise TesseractCellStateMismatchError("Tesseract {} cell-state mismatch: {}.".format(operation, error)) from error
