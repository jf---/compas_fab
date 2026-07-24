"""End-to-end E1 integration: coordinated external-axis layout -> RAPID ``eax``.

Ties T2's ``CoupledGroupLayout`` (arm/external classification derived from the
RobotCell) through ``TesseractRapidEmitter.emit(coupled_axes=...)`` to the
emitter's ``eax`` population. On the ROP reference cell (the positioner joint is
ordered first), a 7-DOF coordinated joint target must emit a ``MoveAbsJ`` whose
``eax`` slot a carries the positioner value in millimetres (never the ``9E+09``
sentinel), round-tripping within display tolerance.
"""

import re
from pathlib import Path

import numpy as np
import tesseract_robotics
from compas_robots import RobotModel
from tesseract_robotics.emitters.rapid import RapidProfile
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_command_language import JointWaypoint
from tesseract_robotics.tesseract_command_language import JointWaypointPoly_wrap_JointWaypoint
from tesseract_robotics.tesseract_command_language import MoveInstruction
from tesseract_robotics.tesseract_command_language import MoveInstructionPoly_wrap_MoveInstruction
from tesseract_robotics.tesseract_command_language import MoveInstructionType_FREESPACE

from compas_fab.backends.tesseract.external_axes import CoupledGroupLayout
from compas_fab.backends.tesseract.rapid_emitter import TesseractRapidEmitter
from compas_fab.robots import RobotCell
from compas_fab.robots import RobotSemantics

_SUPPORT = Path(tesseract_robotics.__file__).parent / "data" / "tesseract" / "support" / "urdf"
_ARM = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
_COUPLED = ("positioner_joint_1", *_ARM)
_PROFILES = {"DEFAULT": RapidProfile(speed="v200", zone="z10", tool="tool0", wobj="wobj0")}
_MM_ATOL = 1e-4  # one .4f mm display step bounds the round-trip rounding error


def _rop_cell():
    urdf = (_SUPPORT / "abb_irb2400_on_positioner.urdf").read_text()
    srdf = (_SUPPORT / "abb_irb2400_on_positioner.srdf").read_text()
    model = RobotModel.from_urdf_string(urdf)
    semantics = RobotSemantics.from_srdf_string(srdf, model)
    return RobotCell(model, semantics)


def _coordinated_program(positions):
    waypoint = JointWaypoint()
    waypoint.setNames(list(_COUPLED))
    waypoint.setPosition(np.asarray(positions, dtype=np.float64))
    move = MoveInstruction(JointWaypointPoly_wrap_JointWaypoint(waypoint), MoveInstructionType_FREESPACE, "DEFAULT")
    composite = CompositeInstruction("t5")
    composite.push_back(MoveInstructionPoly_wrap_MoveInstruction(move))
    return composite


def _moveabsj_targets(rapid: str):
    """Return (robax tokens, eax tokens) from the single ``MoveAbsJ`` jointtarget."""
    match = re.search(r"MoveAbsJ \[\[([^\]]*)\],\s*\[([^\]]*)\]\]", rapid)
    assert match, rapid
    robax = [token.strip() for token in match.group(1).split(",")]
    eax = [token.strip() for token in match.group(2).split(",")]
    return robax, eax


def test_positioner_eax_emitted_in_mm():
    """The coupled layout drives the positioner into eax slot a, in mm."""
    layout = CoupledGroupLayout.build(_rop_cell(), "full_manipulator", "manipulator")
    # positioner_joint_1 (prismatic) at 0.5 m; arm at zero.
    program = _coordinated_program([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    rapid = str(TesseractRapidEmitter.emit(program, _PROFILES, coupled_axes=layout).source)

    robax, eax = _moveabsj_targets(rapid)
    # The six arm joints stay in robax; the positioner is peeled off by NAME.
    assert len(robax) == 6
    assert len(eax) == 6
    # Slot a carries the positioner value converted to mm and round-trips; b..f
    # keep the 9E+09 sentinel (unused eax slots).
    assert abs(float(eax[0]) - 500.0) < _MM_ATOL  # 0.5 m -> 500.0000 mm
    assert all(token == "9E+09" for token in eax[1:])


def test_uncoupled_emit_keeps_all_sentinel_eax():
    """Without a layout, a 6-DOF arm program emits all-sentinel eax (byte-identical)."""
    waypoint = JointWaypoint()
    waypoint.setNames(list(_ARM))
    waypoint.setPosition(np.zeros(6, dtype=np.float64))
    move = MoveInstruction(JointWaypointPoly_wrap_JointWaypoint(waypoint), MoveInstructionType_FREESPACE, "DEFAULT")
    composite = CompositeInstruction("t5-uncoupled")
    composite.push_back(MoveInstructionPoly_wrap_MoveInstruction(move))

    rapid = str(TesseractRapidEmitter.emit(composite, _PROFILES).source)
    _, eax = _moveabsj_targets(rapid)
    assert all(token == "9E+09" for token in eax)
