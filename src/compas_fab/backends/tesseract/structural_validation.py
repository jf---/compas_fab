"""Exact structural validation for COMPAS projections of Tesseract robots."""

from __future__ import annotations

from collections.abc import Iterable
from typing import Optional
from typing import cast

from compas.tolerance import TOL  # type: ignore[import-untyped]
from compas_robots.model import Joint as CompasJoint  # type: ignore[import-untyped]
from tesseract_robotics.planning import Robot
from tesseract_robotics.tesseract_scene_graph import Joint as TesseractJoint

from compas_fab.robots import RobotCell

from .errors import RobotArtifactMismatchError


def validate_robot_cell_structure(
    native_robot: Robot,
    robot_cell: RobotCell,
    artifact_groups: dict[str, Optional[tuple[str, str]]],
    coupled_groups: frozenset[str] = frozenset(),
) -> None:
    """Validate every unit-bearing kinematic field consumed by the backend.

    A coordinated (coupled ROP/REP) group named in ``coupled_groups`` spans two
    scene-graph branches; its native kinematic root is not the COMPAS joint-group base
    and no single serial chain walks it, so for such a group the base-link and chain-walk
    checks that assume a serial group are skipped -- joint names, order, and per-joint
    properties are still validated. Non-coupled groups take the serial path.
    """
    model = robot_cell.robot_model
    semantics = robot_cell.robot_semantics
    if model is None or semantics is None:
        raise RobotArtifactMismatchError("RobotCell requires both robot_model and robot_semantics.")
    native_name = native_robot.env.getName()
    if native_name != model.name:
        raise RobotArtifactMismatchError("Robot name differs: Tesseract {!r}, COMPAS {!r}.".format(native_name, model.name))

    for group in robot_cell.group_names:
        if group not in artifact_groups:
            raise RobotArtifactMismatchError("Exact Tesseract SRDF has no semantic group {!r}.".format(group))
        coupled = group in coupled_groups
        artifact_chain = artifact_groups[group]
        if artifact_chain is not None and not coupled:
            artifact_base, artifact_tip = artifact_chain
            compas_base = robot_cell.get_base_link_name(group)
            compas_tip = robot_cell.get_end_effector_link_name(group)
            if artifact_base != compas_base:
                raise RobotArtifactMismatchError(
                    "Planning group {!r} base link differs: exact SRDF {!r}, COMPAS {!r}.".format(
                        group,
                        artifact_base,
                        compas_base,
                    )
                )
            if artifact_tip != compas_tip:
                raise RobotArtifactMismatchError(
                    "Planning group {!r} tip link differs: exact SRDF {!r}, COMPAS {!r}.".format(
                        group,
                        artifact_tip,
                        compas_tip,
                    )
                )
        compas_joints = robot_cell.get_configurable_joints(group)
        if not compas_joints:
            continue
        try:
            native_group = native_robot.env.getJointGroup(group)
        except (KeyError, RuntimeError) as error:
            raise RobotArtifactMismatchError("Tesseract artifact has no planning group {!r}.".format(group)) from error

        compas_names = [joint.name for joint in compas_joints]
        native_names = list(native_group.getJointNames())
        if native_names != compas_names:
            raise RobotArtifactMismatchError("Planning group {!r} joint order differs: Tesseract {}, COMPAS {}.".format(group, native_names, compas_names))

        if coupled:
            # A coupled ROP/REP group spans two scene-graph branches: its native root
            # (world) is not the COMPAS joint-group base and no serial chain walks it.
            # Validate each configurable joint directly by name; the base-link and
            # chain-walk checks below assume a serial group and do not apply.
            for compas_joint in compas_joints:
                try:
                    native_joint = native_robot.env.getJoint(compas_joint.name)
                except (KeyError, RuntimeError) as error:
                    raise RobotArtifactMismatchError("Tesseract coupled group {!r} is missing joint {!r}.".format(group, compas_joint.name)) from error
                _validate_joint(group, native_joint, compas_joint)
            continue

        native_base = native_group.getBaseLinkName()
        compas_base = robot_cell.get_base_link_name(group)
        if native_base != compas_base:
            raise RobotArtifactMismatchError("Planning group {!r} base link differs: Tesseract {!r}, COMPAS {!r}.".format(group, native_base, compas_base))

        compas_tip = robot_cell.get_end_effector_link_name(group)
        try:
            compas_chain = list(model.iter_joint_chain(compas_base, compas_tip))
        except (KeyError, RuntimeError, ValueError) as error:
            raise RobotArtifactMismatchError(
                "COMPAS planning group {!r} chain cannot be resolved from {!r} to {!r}: {}.".format(
                    group,
                    compas_base,
                    compas_tip,
                    error,
                )
            ) from error
        for compas_joint in compas_chain:
            try:
                native_joint = native_robot.env.getJoint(compas_joint.name)
            except (KeyError, RuntimeError) as error:
                raise RobotArtifactMismatchError(
                    "Tesseract planning group {!r} is missing chain joint {!r}.".format(
                        group,
                        compas_joint.name,
                    )
                ) from error
            _validate_joint(group, native_joint, compas_joint)


def _validate_joint(group: str, native_joint: TesseractJoint, compas_joint: CompasJoint) -> None:
    joint_name = cast(str, compas_joint.name)
    native_type = native_joint.type.name
    compas_type = cast(str, compas_joint.type.name)
    if native_type != compas_type:
        raise RobotArtifactMismatchError(
            "Planning group {!r} joint {!r} type differs: Tesseract {}, COMPAS {}; native values cannot be relabelled across radians/metres.".format(
                group,
                joint_name,
                native_type,
                compas_type,
            )
        )

    _require_equal(group, joint_name, "parent link", native_joint.parent_link_name, compas_joint.parent.link)
    _require_equal(group, joint_name, "child link", native_joint.child_link_name, compas_joint.child.link)

    if compas_type != "FIXED":
        native_axis = [float(value) for value in native_joint.axis]
        compas_axis = [float(value) for value in compas_joint.axis.vector]
        if not _all_exact(native_axis, compas_axis):
            raise RobotArtifactMismatchError("Planning group {!r} joint {!r} axis differs: Tesseract {}, COMPAS {}.".format(group, joint_name, native_axis, compas_axis))

    native_origin = _flatten(native_joint.parent_to_joint_origin_transform.matrix)
    compas_origin = _flatten(compas_joint.origin.to_transformation().matrix)
    if not _all_exact(native_origin, compas_origin):
        raise RobotArtifactMismatchError("Planning group {!r} joint {!r} origin differs.".format(group, joint_name))

    compas_limits = compas_joint.limit
    if compas_type == "FIXED":
        return
    if compas_limits is None:
        raise RobotArtifactMismatchError("Planning group {!r} joint {!r} has no COMPAS limits.".format(group, joint_name))
    limit_fields = ["effort", "velocity"]
    if compas_type in {"REVOLUTE", "PRISMATIC"}:
        limit_fields.extend(["lower", "upper"])
    differences = [field for field in limit_fields if not _exact(float(getattr(native_joint.limits, field)), float(getattr(compas_limits, field)))]
    if differences:
        raise RobotArtifactMismatchError("Planning group {!r} joint {!r} limits differ for: {}.".format(group, joint_name, ", ".join(differences)))


def _require_equal(group: str, joint: str, field: str, native_value: object, compas_value: object) -> None:
    if native_value != compas_value:
        raise RobotArtifactMismatchError("Planning group {!r} joint {!r} {} differs: Tesseract {!r}, COMPAS {!r}.".format(group, joint, field, native_value, compas_value))


def _flatten(rows: Iterable[Iterable[float]]) -> list[float]:
    return [float(value) for row in rows for value in row]


def _all_exact(left: list[float], right: list[float]) -> bool:
    return len(left) == len(right) and all(_exact(a, b) for a, b in zip(left, right))


def _exact(left: float, right: float) -> bool:
    # is_between is atol-only and avoids Tolerance.is_close's relative slack.
    return cast(bool, TOL.is_between(left, right, right, atol=TOL.absolute))
