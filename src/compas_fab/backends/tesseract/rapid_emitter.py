"""Typed delegation to Tesseract's native RAPID emitter."""

from collections.abc import Mapping
from threading import RLock
from typing import Optional

from tesseract_robotics.emitters.rapid import ExternalAxisKind
from tesseract_robotics.emitters.rapid import ExternalAxisLayout
from tesseract_robotics.emitters.rapid import ExternalAxisSpec
from tesseract_robotics.emitters.rapid import RapidProfile
from tesseract_robotics.emitters.rapid import emit_rapid
from tesseract_robotics.tesseract_command_language import CompositeInstruction

from .external_axes import CoupledGroupLayout
from .external_axes import ExternalAxisRole
from .rapid_identity import validate_rapid_name
from .rapid_profiles import normalize_rapid_profiles
from .rapid_program import RapidProgram
from .rapid_program import validate_rapid_program

_RAPID_WRITER_LOCK = RLock()

# COMPAS classifies an external axis by role (TRACK/POSITIONER); the emitter needs
# the joint KIND, which fixes the unit: a TRACK is a prismatic LINEAR axis
# (SI metres -> RAPID mm), a POSITIONER a revolute ROTARY axis (SI radians -> deg).
_ROLE_TO_KIND = {
    ExternalAxisRole.TRACK: ExternalAxisKind.LINEAR,
    ExternalAxisRole.POSITIONER: ExternalAxisKind.ROTARY,
}


def _emitter_external_axis_layout(coupled: CoupledGroupLayout) -> ExternalAxisLayout:
    """Marshal a COMPAS ``CoupledGroupLayout`` to the emitter's ``ExternalAxisLayout``.

    The external axes keep their coupled-group order (``external[0]`` -> ``eax_a``);
    controller-specific slot mapping is a later concern.

    Args:
        coupled: The arm/external classification for a coordinated group.

    Returns:
        The emitter-side layout naming the six arm joints and the external-axis
        specs (name + kind) in eax-slot order.
    """
    specs = [ExternalAxisSpec.build(axis.name, _ROLE_TO_KIND[axis.role]) for axis in coupled.external_axes]
    return ExternalAxisLayout.build(coupled.arm_joint_names, specs)


class TesseractRapidEmitter:
    """Emit exact native Tesseract programs as ABB RAPID source."""

    @staticmethod
    def emit(
        program: CompositeInstruction,
        profiles: Mapping[str, RapidProfile],
        module_name: str = "main_program",
        procedure_name: str = "main",
        coupled_axes: Optional[CoupledGroupLayout] = None,
    ) -> RapidProgram:
        """Delegate one exact native program to Tesseract's RAPID emitter.

        Args:
            program: Exact native program. Wrappers and trajectories are invalid.
            profiles: Exact Tesseract profile names mapped to native RAPID profiles.
            module_name: RAPID module name.
            procedure_name: RAPID procedure name.
            coupled_axes: Optional coupled-group layout. When given, each target's
                external-axis (`eax`) slots carry the group's external axes in
                mm/deg by role; when `None`, emission is byte-identical to an
                uncoupled program (all `eax` slots stay at the `9E9` sentinel).

        Returns:
            Content-addressed RAPID source retaining the supplied program.

        Raises:
            InvalidRapidProgramError: Program is not `CompositeInstruction`.
            InvalidRapidProfileMapError: Profile mapping is malformed.
            InvalidRapidProgramNameError: A module or procedure name is empty.
            RapidEmitterError: Native emission fails. Its exact subtype propagates.
        """
        validated_program = validate_rapid_program(program)
        validated_profiles = normalize_rapid_profiles(profiles)
        validated_module = validate_rapid_name(module_name, "module")
        validated_procedure = validate_rapid_name(procedure_name, "procedure")
        external_axes = _emitter_external_axis_layout(coupled_axes) if coupled_axes is not None else None

        # Tesseract's RapidWriter owns one process-global buffer. Grasshopper can
        # solve components concurrently, so one native emission must remain atomic.
        with _RAPID_WRITER_LOCK:
            source = emit_rapid(
                validated_program,
                validated_profiles,
                external_axes=external_axes,
                module_name=validated_module,
                proc_name=validated_procedure,
            )

        return RapidProgram.build(
            validated_program,
            source,
            validated_module,
            validated_procedure,
        )
