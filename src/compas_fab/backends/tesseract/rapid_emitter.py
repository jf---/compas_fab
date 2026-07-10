"""Typed delegation to Tesseract's native RAPID emitter."""

from collections.abc import Mapping
from threading import RLock

from tesseract_robotics.emitters.rapid import RapidProfile
from tesseract_robotics.emitters.rapid import emit_rapid
from tesseract_robotics.tesseract_command_language import CompositeInstruction

from .rapid_identity import validate_rapid_name
from .rapid_profiles import normalize_rapid_profiles
from .rapid_program import RapidProgram
from .rapid_program import validate_rapid_program

_RAPID_WRITER_LOCK = RLock()


class TesseractRapidEmitter:
    """Emit exact native Tesseract programs as ABB RAPID source."""

    @staticmethod
    def emit(
        program: CompositeInstruction,
        profiles: Mapping[str, RapidProfile],
        module_name: str = "main_program",
        procedure_name: str = "main",
    ) -> RapidProgram:
        """Delegate one exact native program to Tesseract's RAPID emitter.

        Args:
            program: Exact native program. Wrappers and trajectories are invalid.
            profiles: Exact Tesseract profile names mapped to native RAPID profiles.
            module_name: RAPID module name.
            procedure_name: RAPID procedure name.

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

        # Tesseract's RapidWriter owns one process-global buffer. Grasshopper can
        # solve components concurrently, so one native emission must remain atomic.
        with _RAPID_WRITER_LOCK:
            source = emit_rapid(
                validated_program,
                validated_profiles,
                module_name=validated_module,
                proc_name=validated_procedure,
            )

        return RapidProgram.build(
            validated_program,
            source,
            validated_module,
            validated_procedure,
        )
