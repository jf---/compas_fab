"""Immutable RAPID source artifact with explicit filesystem output."""

from __future__ import annotations

from pathlib import Path

from attrs import define
from tesseract_robotics.tesseract_command_language import CompositeInstruction

from .errors import InvalidRapidProgramError
from .errors import InvalidRapidProgramPathError
from .errors import MissingRapidProgramParentError
from .errors import RapidProgramIdentityMismatchError
from .errors import RapidProgramTargetIsDirectoryError
from .errors import RapidProgramWriteError
from .rapid_identity import RapidModuleName
from .rapid_identity import RapidProcedureName
from .rapid_identity import RapidProgramIdentity
from .rapid_identity import RapidSource
from .rapid_identity import validate_rapid_name


def validate_rapid_program(program: object) -> CompositeInstruction:
    """Require an exact native Tesseract program boundary."""
    if not isinstance(program, CompositeInstruction):
        raise InvalidRapidProgramError(
            "RAPID emission requires CompositeInstruction, got {}.".format(
                type(program).__name__,
            )
        )
    return program


@define(frozen=True, slots=True)
class RapidProgram:
    """Exact native program paired with emitted, content-addressed source."""

    program: CompositeInstruction
    source: RapidSource
    module_name: RapidModuleName
    procedure_name: RapidProcedureName
    identity: RapidProgramIdentity

    def __attrs_post_init__(self) -> None:
        validate_rapid_program(self.program)
        if not isinstance(self.source, str) or not self.source:
            raise InvalidRapidProgramError("Emitted RAPID source must be a non-empty string.")
        module = validate_rapid_name(self.module_name, "module")
        procedure = validate_rapid_name(self.procedure_name, "procedure")
        expected_identity = RapidProgramIdentity.build(self.source, module, procedure)
        if self.identity != expected_identity:
            raise RapidProgramIdentityMismatchError("RAPID program identity does not match its exact source and names.")

    @classmethod
    def build(
        cls,
        program: object,
        source: str,
        module_name: object,
        procedure_name: object,
    ) -> RapidProgram:
        """Validate and retain an emitted native RAPID artifact.

        Args:
            program: Exact native program supplied to the emitter.
            source: Exact source returned by the native emitter.
            module_name: Exact RAPID module name.
            procedure_name: Exact RAPID procedure name.

        Returns:
            Immutable emitted program retaining the native program reference.

        Raises:
            InvalidRapidProgramError: Program or source violates the boundary.
            InvalidRapidProgramNameError: Module or procedure name is empty.
        """
        native_program = validate_rapid_program(program)
        if not isinstance(source, str) or not source:
            raise InvalidRapidProgramError("Emitted RAPID source must be a non-empty string.")
        module = validate_rapid_name(module_name, "module")
        procedure = validate_rapid_name(procedure_name, "procedure")
        return cls(
            native_program,
            RapidSource(source),
            RapidModuleName(module),
            RapidProcedureName(procedure),
            RapidProgramIdentity.build(source, module, procedure),
        )

    def write(self, path: Path) -> Path:
        """Write exact UTF-8 source to an explicit path.

        Args:
            path: Exact output path. Its parent must already exist.

        Returns:
            The same path after a successful write.

        Raises:
            InvalidRapidProgramPathError: Path is not a `pathlib.Path`.
            MissingRapidProgramParentError: The parent directory is absent.
            RapidProgramTargetIsDirectoryError: Path targets a directory.
            RapidProgramWriteError: The operating system rejects the write.
        """
        if not isinstance(path, Path):
            raise InvalidRapidProgramPathError("RAPID output path must be pathlib.Path.")
        if not path.parent.exists():
            raise MissingRapidProgramParentError("RAPID output parent does not exist: {}.".format(path.parent))
        if path.is_dir():
            raise RapidProgramTargetIsDirectoryError("RAPID output target is a directory: {}.".format(path))
        try:
            with path.open("w", encoding="utf-8", newline="") as stream:
                stream.write(self.source)
        except OSError as write_error:
            raise RapidProgramWriteError("Cannot write RAPID output {}: {}.".format(path, write_error)) from write_error
        return path
