from pathlib import Path

import pytest
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_command_language import WaitInstruction

from compas_fab.backends.tesseract.errors import InvalidRapidProgramError
from compas_fab.backends.tesseract.errors import InvalidRapidProgramNameError
from compas_fab.backends.tesseract.errors import InvalidRapidProgramPathError
from compas_fab.backends.tesseract.errors import MissingRapidProgramParentError
from compas_fab.backends.tesseract.errors import RapidProgramIdentityMismatchError
from compas_fab.backends.tesseract.errors import RapidProgramTargetIsDirectoryError
from compas_fab.backends.tesseract.errors import RapidProgramWriteError
from compas_fab.backends.tesseract.rapid_identity import RapidProgramIdentity
from compas_fab.backends.tesseract.rapid_program import RapidProgram


SOURCE = "MODULE M\n  PROC main()\n    ! λ\n  ENDPROC\nENDMODULE\n"


def _program():
    program = CompositeInstruction("rapid")
    program.push_back(WaitInstruction(1.0))
    return program


def test_program_retains_exact_composite_reference():
    native_program = _program()

    artifact = RapidProgram.build(native_program, SOURCE, "M", "main")

    assert artifact.program is native_program
    assert artifact.source == SOURCE
    assert artifact.module_name == "M"
    assert artifact.procedure_name == "main"


@pytest.mark.parametrize("program", [object(), None, "program"])
def test_program_rejects_non_composite_values(program):
    with pytest.raises(InvalidRapidProgramError):
        RapidProgram.build(program, SOURCE, "M", "main")


@pytest.mark.parametrize(
    ("source", "module_name", "procedure_name", "error_type"),
    [
        ("", "M", "main", InvalidRapidProgramError),
        (SOURCE, "", "main", InvalidRapidProgramNameError),
        (SOURCE, "M", "   ", InvalidRapidProgramNameError),
        (SOURCE, object(), "main", InvalidRapidProgramNameError),
    ],
)
def test_program_rejects_invalid_artifact_values(source, module_name, procedure_name, error_type):
    with pytest.raises(error_type):
        RapidProgram.build(_program(), source, module_name, procedure_name)


def test_program_raw_constructor_cannot_bypass_native_program_invariant():
    identity = RapidProgramIdentity.build(SOURCE, "M", "main")

    with pytest.raises(InvalidRapidProgramError):
        RapidProgram(object(), SOURCE, "M", "main", identity)


def test_program_raw_constructor_rejects_mismatched_identity():
    mismatched = RapidProgramIdentity.build(SOURCE + "! changed\n", "M", "main")

    with pytest.raises(RapidProgramIdentityMismatchError):
        RapidProgram(_program(), SOURCE, "M", "main", mismatched)


def test_program_write_round_trips_exact_utf8(tmp_path):
    artifact = RapidProgram.build(_program(), SOURCE, "M", "main")
    target = tmp_path / "program.mod"

    result = artifact.write(target)

    assert result == target
    assert target.read_bytes() == SOURCE.encode("utf-8")


def test_program_write_rejects_string_path(tmp_path):
    artifact = RapidProgram.build(_program(), SOURCE, "M", "main")

    with pytest.raises(InvalidRapidProgramPathError):
        artifact.write(str(tmp_path / "program.mod"))


def test_program_write_rejects_missing_parent(tmp_path):
    artifact = RapidProgram.build(_program(), SOURCE, "M", "main")

    with pytest.raises(MissingRapidProgramParentError):
        artifact.write(tmp_path / "missing" / "program.mod")


def test_program_write_rejects_directory_target(tmp_path):
    artifact = RapidProgram.build(_program(), SOURCE, "M", "main")

    with pytest.raises(RapidProgramTargetIsDirectoryError):
        artifact.write(tmp_path)


def test_program_write_translates_os_error(tmp_path, mocker):
    artifact = RapidProgram.build(_program(), SOURCE, "M", "main")
    mocker.patch.object(Path, "open", side_effect=OSError("denied"))

    with pytest.raises(RapidProgramWriteError, match="denied") as caught:
        artifact.write(tmp_path / "program.mod")

    assert isinstance(caught.value.__cause__, OSError)
