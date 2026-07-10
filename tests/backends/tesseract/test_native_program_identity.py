import pytest
from tesseract_robotics.planning import JointTarget

from compas_fab.backends.tesseract.errors import InvalidTesseractMotionProgramError
from compas_fab.backends.tesseract.native_program_builder import build_motion_program
from compas_fab.backends.tesseract.native_program_identity import NATIVE_PROGRAM_IDENTITY_SCHEMA_VERSION
from compas_fab.backends.tesseract.native_program_identity import NativeProgramIdentity
from compas_fab.backends.tesseract.native_program_identity import native_program_digest


@pytest.fixture
def native_composite(tesseract_robot):
    return build_motion_program(
        tesseract_robot,
        [JointTarget([0.0], profile="DEFAULT")],
        "manipulator",
        None,
        "base",
        "DEFAULT",
    ).composite_instruction


def test_native_program_digest_is_stable_and_versioned(native_composite):
    left = native_program_digest(native_composite)
    right = native_program_digest(native_composite)

    assert left == right
    assert len(left.digest) == 64
    assert left.schema_version == NATIVE_PROGRAM_IDENTITY_SCHEMA_VERSION
    assert left.compas_fab_version
    assert left.tesseract_version == "0.35.0.6"


def test_native_program_digest_detects_in_place_mutation(native_composite):
    before = native_program_digest(native_composite)

    native_composite.setDescription("changed")

    assert native_program_digest(native_composite) != before


def test_native_program_digest_covers_dependency_versions(
    native_composite,
    monkeypatch,
):
    baseline = native_program_digest(native_composite)

    def changed_version(distribution):
        versions = {
            "compas-fab": baseline.compas_fab_version,
            "tesseract-robotics-nanobind": "0.35.0.7",
        }
        return versions[distribution]

    monkeypatch.setattr(
        "compas_fab.backends.tesseract.native_program_identity.version",
        changed_version,
    )

    assert native_program_digest(native_composite) != baseline


def test_native_program_digest_rejects_wrong_native_type():
    with pytest.raises(InvalidTesseractMotionProgramError):
        native_program_digest(object())


@pytest.mark.parametrize(
    ("digest", "schema", "compas_version", "tesseract_version"),
    [
        ("bad", NATIVE_PROGRAM_IDENTITY_SCHEMA_VERSION, "2.0.1", "0.35.0.6"),
        ("0" * 64, "unknown", "2.0.1", "0.35.0.6"),
        ("0" * 64, NATIVE_PROGRAM_IDENTITY_SCHEMA_VERSION, "", "0.35.0.6"),
        ("0" * 64, NATIVE_PROGRAM_IDENTITY_SCHEMA_VERSION, "2.0.1", ""),
    ],
)
def test_identity_raw_constructor_cannot_bypass_invariants(
    digest,
    schema,
    compas_version,
    tesseract_version,
):
    with pytest.raises(InvalidTesseractMotionProgramError):
        NativeProgramIdentity(
            digest,
            schema,
            compas_version,
            tesseract_version,
        )
