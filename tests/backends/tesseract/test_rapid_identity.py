import pytest

from compas_fab.backends.tesseract.errors import InvalidRapidProgramIdentityError
from compas_fab.backends.tesseract.rapid_identity import RAPID_IDENTITY_SCHEMA_VERSION
from compas_fab.backends.tesseract.rapid_identity import RapidProgramIdentity


SOURCE = "MODULE M\n  PROC main()\n  ENDPROC\nENDMODULE\n"


def test_identity_is_deterministic_and_versioned():
    left = RapidProgramIdentity.build(SOURCE, "M", "main")
    right = RapidProgramIdentity.build(SOURCE, "M", "main")

    assert left == right
    assert len(left.digest) == 64
    assert left.schema_version == RAPID_IDENTITY_SCHEMA_VERSION
    assert left.compas_fab_version
    assert left.tesseract_version == "0.35.0.6"


@pytest.mark.parametrize(
    ("source", "module_name", "procedure_name"),
    [
        (SOURCE + "! changed\n", "M", "main"),
        (SOURCE, "Changed", "main"),
        (SOURCE, "M", "changed"),
    ],
)
def test_identity_changes_with_every_covered_input(source, module_name, procedure_name):
    baseline = RapidProgramIdentity.build(SOURCE, "M", "main")

    changed = RapidProgramIdentity.build(source, module_name, procedure_name)

    assert changed.digest != baseline.digest


def test_identity_changes_with_tesseract_distribution_version(monkeypatch):
    baseline = RapidProgramIdentity.build(SOURCE, "M", "main")

    def changed_version(distribution):
        versions = {
            "compas-fab": baseline.compas_fab_version,
            "tesseract-robotics-nanobind": "0.35.0.7",
        }
        return versions[distribution]

    monkeypatch.setattr("compas_fab.backends.tesseract.rapid_identity.version", changed_version)

    changed = RapidProgramIdentity.build(SOURCE, "M", "main")

    assert changed.digest != baseline.digest
    assert changed.tesseract_version == "0.35.0.7"


@pytest.mark.parametrize(
    ("digest", "schema_version", "compas_fab_version", "tesseract_version"),
    [
        ("not-a-sha", RAPID_IDENTITY_SCHEMA_VERSION, "2.0.1", "0.35.0.6"),
        ("0" * 64, "unknown", "2.0.1", "0.35.0.6"),
        ("0" * 64, RAPID_IDENTITY_SCHEMA_VERSION, "", "0.35.0.6"),
        ("0" * 64, RAPID_IDENTITY_SCHEMA_VERSION, "2.0.1", ""),
    ],
)
def test_identity_raw_constructor_cannot_bypass_invariants(
    digest,
    schema_version,
    compas_fab_version,
    tesseract_version,
):
    with pytest.raises(InvalidRapidProgramIdentityError):
        RapidProgramIdentity(
            digest,
            schema_version,
            compas_fab_version,
            tesseract_version,
        )
