from attrs import evolve
import pytest

from compas_fab.backends.tesseract.identity import BuildIdentity
from compas_fab.backends.tesseract.scene_identity import DirectSceneGeneration
from compas_fab.backends.tesseract.scene_identity import InvalidNativeSceneContentIdentityError
from compas_fab.backends.tesseract.scene_identity import NativeSceneContentIdentity
from compas_fab.ghpython.tree_identity import IdentityVerification


def _artifact_identity() -> BuildIdentity:
    return BuildIdentity.build("<robot name='r'/>", "<robot name='r'/>", {})


def test_projection_scene_identity_is_verified_and_deterministic() -> None:
    generation = DirectSceneGeneration.build(0)
    first = NativeSceneContentIdentity.build(_artifact_identity(), ("cell", None), generation)
    repeated = NativeSceneContentIdentity.build(_artifact_identity(), ("cell", None), generation)

    assert first == repeated
    assert first.verification is IdentityVerification.VERIFIED


def test_direct_mutation_generation_is_unverifiable_and_content_significant() -> None:
    artifact = _artifact_identity()
    projection = ("cell", "state")
    first = NativeSceneContentIdentity.build(artifact, projection, DirectSceneGeneration.build(1))
    second = NativeSceneContentIdentity.build(artifact, projection, DirectSceneGeneration.build(2))

    assert first.verification is IdentityVerification.UNVERIFIABLE
    assert first.digest != second.digest


def test_raw_scene_identity_cannot_forge_verification() -> None:
    identity = NativeSceneContentIdentity.build(_artifact_identity(), ("cell", None), DirectSceneGeneration.build(1))
    with pytest.raises(InvalidNativeSceneContentIdentityError):
        evolve(identity, verification=IdentityVerification.VERIFIED)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("artifact_digest", "0" * 64),
        ("projection", ("other-cell", None)),
        ("direct_generation", DirectSceneGeneration.build(2)),
    ],
)
def test_raw_scene_identity_cannot_forge_exposed_content(field: str, value: object) -> None:
    identity = NativeSceneContentIdentity.build(_artifact_identity(), ("cell", None), DirectSceneGeneration.build(1))
    with pytest.raises(InvalidNativeSceneContentIdentityError):
        evolve(identity, **{field: value})


def test_no_projection_is_distinct_from_installed_empty_projection() -> None:
    artifact = _artifact_identity()
    generation = DirectSceneGeneration.build(0)
    absent = NativeSceneContentIdentity.build(artifact, None, generation)
    present = NativeSceneContentIdentity.build(artifact, ("", None), generation)

    assert absent.digest != present.digest
