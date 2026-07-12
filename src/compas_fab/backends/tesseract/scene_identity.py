"""Content identity for exact native scene inputs."""

from __future__ import annotations

from hashlib import sha256
from typing import Optional
from typing import Tuple

from attrs import define
from attrs import field

from compas_fab.identity_verification import IdentityVerification

from .errors import InvalidDirectSceneGenerationError
from .errors import InvalidNativeSceneContentIdentityError
from .identity import BuildIdentity

SCENE_IDENTITY_SCHEMA = "compas_fab.tesseract.native_scene/v1"
_SCENE_IDENTITY_FACTORY_TOKEN = object()


def _part(value: bytes) -> bytes:
    return len(value).to_bytes(8, "big") + value


def _valid_digest(value: object) -> bool:
    return type(value) is str and len(value) == sha256().digest_size * 2 and value == value.lower() and all(character in "0123456789abcdef" for character in value)


@define(frozen=True, slots=True)
class DirectSceneGeneration:
    """Monotonic evidence for opaque direct native scene commands."""

    value: int

    @classmethod
    def build(cls, value: int) -> "DirectSceneGeneration":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not int or self.value < 0:
            raise InvalidDirectSceneGenerationError("Direct scene generation must be an exact non-negative integer.")

    def next(self) -> "DirectSceneGeneration":
        return DirectSceneGeneration.build(self.value + 1)


@define(frozen=True, slots=True)
class NativeSceneContentIdentity:
    """Artifact plus canonical projection and opaque-command evidence."""

    digest: str
    artifact_digest: str
    projection: Optional[Tuple[str, Optional[str]]]
    direct_generation: DirectSceneGeneration
    verification: IdentityVerification
    _canonical: bytes = field(eq=False, repr=False)
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(
        cls,
        artifact: BuildIdentity,
        projection: Optional[Tuple[str, Optional[str]]],
        direct_generation: DirectSceneGeneration,
    ) -> "NativeSceneContentIdentity":
        if type(artifact) is not BuildIdentity:
            raise InvalidNativeSceneContentIdentityError("Native scene identity requires an exact BuildIdentity.")
        if not _valid_projection(projection) or type(direct_generation) is not DirectSceneGeneration:
            raise InvalidNativeSceneContentIdentityError("Native scene projection must retain exact cell/state canonical text.")
        canonical = _canonical_scene(artifact.digest, projection, direct_generation)
        verification = IdentityVerification.VERIFIED if direct_generation.value == 0 else IdentityVerification.UNVERIFIABLE
        return cls(
            sha256(canonical).hexdigest(),
            artifact.digest,
            projection,
            direct_generation,
            verification,
            canonical,
            _SCENE_IDENTITY_FACTORY_TOKEN,
        )

    def __attrs_post_init__(self) -> None:
        valid = (
            _valid_digest(self.digest)
            and _valid_digest(self.artifact_digest)
            and _valid_projection(self.projection)
            and type(self.direct_generation) is DirectSceneGeneration
            and type(self.verification) is IdentityVerification
            and self.verification
            is (IdentityVerification.VERIFIED if self.direct_generation.value == 0 else IdentityVerification.UNVERIFIABLE)
            and type(self._canonical) is bytes
            and self._canonical == _canonical_scene(self.artifact_digest, self.projection, self.direct_generation)
            and self.digest == sha256(self._canonical).hexdigest()
            and self._factory_token is _SCENE_IDENTITY_FACTORY_TOKEN
        )
        if not valid:
            raise InvalidNativeSceneContentIdentityError("Native scene content identity fields are inconsistent.")


def _valid_projection(value: object) -> bool:
    return value is None or (
        type(value) is tuple
        and len(value) == 2
        and type(value[0]) is str
        and (value[1] is None or type(value[1]) is str)
    )


def _canonical_scene(
    artifact_digest: str,
    projection: Optional[Tuple[str, Optional[str]]],
    direct_generation: DirectSceneGeneration,
) -> bytes:
    if projection is None:
        projection_payload = b"absent"
    else:
        state = b"absent" if projection[1] is None else b"present" + _part(projection[1].encode("utf-8"))
        projection_payload = b"present" + _part(projection[0].encode("utf-8")) + _part(state)
    return b"".join(
        (
            _part(SCENE_IDENTITY_SCHEMA.encode("utf-8")),
            _part(artifact_digest.encode("ascii")),
            _part(projection_payload),
            _part(str(direct_generation.value).encode("ascii")),
        )
    )
