"""Content identity for exact native scene inputs."""

from __future__ import annotations

from hashlib import sha256
from typing import Optional
from typing import Tuple

from attrs import define
from attrs import field

from compas_fab.ghpython.tree_identity import IdentityVerification

from .identity import BuildIdentity

SCENE_IDENTITY_SCHEMA = "compas_fab.tesseract.native_scene/v1"
_SCENE_IDENTITY_FACTORY_TOKEN = object()


class NativeSceneIdentityError(ValueError):
    """Base failure for native scene content identity."""


class InvalidDirectSceneGenerationError(NativeSceneIdentityError):
    """Raised when direct-scene mutation generation is invalid."""


class InvalidNativeSceneContentIdentityError(NativeSceneIdentityError):
    """Raised when retained scene identity fields disagree."""


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
    projection: Tuple[str, Optional[str]]
    direct_generation: DirectSceneGeneration
    verification: IdentityVerification
    _canonical: bytes = field(eq=False, repr=False)
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(
        cls,
        artifact: BuildIdentity,
        projection: Tuple[str, Optional[str]],
        direct_generation: DirectSceneGeneration,
    ) -> "NativeSceneContentIdentity":
        if type(artifact) is not BuildIdentity:
            raise InvalidNativeSceneContentIdentityError("Native scene identity requires an exact BuildIdentity.")
        if (
            type(projection) is not tuple
            or len(projection) != 2
            or type(projection[0]) is not str
            or (projection[1] is not None and type(projection[1]) is not str)
            or type(direct_generation) is not DirectSceneGeneration
        ):
            raise InvalidNativeSceneContentIdentityError("Native scene projection must retain exact cell/state canonical text.")
        state = b"absent" if projection[1] is None else b"present" + _part(projection[1].encode("utf-8"))
        canonical = b"".join(
            (
                _part(SCENE_IDENTITY_SCHEMA.encode("utf-8")),
                _part(artifact.digest.encode("ascii")),
                _part(projection[0].encode("utf-8")),
                _part(state),
                _part(str(direct_generation.value).encode("ascii")),
            )
        )
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
            and type(self.projection) is tuple
            and len(self.projection) == 2
            and type(self.projection[0]) is str
            and (self.projection[1] is None or type(self.projection[1]) is str)
            and type(self.direct_generation) is DirectSceneGeneration
            and type(self.verification) is IdentityVerification
            and self.verification
            is (IdentityVerification.VERIFIED if self.direct_generation.value == 0 else IdentityVerification.UNVERIFIABLE)
            and type(self._canonical) is bytes
            and self.digest == sha256(self._canonical).hexdigest()
            and self._factory_token is _SCENE_IDENTITY_FACTORY_TOKEN
        )
        if not valid:
            raise InvalidNativeSceneContentIdentityError("Native scene content identity fields are inconsistent.")
