"""Filesystem materialization for content-addressed Tesseract resources."""

from __future__ import annotations

import hashlib
import uuid
from pathlib import Path

from attrs import define
from tesseract_robotics.tesseract_common import Resource
from tesseract_robotics.tesseract_common import ResourceLocator
from tesseract_robotics.tesseract_common import SimpleLocatedResource

from .artifact import RobotArtifact
from .artifact import RobotResource
from .errors import ArtifactMaterializationError
from .errors import RobotResourceContainmentError
from .errors import UnknownRobotResourceError
from .resource_url import PackageResourceUrl


@define(frozen=True, slots=True)
class MaterializedResource:
    """One exact robot resource and its content-addressed file path."""

    url: str
    path: Path


@define(frozen=True, slots=True)
class MaterializedArtifact:
    """Stable filesystem view of an immutable robot artifact."""

    root: Path
    resources: tuple[MaterializedResource, ...]

    @classmethod
    def build(cls, artifact: RobotArtifact, cache_root: Path) -> MaterializedArtifact:
        """Materialize all resources beneath the artifact digest.

        Args:
            artifact: Exact content-addressed robot artifact.
            cache_root: Parent directory for artifact materializations.

        Returns:
            Stable filesystem view of every resource.

        Raises:
            ArtifactMaterializationError: Existing bytes disagree with the artifact.
            UnsafeRobotResourceUrlError: A URL contains unsafe path traversal.
            RobotResourceContainmentError: A resolved target escapes the artifact root.
        """
        root = _contained_artifact_root(cache_root, artifact.identity.digest)
        materialized = tuple(_materialize_resource(root, resource) for resource in artifact.resources)
        return cls(root, materialized)


class ArtifactResourceLocator(ResourceLocator):
    """Resolve exact artifact URLs to content-addressed files."""

    def __init__(self, materialized: MaterializedArtifact) -> None:
        super().__init__()
        self._paths = {resource.url: resource.path for resource in materialized.resources}

    def locateResource(self, url: str) -> Resource:
        """Locate one exact resource URL.

        Args:
            url: URL requested by Tesseract.

        Returns:
            File-backed native resource.

        Raises:
            UnknownRobotResourceError: The URL is absent from the artifact.
        """
        path = self._paths.get(url)
        if path is None:
            raise UnknownRobotResourceError("Robot artifact does not contain resource {!r}.".format(url))
        return SimpleLocatedResource(url, str(path))


def _materialize_resource(root: Path, resource: RobotResource) -> MaterializedResource:
    normalized_url = PackageResourceUrl.build(resource.url)
    target = _contained_target(
        root,
        normalized_url.materialization_path,
        resource.url,
    )
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        _assert_materialized_content(target, resource)
        return MaterializedResource(resource.url, target)

    temporary = target.with_name(".{}.{}.tmp".format(target.name, uuid.uuid4().hex))
    try:
        temporary.write_bytes(resource.content)
        temporary.replace(target)
    finally:
        temporary.unlink(missing_ok=True)
    _assert_materialized_content(target, resource)
    return MaterializedResource(resource.url, target)


def _assert_materialized_content(path: Path, resource: RobotResource) -> None:
    actual_digest = hashlib.sha256(path.read_bytes()).hexdigest()
    expected_digest = hashlib.sha256(resource.content).hexdigest()
    if actual_digest != expected_digest:
        raise ArtifactMaterializationError("Materialized resource {!r} has digest {}, expected {}.".format(resource.url, actual_digest, expected_digest))


def _contained_target(root: Path, relative_path: Path, url: str) -> Path:
    resolved_root = root.resolve()
    resolved_target = (resolved_root / relative_path).resolve()
    try:
        resolved_target.relative_to(resolved_root)
    except ValueError as error:
        raise RobotResourceContainmentError(
            "Robot resource {!r} materializes outside artifact root {}: {}.".format(
                url,
                resolved_root,
                resolved_target,
            )
        ) from error
    return resolved_target


def _contained_artifact_root(cache_root: Path, digest: str) -> Path:
    resolved_cache_root = cache_root.resolve()
    resolved_artifact_root = (resolved_cache_root / digest).resolve()
    if resolved_artifact_root.parent != resolved_cache_root:
        raise RobotResourceContainmentError(
            "Artifact {} resolves outside cache root {}: {}.".format(
                digest,
                resolved_cache_root,
                resolved_artifact_root,
            )
        )
    return resolved_artifact_root
