"""Exact filesystem loading for Tesseract robot artifacts."""

from __future__ import annotations

from pathlib import Path
from pathlib import PurePosixPath
from pathlib import PureWindowsPath
from typing import Sequence
from urllib.parse import urlsplit
from xml.etree import ElementTree

from attrs import define

from .artifact import CollisionMeshPolicy
from .artifact import RobotArtifact
from .errors import AmbiguousRobotPackageError
from .errors import DuplicateRobotResourceRootError
from .errors import InvalidRobotDescriptionEncodingError
from .errors import InvalidRobotResourceError
from .errors import InvalidRobotResourceRootError
from .errors import InvalidSrdfError
from .errors import InvalidUrdfError
from .errors import MissingRobotDescriptionFileError
from .errors import MissingRobotPackageError
from .errors import MissingRobotResourceRootError
from .errors import RobotDescriptionReadError
from .errors import RobotResourceReadError


@define(frozen=True, slots=True)
class ResourceRoot:
    """A directory whose immediate children are `package://` packages."""

    path: Path

    @classmethod
    def build(cls, path: Path) -> ResourceRoot:
        """Validate one filesystem package root.

        Args:
            path: Directory containing package directories by exact package name.

        Returns:
            Validated absolute resource root.

        Raises:
            MissingRobotResourceRootError: The path does not exist.
            InvalidRobotResourceRootError: The path is not a directory.
        """
        if not path.exists():
            raise MissingRobotResourceRootError("Robot resource root does not exist: {}.".format(path))
        if not path.is_dir():
            raise InvalidRobotResourceRootError("Robot resource root is not a directory: {}.".format(path))
        return cls(path.resolve())


@define(frozen=True, slots=True)
class RobotArtifactLoader:
    """Captured exact descriptions and deterministic package resolution."""

    urdf_path: Path
    srdf_path: Path
    resource_roots: tuple[ResourceRoot, ...]
    urdf: str
    srdf: str

    @classmethod
    def build(
        cls,
        urdf_path: Path,
        srdf_path: Path,
        resource_roots: Sequence[ResourceRoot],
    ) -> RobotArtifactLoader:
        """Capture exact source descriptions and package-root configuration.

        Args:
            urdf_path: Exact UTF-8 URDF source file.
            srdf_path: Exact UTF-8 SRDF source file.
            resource_roots: Roots used to resolve `package://<name>/...`.

        Returns:
            Immutable loader whose source contents cannot drift after construction.

        Raises:
            MissingRobotDescriptionFileError: A description path is not a file.
            InvalidRobotDescriptionEncodingError: A description is not UTF-8.
            DuplicateRobotResourceRootError: A root occurs more than once.
        """
        resolved_urdf = _description_path(urdf_path, "URDF")
        resolved_srdf = _description_path(srdf_path, "SRDF")
        roots = tuple(resource_roots)
        root_paths = tuple(root.path for root in roots)
        if len(root_paths) != len(set(root_paths)):
            raise DuplicateRobotResourceRootError("Robot package resource roots must be unique.")
        return cls(
            resolved_urdf,
            resolved_srdf,
            roots,
            _read_description(resolved_urdf, "URDF"),
            _read_description(resolved_srdf, "SRDF"),
        )

    def load(
        self,
        collision_mesh_policy: CollisionMeshPolicy = CollisionMeshPolicy.CONVEX_HULL,
    ) -> RobotArtifact:
        """Load referenced packages and compile a Tesseract-compatible artifact.

        Every file in each directly referenced package is retained. This preserves
        relative mesh sidecars such as COLLADA textures without hashing unrelated
        packages in the same resource root.

        Args:
            collision_mesh_policy: Native collision-mesh treatment. Defaults to
                convex hulls; pass `CollisionMeshPolicy.PRESERVE` for exact
                triangle meshes.

        Returns:
            Immutable, content-addressed Tesseract artifact.

        Raises:
            InvalidUrdfError: URDF XML cannot be parsed.
            InvalidSrdfError: SRDF XML cannot be parsed.
            MissingRobotPackageError: A referenced package cannot be resolved.
            AmbiguousRobotPackageError: Multiple roots contain a package.
            InvalidRobotResourceError: A referenced package URL is unsafe or absent.
        """
        urls = _package_urls(self.urdf, "URDF") | _package_urls(self.srdf, "SRDF")
        package_names = sorted(urlsplit(url).netloc for url in urls)
        packages = {package_name: _resolve_package(package_name, self.resource_roots) for package_name in package_names}
        resources = _read_packages(packages)
        _assert_referenced_resources(urls, resources)
        return RobotArtifact.from_compas_urdf(
            self.urdf,
            self.srdf,
            resources,
            collision_mesh_policy,
        )


def _description_path(path: Path, kind: str) -> Path:
    if not path.is_file():
        raise MissingRobotDescriptionFileError("{} description file does not exist: {}.".format(kind, path))
    return path.resolve()


def _read_description(path: Path, kind: str) -> str:
    try:
        return path.read_bytes().decode("utf-8")
    except OSError as error:
        raise RobotDescriptionReadError("{} description cannot be read from {}: {}.".format(kind, path, error)) from error
    except UnicodeDecodeError as error:
        raise InvalidRobotDescriptionEncodingError("{} description is not UTF-8: {}.".format(kind, path)) from error


def _package_urls(description: str, kind: str) -> set[str]:
    try:
        root = ElementTree.fromstring(description)
    except ElementTree.ParseError as error:
        error_type = InvalidUrdfError if kind == "URDF" else InvalidSrdfError
        raise error_type("{} XML cannot be parsed: {}".format(kind, error)) from error

    urls = {value for element in root.iter() for value in element.attrib.values() if value.startswith("package://")}
    for url in urls:
        parsed = urlsplit(url)
        parts = PurePosixPath(parsed.path).parts
        if parsed.scheme != "package" or not _is_safe_package_authority(parsed.netloc) or not parts or ".." in parts or parsed.query or parsed.fragment:
            raise InvalidRobotResourceError("{} contains invalid package resource URL {!r}.".format(kind, url))
    return urls


def _is_safe_package_authority(authority: str) -> bool:
    windows_path = PureWindowsPath(authority)
    return bool(authority) and authority not in (".", "..") and not windows_path.drive and not windows_path.root and windows_path.parts == (authority,)


def _resolve_package(
    package_name: str,
    resource_roots: tuple[ResourceRoot, ...],
) -> Path:
    candidates: set[Path] = set()
    for root in resource_roots:
        declared_path = root.path / package_name
        if not declared_path.is_dir():
            continue
        resolved_path = declared_path.resolve()
        if resolved_path.parent != root.path:
            raise InvalidRobotResourceError(
                "Referenced robot package {!r} must resolve to a direct child of resource root {}, got {}.".format(
                    package_name,
                    root.path,
                    resolved_path,
                )
            )
        candidates.add(resolved_path)
    if not candidates:
        raise MissingRobotPackageError(
            "Referenced robot package {!r} was not found beneath: {}.".format(
                package_name,
                ", ".join(str(root.path) for root in resource_roots) or "<no resource roots>",
            )
        )
    if len(candidates) > 1:
        raise AmbiguousRobotPackageError(
            "Referenced robot package {!r} exists in multiple roots: {}.".format(package_name, ", ".join(str(candidate) for candidate in sorted(candidates)))
        )
    return next(iter(candidates))


def _read_packages(packages: dict[str, Path]) -> dict[str, bytes]:
    resources: dict[str, bytes] = {}
    for package_name, package_path in packages.items():
        for resource_path in sorted(path for path in package_path.rglob("*") if path.is_file()):
            relative_path = resource_path.relative_to(package_path).as_posix()
            url = "package://{}/{}".format(package_name, relative_path)
            try:
                resources[url] = resource_path.read_bytes()
            except OSError as error:
                raise RobotResourceReadError(
                    "Robot resource {!r} cannot be read from {}: {}.".format(
                        url,
                        resource_path,
                        error,
                    )
                ) from error
    return resources


def _assert_referenced_resources(urls: set[str], resources: dict[str, bytes]) -> None:
    missing = sorted(url for url in urls if url not in resources)
    if missing:
        raise InvalidRobotResourceError("Robot descriptions reference missing package resources: {}.".format(", ".join(missing)))
