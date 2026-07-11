"""Canonical package-resource URL validation for Tesseract artifacts."""

from __future__ import annotations

from pathlib import Path
from pathlib import PurePosixPath
from pathlib import PureWindowsPath
from urllib.parse import urlsplit

from attrs import define

from .errors import UnsafeRobotResourceUrlError


@define(frozen=True, slots=True)
class PackageResourceUrl:
    """One canonical `package://` URL safe on POSIX and Windows."""

    value: str
    package: str
    path_parts: tuple[str, ...]

    def __attrs_post_init__(self) -> None:
        expected = _normalized_fields(self.value)
        if (self.value, self.package, self.path_parts) != expected:
            raise UnsafeRobotResourceUrlError(
                "Robot package resource URL fields differ from normalized URL {!r}.".format(
                    self.value,
                )
            )

    @classmethod
    def build(cls, value: object) -> PackageResourceUrl:
        """Validate one exact package-resource URL.

        Args:
            value: Candidate exact URL.

        Returns:
            Typed canonical URL and platform-neutral relative path fields.

        Raises:
            UnsafeRobotResourceUrlError: The URL can escape or alias a path.
        """
        normalized_value, package, path_parts = _normalized_fields(value)
        return cls(normalized_value, package, path_parts)

    @property
    def materialization_path(self) -> Path:
        """Return the safe relative path beneath an artifact root."""
        return Path("package", self.package, *self.path_parts)


def _normalized_fields(value: object) -> tuple[str, str, tuple[str, ...]]:
    if not isinstance(value, str) or not value or "\x00" in value:
        raise UnsafeRobotResourceUrlError("Robot package resource URL must be non-empty text without NUL bytes.")
    try:
        parsed = urlsplit(value)
    except ValueError as error:
        raise UnsafeRobotResourceUrlError("Robot package resource URL is malformed: {!r}.".format(value)) from error
    if parsed.scheme != "package" or parsed.query or parsed.fragment:
        raise UnsafeRobotResourceUrlError("Robot package resource URL must use only the package scheme and path: {!r}.".format(value))
    if not _is_canonical_package_name(parsed.netloc):
        raise UnsafeRobotResourceUrlError("Robot package resource URL has unsafe package authority: {!r}.".format(value))

    raw_path = parsed.path
    if not raw_path.startswith("/") or raw_path.startswith("//"):
        raise UnsafeRobotResourceUrlError("Robot package resource URL has an absolute or double-root path: {!r}.".format(value))
    relative_path = raw_path[1:]
    if not relative_path or "\\" in relative_path:
        raise UnsafeRobotResourceUrlError("Robot package resource URL has an empty or Windows-rooted path: {!r}.".format(value))

    posix_path = PurePosixPath(relative_path)
    windows_path = PureWindowsPath(relative_path)
    if (
        posix_path.is_absolute()
        or posix_path.root
        or windows_path.is_absolute()
        or windows_path.drive
        or windows_path.root
        or any(":" in part for part in windows_path.parts)
        or any(part in (".", "..") for part in posix_path.parts)
        or any(part in (".", "..") for part in windows_path.parts)
        or relative_path != "/".join(posix_path.parts)
    ):
        raise UnsafeRobotResourceUrlError("Robot package resource URL has unsafe or non-canonical path semantics: {!r}.".format(value))
    return value, parsed.netloc, posix_path.parts


def _is_canonical_package_name(value: str) -> bool:
    windows_path = PureWindowsPath(value)
    return (
        bool(value) and value not in (".", "..") and "\\" not in value and ":" not in value and not windows_path.drive and not windows_path.root and windows_path.parts == (value,)
    )
