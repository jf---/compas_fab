"""Content-addressed identity for a Tesseract robot build."""

from __future__ import annotations

import hashlib
import json
from importlib.metadata import version
from typing import Mapping

from attrs import define

from .errors import EmptyRobotDescriptionError
from .errors import InvalidBuildIdentityError
from .errors import InvalidRobotResourceError

IDENTITY_SCHEMA_VERSION = "1"
SHA256_HEX_LENGTH = hashlib.sha256().digest_size * 2


@define(frozen=True, slots=True)
class BuildIdentity:
    """Reproducible identity of robot inputs and consuming components."""

    digest: str
    schema_version: str
    compas_fab_version: str
    tesseract_version: str

    def __attrs_post_init__(self) -> None:
        _validate_identity(
            self.digest,
            self.schema_version,
            self.compas_fab_version,
            self.tesseract_version,
        )

    @classmethod
    def build(cls, urdf: str, srdf: str, resources: Mapping[str, bytes]) -> BuildIdentity:
        """Build an identity covering descriptions, resources, and versions.

        Args:
            urdf: Complete URDF XML text.
            srdf: Complete SRDF XML text.
            resources: Resource payloads keyed by their exact URDF URL.

        Returns:
            A deterministic content identity.

        Raises:
            EmptyRobotDescriptionError: The URDF or SRDF is empty.
            InvalidRobotResourceError: A URL is empty or a payload is not bytes.
        """
        if not urdf.strip():
            raise EmptyRobotDescriptionError("URDF description is empty.")
        if not srdf.strip():
            raise EmptyRobotDescriptionError("SRDF description is empty.")

        resource_digests = []
        for resource_url in sorted(resources):
            payload = resources[resource_url]
            if not resource_url:
                raise InvalidRobotResourceError("Robot resource URL is empty.")
            if not isinstance(payload, bytes):
                raise InvalidRobotResourceError("Robot resource {!r} must contain bytes, got {}.".format(resource_url, type(payload).__name__))
            resource_digests.append(
                {
                    "url": resource_url,
                    "sha256": hashlib.sha256(payload).hexdigest(),
                }
            )

        compas_fab_version = version("compas-fab")
        tesseract_version = version("tesseract-robotics-nanobind")
        manifest = {
            "schema_version": IDENTITY_SCHEMA_VERSION,
            "components": {
                "compas_fab": compas_fab_version,
                "tesseract_robotics_nanobind": tesseract_version,
            },
            "inputs": {
                "urdf_sha256": hashlib.sha256(urdf.encode("utf-8")).hexdigest(),
                "srdf_sha256": hashlib.sha256(srdf.encode("utf-8")).hexdigest(),
                "resources": resource_digests,
            },
        }
        manifest_bytes = json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode("utf-8")

        return cls(
            digest=hashlib.sha256(manifest_bytes).hexdigest(),
            schema_version=IDENTITY_SCHEMA_VERSION,
            compas_fab_version=compas_fab_version,
            tesseract_version=tesseract_version,
        )


def _validate_identity(
    digest: object,
    schema_version: object,
    compas_fab_version: object,
    tesseract_version: object,
) -> None:
    if not isinstance(digest, str) or len(digest) != SHA256_HEX_LENGTH or any(character not in "0123456789abcdef" for character in digest):
        raise InvalidBuildIdentityError("Robot build digest must be a lowercase SHA-256 hexadecimal string.")
    if schema_version != IDENTITY_SCHEMA_VERSION:
        raise InvalidBuildIdentityError("Robot build identity schema must be {!r}, got {!r}.".format(IDENTITY_SCHEMA_VERSION, schema_version))
    if not isinstance(compas_fab_version, str) or not compas_fab_version:
        raise InvalidBuildIdentityError("Robot build identity COMPAS FAB version must be non-empty text.")
    if not isinstance(tesseract_version, str) or not tesseract_version:
        raise InvalidBuildIdentityError("Robot build identity Tesseract version must be non-empty text.")
