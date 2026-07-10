"""Content-addressed identity for a Tesseract robot build."""

from __future__ import annotations

import hashlib
import json
from importlib.metadata import version
from typing import Mapping

from attrs import define

from .errors import EmptyRobotDescriptionError
from .errors import InvalidRobotResourceError

IDENTITY_SCHEMA_VERSION = "1"


@define(frozen=True, slots=True)
class BuildIdentity:
    """Reproducible identity of robot inputs and consuming components."""

    digest: str
    schema_version: str
    compas_fab_version: str
    tesseract_version: str

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
