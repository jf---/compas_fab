"""Content-addressed identity for emitted RAPID source."""

from __future__ import annotations

import hashlib
from importlib.metadata import version
from typing import NewType

from attrs import define

from .errors import InvalidRapidProgramError
from .errors import InvalidRapidProgramIdentityError
from .errors import InvalidRapidProgramNameError

RAPID_IDENTITY_SCHEMA_VERSION = "1"
SHA256_HEX_LENGTH = hashlib.sha256().digest_size * 2
SHA256_HEX_DIGITS = frozenset("0123456789abcdef")

RapidSource = NewType("RapidSource", str)
RapidModuleName = NewType("RapidModuleName", str)
RapidProcedureName = NewType("RapidProcedureName", str)
Sha256Digest = NewType("Sha256Digest", str)


def _field_bytes(value: str) -> bytes:
    payload = value.encode("utf-8")
    return len(payload).to_bytes(8, byteorder="big") + payload


def validate_rapid_name(value: object, kind: str) -> str:
    """Require a non-empty RAPID module or procedure name."""
    if not isinstance(value, str) or not value.strip():
        raise InvalidRapidProgramNameError("RAPID {} name must be a non-empty string.".format(kind))
    return value


@define(frozen=True, slots=True)
class RapidProgramIdentity:
    """Reproducible identity of RAPID source and consuming components."""

    digest: Sha256Digest
    schema_version: str
    compas_fab_version: str
    tesseract_version: str

    def __attrs_post_init__(self) -> None:
        if not isinstance(self.digest, str) or len(self.digest) != SHA256_HEX_LENGTH or not set(self.digest).issubset(SHA256_HEX_DIGITS):
            raise InvalidRapidProgramIdentityError("RAPID program digest must be a lowercase SHA-256 hex string.")
        if self.schema_version != RAPID_IDENTITY_SCHEMA_VERSION:
            raise InvalidRapidProgramIdentityError(
                "Unknown RAPID identity schema {!r}; expected {!r}.".format(
                    self.schema_version,
                    RAPID_IDENTITY_SCHEMA_VERSION,
                )
            )
        if not isinstance(self.compas_fab_version, str) or not self.compas_fab_version:
            raise InvalidRapidProgramIdentityError("RAPID identity requires a COMPAS FAB version.")
        if not isinstance(self.tesseract_version, str) or not self.tesseract_version:
            raise InvalidRapidProgramIdentityError("RAPID identity requires a Tesseract nanobind version.")

    @classmethod
    def build(
        cls,
        source: str,
        module_name: str,
        procedure_name: str,
    ) -> RapidProgramIdentity:
        """Build an identity covering exact source, names, and versions.

        Args:
            source: Exact emitted RAPID source.
            module_name: Exact RAPID module name.
            procedure_name: Exact RAPID procedure name.

        Returns:
            Deterministic content identity.

        Raises:
            InvalidRapidProgramError: Source is empty or not text.
            InvalidRapidProgramNameError: A module or procedure name is empty.
        """
        if not isinstance(source, str) or not source:
            raise InvalidRapidProgramError("Emitted RAPID source must be a non-empty string.")
        module = validate_rapid_name(module_name, "module")
        procedure = validate_rapid_name(procedure_name, "procedure")
        compas_fab_version = version("compas-fab")
        tesseract_version = version("tesseract-robotics-nanobind")
        fields = (
            RAPID_IDENTITY_SCHEMA_VERSION,
            compas_fab_version,
            tesseract_version,
            source,
            module,
            procedure,
        )
        digest = hashlib.sha256(b"".join(_field_bytes(field) for field in fields)).hexdigest()
        return cls(
            Sha256Digest(digest),
            RAPID_IDENTITY_SCHEMA_VERSION,
            compas_fab_version,
            tesseract_version,
        )
