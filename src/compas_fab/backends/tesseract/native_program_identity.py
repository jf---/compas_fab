"""Versioned content identity for exact native motion programs."""

from __future__ import annotations

import hashlib
from importlib.metadata import version
from typing import NewType

from attrs import define
from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_serialization import composite_instruction_to_binary  # type: ignore[attr-defined]

from .errors import InvalidTesseractMotionProgramError

NATIVE_PROGRAM_IDENTITY_SCHEMA_VERSION = "1"
SHA256_HEX_LENGTH = hashlib.sha256().digest_size * 2
SHA256_HEX_DIGITS = frozenset("0123456789abcdef")

NativeProgramDigest = NewType("NativeProgramDigest", str)


@define(frozen=True, slots=True)
class NativeProgramIdentity:
    """Exact native program bytes paired with consuming versions."""

    digest: NativeProgramDigest
    schema_version: str
    compas_fab_version: str
    tesseract_version: str

    def __attrs_post_init__(self) -> None:
        if not isinstance(self.digest, str) or len(self.digest) != SHA256_HEX_LENGTH or not set(self.digest).issubset(SHA256_HEX_DIGITS):
            raise InvalidTesseractMotionProgramError("Native program digest must be lowercase SHA-256 hex.")
        if self.schema_version != NATIVE_PROGRAM_IDENTITY_SCHEMA_VERSION:
            raise InvalidTesseractMotionProgramError("Unknown native program identity schema {!r}.".format(self.schema_version))
        if not isinstance(self.compas_fab_version, str) or not self.compas_fab_version:
            raise InvalidTesseractMotionProgramError("Native program identity requires a COMPAS FAB version.")
        if not isinstance(self.tesseract_version, str) or not self.tesseract_version:
            raise InvalidTesseractMotionProgramError("Native program identity requires a Tesseract nanobind version.")


def native_program_digest(program: object) -> NativeProgramIdentity:
    """Hash exact native binary serialization and consuming versions."""
    if not isinstance(program, CompositeInstruction):
        raise InvalidTesseractMotionProgramError("Native program identity requires CompositeInstruction, got {}.".format(type(program).__name__))
    payload = bytes(composite_instruction_to_binary(program))
    compas_fab_version = version("compas-fab")
    tesseract_version = version("tesseract-robotics-nanobind")
    digest = hashlib.sha256(
        b"".join(
            (
                _field_bytes(NATIVE_PROGRAM_IDENTITY_SCHEMA_VERSION.encode("utf-8")),
                _field_bytes(compas_fab_version.encode("utf-8")),
                _field_bytes(tesseract_version.encode("utf-8")),
                _field_bytes(payload),
            )
        )
    ).hexdigest()
    return NativeProgramIdentity(
        NativeProgramDigest(digest),
        NATIVE_PROGRAM_IDENTITY_SCHEMA_VERSION,
        compas_fab_version,
        tesseract_version,
    )


def _field_bytes(payload: bytes) -> bytes:
    return len(payload).to_bytes(8, byteorder="big") + payload
