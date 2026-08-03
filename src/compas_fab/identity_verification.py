"""Neutral content-identity verification state."""

from enum import Enum


class IdentityVerification(Enum):
    VERIFIED = "verified"
    UNVERIFIABLE = "unverifiable"
