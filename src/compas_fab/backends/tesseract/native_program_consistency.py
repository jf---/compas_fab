"""Complete native program identity normalized only for generated UUIDs."""

from __future__ import annotations

from typing import cast
from xml.etree import ElementTree

from tesseract_robotics.tesseract_command_language import CompositeInstruction
from tesseract_robotics.tesseract_serialization import composite_instruction_to_xml  # type: ignore[attr-defined]

from .errors import InvalidTesseractMotionProgramError

_GENERATED_ID_TAGS = frozenset(("uuid", "parent_uuid"))
_NIL_UUID = "00000000-0000-0000-0000-000000000000"


def native_program_semantic_identity(program: CompositeInstruction) -> bytes:
    """Serialize every native field while erasing generated UUID identity only."""
    try:
        root = ElementTree.fromstring(composite_instruction_to_xml(program))
    except (RuntimeError, TypeError, ValueError, ElementTree.ParseError) as serialization_error:
        raise InvalidTesseractMotionProgramError("Native program consistency serialization failed: {}.".format(serialization_error)) from serialization_error
    for element in root.iter():
        if element.tag in _GENERATED_ID_TAGS:
            data = element.find("data")
            if data is not None:
                data.text = _NIL_UUID
    return cast(bytes, ElementTree.tostring(root, encoding="utf-8"))
