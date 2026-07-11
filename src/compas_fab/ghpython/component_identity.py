from __future__ import annotations

from hashlib import sha256
from typing import Sequence
from typing import Tuple

from attrs import define


class InvalidComponentIdentityError(ValueError):
    pass


def _part(value: bytes) -> bytes:
    return len(value).to_bytes(8, "big") + value


@define(frozen=True, slots=True)
class ComponentInstanceId:
    value: str

    @classmethod
    def build(cls, value: str) -> "ComponentInstanceId":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if type(self.value) is not str or not self.value:
            raise InvalidComponentIdentityError("Component instance ID must be non-empty str.")


@define(frozen=True, slots=True)
class CanonicalField:
    name: str
    payload: bytes

    @classmethod
    def text(cls, name: str, value: str) -> "CanonicalField":
        if type(value) is not str:
            raise InvalidComponentIdentityError("Text field value must be str.")
        return cls(name, value.encode("utf-8"))

    @classmethod
    def bytes(cls, name: str, value: bytes) -> "CanonicalField":
        return cls(name, bytes(value))

    def __attrs_post_init__(self) -> None:
        if type(self.name) is not str or not self.name or type(self.payload) is not bytes:
            raise InvalidComponentIdentityError("Canonical field requires a name and exact bytes.")


def _digest(component: ComponentInstanceId, schema: str, fields: Tuple[CanonicalField, ...]) -> str:
    payload = _part(component.value.encode("utf-8")) + _part(schema.encode("utf-8"))
    for field in fields:
        payload += _part(field.name.encode("utf-8")) + _part(field.payload)
    return sha256(payload).hexdigest()


@define(frozen=True, slots=True)
class ComponentInputIdentity:
    component: ComponentInstanceId
    schema: str
    fields: Tuple[CanonicalField, ...]
    digest: str

    @classmethod
    def build(cls, component: ComponentInstanceId, schema: str, fields: Sequence[CanonicalField]) -> "ComponentInputIdentity":
        retained = tuple(fields)
        return cls(component, schema, retained, _digest(component, schema, retained))

    def __attrs_post_init__(self) -> None:
        invalid = not isinstance(self.component, ComponentInstanceId) or type(self.schema) is not str or not self.schema
        invalid = invalid or any(not isinstance(field, CanonicalField) for field in self.fields)
        invalid = invalid or self.digest != _digest(self.component, self.schema, self.fields)
        if invalid:
            raise InvalidComponentIdentityError("Component input identity is inconsistent.")
