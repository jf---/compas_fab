"""Deterministic, root-free content identities for Grasshopper trees."""

from __future__ import annotations

from enum import Enum
from hashlib import sha256
from math import isfinite
from struct import pack
from typing import Callable
from typing import Dict
from typing import Generic
from typing import Optional
from typing import Tuple
from typing import Type
from typing import TypeVar
from typing import Union
from typing import cast

from attrs import define
from attrs import field
from compas.geometry import Frame  # type: ignore[import-untyped]

from compas_fab.ghpython.component_identity import CanonicalField
from compas_fab.ghpython.port_semantics import PortSemantics
from compas_fab.ghpython.tree_coordinates import BranchCoordinate
from compas_fab.ghpython.tree_coordinates import GhPath
from compas_fab.ghpython.tree_coordinates import TreeCoordinate
from compas_fab.ghpython.tree_coordinates import TreeRootId
from compas_fab.ghpython.tree_diagnostics import SourceCoordinateMap
from compas_fab.ghpython.tree_values import Tree
from compas_fab.ghpython.tree_values import TreeBranch
from compas_fab.ghpython.tree_values import TreeTopology
from compas_fab.identity_verification import IdentityVerification

T = TypeVar("T")
SHA256_HEX_LENGTH = sha256().digest_size * 2
SOURCE_TREE_SCHEMA = "compas_fab.ghpython.source_tree/v1"
STAGE_TREE_SCHEMA = "compas_fab.ghpython.stage_tree/v1"
_IDENTITY_FACTORY_TOKEN = object()
_AUDITED_REGISTRATION_TOKEN = object()


class TreeIdentityContractError(ValueError):
    """Base failure for deterministic tree identities."""


class InvalidTreeContentDigestError(TreeIdentityContractError):
    """Raised when a tree digest is not lowercase SHA-256 hexadecimal."""


class InvalidBranchContentDigestError(TreeIdentityContractError):
    """Raised when a branch digest is not lowercase SHA-256 hexadecimal."""


class InvalidExactItemCodecError(TreeIdentityContractError):
    """Raised when an exact item codec is malformed or absent."""


class InvalidItemPayloadError(TreeIdentityContractError):
    """Raised when an item cannot be encoded exactly by its declared codec."""


class InvalidStageParameterError(TreeIdentityContractError):
    """Raised when a typed stage parameter is malformed."""


class InvalidSourceTreeIdentityError(TreeIdentityContractError):
    """Raised when source-tree identity fields are inconsistent."""


class InvalidStageTreeIdentityError(TreeIdentityContractError):
    """Raised when stage-tree identity fields or provenance are inconsistent."""


class UnknownStageBuilderSchemaError(InvalidStageTreeIdentityError):
    """Raised when a stage claims verification without an audited builder schema."""


class UnknownStageOutputCoordinateError(InvalidStageTreeIdentityError):
    """Raised when stage attribution names an absent output coordinate."""


class UnknownStageSourceCoordinateError(InvalidStageTreeIdentityError):
    """Raised when stage attribution names an absent prior coordinate."""


class UnknownStageSourceBranchError(InvalidStageTreeIdentityError):
    """Raised when empty-branch attribution names an absent prior branch."""


class NonEmptyStageSourceBranchError(InvalidStageTreeIdentityError):
    """Raised when empty-branch attribution names a nonempty prior branch."""


class IncompleteStageSourceCoordinatesError(InvalidStageTreeIdentityError):
    """Raised when stage attribution does not meet its audited coverage contract."""


class InvalidAuditedStageSchemaError(TreeIdentityContractError):
    """Raised when an implementation-coupled stage schema is malformed."""


class DuplicateAuditedStageSchemaError(InvalidAuditedStageSchemaError):
    """Raised when a schema is registered more than once."""


class UnregisteredStageImplementationError(InvalidStageTreeIdentityError):
    """Raised when verified construction lacks its exact registered builder."""


class InvalidStagePriorBindingError(InvalidStageTreeIdentityError):
    """Raised when semantic roles, runtime roots, or prior identities disagree."""


class InvalidStageBranchEvidenceError(InvalidStageTreeIdentityError):
    """Raised when path-keyed native evidence is incomplete or malformed."""


class UnknownIdentityBranchError(TreeIdentityContractError):
    """Raised when an identity has no branch at a requested path."""


class SourceCoordinateCoverage(Enum):
    """Observed root-free output attribution coverage."""

    COMPLETE_OUTPUTS = "complete_outputs"
    PARTIAL_OUTPUTS = "partial_outputs"


class SourceCoordinateRequirement(Enum):
    """Audited source-attribution requirement for a deterministic builder."""

    COMPLETE_OUTPUTS = "complete_outputs"


@define(frozen=True, slots=True)
class _AuditedStageSchema:
    """Private schema evidence coupled to one implemented deterministic builder."""

    schema: str
    implementation: Callable[..., object] = field(eq=False, repr=False)
    source_requirement: SourceCoordinateRequirement
    _registration_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(
        cls,
        schema: str,
        implementation: Callable[..., object],
        source_requirement: SourceCoordinateRequirement,
    ) -> "_AuditedStageSchema":
        return cls(schema, implementation, source_requirement)

    def __attrs_post_init__(self) -> None:
        if type(self.schema) is not str or not self.schema or self.schema != self.schema.strip():
            raise InvalidAuditedStageSchemaError("Audited stage schema must be non-empty canonical text.")
        if not callable(self.implementation):
            raise InvalidAuditedStageSchemaError("Audited stage schema must reference its deterministic implementation.")
        if type(self.source_requirement) is not SourceCoordinateRequirement:
            raise InvalidAuditedStageSchemaError("Audited stage schema requires an exact source-coordinate contract.")
        if self._registration_token is not None and self._registration_token is not _AUDITED_REGISTRATION_TOKEN:
            raise InvalidAuditedStageSchemaError("Audited stage schema registration evidence is invalid.")


_AUDITED_STAGE_SCHEMAS: Dict[str, _AuditedStageSchema] = {}


def _register_audited_stage_schema(
    schema: str,
    implementation: Callable[..., object],
    source_requirement: SourceCoordinateRequirement,
) -> None:
    """Register one package-owned deterministic builder exactly once."""
    if schema in _AUDITED_STAGE_SCHEMAS:
        raise DuplicateAuditedStageSchemaError("Audited stage schema is already registered.")
    _AUDITED_STAGE_SCHEMAS[schema] = _AuditedStageSchema(
        schema,
        implementation,
        source_requirement,
        _AUDITED_REGISTRATION_TOKEN,
    )


def _valid_digest(value: object) -> bool:
    if type(value) is not str:
        return False
    return len(value) == SHA256_HEX_LENGTH and value == value.lower() and all(character in "0123456789abcdef" for character in value)


@define(frozen=True, slots=True)
class TreeContentDigest:
    """SHA-256 digest of complete root-free tree content."""

    value: str

    @classmethod
    def build(cls, value: str) -> "TreeContentDigest":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if not _valid_digest(self.value):
            raise InvalidTreeContentDigestError("Tree content digest must be lowercase SHA-256 hexadecimal.")


@define(frozen=True, slots=True)
class BranchContentDigest:
    """SHA-256 digest of one root-free branch and its provenance."""

    value: str

    @classmethod
    def build(cls, value: str) -> "BranchContentDigest":
        return cls(value)

    def __attrs_post_init__(self) -> None:
        if not _valid_digest(self.value):
            raise InvalidBranchContentDigestError("Branch content digest must be lowercase SHA-256 hexadecimal.")


def _part(payload: bytes) -> bytes:
    return len(payload).to_bytes(8, "big") + payload


def _text(value: str) -> bytes:
    return value.encode("utf-8")


def _integer(value: int) -> bytes:
    return str(value).encode("ascii")


def _digest(payload: bytes) -> str:
    return sha256(payload).hexdigest()


@define(frozen=True, slots=True)
class ExactItemCodec(Generic[T]):
    """Explicit schema and exact-type encoder for source values."""

    schema: str
    item_type: Type[T]
    encoder: Callable[[T], bytes]

    @classmethod
    def build(
        cls,
        schema: str,
        item_type: Type[T],
        encoder: Callable[[T], bytes],
    ) -> "ExactItemCodec[T]":
        return cls(schema, item_type, encoder)

    def __attrs_post_init__(self) -> None:
        if type(self.schema) is not str or not self.schema or self.schema != self.schema.strip() or not isinstance(self.item_type, type) or not callable(self.encoder):
            raise InvalidExactItemCodecError("Exact item codec requires a stable schema, exact type, and encoder.")

    def encode(self, item: T) -> bytes:
        """Encode one exact-type item or fail with the codec boundary error."""
        if type(item) is not self.item_type:
            raise InvalidItemPayloadError("Tree item does not match the exact codec type.")
        try:
            payload = self.encoder(item)
        except InvalidItemPayloadError:
            raise
        except (OverflowError, TypeError, ValueError) as error:
            raise InvalidItemPayloadError("Exact item codec rejected the item payload.") from error
        if type(payload) is not bytes:
            raise InvalidItemPayloadError("Exact item codec must return exact bytes.")
        return payload


def _encode_text(value: str) -> bytes:
    return value.encode("utf-8")


def _encode_bool(value: bool) -> bytes:
    return b"\x01" if value else b"\x00"


def _encode_f64(value: float) -> bytes:
    if not isfinite(value):
        raise InvalidItemPayloadError("IEEE-754 source floats must be finite.")
    return pack(">d", value)


def _encode_frame(value: Frame) -> bytes:
    coordinates = (
        value.point.x,
        value.point.y,
        value.point.z,
        value.xaxis.x,
        value.xaxis.y,
        value.xaxis.z,
        value.yaxis.x,
        value.yaxis.y,
        value.yaxis.z,
        value.zaxis.x,
        value.zaxis.y,
        value.zaxis.z,
    )
    if any(not isfinite(coordinate) for coordinate in coordinates):
        raise InvalidItemPayloadError("COMPAS Frame identity fields must all be finite.")
    return pack(">12d", *coordinates)


TEXT_CODEC: ExactItemCodec[str] = ExactItemCodec.build("utf8-text/v1", str, _encode_text)
BOOL_CODEC: ExactItemCodec[bool] = ExactItemCodec.build("strict-bool/v1", bool, _encode_bool)
FLOAT64_CODEC: ExactItemCodec[float] = ExactItemCodec.build("ieee754-f64/v1", float, _encode_f64)
FRAME_CODEC: ExactItemCodec[Frame] = ExactItemCodec.build("compas-frame-12xf64/v1", Frame, _encode_frame)


def _path_bytes(path: GhPath) -> bytes:
    return b"".join(_part(_integer(index)) for index in path.indices)


def _shape_bytes(semantics: PortSemantics) -> bytes:
    shape = semantics.item_shape
    tag = b"" if shape.tag is None else _text(shape.tag.value)
    length = b"" if shape.length is None else _integer(shape.length)
    return b"".join(
        (
            _part(_text(semantics.topology_role.value)),
            _part(_text(semantics.branch_semantics.value)),
            _part(_text(shape.kind.value)),
            _part(tag),
            _part(length),
        )
    )


def _branch_bytes(branch: TreeBranch[T], codec: ExactItemCodec[T]) -> bytes:
    item_payloads = []
    for item in branch.items:
        if item.is_null:
            item_payloads.append(_part(b"null"))
        else:
            item_payloads.append(_part(b"value") + _part(codec.encode(cast(T, item.item))))
    return _part(_path_bytes(branch.path)) + _part(b"".join(_part(payload) for payload in item_payloads))


def _topology_branch_bytes(topology: TreeTopology, index: int) -> bytes:
    bitmap = b"".join(b"\x01" if bit else b"\x00" for bit in topology.null_bitmaps[index])
    return b"".join(
        (
            _part(_path_bytes(topology.paths[index])),
            _part(_integer(topology.item_counts[index])),
            _part(bitmap),
        )
    )


def _topology_bytes(topology: TreeTopology) -> bytes:
    return b"".join(_part(_topology_branch_bytes(topology, index)) for index in range(len(topology.paths)))


@define(frozen=True, slots=True)
class SourceTreeIdentity:
    """Verified identity of exact source values and their port meaning."""

    digest: TreeContentDigest
    branch_digests: Tuple[BranchContentDigest, ...]
    topology: TreeTopology
    semantics: PortSemantics
    verification: IdentityVerification
    _canonical: bytes
    _branch_canonical: Tuple[bytes, ...]
    codec_schema: str = ""
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)

    @classmethod
    def build(
        cls,
        tree: Tree[T],
        codec: ExactItemCodec[T],
        semantics: PortSemantics,
    ) -> "SourceTreeIdentity":
        if type(tree) is not Tree:
            raise InvalidSourceTreeIdentityError("Source identity requires an exact Tree value.")
        if type(codec) is not ExactItemCodec:
            raise InvalidExactItemCodecError("Source identity requires an explicit ExactItemCodec.")
        if type(semantics) is not PortSemantics:
            raise InvalidSourceTreeIdentityError("Source identity requires exact port semantics.")
        semantic_payload = _shape_bytes(semantics)
        branch_canonical = tuple(_branch_bytes(branch, codec) for branch in tree.branches)
        branch_digests = tuple(
            BranchContentDigest.build(_digest(_part(_text(SOURCE_TREE_SCHEMA)) + _part(_text(codec.schema)) + _part(semantic_payload) + _part(branch_payload)))
            for branch_payload in branch_canonical
        )
        canonical = b"".join(
            (
                _part(_text(SOURCE_TREE_SCHEMA)),
                _part(_text(codec.schema)),
                _part(semantic_payload),
                _part(b"".join(_part(payload) for payload in branch_canonical)),
            )
        )
        return cls(
            TreeContentDigest.build(_digest(canonical)),
            branch_digests,
            tree.topology,
            semantics,
            IdentityVerification.VERIFIED,
            canonical,
            branch_canonical,
            codec.schema,
            _IDENTITY_FACTORY_TOKEN,
        )

    def __attrs_post_init__(self) -> None:
        valid = (
            type(self.digest) is TreeContentDigest
            and type(self.branch_digests) is tuple
            and all(type(digest) is BranchContentDigest for digest in self.branch_digests)
            and type(self.topology) is TreeTopology
            and type(self.semantics) is PortSemantics
            and self.verification is IdentityVerification.VERIFIED
            and type(self._canonical) is bytes
            and type(self._branch_canonical) is tuple
            and all(type(payload) is bytes for payload in self._branch_canonical)
            and type(self.codec_schema) is str
            and bool(self.codec_schema)
            and len(self.branch_digests) == len(self.topology.paths) == len(self._branch_canonical)
            and self.digest.value == _digest(self._canonical)
            and self._canonical
            == b"".join(
                (
                    _part(_text(SOURCE_TREE_SCHEMA)),
                    _part(_text(self.codec_schema)),
                    _part(_shape_bytes(self.semantics)),
                    _part(b"".join(_part(payload) for payload in self._branch_canonical)),
                )
            )
            and all(
                digest.value == _digest(_part(_text(SOURCE_TREE_SCHEMA)) + _part(_text(self.codec_schema)) + _part(_shape_bytes(self.semantics)) + _part(payload))
                for digest, payload in zip(self.branch_digests, self._branch_canonical)
            )
            and self._factory_token is _IDENTITY_FACTORY_TOKEN
        )
        if not valid:
            raise InvalidSourceTreeIdentityError("Source-tree identity fields are inconsistent.")

    def branch(self, path: GhPath) -> BranchContentDigest:
        """Return the content digest for one canonical branch path."""
        if type(path) is not GhPath:
            raise UnknownIdentityBranchError("Branch identity lookup requires an exact GhPath.")
        for candidate, digest in zip(self.topology.paths, self.branch_digests):
            if candidate == path:
                return digest
        raise UnknownIdentityBranchError("Tree identity has no branch at the requested path.")


class StageParameterKind(Enum):
    TEXT = "text"
    F64 = "f64"
    BOOL = "bool"


StageParameterValue = Union[str, float, bool]


@define(frozen=True, slots=True)
class StageParameter:
    """Validated, typed input to a deterministic builder stage."""

    name: str
    kind: StageParameterKind
    value: StageParameterValue

    @classmethod
    def text(cls, name: str, value: str) -> "StageParameter":
        return cls(name, StageParameterKind.TEXT, value)

    @classmethod
    def f64(cls, name: str, value: float) -> "StageParameter":
        return cls(name, StageParameterKind.F64, value)

    @classmethod
    def bool(cls, name: str, value: bool) -> "StageParameter":
        return cls(name, StageParameterKind.BOOL, value)

    def __attrs_post_init__(self) -> None:
        if type(self.name) is not str or not self.name or self.name != self.name.strip():
            raise InvalidStageParameterError("Stage parameter name must be non-empty canonical text.")
        if type(self.kind) is not StageParameterKind:
            raise InvalidStageParameterError("Stage parameter requires an exact declared type.")
        if self.kind is StageParameterKind.TEXT:
            valid = type(self.value) is str
        elif self.kind is StageParameterKind.F64:
            valid = type(self.value) is float and isfinite(self.value)
        else:
            valid = type(self.value) is bool
        if not valid:
            raise InvalidStageParameterError("Stage parameter value does not match its declared exact type.")

    def identity_bytes(self) -> bytes:
        """Encode the validated tag and value with explicit boundaries."""
        if self.kind is StageParameterKind.TEXT:
            payload = _text(cast(str, self.value))
        elif self.kind is StageParameterKind.F64:
            payload = pack(">d", cast(float, self.value))
        else:
            payload = b"\x01" if cast(bool, self.value) else b"\x00"
        return _part(_text(self.name)) + _part(_text(self.kind.value)) + _part(payload)


@define(frozen=True, slots=True)
class StagePriorBinding:
    """One semantic input role routed from an exact runtime tree root."""

    role: str
    root_id: TreeRootId = field(eq=False)
    identity: "TreeIdentity"

    @classmethod
    def build(
        cls,
        role: str,
        root_id: TreeRootId,
        identity: "TreeIdentity",
    ) -> "StagePriorBinding":
        return cls(role, root_id, identity)

    def __attrs_post_init__(self) -> None:
        if (
            type(self.role) is not str
            or not self.role
            or self.role != self.role.strip()
            or type(self.root_id) is not TreeRootId
            or not _is_tree_identity_value(self.identity)
        ):
            raise InvalidStagePriorBindingError("Stage prior binding requires a unique semantic role, runtime root, and exact identity.")

    def declaration_bytes(self) -> bytes:
        """Encode semantic role and whole content while excluding routing root."""
        return _part(_text(self.role)) + _part(_text(self.identity.digest.value))


@define(frozen=True, slots=True)
class StageBranchEvidence:
    """Exact native or deterministic evidence scoped to one output path."""

    path: GhPath
    fields: Tuple[CanonicalField, ...]

    @classmethod
    def build(
        cls,
        path: GhPath,
        fields: Tuple[CanonicalField, ...],
    ) -> "StageBranchEvidence":
        return cls(path, fields)

    def __attrs_post_init__(self) -> None:
        if (
            type(self.path) is not GhPath
            or type(self.fields) is not tuple
            or not self.fields
            or any(type(value) is not CanonicalField for value in self.fields)
            or len({value.name for value in self.fields}) != len(self.fields)
        ):
            raise InvalidStageBranchEvidenceError("Stage branch evidence requires one path and non-empty unique exact fields.")

    def identity_bytes(self) -> bytes:
        """Encode the root-free path and ordered typed fields."""
        fields = b"".join(_part(_text(value.name)) + _part(value.payload) for value in self.fields)
        return _part(_path_bytes(self.path)) + _part(fields)


def _coordinate_key(coordinate: TreeCoordinate) -> Tuple[Tuple[int, ...], int]:
    return coordinate.branch.path.canonical_key(), coordinate.item_index.value


def _topology_coordinate_keys(topology: TreeTopology) -> frozenset[Tuple[Tuple[int, ...], int]]:
    return frozenset((path.canonical_key(), item_index) for path, item_count in zip(topology.paths, topology.item_counts) for item_index in range(item_count))


def _validate_source_coordinates(
    mapping: SourceCoordinateMap,
    output_topology: TreeTopology,
    prior_topology: TreeTopology,
) -> SourceCoordinateCoverage:
    output_keys = _topology_coordinate_keys(output_topology)
    prior_keys = _topology_coordinate_keys(prior_topology)
    prior_branch_item_counts = {path.canonical_key(): item_count for path, item_count in zip(prior_topology.paths, prior_topology.item_counts)}
    mapped_output_keys = set()
    for entry in mapping.entries:
        output_key = _coordinate_key(entry.output)
        if output_key not in output_keys:
            raise UnknownStageOutputCoordinateError("Stage source mapping output must exist in the output topology.")
        mapped_output_keys.add(output_key)
        for source in entry.sources:
            if _coordinate_key(source) not in prior_keys:
                raise UnknownStageSourceCoordinateError("Stage source mapping source must exist in the prior topology.")
        if entry.empty_source_branch is not None:
            source_branch_key = entry.empty_source_branch.path.canonical_key()
            if source_branch_key not in prior_branch_item_counts:
                raise UnknownStageSourceBranchError("Stage empty-branch attribution must name a branch in the prior topology.")
            if prior_branch_item_counts[source_branch_key] != 0:
                raise NonEmptyStageSourceBranchError("Stage empty-branch attribution must name an empty prior branch.")
    if mapped_output_keys == output_keys:
        return SourceCoordinateCoverage.COMPLETE_OUTPUTS
    return SourceCoordinateCoverage.PARTIAL_OUTPUTS


def _binding_by_root(
    bindings: Tuple[StagePriorBinding, ...],
) -> Dict[TreeRootId, StagePriorBinding]:
    return {binding.root_id: binding for binding in bindings}


def _validate_prior_bindings(
    prior: "TreeIdentity",
    bindings: Tuple[StagePriorBinding, ...],
) -> None:
    if type(bindings) is not tuple or any(type(binding) is not StagePriorBinding for binding in bindings):
        raise InvalidStagePriorBindingError("Stage prior bindings must be an exact tuple.")
    if not bindings:
        return
    roles = tuple(binding.role for binding in bindings)
    roots = tuple(binding.root_id for binding in bindings)
    if len(set(roles)) != len(roles) or len(set(roots)) != len(roots):
        raise InvalidStagePriorBindingError("Stage prior roles and runtime roots must each be unique.")
    if bindings[0].identity != prior:
        raise InvalidStagePriorBindingError("First stage prior binding must retain the primary prior identity.")


def _validate_bound_source_coordinates(
    mapping: SourceCoordinateMap,
    output_topology: TreeTopology,
    bindings: Tuple[StagePriorBinding, ...],
) -> SourceCoordinateCoverage:
    output_keys = _topology_coordinate_keys(output_topology)
    binding_by_root = _binding_by_root(bindings)
    mapped_output_keys = set()
    for entry in mapping.entries:
        output_key = _coordinate_key(entry.output)
        if output_key not in output_keys:
            raise UnknownStageOutputCoordinateError("Stage source mapping output must exist in the output topology.")
        mapped_output_keys.add(output_key)
        for source in entry.sources:
            binding = binding_by_root.get(source.branch.root_id)
            if binding is None:
                raise InvalidStagePriorBindingError("Stage source root has no semantic prior binding.")
            if _coordinate_key(source) not in _topology_coordinate_keys(binding.identity.topology):
                raise UnknownStageSourceCoordinateError("Stage source mapping source must exist in its bound prior topology.")
        if entry.empty_source_branch is not None:
            empty = entry.empty_source_branch
            binding = binding_by_root.get(empty.root_id)
            if binding is None:
                raise InvalidStagePriorBindingError("Stage empty-source root has no semantic prior binding.")
            counts = {
                path.canonical_key(): count
                for path, count in zip(binding.identity.topology.paths, binding.identity.topology.item_counts)
            }
            key = empty.path.canonical_key()
            if key not in counts:
                raise UnknownStageSourceBranchError("Stage empty-branch attribution must name a branch in its bound prior.")
            if counts[key] != 0:
                raise NonEmptyStageSourceBranchError("Stage empty-branch attribution must name an empty prior branch.")
    if mapped_output_keys == output_keys:
        return SourceCoordinateCoverage.COMPLETE_OUTPUTS
    return SourceCoordinateCoverage.PARTIAL_OUTPUTS


def _role_aware_mapping_bytes(
    mapping: SourceCoordinateMap,
    bindings: Tuple[StagePriorBinding, ...],
) -> bytes:
    binding_by_root = _binding_by_root(bindings)
    payloads = []
    for entry in mapping.entries:
        output = _part(_path_bytes(entry.output.branch.path)) + _part(_integer(entry.output.item_index.value))
        if entry.empty_source_branch is not None:
            empty = entry.empty_source_branch
            binding = binding_by_root[empty.root_id]
            source = _part(_text(binding.role)) + _part(_path_bytes(empty.path))
            payloads.append(_part(b"empty_branch") + _part(output) + _part(source))
            continue
        sources = []
        for source_coordinate in entry.sources:
            binding = binding_by_root[source_coordinate.branch.root_id]
            sources.append(
                _part(_text(binding.role))
                + _part(_path_bytes(source_coordinate.branch.path))
                + _part(_integer(source_coordinate.item_index.value))
            )
        payloads.append(_part(b"items") + _part(output) + _part(b"".join(_part(value) for value in sources)))
    return b"".join(_part(value) for value in payloads)


def _validate_branch_evidence(
    evidence: Tuple[StageBranchEvidence, ...],
    topology: TreeTopology,
) -> None:
    if type(evidence) is not tuple or any(type(value) is not StageBranchEvidence for value in evidence):
        raise InvalidStageBranchEvidenceError("Stage branch evidence must be an exact tuple.")
    if evidence and tuple(value.path for value in evidence) != topology.paths:
        raise InvalidStageBranchEvidenceError("Stage branch evidence must cover every output path once in canonical order.")


def _branch_evidence_payload(
    evidence: Tuple[StageBranchEvidence, ...],
    path: GhPath,
) -> bytes:
    for value in evidence:
        if value.path == path:
            return value.identity_bytes()
    return b""


@define(frozen=True, slots=True)
class StageTreeIdentity:
    """Identity derived from prior content and registered builder provenance."""

    digest: TreeContentDigest
    branch_digests: Tuple[BranchContentDigest, ...]
    prior_digest: TreeContentDigest
    builder_schema: str
    parameters: Tuple[StageParameter, ...]
    topology: TreeTopology
    source_coordinates: SourceCoordinateMap = field(eq=False)
    source_coverage: SourceCoordinateCoverage
    verification: IdentityVerification
    _canonical: bytes
    _branch_canonical: Tuple[bytes, ...]
    _factory_token: Optional[object] = field(default=None, eq=False, repr=False)
    prior_bindings: Tuple[StagePriorBinding, ...] = ()
    branch_evidence: Tuple[StageBranchEvidence, ...] = ()
    _builder_implementation: Optional[Callable[..., object]] = field(default=None, eq=False, repr=False)
    _bound_prior_mode: bool = field(default=False, eq=False, repr=False)

    @classmethod
    def build(
        cls,
        prior: "TreeIdentity",
        builder_schema: str,
        parameters: Tuple[StageParameter, ...],
        topology: TreeTopology,
        source_coordinates: Optional[SourceCoordinateMap] = None,
        verification: Optional[IdentityVerification] = None,
        *,
        implementation: Optional[Callable[..., object]] = None,
        prior_binding: Optional[StagePriorBinding] = None,
        additional_priors: Tuple[StagePriorBinding, ...] = (),
        branch_evidence: Tuple[StageBranchEvidence, ...] = (),
    ) -> "StageTreeIdentity":
        if type(prior) is not SourceTreeIdentity and type(prior) is not StageTreeIdentity:
            raise InvalidStageTreeIdentityError("Stage identity requires an exact prior tree identity.")
        if type(builder_schema) is not str or not builder_schema or builder_schema != builder_schema.strip():
            raise InvalidStageTreeIdentityError("Stage identity requires a non-empty canonical builder schema.")
        if type(parameters) is not tuple or any(type(parameter) is not StageParameter for parameter in parameters):
            raise InvalidStageTreeIdentityError("Stage identity requires an exact parameter tuple.")
        names = tuple(parameter.name for parameter in parameters)
        if len(set(names)) != len(names):
            raise InvalidStageTreeIdentityError("Stage parameter names must be unique.")
        if type(topology) is not TreeTopology:
            raise InvalidStageTreeIdentityError("Stage identity requires an exact output topology.")
        mapping = SourceCoordinateMap.build(()) if source_coordinates is None else source_coordinates
        if type(mapping) is not SourceCoordinateMap:
            raise InvalidStageTreeIdentityError("Stage identity requires an exact source-coordinate map.")
        if prior_binding is None:
            if additional_priors:
                raise InvalidStagePriorBindingError("Additional stage priors require an explicit primary prior binding.")
            bindings: Tuple[StagePriorBinding, ...] = ()
        else:
            bindings = (prior_binding,) + additional_priors
        _validate_prior_bindings(prior, bindings)
        _validate_branch_evidence(branch_evidence, topology)
        if verification is not None and verification is not IdentityVerification.UNVERIFIABLE:
            raise InvalidStageTreeIdentityError("Caller-supplied stage verification may only downgrade to UNVERIFIABLE.")
        audited_schema = _AUDITED_STAGE_SCHEMAS.get(builder_schema)
        if audited_schema is None and verification is not IdentityVerification.UNVERIFIABLE:
            raise UnknownStageBuilderSchemaError("Unknown builder schemas require explicit UNVERIFIABLE identity.")
        if audited_schema is not None and audited_schema.schema != builder_schema:
            raise InvalidStageTreeIdentityError("Audited builder schema registry key and descriptor disagree.")
        if (
            audited_schema is not None
            and audited_schema._registration_token is _AUDITED_REGISTRATION_TOKEN
            and implementation is not audited_schema.implementation
        ):
            raise UnregisteredStageImplementationError("Verified stage construction requires its exact registered implementation.")
        resolved_verification = prior.verification if verification is None else verification
        source_coverage = (
            _validate_bound_source_coordinates(mapping, topology, bindings)
            if bindings
            else _validate_source_coordinates(mapping, topology, prior.topology)
        )
        if (
            resolved_verification is IdentityVerification.VERIFIED
            and audited_schema is not None
            and audited_schema.source_requirement is SourceCoordinateRequirement.COMPLETE_OUTPUTS
            and source_coverage is not SourceCoordinateCoverage.COMPLETE_OUTPUTS
        ):
            raise IncompleteStageSourceCoordinatesError("Verified stage source attribution does not satisfy its audited schema contract.")
        parameter_payload = b"".join(_part(parameter.identity_bytes()) for parameter in parameters)
        bindings_payload = b"".join(_part(binding.declaration_bytes()) for binding in bindings)
        evidence_payload = b"".join(_part(value.identity_bytes()) for value in branch_evidence)
        common = b"".join(
            (
                _part(_text(STAGE_TREE_SCHEMA)),
                _part(_text(builder_schema)),
                _part(_text(prior.digest.value)),
                _part(bindings_payload),
                _part(parameter_payload),
                _part(_text(source_coverage.value)),
                _part(_text(resolved_verification.value)),
            )
        )
        mapping_payload = _role_aware_mapping_bytes(mapping, bindings) if bindings else mapping.root_free_identity_bytes()
        branch_canonical = tuple(
            (
                _bound_stage_branch_bytes(
                    bindings,
                    builder_schema,
                    parameter_payload,
                    resolved_verification,
                    topology,
                    index,
                    mapping,
                    branch_evidence,
                )
                if bindings
                else _stage_branch_bytes(
                    prior,
                    builder_schema,
                    parameter_payload,
                    resolved_verification,
                    topology,
                    index,
                    mapping,
                )
            )
            for index in range(len(topology.paths))
        )
        branch_digests = tuple(BranchContentDigest.build(_digest(payload)) for payload in branch_canonical)
        canonical = common + _part(_topology_bytes(topology)) + _part(mapping_payload) + _part(evidence_payload)
        return cls(
            TreeContentDigest.build(_digest(canonical)),
            branch_digests,
            prior.digest,
            builder_schema,
            parameters,
            topology,
            mapping,
            source_coverage,
            resolved_verification,
            canonical,
            branch_canonical,
            _IDENTITY_FACTORY_TOKEN,
            bindings,
            branch_evidence,
            implementation,
            bool(bindings),
        )

    def __attrs_post_init__(self) -> None:
        if type(self.topology) is not TreeTopology:
            raise InvalidStageTreeIdentityError("Stored stage identity requires exact output topology.")
        _validate_branch_evidence(self.branch_evidence, self.topology)
        if type(self.prior_bindings) is not tuple or any(type(value) is not StagePriorBinding for value in self.prior_bindings):
            raise InvalidStagePriorBindingError("Stored stage prior bindings must be an exact tuple.")
        if type(self._bound_prior_mode) is not bool or self._bound_prior_mode != bool(self.prior_bindings):
            raise InvalidStagePriorBindingError("Stored stage prior mode must match retained semantic bindings.")
        if self.prior_bindings:
            roles = tuple(value.role for value in self.prior_bindings)
            roots = tuple(value.root_id for value in self.prior_bindings)
            if len(set(roles)) != len(roles) or len(set(roots)) != len(roots):
                raise InvalidStagePriorBindingError("Stored stage prior roles and roots must be unique.")
            if self.prior_bindings[0].identity.digest != self.prior_digest:
                raise InvalidStagePriorBindingError("Stored primary prior binding must match retained prior digest.")
        descriptor = _AUDITED_STAGE_SCHEMAS.get(self.builder_schema)
        if (
            descriptor is not None
            and descriptor._registration_token is _AUDITED_REGISTRATION_TOKEN
            and self.verification is IdentityVerification.VERIFIED
            and self._builder_implementation is not descriptor.implementation
        ):
            raise UnregisteredStageImplementationError("Stored verified stage lacks its exact registered implementation.")
        bound_payloads_valid = True
        if self.prior_bindings:
            coverage = _validate_bound_source_coordinates(self.source_coordinates, self.topology, self.prior_bindings)
            parameter_payload = b"".join(_part(parameter.identity_bytes()) for parameter in self.parameters)
            bindings_payload = b"".join(_part(binding.declaration_bytes()) for binding in self.prior_bindings)
            evidence_payload = b"".join(_part(value.identity_bytes()) for value in self.branch_evidence)
            common = b"".join(
                (
                    _part(_text(STAGE_TREE_SCHEMA)),
                    _part(_text(self.builder_schema)),
                    _part(_text(self.prior_digest.value)),
                    _part(bindings_payload),
                    _part(parameter_payload),
                    _part(_text(coverage.value)),
                    _part(_text(self.verification.value)),
                )
            )
            expected_branches = tuple(
                _bound_stage_branch_bytes(
                    self.prior_bindings,
                    self.builder_schema,
                    parameter_payload,
                    self.verification,
                    self.topology,
                    index,
                    self.source_coordinates,
                    self.branch_evidence,
                )
                for index in range(len(self.topology.paths))
            )
            mapping_payload = _role_aware_mapping_bytes(self.source_coordinates, self.prior_bindings)
            expected_canonical = common + _part(_topology_bytes(self.topology)) + _part(mapping_payload) + _part(evidence_payload)
            bound_payloads_valid = (
                coverage is self.source_coverage
                and expected_branches == self._branch_canonical
                and expected_canonical == self._canonical
            )
        valid = (
            type(self.digest) is TreeContentDigest
            and type(self.branch_digests) is tuple
            and all(type(digest) is BranchContentDigest for digest in self.branch_digests)
            and type(self.prior_digest) is TreeContentDigest
            and type(self.builder_schema) is str
            and bool(self.builder_schema)
            and self.builder_schema == self.builder_schema.strip()
            and type(self.parameters) is tuple
            and all(type(parameter) is StageParameter for parameter in self.parameters)
            and type(self.topology) is TreeTopology
            and type(self.source_coordinates) is SourceCoordinateMap
            and type(self.source_coverage) is SourceCoordinateCoverage
            and type(self.verification) is IdentityVerification
            and type(self._canonical) is bytes
            and type(self._branch_canonical) is tuple
            and all(type(payload) is bytes for payload in self._branch_canonical)
            and len(self.branch_digests) == len(self.topology.paths) == len(self._branch_canonical)
            and self.digest.value == _digest(self._canonical)
            and all(digest.value == _digest(payload) for digest, payload in zip(self.branch_digests, self._branch_canonical))
            and self._factory_token is _IDENTITY_FACTORY_TOKEN
            and type(self.branch_evidence) is tuple
            and type(self._bound_prior_mode) is bool
            and bound_payloads_valid
        )
        if not valid:
            raise InvalidStageTreeIdentityError("Stage-tree identity fields are inconsistent.")

    def branch(self, path: GhPath) -> BranchContentDigest:
        """Return the stage digest for one canonical output branch."""
        if type(path) is not GhPath:
            raise UnknownIdentityBranchError("Branch identity lookup requires an exact GhPath.")
        for candidate, digest in zip(self.topology.paths, self.branch_digests):
            if candidate == path:
                return digest
        raise UnknownIdentityBranchError("Tree identity has no branch at the requested path.")

    @property
    def requires_fresh_compute_token(self) -> bool:
        """Whether runtime execution must prove a fresh compute attempt."""
        return self.verification is IdentityVerification.UNVERIFIABLE

    @property
    def reusable_from_content_cache(self) -> bool:
        """Whether deterministic content-cache reuse is truthful."""
        return self.verification is IdentityVerification.VERIFIED


TreeIdentity = Union[SourceTreeIdentity, StageTreeIdentity]


def _is_tree_identity_value(value: object) -> bool:
    return type(value) is SourceTreeIdentity or type(value) is StageTreeIdentity


def _stage_branch_bytes(
    prior: TreeIdentity,
    builder_schema: str,
    parameter_payload: bytes,
    verification: IdentityVerification,
    topology: TreeTopology,
    branch_index: int,
    mapping: SourceCoordinateMap,
) -> bytes:
    output_path = topology.paths[branch_index]
    relevant_entries = tuple(entry for entry in mapping.entries if entry.output.branch.path == output_path)
    mapped_item_indices = {entry.output.item_index.value for entry in relevant_entries}
    expected_item_indices = set(range(topology.item_counts[branch_index]))
    source_coverage = SourceCoordinateCoverage.COMPLETE_OUTPUTS if mapped_item_indices == expected_item_indices else SourceCoordinateCoverage.PARTIAL_OUTPUTS
    source_paths = []
    for entry in relevant_entries:
        for source in entry.sources:
            if source.branch.path not in source_paths:
                source_paths.append(source.branch.path)
        if entry.empty_source_branch is not None and entry.empty_source_branch.path not in source_paths:
            source_paths.append(entry.empty_source_branch.path)
    if source_paths:
        prior_digests = tuple(prior.branch(path) for path in source_paths)
        prior_payload = b"".join(_part(_text(digest.value)) for digest in prior_digests)
    elif topology.item_counts[branch_index] == 0 and output_path in prior.topology.paths:
        prior_payload = _part(_text(prior.branch(output_path).value))
    else:
        prior_payload = _part(_text(prior.digest.value))
    mapping_payload = SourceCoordinateMap.build(relevant_entries).root_free_identity_bytes()
    return b"".join(
        (
            _part(_text(STAGE_TREE_SCHEMA)),
            _part(_text(builder_schema)),
            _part(prior_payload),
            _part(parameter_payload),
            _part(_text(source_coverage.value)),
            _part(_text(verification.value)),
            _part(_topology_branch_bytes(topology, branch_index)),
            _part(mapping_payload),
        )
    )


def _bound_stage_branch_bytes(
    bindings: Tuple[StagePriorBinding, ...],
    builder_schema: str,
    parameter_payload: bytes,
    verification: IdentityVerification,
    topology: TreeTopology,
    branch_index: int,
    mapping: SourceCoordinateMap,
    evidence: Tuple[StageBranchEvidence, ...],
) -> bytes:
    output_path = topology.paths[branch_index]
    relevant_entries = tuple(entry for entry in mapping.entries if entry.output.branch.path == output_path)
    mapped_item_indices = {entry.output.item_index.value for entry in relevant_entries}
    expected_item_indices = set(range(topology.item_counts[branch_index]))
    coverage = SourceCoordinateCoverage.COMPLETE_OUTPUTS if mapped_item_indices == expected_item_indices else SourceCoordinateCoverage.PARTIAL_OUTPUTS
    binding_by_root = _binding_by_root(bindings)
    source_keys = []
    for entry in relevant_entries:
        coordinates: Tuple[BranchCoordinate, ...]
        if entry.empty_source_branch is not None:
            coordinates = (entry.empty_source_branch,)
        else:
            coordinates = tuple(source.branch for source in entry.sources)
        for coordinate in coordinates:
            binding = binding_by_root[coordinate.root_id]
            key = (binding.role, coordinate.path)
            if key not in source_keys:
                source_keys.append(key)
    if not source_keys:
        for binding in bindings:
            if output_path in binding.identity.topology.paths:
                source_keys.append((binding.role, output_path))
    prior_payloads = []
    for role, path in source_keys:
        binding = next(value for value in bindings if value.role == role)
        prior_payloads.append(
            _part(_text(role))
            + _part(_path_bytes(path))
            + _part(_text(binding.identity.branch(path).value))
        )
    branch_mapping = SourceCoordinateMap.build(relevant_entries)
    mapping_payload = _role_aware_mapping_bytes(branch_mapping, bindings)
    return b"".join(
        (
            _part(_text(STAGE_TREE_SCHEMA)),
            _part(_text(builder_schema)),
            _part(b"".join(_part(value) for value in prior_payloads)),
            _part(parameter_payload),
            _part(_text(coverage.value)),
            _part(_text(verification.value)),
            _part(_topology_branch_bytes(topology, branch_index)),
            _part(mapping_payload),
            _part(_branch_evidence_payload(evidence, output_path)),
        )
    )
