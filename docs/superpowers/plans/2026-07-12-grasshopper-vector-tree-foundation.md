# Grasshopper Vector/Tree Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development. Dispatch a fresh implementation subagent per task, then fresh specification and quality reviewers. Track every checkbox.

**Goal:** Establish a host-independent ordered-tree contract, explicit bounded expansion, exact-native Cartesian/joint/state series authoring, branch-planning foundations, and pure native-result expansion without claiming unproven asynchronous Grasshopper lifecycle or viewport behavior.

**Architecture:** Immutable tree/domain values separate content identity from runtime routing and generations. Exact stage-provenance identities bridge source frames and group-typed fixed vectors through native programs into branch planning; Tesseract remains authoritative, and the baseline serializes one shared runtime/composer while each sequential call receives its own native robot clone. Seven additive pure Grasshopper nodes exercise exact tree transport, explicit cross-product, authoring, and inspection under a host-behavior emulation model; Windows Rhino owns later async-node lifecycle, progressive-port, viewport, bake, compilation, and save/reopen proof.

**Tech Stack:** Python 3.9/3.12, attrs, SHA-256, Hypothesis, `tesseract-robotics-nanobind==0.35.0.6`, Grasshopper CPython source contracts, pytest-xdist/testmon, strict mypy, Ruff, Pixi, MkDocs.

## Global Constraints

- Exact native `Pose`, `CartesianTarget`, `JointTarget`, `StateTarget`, `MotionProgram`, `CompositeInstruction`, `ProfileDictionary`, `TesseractPlanningRequest`, `TesseractPlanningResult`, and native trajectory points remain authoritative.
- Series are primary. A scalar setting is accepted only through `ScalarFromTree`, which requires one canonical branch containing one non-null item. A one-item domain tree is never implicitly broadcast.
- `TreeRootId` is runtime routing only. It never enters `TreeContentDigest`, `BranchContentDigest`, stage provenance, or `BuildIdentity`.
- `Tree.build` requires canonical host path order: lexicographic integer-segment order with a shorter equal prefix first. It rejects unsorted or duplicate paths; capture observes rather than repairs host order.
- Paths, empty branches, item order, and null slots survive. Never flatten, simplify, graft, renumber, repeat-last, truncate, skip nulls, or use longest-list matching.
- Only a port declared `BROADCASTABLE_SCALAR` broadcasts. Trees use equal branch-local zip. Cross-product is explicit, pure, deterministic, and bounded.
- Content identity uses verified codecs or typed stage provenance. Never pickle, reflect, hash object addresses, or pretend arbitrary nanobind objects serialize canonically.
- Every topology-bearing output has parallel status; source-shaped item diagnostics remain source-shaped; branch-only failures use `BranchDiagnosticMap`.
- Host capture retains exact ordered paths, integer segments, branch order, item indices, and explicit null positions. Host emission uses `EnsurePath` for every branch followed by indexed `Insert` for every slot including `None`, then self-audits the built structure before publication.
- Coordinate-derived labels are display-only. They never simplify paths, mutate exact native names, or enter content identity.
- All values are frozen/slotted attrs classes. Public `build` factories and `__attrs_post_init__` enforce the same invariants. Every failure mode is named.
- One responsibility per file. Package `__init__.py` files stay minimal and add no `__all__`.
- Python 3.9 syntax only. Imports are unconditional. No fallback, optional import, `HAS_*`, broad swallowed exception, skip, skip-if, or xfail.
- Existing scalar nodes remain untouched. New series nodes normalize one-item trees only on ports declared as scalar settings; series ports retain every branch, item, empty branch, and null slot unchanged. They do not alter old component layouts or behavior.
- Use `apply_patch` and Pixi. Every pytest invocation includes `-n auto`. Every task runs affected `pytest --testmon -n auto` before review.
- Never commit automatically. After both task reviewers approve, stop and obtain explicit user authorization for that commit. If authorized, set author and committer to `Jelle Feringa <jelleferinga@gmail.com>` and do not push.
- Run each listed command separately. Do not chain commands.

## Deliberate Boundary

This tranche ships tree/domain foundations, a Grasshopper-visible bounded pure cross-product, pure Cartesian/joint/state series authoring nodes, a host-independent planning owner with explicit close, and a pure result-inspection node. It does not expose an asynchronous planning Grasshopper node or document owner registry. The next host-runtime tranche has exactly seven planned nodes: `Tesseract Planning Job`, `Tesseract Cancel Planning`, `Tesseract Planning Result`, `Tesseract Trajectory Scrub`, `Tesseract Scene Preview`, `Tesseract Path Preview`, and the separate edge-triggered `Tesseract Bake Prepared Geometry`. Windows Rhino must first prove lifecycle binding, scheduled solves, document close, compilation, progressive ports/menu, undo, arity, save/reopen, current-sample/full-series source identity, preview sibling retention, bake idempotency/stale rejection, and multi-iteration behavior. No viewport, bake, packaged-GHX, controller, scene, mobile/VKC, concurrent TrajOpt, or hardware claim is made by this foundation tranche.

The follow-on tranche receives Task 7's branch planning owner and Task 8's identified result/sample expansion without changing either contract. Planning Job publishes one handle/snapshot per exact program coordinate; Cancel and Result retain the same coordinates/status/diagnostic surfaces. Trajectory Scrub emits both the selected current sample and the complete sample series with one composed `SourceCoordinateMap`; Scene Preview consumes only the immutable selected current sample, while Path Preview consumes the immutable complete series/trajectory. Changed preview branches disappear without touching unchanged sibling display. Bake consumes only immutable prepared geometry and a fresh edge token, records source/build identity in a deterministic object map, rejects stale generations, and returns the same receipt without duplicating objects when the token repeats.

That tranche must add compiled-user-object acceptance plus actual-Rhino tests for exact nonzero/prefix paths, branch order, empty branches, explicit null positions, matching-mode immunity, group-typed fixed-vector atomicity, optional absence versus connected null, bounded cross-product, branch reorder/insert/remove stability, current/full-series identity, sibling preview retention, progressive-port undo/wire/value/save-reopen behavior, and bake duplicate/stale/partial-failure reconciliation. These tests are mandatory release gates, not evidence claimed by Tasks 1–9.

## File Map

| File | One responsibility |
|---|---|
| `src/compas_fab/ghpython/tree_errors.py` | named tree/match/identity failures |
| `src/compas_fab/ghpython/tree_coordinates.py` | canonical paths and coordinates |
| `src/compas_fab/ghpython/tree_values.py` | immutable null-aware ordered trees |
| `src/compas_fab/ghpython/port_semantics.py` | orthogonal port and policy declarations |
| `src/compas_fab/ghpython/tree_diagnostics.py` | statuses, source maps, branch diagnostics, reduced outputs |
| `src/compas_fab/ghpython/coordinate_labels.py` | deterministic display-only labels from exact coordinates |
| `src/compas_fab/ghpython/optional_tree_input.py` | explicit absent-versus-present optional tree input |
| `src/compas_fab/ghpython/group_shape.py` | planning-group identity, ordered joint IDs, and DOF |
| `src/compas_fab/ghpython/tree_identity.py` | source content and stage-provenance identity |
| `src/compas_fab/ghpython/tree_matching.py` | scalar broadcast and branch-local zip |
| `src/compas_fab/ghpython/tree_expansion.py` | explicit bounded cross-product |
| `src/compas_fab/ghpython/cross_product_projection.py` | aligned Grasshopper trees projected from exact product pairs |
| `src/compas_fab/ghpython/tree_expansion_codec.py` | one reversible coordinate/path codec |
| `src/compas_fab/ghpython/tree_codec.py` | canonical host-neutral tree wire codec |
| `src/compas_fab/ghpython/branch_runtime_identity.py` | runtime generations and dependency closure |
| `src/compas_fab/ghpython/branch_current_output.py` | branch current-or-absent publication |
| `src/compas_fab/backends/tesseract/scene_identity.py` | exact artifact/scene content digest |
| `src/compas_fab/ghpython/tesseract_pose_series.py` | exact pose-series construction |
| `src/compas_fab/ghpython/tesseract_cartesian_target_series.py` | exact Cartesian-target series construction |
| `src/compas_fab/ghpython/tesseract_joint_target_series.py` | exact group-typed joint-target series construction |
| `src/compas_fab/ghpython/tesseract_state_target_series.py` | exact group-typed state-target series construction and optional-field absence |
| `src/compas_fab/ghpython/tesseract_program_series.py` | one exact program per ordered branch |
| `src/compas_fab/ghpython/planning_content_identity.py` | program/planner/scene/profile request identity |
| `src/compas_fab/ghpython/branch_planning_state.py` | truthful pure planning reducer |
| `src/compas_fab/ghpython/branch_planning.py` | bounded host-independent execution owner |
| `src/compas_fab/ghpython/tesseract_result_series.py` | pure result/sample expansion |
| `src/compas_fab/ghpython/grasshopper_tree_adapter.py` | tree-access capture/emission and scalar decoding |
| `tests/ghpython/tree_host_emulation.py` | pending-Rhino host-behavior model |

---

### Task 1: Canonical Tree Values and Orthogonal Port Meaning

**Files:**
- Create: `src/compas_fab/ghpython/tree_errors.py`
- Create: `src/compas_fab/ghpython/tree_coordinates.py`
- Create: `src/compas_fab/ghpython/tree_values.py`
- Create: `src/compas_fab/ghpython/port_semantics.py`
- Create: `tests/ghpython/test_tree_values.py`
- Modify: `pyproject.toml`
- Modify: `pixi.lock`

**Interfaces:**
- `TreeRootId`, `ItemIndex`, `GhPath`, `BranchCoordinate`, `TreeCoordinate`, `TreeItem[T]`, `TreeBranch[T]`, `Tree[T]`, `TreeTopology`.
- `ShapeTag.build(value)`, `ItemShape.scalar/domain_atomic/fixed_vector`, `PortSemantics.build`, `Scalar[T]`, `ScalarFromTree[T]`, `AtomicFromTree[T]`, `FixedVector[T]`.
- `ItemValidationPolicy`, `SequenceReductionPolicy`, and the single canonical `BatchPublicationPolicy` with exact values `PUBLISH_INDEPENDENT` and `FAIL_BATCH`.

- [ ] **Step 1: Write RED examples and Hypothesis properties**

```python
@given(st.lists(st.lists(st.integers(min_value=0), min_size=1).map(tuple), min_size=1, unique=True))
def test_tree_accepts_only_canonical_host_path_order(raw_paths) -> None:
    paths = [GhPath.build(*parts) for parts in raw_paths]
    canonical = sorted(paths, key=lambda path: path.indices)
    branches = tuple(TreeBranch.build(path, ()) for path in canonical)
    assert Tree.build(TreeRootId.build("routing-a"), branches).branches == branches
    if paths != canonical:
        with pytest.raises(NonCanonicalTreeOrderError):
            Tree.build(TreeRootId.build("routing-a"), tuple(TreeBranch.build(path, ()) for path in paths))


def test_tree_preserves_empty_branch_null_slot_and_fixed_vector_atomicity() -> None:
    shape = ItemShape.fixed_vector(ShapeTag.build("group/joints/6"), 6)
    vector = FixedVector.build(shape, (1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
    tree = Tree.build(
        TreeRootId.build("routing-a"),
        (
            TreeBranch.build(GhPath.build(0, 2), (TreeItem.value(vector), TreeItem.null())),
            TreeBranch.build(GhPath.build(3), ()),
        ),
    )
    assert tree.branches[0].items[0].item is vector
    assert tree.branches[1].items == ()
```

Also test duplicate paths, negative/boolean segments, unsorted prefix paths, empty/invalid `ShapeTag`, fixed-vector length mismatch, raw-constructor bypass, `ScalarFromTree` rejection of zero/multiple branches, zero/multiple items, and null, plus `AtomicFromTree` retaining one exact domain object without labelling it scalar.

- [ ] **Step 2: Verify RED**

Run: `pixi run pytest tests/ghpython/test_tree_values.py -n auto -q`

Expected: collection fails because production modules are absent.

- [ ] **Step 3: Implement exact invariants**

`GhPath.build` accepts at least one exact non-negative integer and exposes `canonical_key()` equal to its integer tuple. `Tree.build` retains already-canonical branches and fails unsorted/duplicate input. `TreeRootId.build` validates non-empty text but remains routing metadata. `TreeItem` distinguishes a null slot from a value; `TreeTopology` records ordered paths, counts, and null bitmaps.

`ShapeTag.build` accepts non-empty text with no surrounding whitespace. `ItemShape` enforces: scalar has no tag/length; domain-atomic has a tag and no length; fixed-vector has a tag and positive exact integer length. `ScalarFromTree.build(tree)` requires exactly one branch, one non-null item, and returns `Scalar`; it never accepts a one-item tree as broadcast unless the consuming port declares `BROADCASTABLE_SCALAR`. `AtomicFromTree.build(tree, expected_type)` has the same cardinality rule, retains one exact domain object, and returns an atomic wrapper rather than `Scalar`.

Add direct locked Pixi dependencies: `hypothesis = ">=6"` in `[tool.pixi.dependencies]` so every test environment imports it, and `librsvg = "*"` in `[tool.pixi.feature.dev.dependencies]` so default-environment icon tasks resolve `rsvg-convert`. Update `pixi.lock`; do not add PyPI fallbacks.

Run: `pixi lock`

Expected: lock resolves both workspace dependencies on supported platforms.

- [ ] **Step 4: Verify and review**

Run: `pixi run pytest tests/ghpython/test_tree_values.py -n auto -q`

Run: `pixi run pytest --testmon -n auto -q`

Run: `pixi run python -c "import hypothesis"`

Run: `pixi run rsvg-convert --version`

Run: `pixi run mypy --strict src/compas_fab/ghpython/tree_errors.py src/compas_fab/ghpython/tree_coordinates.py src/compas_fab/ghpython/tree_values.py src/compas_fab/ghpython/port_semantics.py`

Run: `pixi run ruff check src/compas_fab/ghpython/tree_errors.py src/compas_fab/ghpython/tree_coordinates.py src/compas_fab/ghpython/tree_values.py src/compas_fab/ghpython/port_semantics.py tests/ghpython/test_tree_values.py`

Expected: all commands pass. Dispatch fresh specification and quality reviewers. Stop before git mutation and request explicit commit authorization.

### Task 2: Source Coordinates, Diagnostics, Reduction, and Content Identity

**Files:**
- Create: `src/compas_fab/ghpython/tree_diagnostics.py`
- Create: `src/compas_fab/ghpython/tree_identity.py`
- Create: `tests/ghpython/test_tree_diagnostics.py`
- Create: `tests/ghpython/test_tree_identity.py`

**Interfaces:**
- `SourceCoordinateEntry(output, sources)`, `SourceCoordinateMap.build`, `BranchDiagnosticMap.build`, `TopologyOutput.build`, `ReducedTopologyOutput.build`.
- `IdentityVerification` (`VERIFIED`, `UNVERIFIABLE`), `TreeContentDigest`, `BranchContentDigest`, `ExactItemCodec[T]`, `StageParameter.text/f64/bool`, `SourceTreeIdentity.build`, `StageTreeIdentity.build`, and `TreeIdentity` as the strict union of the two identity classes.
- Task 2 alone defines and owns `ReducedTopologyOutput`.

- [ ] **Step 1: Write RED diagnostic direction/cardinality tests**

```python
def test_reduction_maps_one_output_to_ordered_nonempty_sources() -> None:
    output = TreeCoordinate.build(BranchCoordinate.build(TreeRootId.build("out"), GhPath.build(4)), ItemIndex.build(0))
    sources = tuple(
        TreeCoordinate.build(BranchCoordinate.build(TreeRootId.build("in"), GhPath.build(4)), ItemIndex.build(index))
        for index in range(3)
    )
    mapping = SourceCoordinateMap.build((SourceCoordinateEntry.build(output, sources),))
    assert mapping.sources_for(output) == sources
    with pytest.raises(DuplicateSourceOutputError):
        SourceCoordinateMap.build((SourceCoordinateEntry.build(output, sources), SourceCoordinateEntry.build(output, sources)))
    with pytest.raises(EmptySourceCoordinateError):
        SourceCoordinateEntry.build(output, ())
```

Test `TopologyOutput` identical topology; `ReducedTopologyOutput` exactly one aggregate value/status per source branch, source-shaped item diagnostics, empty-branch `BranchDiagnosticMap`, complete many-source-to-one-output mapping, and raw bypass. Task 6 consumes these tests and never tests class absence.

- [ ] **Step 2: Write RED identity properties**

```python
@given(root_a=st.text(min_size=1), root_b=st.text(min_size=1))
def test_runtime_root_never_changes_content_digest(root_a: str, root_b: str) -> None:
    assume(root_a.strip() and root_b.strip())
    left = text_tree(TreeRootId.build(root_a.strip()), {(0,): ("a", None), (2,): ()})
    right = text_tree(TreeRootId.build(root_b.strip()), {(0,): ("a", None), (2,): ()})
    assert SourceTreeIdentity.build(left, TEXT_CODEC, TEXT_TREE_SEMANTICS) == SourceTreeIdentity.build(right, TEXT_CODEC, TEXT_TREE_SEMANTICS)


def test_stage_identity_is_provenance_not_native_object_serialization() -> None:
    source = SourceTreeIdentity.build(frame_tree(), FRAME_CODEC, FRAME_TREE_SEMANTICS)
    built = StageTreeIdentity.build(
        source,
        "compas_fab.tesseract.pose_series/v1",
        (StageParameter.f64("metres_per_user_unit", 0.001), StageParameter.text("working_frame", "base_link")),
        source.topology,
    )
    assert built.verification is IdentityVerification.VERIFIED
    assert built.digest != source.digest
```

Add properties proving path/item/null/semantics/order changes alter digests; solve/request generations and roots do not; length-prefix collision resistance; non-finite frame fields fail; raw construction fails; arbitrary native objects cannot enter `SourceTreeIdentity` without an `ExactItemCodec`.

- [ ] **Step 3: Verify RED**

Run: `pixi run pytest tests/ghpython/test_tree_diagnostics.py tests/ghpython/test_tree_identity.py -n auto -q`

Expected: collection fails because both modules are absent.

- [ ] **Step 4: Implement deterministic identities and explicit verification**

`SourceCoordinateEntry` direction is always output coordinate to a non-empty ordered tuple of source coordinates. Output keys are unique. Reduction uses many sources for one output; expansion creates one entry per output with its exact source coordinate or coordinates. Runtime entries retain roots for routing, but their identity encoding contains only each coordinate's canonical path and item index; source and output roots are excluded.

`SourceTreeIdentity` hashes schema, canonical paths, empty branches, null bitmap, item order, port semantics, and exact codec bytes; it excludes root. Provide exact codecs only for source primitives used here: UTF-8 text, strict bool, IEEE-754 finite float, and COMPAS `Frame` as twelve ordered finite IEEE-754 coordinates/axes values. `StageParameter` is a validated tagged field, not caller-supplied digest bytes. `StageTreeIdentity` hashes prior digest, builder schema/version, ordered typed stage parameters, root-free output topology/null status, root-free source-map coordinates, and verification state. A verified stage requires verified prior identity plus a registered deterministic builder schema. External native pose/target trees are `UNVERIFIABLE`, require a fresh compute token, and are never reusable from content cache. No generic native encoder exists.

- [ ] **Step 5: Verify and review**

Run: `pixi run pytest tests/ghpython/test_tree_diagnostics.py tests/ghpython/test_tree_identity.py -n auto -q`

Run: `pixi run pytest --testmon -n auto -q`

Run: `pixi run mypy --strict src/compas_fab/ghpython/tree_diagnostics.py src/compas_fab/ghpython/tree_identity.py`

Run: `pixi run ruff check src/compas_fab/ghpython/tree_diagnostics.py src/compas_fab/ghpython/tree_identity.py tests/ghpython/test_tree_diagnostics.py tests/ghpython/test_tree_identity.py`

Expected: all pass. Dispatch both reviewers. Stop and request explicit commit authorization.

### Task 3: Exact Matching and One Expansion Codec

**Files:**
- Create: `src/compas_fab/ghpython/tree_matching.py`
- Create: `src/compas_fab/ghpython/tree_expansion.py`
- Create: `src/compas_fab/ghpython/tree_expansion_codec.py`
- Create: `tests/ghpython/test_tree_matching.py`
- Create: `tests/ghpython/test_tree_expansion.py`

**Interfaces:**
- `MatchRole` (`EXACT_TREE`, `BROADCASTABLE_SCALAR`, `GLOBAL_ATOMIC`), `MatchInput`, `MatchPolicy`, `MatchedTree`, `match_inputs`.
- `RequestOrdinal.build`, `ResultOrdinal.build`, `CrossProductCoordinate(left, right)`, `ExpandedBranchCoordinate(source_branch, request_ordinal, result_ordinal)`, `ExpansionCoordinate` tagged union, `ExpansionPathCodec.encode/decode`.
- `MaximumExpandedItems`, `CrossProductPolicy`, `cross_product`.

- [ ] **Step 1: Write RED matching properties**

```python
@given(st.lists(st.integers(), min_size=0), st.lists(st.integers(), min_size=0))
def test_equal_branch_zip_never_repeats_or_truncates(left, right) -> None:
    trees = (MatchInput.tree("left", one_branch(left)), MatchInput.tree("right", one_branch(right)))
    if len(left) != len(right):
        with pytest.raises(BranchLengthMismatchError):
            match_inputs(trees, EXACT_PAIR_POLICY)
    else:
        rows = match_inputs(trees, EXACT_PAIR_POLICY).branches[0].rows
        assert [tuple(item.item for item in row.items) for row in rows] == list(zip(left, right))


def test_only_declared_scalar_broadcasts() -> None:
    poses = two_item_tree()
    scalar = MatchInput.scalar("profile", Scalar.build("DEFAULT"))
    assert len(match_inputs((MatchInput.tree("pose", poses), scalar), BROADCAST_POLICY).branches[0].rows) == 2
    with pytest.raises(ImplicitSingletonBroadcastError):
        match_inputs((MatchInput.tree("pose", poses), MatchInput.tree("profile", one_item_tree("DEFAULT"))), EXACT_PAIR_POLICY)
```

Cover ragged independent branches, empty branches, null retention, branch-set mismatch, no tree anchor, raw policy bypass, and canonical output order.

- [ ] **Step 2: Write RED expansion/codec tests**

```python
def test_one_codec_round_trips_cross_product_and_reserved_result_coordinates() -> None:
    cross = CrossProductCoordinate.build(tree_coordinate((1,), 0), tree_coordinate((1, 2), 3))
    result = ExpandedBranchCoordinate.build(branch_coordinate((7, 4)), RequestOrdinal.build(2), ResultOrdinal.build(5))
    assert ExpansionPathCodec.decode(ExpansionPathCodec.encode(cross)) == cross
    assert ExpansionPathCodec.decode(ExpansionPathCodec.encode(result)) == result
    assert ExpansionPathCodec.encode(cross) != ExpansionPathCodec.encode(result)
```

Test prefix-related source paths, malformed tags/lengths, null operands, multiplication bound before allocation, deterministic axis order, and `CrossProductLimitError`.

- [ ] **Step 3: Verify RED**

Run: `pixi run pytest tests/ghpython/test_tree_matching.py tests/ghpython/test_tree_expansion.py -n auto -q`

Expected: collection fails because production modules are absent.

- [ ] **Step 4: Implement exact algorithms**

The first `EXACT_TREE` is the topology anchor. Require exact ordered policy/input names, identical canonical branch sets for other exact trees, and equal lengths per corresponding branch. Broadcast only exact `Scalar.value` from `BROADCASTABLE_SCALAR`. Retain `GLOBAL_ATOMIC` values once per operation without treating them as scalars or matching them by coordinate. Preserve null slots and defer item validation to the consumer.

The expansion codec begins with a coordinate-kind tag, then encodes every path as segment count plus exact segments and ordinals. Decode consumes all segments and rebuilds through factories. Cross-product validates the derived multiplication against `MaximumExpandedItems` before allocating; each output maps to the exact left/right source coordinates. `ExpandedBranchCoordinate` is defined here but used only by Task 8 result expansion.

- [ ] **Step 5: Verify and review**

Run: `pixi run pytest tests/ghpython/test_tree_matching.py tests/ghpython/test_tree_expansion.py -n auto -q`

Run: `pixi run pytest --testmon -n auto -q`

Run: `pixi run mypy --strict src/compas_fab/ghpython/tree_matching.py src/compas_fab/ghpython/tree_expansion.py src/compas_fab/ghpython/tree_expansion_codec.py`

Run: `pixi run ruff check src/compas_fab/ghpython/tree_matching.py src/compas_fab/ghpython/tree_expansion.py src/compas_fab/ghpython/tree_expansion_codec.py tests/ghpython/test_tree_matching.py tests/ghpython/test_tree_expansion.py`

Expected: all pass. Dispatch both reviewers. Stop and request explicit commit authorization.

### Task 4: Canonical Wire Codec and Pending-Rhino Host Emulation

**Files:**
- Create: `src/compas_fab/ghpython/tree_codec.py`
- Create: `tests/ghpython/tree_host_emulation.py`
- Create: `tests/ghpython/test_tree_codec.py`
- Create: `tests/ghpython/test_tree_host_emulation.py`

**Interfaces:**
- `EncodedSlot`, `EncodedBranch`, `EncodedTree`, `TreeValueCodec`, `encode_tree`, `decode_tree`.
- Test-only `EmulatedAccess`, `EmulatedParameter`, `EmulatedInvocation`, `PendingRhinoSolveModel`.

- [ ] **Step 1: Write RED codec properties**

```python
@given(canonical_text_trees())
def test_tree_codec_round_trips_canonical_topology(tree) -> None:
    encoded = encode_tree(tree, UTF8_CODEC)
    assert decode_tree(encoded, UTF8_CODEC) == tree
    assert encode_tree(decode_tree(encoded, UTF8_CODEC), UTF8_CODEC).canonical_json == encoded.canonical_json


def test_decode_rejects_unsorted_host_paths_instead_of_reordering() -> None:
    encoded = encoded_tree(paths=((2,), (1,)))
    with pytest.raises(NonCanonicalTreeOrderError):
        decode_tree(encoded, UTF8_CODEC)
```

Cover prefix paths, nonzero paths, empty branches, null versus empty payload, duplicate paths, wrong schema, malformed base64, and canonical JSON byte equality.

- [ ] **Step 2: Write RED emulation tests**

Model item/list/tree and multiple invocation behavior as pending-Rhino hypotheses only. Test that tree access yields one emulated invocation containing the full tree, item/list access may yield multiple invocations, fixed vectors remain atomic domain items, and no helper flattens a tree. Name every assertion and docstring “emulated” or “pending Rhino”; never use release-evidence wording.

- [ ] **Step 3: Verify RED**

Run: `pixi run pytest tests/ghpython/test_tree_codec.py tests/ghpython/test_tree_host_emulation.py -n auto -q`

Expected: collection fails because production/test-support modules are absent.

- [ ] **Step 4: Implement codec and emulation model**

Schema is `compas_fab.gh_tree/v1`; canonical JSON is sorted-key compact UTF-8; payload bytes use base64. Encode excludes `TreeRootId` from content payload and carries it only in a separate runtime-routing envelope. Decode validates canonical path order before `Tree.build`, rebuilds through factories, re-encodes, and requires byte equality.

The emulation model has no production import and makes no Rhino claim. It records assumed access/invocation behavior so the later self-hosted test can replace assumptions with observed evidence.

- [ ] **Step 5: Verify and review**

Run: `pixi run pytest tests/ghpython/test_tree_codec.py tests/ghpython/test_tree_host_emulation.py -n auto -q`

Run: `pixi run pytest --testmon -n auto -q`

Run: `pixi run mypy --strict src/compas_fab/ghpython/tree_codec.py tests/ghpython/tree_host_emulation.py`

Run: `pixi run ruff check src/compas_fab/ghpython/tree_codec.py tests/ghpython/tree_host_emulation.py tests/ghpython/test_tree_codec.py tests/ghpython/test_tree_host_emulation.py`

Expected: all pass. Dispatch both reviewers. Stop and request explicit commit authorization.

### Task 5: Runtime Generations and Branch Currentness

**Files:**
- Create: `src/compas_fab/ghpython/branch_runtime_identity.py`
- Create: `src/compas_fab/ghpython/branch_current_output.py`
- Create: `tests/ghpython/test_branch_current_output.py`

**Interfaces:**
- `SolveGeneration`, `BranchRequestGeneration`, `BranchRuntimeIdentity`, `TreeRuntimeSnapshot`, `SharedInputsChanged`, `SharedInputsUnchanged`, `advance_runtime`.
- `BranchDecision`, `BranchOutputState[T]`.

- [ ] **Step 1: Write RED dependency-closure tests**

```python
def test_branch_edit_preserves_unchanged_sibling_but_shared_edit_clears_all() -> None:
    first = runtime_snapshot(content_tree("a", "b"), solve=0, requests=(0, 0))
    state = published_state(first, values=("left", "right"))
    local = advance_runtime(first, content_tree("changed", "b"), SharedInputsUnchanged.build())
    assert local.solve_generation == first.solve_generation
    assert state.reconcile(local)[GhPath.build(0)] is BranchDecision.CLEARED
    assert state.current(local.branch(GhPath.build(1))) == "right"
    shared = advance_runtime(local, local.content, SharedInputsChanged.build((CanonicalField.text("scene", "next"),)))
    assert shared.solve_generation == SolveGeneration.build(1)
    assert set(state.reconcile(shared).values()) == {BranchDecision.CLEARED}
```

Test topology changes, null/order edits, empty-branch add/remove, sequence-reduction branch closure, stale late publish/fail/cancel, unknown path, raw snapshots, unchanged idempotence, and root changes that do not change content digest.

- [ ] **Step 2: Verify RED**

Run: `pixi run pytest tests/ghpython/test_branch_current_output.py -n auto -q`

Expected: collection fails because both modules are absent.

- [ ] **Step 3: Implement exact runtime-only identities**

`SharedInputsChanged.build` accepts an exact non-empty tuple of `CanonicalField`; it rejects lists, empty tuples, duplicates, and unvalidated fields. Topology/shared change increments root solve generation and every surviving branch request generation. Same topology plus changed branch digest increments only that branch. `BranchOutputState` keys exact branch coordinate, clears dependency closure immediately, and rejects every stale transition with `StaleBranchGenerationError`.

`TreeRuntimeSnapshot.initial(tree_identity)` creates solve generation zero and request generation zero for every canonical branch. Runtime root changes reroute coordinates but do not alter the supplied content digests.

- [ ] **Step 4: Verify and review**

Run: `pixi run pytest tests/ghpython/test_branch_current_output.py -n auto -q`

Run: `pixi run pytest --testmon -n auto -q`

Run: `pixi run mypy --strict src/compas_fab/ghpython/branch_runtime_identity.py src/compas_fab/ghpython/branch_current_output.py`

Run: `pixi run ruff check src/compas_fab/ghpython/branch_runtime_identity.py src/compas_fab/ghpython/branch_current_output.py tests/ghpython/test_branch_current_output.py`

Expected: all pass. Dispatch both reviewers. Stop and request explicit commit authorization.

### Task 6: Stage-Provenance Native Series and Planning Identity Bridge

**Files:**
- Create: `src/compas_fab/backends/tesseract/scene_identity.py`
- Modify: `src/compas_fab/backends/tesseract/client.py`
- Modify: `src/compas_fab/backends/tesseract/errors.py`
- Modify: `src/compas_fab/backends/tesseract/native_plan.py`
- Modify: `src/compas_fab/backends/tesseract/planner.py`
- Create: `src/compas_fab/ghpython/optional_tree_input.py`
- Create: `src/compas_fab/ghpython/group_shape.py`
- Create: `src/compas_fab/ghpython/tesseract_pose_series.py`
- Create: `src/compas_fab/ghpython/tesseract_cartesian_target_series.py`
- Create: `src/compas_fab/ghpython/tesseract_joint_target_series.py`
- Create: `src/compas_fab/ghpython/tesseract_state_target_series.py`
- Create: `src/compas_fab/ghpython/tesseract_program_series.py`
- Create: `src/compas_fab/ghpython/planning_content_identity.py`
- Create: `tests/ghpython/test_optional_tree_input.py`
- Create: `tests/ghpython/test_group_shape.py`
- Create: `tests/ghpython/test_tesseract_pose_series.py`
- Create: `tests/ghpython/test_tesseract_cartesian_target_series.py`
- Create: `tests/ghpython/test_tesseract_joint_target_series.py`
- Create: `tests/ghpython/test_tesseract_state_target_series.py`
- Create: `tests/ghpython/test_tesseract_program_series.py`
- Create: `tests/ghpython/test_planning_content_identity.py`
- Modify: `tests/backends/tesseract/test_native_plan_component.py`

**Interfaces:**
- `NativeSceneContentIdentity`, `DirectSceneGeneration.build`, public `TesseractPlanner.native_artifact_digest`, and public `TesseractPlanner.native_scene_content_identity`.
- `OptionalTreeInput.absent()` and `OptionalTreeInput.present(tree)` form an immutable tagged union introduced here; absence has no topology, while every present tree retains its empty branches and null slots.
- `PlanningGroupId.build`, `JointId.build`, `DegreesOfFreedom.build`, `GroupShape.build(group_id, ordered_joint_ids)`, and `GroupShapeFactory.from_native_robot(native_robot, group_id)`; `GroupShape` owns the group ID, exact ordered joint IDs, and derived DOF.
- `PoseSeriesBuild`, `CartesianTargetSeriesBuild`, `JointTargetSeriesBuild`, `StateTargetSeriesBuild`, `ProgramSeriesBuild`; each contains exact values/output surfaces plus `StageTreeIdentity`.
- `build_pose_series`, `build_cartesian_target_series`, `build_joint_target_series`, `build_state_target_series`, and `build_motion_program_series` are the only public native-series factories. Joint/state factories accept a `GroupShape`, group-compatible position vectors, optional joint-name vectors, and state optional dynamics/time through `OptionalTreeInput`; omitted names remain exact native `None`.
- `ProfileGeneration.build`, `ComputeToken.build`, `UnverifiableProfileIdentity.build`, `PlanningSharedInputs.build`, `PlanningTreeIdentity.build`, `BranchPlanAttemptId`, `BranchPlanAttempt.build`, `BranchPlanRequest.build`, and `build_branch_plan_requests`.

- [ ] **Step 1: Write RED native-series behavior tests**

```python
def test_exact_series_builds_one_program_per_branch_with_provenance(native_robot) -> None:
    poses = build_pose_series(frame_source(), pose_parameters())
    targets = build_cartesian_target_series(poses, target_parameters())
    programs = build_motion_program_series(native_robot, targets, program_parameters())
    assert [branch.path for branch in programs.output.values.branches] == [GhPath.build(0), GhPath.build(4, 1)]
    assert all(len(branch.items) == 1 for branch in programs.output.values.branches)
    first_target = targets.output.values.branches[0].items[0].item
    assert first_target.pose is poses.output.values.branches[0].items[0].item
    assert programs.output.values.branches[0].items[0].item.motion_program.targets[0] is first_target
    assert programs.identity.verification is IdentityVerification.VERIFIED


def test_full_native_authoring_identity_ignores_routing_root(native_robot) -> None:
    left = build_program_pipeline(native_robot, source_root=TreeRootId.build("document-a"))
    right = build_program_pipeline(native_robot, source_root=TreeRootId.build("document-b"))
    assert left.identity == right.identity
    assert left.output.source_coordinates.root_free_identity_bytes() == right.output.source_coordinates.root_free_identity_bytes()
```

Test typed scalar broadcast, equal move/profile zip, unequal mismatch, ragged branches, null item invalidating only its program branch, empty ordered branch diagnostics, one-item convenience, exact native types/object retention, source-map completeness, raw bypass, and externally supplied native values becoming `UNVERIFIABLE`. `GroupShape.build` rejects empty/duplicate/invalid joint IDs; `GroupShapeFactory.from_native_robot` retains the exact native group order and derives `DegreesOfFreedom == len(ordered_joint_ids)`. Build joint targets from group-tagged fixed position vectors plus optional group-compatible name vectors, and state targets from positions plus optional names/velocity/acceleration/time. Prove a fixed vector is one atomic tree item, wrong group tags/lengths fail before native work, absent JointTarget names produce exact native `names=None`, and absent StateTarget optional fields remain exact native absence. A present optional tree containing a null slot remains topology-bearing invalid input at that exact coordinate rather than becoming absence. `ReducedTopologyOutput` behavior is tested directly, never its absence.

- [ ] **Step 2: Write RED exact bridge tests**

```python
def test_program_tree_and_shared_native_inputs_bridge_to_branch_requests(shared_planner, exact_profiles) -> None:
    programs = verified_program_series(shared_planner)
    shared = PlanningSharedInputs.build(
        shared_planner,
        shared_planner.native_artifact_digest,
        shared_planner.native_scene_content_identity,
        "DescartesFPipeline",
        UnverifiableProfileIdentity.build(ProfileGeneration.build(3)),
        exact_profiles,
        True,
        ComputeToken.build("solve-7"),
    )
    planning_identity = PlanningTreeIdentity.build(programs, shared)
    runtime = TreeRuntimeSnapshot.initial(planning_identity.tree_identity)
    requests = build_branch_plan_requests(programs, shared, planning_identity, runtime)
    assert all(request.call.planner is shared_planner for request in requests)
    assert all(request.content_identity == planning_identity.branch(request.identity.coordinate.path) for request in requests)
    assert all(request.compute_token == ComputeToken.build("solve-7") for request in requests)
    assert all(request.attempt.runtime_identity == request.identity for request in requests)
    assert all(request.attempt.compute_token == request.compute_token for request in requests)
    assert planning_identity.verification is IdentityVerification.UNVERIFIABLE


def test_new_token_changes_attempt_not_content_identity(shared_request_inputs) -> None:
    first = build_request(shared_request_inputs, ComputeToken.build("attempt-a"))
    repeated = build_request(shared_request_inputs, ComputeToken.build("attempt-a"))
    second = build_request(shared_request_inputs, ComputeToken.build("attempt-b"))
    assert first.attempt == repeated.attempt
    assert first.attempt.id != second.attempt.id
    assert first.content_identity == second.content_identity
```

Test that every exact `ProfileDictionary` is `UNVERIFIABLE`, requires explicit generation plus fresh compute token, and never caches; no caller can forge a verified profile wrapper. Program branch identity uses existing `native_program_digest`; result identity binds exact request identity; artifact/scene/pipeline/profile-generation/auto-seed changes alter planning identity; runtime routing roots and solve/request generations do not. A projection-derived scene identity is verified; direct native scene mutation increments `DirectSceneGeneration`, makes scene verification `UNVERIFIABLE`, requires a fresh token, and prevents reuse.

Add native-call forced-interleaving tests in `test_native_plan_component.py`: mutate queued program bytes, planner scene revision, profile object identity, pipeline, and auto-seed before execution and assert `NativePlanInputsChangedBeforeExecutionError` plus zero planner calls. Mutate program or scene revision while a controlled native call is active and assert `NativePlanInputsChangedDuringExecutionError`; the returned native result is discarded. Assert unchanged calls execute once. Explicitly assert internal `ProfileDictionary` content is not inspected because 0.35.0.6 exposes no canonical serialization; it remains `UNVERIFIABLE` rather than falsely checked.

- [ ] **Step 3: Verify RED**

Run: `pixi run pytest tests/ghpython/test_optional_tree_input.py tests/ghpython/test_group_shape.py tests/ghpython/test_tesseract_pose_series.py tests/ghpython/test_tesseract_cartesian_target_series.py tests/ghpython/test_tesseract_joint_target_series.py tests/ghpython/test_tesseract_state_target_series.py tests/ghpython/test_tesseract_program_series.py tests/ghpython/test_planning_content_identity.py tests/backends/tesseract/test_native_plan_component.py -n auto -q`

Expected: collection fails because new modules/properties are absent.

- [ ] **Step 4: Implement exact provenance chain**

`NativeSceneContentIdentity` hashes artifact `BuildIdentity.digest` plus the existing canonical robot-cell/state projection bytes. Projection-derived identities are `VERIFIED`. `_mark_native_scene_changed` cannot derive direct command content from the current revision, so it increments `DirectSceneGeneration`, publishes an `UNVERIFIABLE` scene identity, and never claims revision alone is content. Expose the complete identity read-only through client/planner. Pose identity derives frame-source digest plus pose builder schema/version, scale, working frame, and root-free output topology. Cartesian-target identity derives pose identity plus move/profile source identities and exact match/item policies. Joint/state-target identities derive the exact `GroupShape` group ID, ordered joint IDs, DOF, complete ordered fixed-vector values, optional-name/dynamics/time presence tags, move/profile sources, output topology, and source maps; absent and present-null optional inputs cannot share identity. Program identity derives target identity plus group/TCP/working/profile parameters, root-free source map, and every exact `native_program_digest`; mismatch between provenance and native digest fails.

`PlanningTreeIdentity` derives program identity plus artifact digest, complete scene identity including verification/direct generation, pipeline, unverifiable profile generation, auto-seed, and planning schema/version. Because `ProfileDictionary` has no sealed canonical binding in 0.35.0.6, every baseline planning identity is `UNVERIFIABLE`; a verified profile path is deferred. Unverified scene/profile inputs require a fresh compute token and are never reusable. The identity exposes `branch(path)` while retaining exact shared planner/profile objects outside the digest. `ComputeToken` is runtime edge evidence and never enters content identity. `BranchPlanAttempt.build(runtime_identity, compute_token)` derives `BranchPlanAttemptId` from the complete runtime identity plus token; same inputs are idempotent, while a new token creates a distinct attempt without changing content identity. Task 6 owns frozen `BranchPlanRequest(attempt, content_identity, call)` and exposes its runtime identity/token through the attempt. `build_branch_plan_requests` is its only batch factory: it verifies matching tree/branch content identities and runtime snapshot before building exact `NativePlanCall`s. A result record retains its request content identity; no arbitrary `PlanningResult` serialization is attempted.

`NativePlanCall.validate_inputs_before_execution` recomputes the observable `NativePlanSignature` from current program digest, planner scene revision, profile object identity, pipeline, and auto-seed. A mismatch raises `NativePlanInputsChangedBeforeExecutionError` and performs no native call. `execute` validates immediately before `plan_native`, then recomputes after return; a post-call mismatch raises `NativePlanInputsChangedDuringExecutionError` and discards the result. Internal `ProfileDictionary` content is intentionally not claimed observable.

- [ ] **Step 5: Verify and review**

Run: `pixi run pytest tests/ghpython/test_optional_tree_input.py tests/ghpython/test_group_shape.py tests/ghpython/test_tesseract_pose_series.py tests/ghpython/test_tesseract_cartesian_target_series.py tests/ghpython/test_tesseract_joint_target_series.py tests/ghpython/test_tesseract_state_target_series.py tests/ghpython/test_tesseract_program_series.py tests/ghpython/test_planning_content_identity.py tests/backends/tesseract/test_native_plan_component.py -n auto -q`

Run: `pixi run pytest --testmon -n auto -q`

Run: `pixi run mypy --strict src/compas_fab/backends/tesseract/scene_identity.py src/compas_fab/backends/tesseract/client.py src/compas_fab/backends/tesseract/native_plan.py src/compas_fab/backends/tesseract/planner.py src/compas_fab/ghpython/optional_tree_input.py src/compas_fab/ghpython/group_shape.py src/compas_fab/ghpython/tesseract_pose_series.py src/compas_fab/ghpython/tesseract_cartesian_target_series.py src/compas_fab/ghpython/tesseract_joint_target_series.py src/compas_fab/ghpython/tesseract_state_target_series.py src/compas_fab/ghpython/tesseract_program_series.py src/compas_fab/ghpython/planning_content_identity.py`

Run: `pixi run ruff check src/compas_fab/backends/tesseract/scene_identity.py src/compas_fab/backends/tesseract/client.py src/compas_fab/backends/tesseract/errors.py src/compas_fab/backends/tesseract/native_plan.py src/compas_fab/backends/tesseract/planner.py src/compas_fab/ghpython/optional_tree_input.py src/compas_fab/ghpython/group_shape.py src/compas_fab/ghpython/tesseract_pose_series.py src/compas_fab/ghpython/tesseract_cartesian_target_series.py src/compas_fab/ghpython/tesseract_joint_target_series.py src/compas_fab/ghpython/tesseract_state_target_series.py src/compas_fab/ghpython/tesseract_program_series.py src/compas_fab/ghpython/planning_content_identity.py tests/ghpython/test_optional_tree_input.py tests/ghpython/test_group_shape.py tests/ghpython/test_tesseract_pose_series.py tests/ghpython/test_tesseract_cartesian_target_series.py tests/ghpython/test_tesseract_joint_target_series.py tests/ghpython/test_tesseract_state_target_series.py tests/ghpython/test_tesseract_program_series.py tests/ghpython/test_planning_content_identity.py tests/backends/tesseract/test_native_plan_component.py`

Expected: all pass. Dispatch both reviewers. Stop and request explicit commit authorization.

### Task 7: Truthful Branch Planning with Native Execution Capacity

**Files:**
- Create: `src/compas_fab/ghpython/branch_planning_state.py`
- Create: `src/compas_fab/ghpython/branch_planning.py`
- Create: `tests/ghpython/test_branch_planning_state.py`
- Create: `tests/ghpython/test_branch_planning.py`

**Interfaces:**
- Consumes Task 6 `BranchPlanRequest`; produces `IdentifiedPlanningResult(value, attempt_id, producing_runtime_identity, request_content_identity)`, `BatchPublicationOutcome` (`PUBLISHABLE`, `WITHHELD`), `BranchPlanStatus`, `BranchPlanRecord`, `BranchPlanSnapshot`, and pure `reduce_branch_plan`.
- `PlanningQueueCapacity.build`, `NativeExecutionCapacity.build(configured_shared_runtime_capacity)`, `NativeExecutionLeasePool`, `BranchPlanningOwner.build/submit/cancel/snapshot/close`.
- Consumes `BatchPublicationPolicy` only from Task 1; Task 7 never redeclares it.

- [ ] **Step 1: Write RED truthful-publication tests**

```python
def test_fail_batch_withholds_without_rewriting_truth() -> None:
    snapshot = completed_batch(policy=BatchPublicationPolicy.FAIL_BATCH, outcomes=(failed("left"), succeeded("right")))
    assert snapshot.records[0].status is BranchPlanStatus.FAILED
    assert snapshot.records[1].status is BranchPlanStatus.SUCCEEDED
    assert snapshot.records[1].result.value.native_result.message == "right"
    assert snapshot.publication_outcome is BatchPublicationOutcome.WITHHELD
```

Test independent publication, queued/running/cancel-requested/cancelled transitions, stale completion discard, exact request/result identity retention, illegal transition no-effect, malformed raw records, and batch outcome becoming publishable only under the declared policy.

- [ ] **Step 2: Write RED serialized shared-runtime tests**

```python
def test_shared_runtime_serializes_calls_and_each_call_gets_distinct_native_clone(instrumented_client) -> None:
    planner = TesseractPlanner(instrumented_client)
    owner = planning_owner(
        execution_capacity=NativeExecutionCapacity.build(1),
        queue_capacity=PlanningQueueCapacity.build(2),
    )
    owner.submit(two_requests_using_same_planner(planner), BatchPublicationPolicy.PUBLISH_INDEPENDENT)
    complete_all_with_controlled_executor(owner)
    assert instrumented_client.maximum_composer_concurrency == 1
    assert len({id(robot) for robot in instrumented_client.returned_robots}) == 2
    owner.close()


def test_capacity_above_shared_runtime_baseline_fails_named() -> None:
    with pytest.raises(UnsupportedConcurrentNativePlanningError):
        NativeExecutionCapacity.build(2)
```

Test deterministic branch queue order, nonblocking submit, overflow before partial admission, failure releasing the single lease, same planner accepted for every queued branch, and no concurrent pipeline claim. Close marks the owner closed, cancels queued records, discards late active completion, and rejects new submit. The active native call may finish; a test-only helper waits for executor termination. No thread kill/cancellation claim is permitted. Concurrent native runtime pools and concurrent TrajOpt are explicitly outside this tranche.

Add forced interleavings: queue two attempts, mutate the second program or scene before its lease, and assert named pre-execution failure with no second native call. Mutate scene revision during the active controlled call and assert named during-execution failure with no published result. Submit the same attempt twice and assert one record/dispatch; submit the same content/runtime identity with a fresh compute token and assert a distinct attempt executes sequentially.

- [ ] **Step 3: Verify RED**

Run: `pixi run pytest tests/ghpython/test_branch_planning_state.py tests/ghpython/test_branch_planning.py -n auto -q`

Expected: collection fails because both modules are absent.

- [ ] **Step 4: Implement reducer and owner**

Branch status always reports what happened to that branch. On checked success, the reducer creates `IdentifiedPlanningResult(value, attempt_id, producing_runtime_identity, request_content_identity)` only from the exact completing `BranchPlanRequest` and exact result. `FAIL_BATCH` changes only `BatchPublicationOutcome` to `WITHHELD`; it never changes a successful record/result to failed. `PUBLISH_INDEPENDENT` exposes every terminal branch independently. Attempt ID is the admission/idempotency key: reusing one compute token for the same runtime request never dispatches twice; intentional repetition with a new token creates a distinct attempt while content identity remains equal.

`NativeExecutionCapacity.build` accepts exactly one because the current planner shares one runtime/composer; any larger exact integer raises `UnsupportedConcurrentNativePlanningError`. `NativeExecutionLeasePool` wraps a one-permit bounded semaphore. `PlanningQueueCapacity` is a separate explicit positive bound. One worker consumes branches in canonical request order, acquires the lease, calls `NativePlanCall.validate_inputs_before_execution` at the lease boundary, then calls checked `NativePlanCall.execute`, and releases the lease in `finally`. The shared `TesseractPlanner.plan_native` remains unchanged and clones internally for each sequential call. Named before/during input-change errors become truthful failed records; the reducer publishes success only from an execute return that passed post-validation. Submit validates attempt/content/currentness, enqueues without blocking, and returns an immutable snapshot. `close` is explicit/idempotent: mark closed, cancel queued records, reject new work, and discard completion from an already-active call after it finishes. It does not kill a thread or native call; there is no document lifecycle claim or registry.

- [ ] **Step 5: Verify and review**

Run: `pixi run pytest tests/ghpython/test_branch_planning_state.py tests/ghpython/test_branch_planning.py -n auto -q`

Run: `pixi run pytest --testmon -n auto -q`

Run: `pixi run mypy --strict src/compas_fab/ghpython/branch_planning_state.py src/compas_fab/ghpython/branch_planning.py`

Run: `pixi run ruff check src/compas_fab/ghpython/branch_planning_state.py src/compas_fab/ghpython/branch_planning.py tests/ghpython/test_branch_planning_state.py tests/ghpython/test_branch_planning.py`

Expected: all pass. Dispatch both reviewers. Stop and request explicit commit authorization.

### Task 8: Pure Native Result and Sample Expansion

**Files:**
- Create: `src/compas_fab/ghpython/tesseract_result_series.py`
- Create: `tests/ghpython/test_tesseract_result_series.py`

**Interfaces:**
- Consumes Task 7 `IdentifiedPlanningResult`; produces `ResultContentIdentity(request_content_identity, result_ordinal)`, `ResultRuntimeIdentity(attempt_id, result_ordinal)`, `TrajectorySample`, `ResultSeriesInspection`, and `inspect_result_series`.
- Uses Task 3 `ExpandedBranchCoordinate` and `ExpansionPathCodec`; no second expansion codec.

- [ ] **Step 1: Write RED pure-inspection tests**

```python
def test_result_expansion_preserves_exact_objects_absence_and_sources(native_results) -> None:
    identified = identified_result_tree(native_results)
    inspection = inspect_result_series(identified)
    assert inspection.results.branches[0].items[0].item is native_results[0]
    view = TesseractNativeResultView.build(native_results[0])
    assert inspection.samples.branches[0].items[0].item.native_point is view.trajectory_points[0]
    assert inspection.samples.branches[0].items[0].item.velocity is None
    coordinate = ExpansionPathCodec.decode(inspection.samples.branches[0].path)
    assert isinstance(coordinate, ExpandedBranchCoordinate)
    assert inspection.source_coordinates.sources_for(tree_coordinate_from_sample(inspection, 0, 0)) == (tree_coordinate_from_result(inspection, 0),)
```

Test exact request/native result/raw program/message retention and distinct content/runtime result identities inherited from `IdentifiedPlanningResult`. Two intentional executions with equal content/runtime branch identity but fresh compute tokens must yield equal `ResultContentIdentity` and unequal `ResultRuntimeIdentity`; different producing runtime generations also differ. Reject a wrapper whose attempt, request content, or producing runtime identity differs from its successful branch record. Cover fixed-vector joint names/position, optional velocity/acceleration/time remaining absent, prefix/nonzero paths, multiple results per source branch, empty/failed branches, output status, source maps, and raw bypass. Make no viewport, decimation, geometry, scene, or preview assertion.

- [ ] **Step 2: Verify RED**

Run: `pixi run pytest tests/ghpython/test_tesseract_result_series.py -n auto -q`

Expected: collection fails because the module is absent.

- [ ] **Step 3: Implement pure expansion**

Build `TesseractNativeResultView` once per exact result. `ExpandedBranchCoordinate` combines source branch plus request/result ordinals; sample items remain ordered beneath its encoded path. `TrajectorySample` retains exact native point, typed fixed vectors, optional dynamics/time, both result identities, attempt ID, producing runtime identity, and exact source result coordinate. `ResultContentIdentity` hashes request content plus result ordinal and excludes runtime generation/root. `ResultRuntimeIdentity` binds `BranchPlanAttemptId` plus result ordinal, so a fresh compute token remains distinguishable downstream even when content and branch runtime identity are unchanged. Neither serializes arbitrary native results.

- [ ] **Step 4: Verify and review**

Run: `pixi run pytest tests/ghpython/test_tesseract_result_series.py -n auto -q`

Run: `pixi run pytest --testmon -n auto -q`

Run: `pixi run mypy --strict src/compas_fab/ghpython/tesseract_result_series.py`

Run: `pixi run ruff check src/compas_fab/ghpython/tesseract_result_series.py tests/ghpython/test_tesseract_result_series.py`

Expected: all pass. Dispatch both reviewers. Stop and request explicit commit authorization.

### Task 9: Seven Additive Pure Tree Nodes and Truthful Documentation

**Files:**
- Create: `src/compas_fab/ghpython/grasshopper_tree_adapter.py`
- Create: `src/compas_fab/ghpython/coordinate_labels.py`
- Create: `src/compas_fab/ghpython/cross_product_projection.py`
- Modify: `src/compas_fab/ghpython/tree_errors.py`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractPoseSeries/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractPoseProfileCrossProduct/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractCartesianTargetSeries/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractJointTargetSeries/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractStateTargetSeries/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractMotionProgramSeries/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractResultInspectSeries/{code.py,metadata.json,icon.svg,icon.png}`
- Create: `tests/ghpython/test_tesseract_series_components.py`
- Create: `tests/ghpython/test_coordinate_labels.py`
- Create: `tests/ghpython/test_cross_product_projection.py`
- Create: `docs/frontends/ghpython-tesseract-series.md`
- Create: `docs/frontends/examples/tesseract-series-tree.json`
- Modify: `docs/frontends/ghpython.md`
- Modify: `mkdocs.yml`
- Modify: `pyproject.toml`
- Test: `tests/backends/tesseract/test_documented_examples.py`

**Exact node count:** seven. No planning-job owner registry, async component, cancel component, scrub, preview, or bake node belongs to this foundation tranche.

**Adapter interfaces:** `HostInputPresence.build(source_count, persistent_data_count)`, `capture_tree(host_tree, root_id, input_name) -> Tree[object]`, `capture_optional_tree(host_tree, root_id, input_name, presence) -> OptionalTreeInput[object]`, `capture_fixed_vector_branches(host_tree, root_id, input_name, group_shape, quantity_shape) -> FixedVectorBranchCapture[object]`, `capture_scalar_branches(host_tree, root_id, input_name, quantity_shape) -> ScalarBranchCapture[object]`, `emit_tree(tree, output_name) -> Grasshopper.DataTree[object]`, and `audit_emitted_tree(expected, emitted, output_name) -> None`. `HostInputPresence` accepts exact non-negative integer counts and is absent only when both counts are zero; any source or persistent data makes the input present. `FixedVectorBranchCapture` carries one group-tagged atomic vector item per ordinary ordered scalar branch; `ScalarBranchCapture` carries one scalar item from each exact one-item branch. Both carry parallel status, source-shaped diagnostics, branch diagnostics, and source-coordinate maps. `HostInvocationId.build(component_instance, iteration)` guards invocation identity. Only `emit_tree` constructs host trees; only `audit_emitted_tree` reads emitted trees for post-build verification.

**Additive projection interfaces:** `PoseProfileCrossProductProjection.build(result, policy, pose_identity, profile_identity, pose_root, profile_root)`, `CoordinateLabelSubject` (`BRANCH`, `ITEM`), `CoordinateLabelState` (`EMPTY`, `VALUE`, `NULL`), `CoordinateLabelContext.build(subject, state, coordinate)`, and `CoordinateLabelPolicy.format(context)`. Projection consumes the committed `CrossProductResult.pairs` and `source_coordinates` unchanged; it does not add fields, change ordering, or replace Task 3 factories. The existing left-pose/right-profile pair order and output names are authoritative; projection identity contains the two input identities, exact existing pair coordinate order, aligned output topology, source map, and `CrossProductPolicy.maximum_expanded_items`.

**Exact port contract:** every input uses `scriptParamAccess: 2` (tree access), including settings. The current componentizer does not apply output `scriptParamAccess`; output metadata therefore asserts names/order only. Each script constructs and returns explicit Grasshopper `DataTree`/tree structures through the adapter, with emitted-structure tests kept pending real-Rhino confirmation. Every component rejects `ghenv.Iteration != 0` with `UnexpectedHostIterationError` before domain work.

| Node | Inputs in exact order | Defaults and decoding | Outputs in exact order |
|---|---|---|---|
| `Tesseract Pose Series` | `frames`, `metres_per_user_unit`, `working_frame` | frames required tree; scale required `ScalarFromTree[float]`; working frame one-item tree default `base_link` | `poses`, `status`, `item_diagnostics`, `branch_diagnostics`, `identity` |
| `Tesseract Pose Profile Cross Product` | `poses`, `profiles`, `maximum_results` | exact left-pose and right-profile trees required; maximum required positive `ScalarFromTree[int]`; pair order is canonical left pose/right profile | `poses_expanded`, `profiles_expanded`, `status`, `item_diagnostics`, `branch_diagnostics`, `source_coordinates`, `identity` |
| `Tesseract Cartesian Target Series` | `poses`, `move_types`, `move_type_match`, `profiles`, `profile_match` | poses required tree; move types default one-item `FREESPACE`; profiles default one-item `DEFAULT`; match tokens default `BROADCASTABLE_SCALAR`, accept exact `BROADCASTABLE_SCALAR` or `EXACT_TREE` | `targets`, `status`, `item_diagnostics`, `branch_diagnostics`, `source_coordinates`, `identity` |
| `Tesseract Joint Target Series` | `native_robot`, `group_name`, `positions`, `joint_names`, `move_types`, `move_type_match`, `profiles`, `profile_match` | robot/group resolve exact `GroupShape`; each ordinary numeric positions branch becomes one atomic group-tagged vector item; joint-name branches are optional and reduce identically when present; absent names remain native `None`; move/profile defaults and match tokens equal Cartesian Target Series | `targets`, `status`, `item_diagnostics`, `branch_diagnostics`, `source_coordinates`, `identity` |
| `Tesseract State Target Series` | `native_robot`, `group_name`, `positions`, `joint_names`, `velocities`, `accelerations`, `times`, `move_types`, `move_type_match`, `profiles`, `profile_match` | robot/group resolve exact `GroupShape`; each ordinary numeric/name branch reduces to one atomic group-tagged vector item; disconnected names/dynamics/time are absent; any source or persistent data makes the tree present, including null slots; move/profile defaults and match tokens equal Cartesian Target Series | `targets`, `status`, `item_diagnostics`, `branch_diagnostics`, `source_coordinates`, `identity` |
| `Tesseract Motion Program Series` | `native_robot`, `targets`, `group_name`, `tcp_selection`, `working_frame`, `profile` | robot/group are required one-item trees; targets ordered tree; TCP selection defaults to exact token `AUTO` or accepts exact link text and records the choice in provenance; working/profile default `base_link`/`DEFAULT` | `motion_programs`, `programs`, `joint_names`, `tcp_frames`, `status`, `item_diagnostics`, `branch_diagnostics`, `source_coordinates`, `identity` |
| `Tesseract Result Inspect Series` | `identified_results` | exact `IdentifiedPlanningResult` tree from the Python planning foundation; no defaults or compute token | `requests`, `native_results`, `raw_programs`, `messages`, `trajectory_points`, `joint_names`, `positions`, `velocities`, `accelerations`, `times`, `status`, `item_diagnostics`, `branch_diagnostics`, `source_coordinates` |

Host tree access is only transport. Each input independently declares domain meaning:

| Node input | `TopologyRole` | `BranchSemantics` | `ItemShape` | `MatchRole` |
|---|---|---|---|---|
| Pose `frames` | `TREE` | `ELEMENTWISE` | `DOMAIN_ATOMIC[CompasFrame]` | `EXACT_TREE` |
| Pose `metres_per_user_unit` | `ATOMIC` | `ELEMENTWISE` | `SCALAR[MetersPerUserUnit]` | `BROADCASTABLE_SCALAR` |
| Pose `working_frame` | `ATOMIC` | `ELEMENTWISE` | `SCALAR[WorkingFrameName]` | `BROADCASTABLE_SCALAR` |
| Cross Product `poses` | `TREE` | `ELEMENTWISE` | `DOMAIN_ATOMIC[WorkingFramePose]` | explicit product left operand |
| Cross Product `profiles` | `TREE` | `ELEMENTWISE` | `SCALAR[ProfileName]` | explicit product right operand |
| Cross Product `maximum_results` | `ATOMIC` | `ELEMENTWISE` | `SCALAR[MaximumExpandedItems]` | `GLOBAL_ATOMIC` |
| Cartesian `poses` | `TREE` | `ELEMENTWISE` | `DOMAIN_ATOMIC[WorkingFramePose]` | `EXACT_TREE` |
| Cartesian `move_types` | token-selected `ATOMIC` or `TREE` | `ELEMENTWISE` | `SCALAR[MoveType]` | token-selected `BROADCASTABLE_SCALAR` or `EXACT_TREE` |
| Cartesian `move_type_match` | `ATOMIC` | `ELEMENTWISE` | `SCALAR[MatchRole]` | `GLOBAL_ATOMIC` |
| Cartesian `profiles` | token-selected `ATOMIC` or `TREE` | `ELEMENTWISE` | `SCALAR[ProfileName]` | token-selected `BROADCASTABLE_SCALAR` or `EXACT_TREE` |
| Cartesian `profile_match` | `ATOMIC` | `ELEMENTWISE` | `SCALAR[MatchRole]` | `GLOBAL_ATOMIC` |
| Joint/State `native_robot` | `ATOMIC` | `ELEMENTWISE` | `DOMAIN_ATOMIC[NativeRobot]` | `GLOBAL_ATOMIC` via `AtomicFromTree` |
| Joint/State `group_name` | `ATOMIC` | `ELEMENTWISE` | `SCALAR[PlanningGroupId]` | `GLOBAL_ATOMIC` |
| Joint raw `positions` | `TREE` | `ORDERED_SEQUENCE` | `SCALAR[NativeJointPosition]` | branch-reduction anchor |
| Joint connected raw `joint_names` | `TREE` | `ORDERED_SEQUENCE` | `SCALAR[JointId]` | exact branch set before reduction |
| Joint move/profile values and match tokens | token-selected `ATOMIC` or `TREE` | `ELEMENTWISE` | typed scalars | token-selected broadcast/exact or `GLOBAL_ATOMIC` |
| State raw `positions` | `TREE` | `ORDERED_SEQUENCE` | `SCALAR[NativeJointPosition]` | branch-reduction anchor |
| State connected raw `joint_names` | `TREE` | `ORDERED_SEQUENCE` | `SCALAR[JointId]` | exact branch set before reduction |
| State connected raw `velocities` | `TREE` | `ORDERED_SEQUENCE` | `SCALAR[NativeJointVelocity]` | exact branch set before reduction |
| State connected raw `accelerations` | `TREE` | `ORDERED_SEQUENCE` | `SCALAR[NativeJointAcceleration]` | exact branch set before reduction |
| State connected raw `times` | `TREE` | `ORDERED_SEQUENCE` | `SCALAR[NativeTime]` | exact one-item branch before reduction |
| State move/profile values and match tokens | token-selected `ATOMIC` or `TREE` | `ELEMENTWISE` | typed scalars | token-selected broadcast/exact or `GLOBAL_ATOMIC` |
| Program `native_robot` | `ATOMIC` | `ELEMENTWISE` | `DOMAIN_ATOMIC[NativeRobot]` | `GLOBAL_ATOMIC` via `AtomicFromTree` |
| Program `targets` | `TREE` | `ORDERED_SEQUENCE` | `DOMAIN_ATOMIC[NativeTarget]` | `EXACT_TREE` |
| Program `group_name` | `ATOMIC` | `ELEMENTWISE` | `SCALAR[GroupName]` | `GLOBAL_ATOMIC` |
| Program `tcp_selection` | `ATOMIC` | `ELEMENTWISE` | `SCALAR[TcpSelection]` | `GLOBAL_ATOMIC` |
| Program `working_frame` | `ATOMIC` | `ELEMENTWISE` | `SCALAR[WorkingFrameName]` | `GLOBAL_ATOMIC` |
| Program `profile` | `ATOMIC` | `ELEMENTWISE` | `SCALAR[ProfileName]` | `GLOBAL_ATOMIC` |
| Inspect `identified_results` | `TREE` | `ELEMENTWISE` | `DOMAIN_ATOMIC[IdentifiedPlanningResult]` | `EXACT_TREE` |

Adapter reduction is a separate typed transformation, not a reinterpretation of the raw ports:

| Normalized value | `TopologyRole` | `BranchSemantics` | `ItemShape` | Cardinality |
|---|---|---|---|---|
| Joint/State positions | `TREE` | `ELEMENTWISE` | `FIXED_VECTOR[JointPositions, GroupShape]` | one item per nonempty source branch |
| Present Joint/State names | `TREE` | `ELEMENTWISE` | `FIXED_VECTOR[JointNames, GroupShape]` | one item per nonempty source branch |
| Present State velocities | `TREE` | `ELEMENTWISE` | `FIXED_VECTOR[JointVelocities, GroupShape]` | one item per nonempty source branch |
| Present State accelerations | `TREE` | `ELEMENTWISE` | `FIXED_VECTOR[JointAccelerations, GroupShape]` | one item per nonempty source branch |
| Present State time | `TREE` | `ELEMENTWISE` | `SCALAR[NativeTime]` | one item per exact one-item source branch |

There is no compute token on pure nodes. Planner, pipeline, profiles, auto-seed, compute token, and publication policy route only through the Task 6/7 Python planning foundation, not these seven components. A future async Grasshopper node must expose them only after Windows lifecycle proof.

- [ ] **Step 1: Write RED adapter/metadata/component tests**

Test exact node count, directory names, input port order and tree access, every input's exact topology/branch/item/match declaration from both tables, output name order without output-access metadata assertions, defaults, exact nanobind directive, category `COMPAS FAB`, subcategories `Tesseract · Author` for the six authoring/expansion nodes and `Tesseract · Inspect` for result inspection, no hidden planner/profile routing, and no async registry import. Execute each component once with `ghenv.Iteration == 0`; assert nonzero iteration raises/reports `UnexpectedHostIterationError` and emits absent outputs. Prove only scalar-setting ports accept one-item trees through `ScalarFromTree`; series ports retain full topology. Assert every raw Joint/State vector-valued numeric/name port is `TREE + ORDERED_SEQUENCE + SCALAR[quantity]`, then prove adapter reduction yields `TREE + ELEMENTWISE + FIXED_VECTOR[GroupShape]` with exactly one vector item per nonempty source branch, the same path, exact branch-local DOF, and a many-source-to-one coordinate map. State time reduces separately from an exact one-item raw scalar branch to one normalized scalar item. Retain `native_robot` through `AtomicFromTree`. Test `HostInputPresence` for `(0, 0)`, sources only, persistent data only, and both; only `(0, 0)` yields absence, while a present tree containing null remains present. JointTarget names absent must reach the exact native target as `None`. The compiled cross-product node must expose two aligned expanded trees from canonical left-pose/right-profile pairs, require an explicit finite bound, and reject over-limit input before allocation. Existing scalar component files and reference tests remain byte-unchanged.

The fake host uses exact paths `{1}`, `{1;2}`, and `{7;0}`, an empty branch, and explicit nulls at nonzero item indices. Assert capture retains host path order/segments and item indices; emission calls `EnsurePath` once per branch before any branch insert and indexed `Insert` once per slot including `None`; independent readback matches exact paths/counts/null positions. Inject path, count, and null-position corruption and require `GrasshopperTreeSelfAuditError` with no output. Inject capture/emission faults and require their distinct named errors. Static compiled-node inspection asserts every input is tree access and no host data-matching API is called; actual matching-mode immunity remains a mandatory follow-on Rhino acceptance test.

Build the existing `CrossProductResult`, snapshot its `pairs` and `source_coordinates`, then project it. Assert the projection leaves both snapshots equal, creates pose/profile trees with identical one-item branches in exact `pair.path` order, retains each canonical left-pose/right-profile value/null slot, reuses the exact source-coordinate entries, and emits one parallel status item plus source-shaped item diagnostics for every pair. Valid zero-pair products emit aligned empty trees and empty diagnostic maps. Identity contains input identities, policy bound, pair-coordinate order, aligned topology, and source map; changing runtime roots does not change it. Feed both projected trees directly to Cartesian Target Series under `EXACT_TREE`; no implicit product exists and no axis-name port/type exists.

Test `CoordinateLabelContext` exhaustively: `BRANCH+EMPTY+BranchCoordinate`, `ITEM+VALUE+TreeCoordinate`, and `ITEM+NULL+TreeCoordinate` are the only valid combinations. Every other subject/state/coordinate combination raises a named label-context error. Labels retain unsimplified coordinates, never mutate native values, and cannot enter codecs, stage parameters, or content identity.

- [ ] **Step 2: Verify RED**

Run: `pixi run pytest tests/ghpython/test_coordinate_labels.py tests/ghpython/test_cross_product_projection.py tests/ghpython/test_tesseract_series_components.py tests/backends/tesseract/test_documented_examples.py -n auto -q`

Expected: fail because adapter, seven nodes, and documented fixture are absent.

- [ ] **Step 3: Implement thin pure adapters**

`HostInvocationId.build(component_instance, iteration)` validates exact IDs and iteration. Add `UnexpectedHostIterationError`, `GrasshopperTreeCaptureError`, `GrasshopperTreeEmissionError`, `GrasshopperTreeSelfAuditError`, `InvalidFixedVectorBranchError`, and `InvalidCoordinateLabelContextError` to `tree_errors.py`. Each `RunScript` first requires iteration zero, captures `tree.Paths` in exact host order without sorting, copies every exact integer `GH_Path.Indices` segment, walks every branch by exact item index, and records every `None` as `TreeItem.null`. `HostInputPresence.build(SourceCount, PersistentDataCount)` is the sole optional-presence authority: absence requires both exact counts to be zero; any source or persistent datum calls `OptionalTreeInput.present(capture_tree(...))`, even for a null/empty tree. Capture rejects duplicate/noncanonical paths or an unobservable host slot with `GrasshopperTreeCaptureError` before domain work.

Raw Joint/State vector-valued ports enter `capture_fixed_vector_branches` as `TREE + ORDERED_SEQUENCE + SCALAR[quantity]`. The adapter treats each ordinary Grasshopper branch as one vector coordinate at the same path, validates every scalar/name slot, requires exact `GroupShape.dof`, and emits normalized `TREE + ELEMENTWISE + FIXED_VECTOR[GroupShape]` with one vector item per nonempty source branch. It maps that vector coordinate to every original scalar coordinate in order and retains source-shaped diagnostics. An empty branch or null component cannot become a shorter vector; it emits exact branch/item diagnostics and no native target. `capture_scalar_branches` separately requires exactly one valid time item and emits one normalized scalar item at the same path. Joint/State nodes build their `GroupShape` from exact `native_robot` plus `group_name` before converting positions and any present names/dynamics branches. Optional names absent pass `None` unchanged to `joint_target_from_native`/`state_target_from_native`.

Emission constructs one `Grasshopper.DataTree[object]`, calls `EnsurePath(exact_path)` for every branch including empty branches, then calls indexed `Insert(value_or_None, exact_path, item_index)` for every slot including explicit nulls. It catches only named expected host failures and raises `GrasshopperTreeEmissionError` with output name and coordinate. Before returning, it reads the built structure back through an independent audit path and requires exact ordered path segments, branch count, item counts, and null positions; mismatch raises `GrasshopperTreeSelfAuditError` and publishes no partial tree. The adapter never invokes host item/list matching, path simplification, generic nested-list conversion, or append-only emission. The fake sink proves the algorithm only; identical actual-Rhino capture/emission proof remains mandatory.

`PoseProfileCrossProductProjection.build` is a one-way projection over the committed `CrossProductResult`: iterate `result.pairs` once in existing canonical left-pose/right-profile order, create one `poses_expanded` branch and one `profiles_expanded` branch at each exact `pair.path`, retain left/right value or null state, build parallel status and diagnostics, and require the projected source map to equal `result.source_coordinates`. It never reconstructs pairs or calls a second product implementation. `CoordinateLabelContext` factories enforce the subject/state/coordinate matrix before `CoordinateLabelPolicy` formats exact unsimplified coordinates; neither type is admitted to identity codecs or native values.

Create seven independent SVGs using the existing branch motif. Add these exact private Pixi tasks and ordered no-command wrapper to `pyproject.toml`; do not add CairoSVG or change `pixi.lock`:

```toml
_render-gh-pose-series-icon = { cmd = "rsvg-convert -w 24 -h 24 src/compas_fab/ghpython/components_cpython/Cf_TesseractPoseSeries/icon.svg -o src/compas_fab/ghpython/components_cpython/Cf_TesseractPoseSeries/icon.png" }
_render-gh-pose-profile-cross-product-icon = { cmd = "rsvg-convert -w 24 -h 24 src/compas_fab/ghpython/components_cpython/Cf_TesseractPoseProfileCrossProduct/icon.svg -o src/compas_fab/ghpython/components_cpython/Cf_TesseractPoseProfileCrossProduct/icon.png" }
_render-gh-target-series-icon = { cmd = "rsvg-convert -w 24 -h 24 src/compas_fab/ghpython/components_cpython/Cf_TesseractCartesianTargetSeries/icon.svg -o src/compas_fab/ghpython/components_cpython/Cf_TesseractCartesianTargetSeries/icon.png" }
_render-gh-joint-target-series-icon = { cmd = "rsvg-convert -w 24 -h 24 src/compas_fab/ghpython/components_cpython/Cf_TesseractJointTargetSeries/icon.svg -o src/compas_fab/ghpython/components_cpython/Cf_TesseractJointTargetSeries/icon.png" }
_render-gh-state-target-series-icon = { cmd = "rsvg-convert -w 24 -h 24 src/compas_fab/ghpython/components_cpython/Cf_TesseractStateTargetSeries/icon.svg -o src/compas_fab/ghpython/components_cpython/Cf_TesseractStateTargetSeries/icon.png" }
_render-gh-program-series-icon = { cmd = "rsvg-convert -w 24 -h 24 src/compas_fab/ghpython/components_cpython/Cf_TesseractMotionProgramSeries/icon.svg -o src/compas_fab/ghpython/components_cpython/Cf_TesseractMotionProgramSeries/icon.png" }
_render-gh-result-series-icon = { cmd = "rsvg-convert -w 24 -h 24 src/compas_fab/ghpython/components_cpython/Cf_TesseractResultInspectSeries/icon.svg -o src/compas_fab/ghpython/components_cpython/Cf_TesseractResultInspectSeries/icon.png" }
render-gh-series-icons = { depends-on = ["_render-gh-pose-series-icon", "_render-gh-pose-profile-cross-product-icon", "_render-gh-target-series-icon", "_render-gh-joint-target-series-icon", "_render-gh-state-target-series-icon", "_render-gh-program-series-icon", "_render-gh-result-series-icon"], description = "Render series component icons" }
```

Run: `pixi run render-gh-series-icons`

Expected: exactly seven 24×24 PNGs.

- [ ] **Step 4: Document only proven scope**

The page documents exact tree semantics, scalar-from-tree settings, matching, display-only coordinate labels, provenance verification, pure-node ports, errors, and the host-emulation limitation. The JSON fixture covers canonical prefix/nonzero paths, an empty branch, null slot, equal zip, unequal mismatch evidence, explicit bounded cross-product coordinates, group-typed joint/state fixed vectors, optional dynamics absence versus a present null slot, and pure result expansion. It schedules the exact seven-node host-runtime tranche and its acceptance gates but does not claim compiled `.ghuser`, packaged GHX behavior, observed Rhino equivalence, async planning node, lifecycle cleanup, viewport or bake behavior, progressive ports/menu, undo, or save/reopen.

- [ ] **Step 5: Run release gates**

Run: `pixi run pytest tests/ghpython tests/backends/tesseract -n auto -q`

Run: `pixi run pytest --testmon -n auto -q`

Run: `pixi run mypy --strict src/compas_fab/ghpython/tree_errors.py src/compas_fab/ghpython/tree_coordinates.py src/compas_fab/ghpython/tree_values.py src/compas_fab/ghpython/port_semantics.py src/compas_fab/ghpython/tree_diagnostics.py src/compas_fab/ghpython/coordinate_labels.py src/compas_fab/ghpython/optional_tree_input.py src/compas_fab/ghpython/group_shape.py src/compas_fab/ghpython/tree_identity.py src/compas_fab/ghpython/tree_matching.py src/compas_fab/ghpython/tree_expansion.py src/compas_fab/ghpython/tree_expansion_codec.py src/compas_fab/ghpython/cross_product_projection.py src/compas_fab/ghpython/tree_codec.py src/compas_fab/ghpython/branch_runtime_identity.py src/compas_fab/ghpython/branch_current_output.py src/compas_fab/ghpython/tesseract_pose_series.py src/compas_fab/ghpython/tesseract_cartesian_target_series.py src/compas_fab/ghpython/tesseract_joint_target_series.py src/compas_fab/ghpython/tesseract_state_target_series.py src/compas_fab/ghpython/tesseract_program_series.py src/compas_fab/ghpython/planning_content_identity.py src/compas_fab/ghpython/branch_planning_state.py src/compas_fab/ghpython/branch_planning.py src/compas_fab/ghpython/tesseract_result_series.py src/compas_fab/ghpython/grasshopper_tree_adapter.py src/compas_fab/backends/tesseract/scene_identity.py src/compas_fab/backends/tesseract/client.py src/compas_fab/backends/tesseract/native_plan.py src/compas_fab/backends/tesseract/planner.py`

Run: `pixi run ruff check src/compas_fab/ghpython src/compas_fab/backends/tesseract/scene_identity.py tests/ghpython`

Run: `pixi run mkdocs build --strict`

Run: `pixi run pytest -n auto -q`

Run: `git diff --check`

Expected: all exit zero, no reference test changes, and no new skip/xfail.

- [ ] **Step 6: Review and stop for authorization**

Fresh specification review checks every exact port and all audit invariants. Fresh quality review checks strict typing, factory/raw safety, content/runtime separation, truthful result/batch states, shared-planner clone capacity, no generic native serialization, no native weakening, and truthful platform claims. Stop before `git add` or `git commit`; request explicit user authorization for the reviewed task changes.

## Plan Self-Review

- Task 2 alone owns `ReducedTopologyOutput`; Task 6 tests behavior.
- `CrossProductCoordinate` and result-only `ExpandedBranchCoordinate` share one tagged codec.
- Task 9 projects the existing immutable canonical left-pose/right-profile `CrossProductResult.pairs` into explicitly named aligned output trees without modifying Task 3; no axis-name input/type exists, and coordinate labels remain excluded from identity/codecs.
- Shared planners are valid; baseline execution capacity is exactly one, calls are serialized deterministically, and each sequential `plan_native` call clones internally.
- `FAIL_BATCH` withholds publication without falsifying branch status/result.
- Identity is executable source codec plus typed stage provenance; program digest is native; results bind requests.
- `NativePlanCall` rejects observable pre-execution drift before native work and discards results after during-execution drift; opaque profile contents stay explicitly unobservable.
- The bridge is explicit: program tree and shared planner/scene/pipeline/profile/auto-seed identity → `TreeIdentity` → runtime snapshot → `BranchPlanRequest`.
- Attempt identity binds runtime identity plus compute token: repeated tokens are idempotent, fresh tokens distinguish intentional executions without changing content identity, and runtime result identity carries the attempt downstream.
- Seven pure tree-access nodes have exact ports; nonzero host iteration fails named; existing scalar nodes stay untouched.
- The exact seven-node host-runtime tranche owns async Grasshopper exposure, current/full-series scrub, preview, separate bake, lifecycle cleanup, progressive ports/menu, componentizer, and real-Rhino proof; all remain deferred without product claims here.
- Host emission uses `EnsurePath` then indexed `Insert` for every slot and self-audits exact paths/counts/null positions before publication; fake-host success never substitutes for Rhino evidence.
- Task 6 owns `GroupShape` and optional tree presence; Task 9 alone owns host presence derivation, ordinary-branch fixed-vector reduction, coordinate-label subject/state, and aligned product projection.
- Optional names preserve exact native `None`; any host source or persistent data makes an optional tree present, including present-null.
- `TreeRootId` is routing-only; canonical host order is validated; source-map direction/cardinality is explicit.
- Hypothesis properties cover topology, matching, codec, and identity. Every task runs testmon.
- Hypothesis and the existing `rsvg-convert` provider are direct locked Pixi dependencies; icon rendering adds tasks but no second renderer.
- Every commit waits for explicit authorization and every command is unchained.
