# Grasshopper, Tesseract, and ABB Product Design

**Status:** master product contract for implementation tranches
**Date:** 2026-07-11

## 1. Product contract and provenance

This product makes Rhino 8 Grasshopper a primary surface for fixed-base robotic fabrication: exact Tesseract resources, native scene, authoring, profiles, planning, contacts, diagnostics, result projection, preview, native RAPID, hardened ABB Robot Web Services (RWS), and a separate advanced binary authored-playback backend.

Two independent release gates apply:

1. **Ergonomics:** canonical work uses the fewest wires/actions compatible with explicit robot-safety decisions; generated selectors, readable summaries, local diagnostics, responsive solves, progressive ports, docs, and installed examples are mandatory.
2. **Engineering:** frames, units, groups, tasks, controllers, identities, lifetimes, concurrency, state, errors, reproducibility, and downstream contracts are explicit and mechanically enforced.

A tranche fails if either gate fails. Convenience never weakens native Tesseract, and rigor never excuses a hostile Grasshopper workflow.

The design derives only from the current COMPAS FAB repository/specifications; the installed public API of `tesseract-robotics-nanobind==0.35.0.6`; the public Apache-2.0 contracts of `abb-robot-client==0.5.0` and `abb-motion-program-exec==0.8.0`; public ABB RWS session, subscription, file, RAPID task, program-load, execution, state, event-log, and I/O concepts; and this product brief. Component names follow Section 7; icons extend COMPAS FAB's existing language; manifests are defined here. No external component catalog, asset set, graph, identifier registry, or serialization format is an input.

## 2. Binding invariants

### 2.1 Native-first

- Exact native `Robot`, `Pose`, `CartesianTarget`, `JointTarget`, `StateTarget`, `MotionProgram`, `CompositeInstruction`, `ProfileDictionary`, `PlanningResult`, environment `Command`, `ContactResult`, and `ContactResultMap` remain available to Grasshopper/Python consumers.
- COMPAS values enter and leave only at named, typed boundaries. A projection retains its exact source and a machine-readable loss report; it never becomes the authority for native inspection.
- Every released native option is exposed, accepted as an exact native override, or remains absent so its released default applies. No smaller COMPAS option set replaces it.
- Native RAPID consumes the authored `CompositeInstruction`, never a dense planned result or COMPAS trajectory.
- Native scene commands remain the ceiling: convenience nodes validate common tool/body/attachment/touch-link work without narrowing the exact command path.

### 2.2 Grasshopper behavior

- Pure nodes solve synchronously. Resource I/O, native initialization/planning/collision, controller work, deployment, and playback run outside Grasshopper UI/solve threads; those threads only validate, enqueue without blocking, read immutable snapshots, schedule a future solve, and draw prepared geometry.
- Every output matches the complete observable input identity or is absent. Changed inputs, failed recomputation, cancellation, queue saturation, disconnect, or conflict clear stale output immediately.
- Remote output carries controller/session generation, observation revision, source/receive time, freshness, and reconciliation state. Mutation requires an edge-triggered unique token; recompute cannot repeat it.
- Optional ports are progressive: canonical ports start visible; one menu action shows native controls; show/hide/reset is undoable; order, values, and wires persist through GHX serialization.
- Selectors are generated from exact upstream identities between solves. User selectors are not overwritten. A vanished choice remains visibly invalid and raises a local named error; the first item is never silently chosen.
- Each node emits the exact typed value plus a concise panel-readable summary. Diagnostics identify the local port/program index/link and an actionable next step.

### 2.3 Series, trees, and matching

- Grasshopper topology is observable product data. `Tree[T]` retains an ordered tuple of `TreeBranch[T]`; each branch retains its exact `GhPath`, item order, empty-branch presence, and explicit null slots. `BranchCoordinate` is `(TreeRootId, GhPath)` and remains valid for an empty branch; `TreeCoordinate` adds `ItemIndex` for an existing or null item slot. No adapter may drop nulls, collapse empty branches, simplify paths, flatten, renumber, or infer a different topology.
- The Grasshopper adapter captures the host's exact ordered `GH_Path` sequence, every integer path segment, branch count, item index, and explicit `None`/null position. Emission creates every path with `EnsurePath` before inserting every slot at its exact index with indexed `Insert`, including explicit `None`/null slots. It then reads the emitted structure back and requires exact equality of ordered paths, branch counts, item counts, and null positions before publication. Capture raises `GrasshopperTreeCaptureError`, host construction raises `GrasshopperTreeEmissionError`, and post-build mismatch raises `GrasshopperTreeSelfAuditError`; host-independent emulation is insufficient evidence without the same round trip on actual Rhino.
- Port meaning is orthogonal. `TopologyRole` is `ATOMIC` or `TREE`; `BranchSemantics` is `ELEMENTWISE`, `ORDERED_SEQUENCE`, or `BATCH`; `ItemShape` is `SCALAR`, `FIXED_VECTOR[ShapeTag]`, or `DOMAIN_ATOMIC`. Every port declares all three. Thus a ragged tree of ordered fixed-vector joint items is `TREE + ORDERED_SEQUENCE + FIXED_VECTOR[GroupShape]`; a native profile dictionary is `ATOMIC + ELEMENTWISE + DOMAIN_ATOMIC`. Grasshopper item/list/tree access is an encoding choice and never substitutes for these declarations.
- Every multi-input node declares a typed `MatchPolicy`. Only a port explicitly declared `BROADCASTABLE_SCALAR` may broadcast one `Scalar[T]` over the other inputs' coordinates. A one-item series/tree is not a scalar. Non-scalar inputs require identical branch sets and equal lengths within each matched branch; ragged lengths across different branches are valid. Default matching never repeats last/first items, truncates, skips nulls, zips across branches, or performs Grasshopper longest-list behavior.
- Cartesian products require an explicit pure `CrossProductPolicy` with a derived finite result bound and deterministic output paths formed from both source coordinates. Canonical pair order is always left pose then right profile for the visible pose/profile projection; output port names make those axes observable without extra inputs. Cross-products are forbidden for controller mutations, deployment, reset/start/stop, I/O writes, and binary execution.
- Policies are also orthogonal. `ItemValidationPolicy` governs invalid/null elementwise items, `SequenceReductionPolicy` governs branch-to-aggregate reductions, and `BatchPublicationPolicy` governs whether independent async results may publish when siblings fail. Ordered program construction uses `SequenceReductionPolicy.REQUIRE_ALL_VALID`: one invalid/null target invalidates the branch and emits no program because a native program cannot contain a null instruction.
- Pure elementwise authoring maps each valid input coordinate to the same output coordinate. Ordered-sequence consumers map one input branch to one output program at the original `GhPath` and emit a `SourceCoordinateMap` from every program/instruction/diagnostic index back to the exact source coordinate. An empty ordered sequence records `EmptyOrderedSeriesError` in a typed `BranchDiagnosticMap` keyed by its `BranchCoordinate`; its source-shaped item diagnostic branch remains empty because no item coordinate exists. It never disappears or produces an empty native program silently.
- Every topology-bearing output has a parallel `Tree[ItemStatus]` with identical paths and item counts. When a branch reduction maps an ordered input series to one aggregate output, that aggregate has one parallel output-status item while a separate `Tree[Diagnostic]` retains the original input paths, item counts, null slots, and source coordinates. Component summaries aggregate counts without replacing either status surface. Exact native values inside tree items retain object identity.
- `CoordinateLabelPolicy` produces deterministic panel/preview labels only from a valid typed subject/state context: branch+empty, item+value, or item+null. The first uses `BranchCoordinate`; the latter two require `TreeCoordinate`. Labels are display-only: they never simplify paths, mutate exact native object names, enter a codec, `BuildIdentity`, or a content digest, or replace the typed source-coordinate map.

### 2.4 Controller separation and safety

- **RWS observe** owns reads/subscriptions; **RWS control** owns upload/readback/load/reset/start/program-stop/I/O writes; **binary playback** owns its backend-native authored program/interpreter flow.
- Native RAPID source, dense Tesseract results, COMPAS projections, and binary bytes are non-interchangeable types with separate identities, nodes, icons, owners, and docs.
- Deploy never stops, resets, starts, changes motor state/mode, or writes unrelated I/O. Reset/start/I/O writes require arm then issue; RAPID program-stop and binary stop are separate immediate unarmed scalar-only safety-priority actions and are explicitly normal controller stops, not safety-rated or emergency stops.
- Controller configuration installation, safety-system configuration, and automatic motor-state changes are out of scope.
- Controller mutation nodes do not inherit pure-node lifting. Armed mutation nodes accept exactly one command intent, one arm, and one invocation token by default. Multiple armed mutations require an explicit ordered `ControllerCommandBatch` with a derived finite bound, per-item arm/precondition, sequential journal admission, and per-item receipt/reconciliation; recompute and tree matching can never manufacture a batch. RAPID program-stop and binary stop remain separate immediate, non-armed scalar-only actions and both are forbidden inside every ordered batch.
- Ordered mutation batches stop on the first dispatched, precondition, verification, indeterminate, or reconciliation failure. `CONTINUE_INDEPENDENT` is valid only after a proven pre-dispatch failure with no effect, or where a formal resource-independence proof plus unchanged reconciled preconditions exists for every later item. An indeterminate item always blocks the remainder; no later mutation may dispatch until reconciliation resolves it.

## 3. Scope and terminal completeness

Terminal completeness is a single installed fixed-base definition that:

1. loads content-addressed URDF/SRDF/resources and initializes native runtime;
2. creates native tools/rigid bodies, attachments, touch links, and scene commands;
3. authors exact poses/Cartesian/joint/state targets and programs;
4. selects an available native pipeline and builds exact profiles;
5. plans asynchronously, exposes exact diagnostics/contacts/results, and explicitly projects when needed;
6. scrubs/previews locally and emits native RAPID from the authored program;
7. connects/observes an ABB controller and deploys, verifies, loads, resets, starts, monitors, program-stops, reads/writes I/O, and reconciles state through RWS;
8. independently authors and executes a supported fixed-base binary program against its preinstalled controller interpreter.

The installed local definition is series-first: ordered frame/pose/target branches produce ordered programs. It proves typed scalar broadcast, equal branch-local paired-series zip and unequal-length mismatch rejection, ragged independent branches, an explicit bounded pure cross-product, one-branch async failure, and topology-preserving diagnostics/preview. A one-item ordered series is the concise convenience case, not a scalar or separate architecture.

Included: fixed base, one controller, one motion task; released 0.35.0.6 profile factories; RW6/RW7 RWS profiles; fake/recorded/RobotStudio tests; physical RW6 acceptance; macOS ARM64 Python 3.12 development; Rhino 8 Windows CPython 3.9 product builds.

Deferred: mobile bases, root-changing external axes, VKC, MultiMove, concurrent TrajOpt, EGM, lossy dense-result-to-RAPID/binary conversion, and any current physical RW7 validation claim. RW7 release claims stop at fake, recorded, and RobotStudio evidence until a supervised physical record exists.

## 4. Canonical workflow task matrix

This matrix comes from user intent, before component design, and is the interaction acceptance oracle. Counts exclude optional expert/diagnostic wires but include safety actions.

| Task | Authoritative input/output | Explicit decision/action budget | Canonical wire budget |
|---|---|---:|---:|
| Load robot | sources/plugins → artifact, digest, summary | one build token on source change | artifact to runtime: 1 |
| Add/attach scene object | topology-preserving geometry, parent, transform, touch links → exact command branches/batch | one authoring action per tree; explicit batch policy | object to batch: 1 |
| Author Cartesian move | framed SI-scaled pose tree, scalar/type/profile match policy → exact target tree/status | one target action | pose→target→program: 2 |
| Author joint/state move | group-typed fixed vectors/ordered series, optional dynamics → exact target tree/status | one target action | target→program: 1 |
| Compose programs | ordered target branches → one exact program per branch + source-coordinate map | one composition action | targets→program: 1 |
| Choose planning | runtime pipeline registry, exact profiles → identities/summaries | one selector choice | scene/program/profiles→job: 3 |
| Plan/inspect | exact request tree → isolated jobs/results/parallel diagnostics with retained topology | one compute action; explicit batch failure policy | job→inspect: 1 |
| Project/preview | exact result tree + explicit policy → topology-preserving projection/report/sample/display | one projection choice; slider only thereafter | result→project/scrub→preview: 2 |
| Generate RAPID | authored program + profiles → exact source/bundle identity | one pure generation | program/profiles→generate: 2 |
| Observe controller | endpoint/session/resources → reconciled snapshot | one connect and one subscribe | session→observe: 1 |
| Deploy RAPID | one sealed bundle + session/task → verified loaded deployment; explicit batch node only | arm then deploy per item: 2 actions | bundle/session→deploy: 2 |
| Reset/start | ready deployment + reconciled state → execution handle | arm+reset; arm+start | deployment/session→command: 2 |
| Immediate stop | live RAPID session/task or binary execution → acknowledged observed stop | one press; never armed/batched | handle→stop: 1 |
| I/O | signal spec/session → sample or verified write receipt | read: 0; write: arm+issue | session/spec→I/O: 2 |
| Binary playback | exact backend program + compatible session → execution/log | arm then execute | artifact/session→execute: 2 |

## 5. Architecture, types, and ownership

### 5.1 Selected architecture

```text
Grasshopper UI/solve: pure adapters, intents, immutable snapshots, viewport draw
                     |
              document handle registry (routing only)
             /                              \
local native owners                    process controller registry
artifact/scene/planning/contact         one ControllerOwner per ControllerId
isolated robot clones                   document leases; one async session/journal
```

Selected: exact domain cores plus asynchronous owners. Rejected: a universal workflow facade (god object/lifetime mixing), direct component-to-controller calls (races/recompute side effects/UI stalls), and one normalized trajectory model (native loss).

The process-wide `ControllerLeaseRegistry` guarantees one `ControllerOwner` per `ControllerId`; documents hold leases, and the last release closes the session. Mutation ownership is exclusive per task. The registry routes leases only. It does not absorb session, subscription, command, deployment, execution, or reconciliation state.

One production file owns one responsibility. Values with different lifetimes, consumers, or change rates live in different files; no runtime, planner, controller, or component object may accumulate another domain's state. Public protocols declare each owner/adapter contract, and private helpers cannot leak through package imports.

### 5.2 Typed immutable boundaries

Strict mypy is a release gate. Non-interchangeable `NewType` values cover `ControllerId`, `SessionId`, `SessionGeneration`, `SubscriptionId`, `ObservationRevision`, `CommandId`, `CommandSequence`, `DeploymentId`, `ExecutionId`, `DocumentId`, `ComponentInstanceId`, `RobotArtifactId`, `SceneRevision`, `GroupId`, `TaskId`, `MechanicalUnitId`, `FrameId`, `LinkId`, `JointId`, `SignalId`, `ProfileId`, `PipelineId`, `CredentialHandle`, `TreeRootId`, `TreeContentDigest`, `BranchContentDigest`, `SolveGeneration`, `BranchRequestGeneration`, `ItemIndex`, `RequestOrdinal`, `ResultOrdinal`, `MetersPerUserUnit`, `Seconds`, `Radians`, and `Meters`.

`TaggedFrame[FrameTag, UnitTag]` distinguishes user/world/base/working/tool/controller frames and metres/user units. `JointVector[GroupTag, QuantityTag]` retains group, ordered joint identities, and position/velocity/acceleration/effort meaning. Conversion consumes one exact tag and returns another; bare floats/strings do not cross subsystem boundaries.

`Scalar[T]`, `FixedVector[T, ShapeTag]`, `TreeBranch[T]`, `Tree[T]`, `OptionalTreeInput[T]`, `HostInputPresence`, `TreeTopology`, `BranchCoordinate`, `TreeCoordinate`, `ExpandedBranchCoordinate`, `GroupShape`, `BranchDiagnosticMap`, `MatchPolicy`, `SourceCoordinateMap`, `CoordinateLabelSubject`, `CoordinateLabelState`, `CoordinateLabelContext`, `CoordinateLabelPolicy`, `TreeContentDigest`, `BranchContentDigest`, `SolveGeneration`, `BranchRequestGeneration`, `ItemValidationPolicy`, `SequenceReductionPolicy`, and `BatchPublicationPolicy` are immutable typed boundaries rather than lists/dicts with conventions. `Tree.build(...)` validates unique exact paths, branch order, item/null counts, and root identity. `HostInputPresence` is absent only when both Grasshopper source count and persistent-data count are zero; any source or persistent datum creates `OptionalTreeInput.present(...)`, so a connected/persistent tree containing empty branches or null slots never becomes absence. Matching returns a typed `MatchedTree[...]` or raises a named topology error before domain work begins. `BranchDiagnosticMap.build(...)` validates unique branch coordinates and contains only diagnostics that cannot truthfully occupy an item slot.

`GroupShape` owns `PlanningGroupId`, the exact ordered tuple of `JointId`, and derived positive `DegreesOfFreedom`. `GroupShape.build(...)` validates non-empty unique joint IDs; `GroupShapeFactory.from_native_robot(...)` is the host factory and retains the native group's exact joint order. Raw Joint/State position, name, velocity, and acceleration ports are `TREE + ORDERED_SEQUENCE + SCALAR[quantity]` at the Grasshopper boundary. Adapter reduction produces normalized `TREE + ELEMENTWISE + FIXED_VECTOR[GroupShape]` with exactly one atomic vector item at each nonempty source branch's same path, a many-source-to-one-vector coordinate map, and source-shaped diagnostics. Raw time is an ordered scalar branch and reduces only when it contains exactly one valid item. Empty, null-containing, wrong-length, or wrong-group branches fail before native target construction; no branch is shortened.

Boundary values are frozen/slotted `attrs` classes. Public `Class.build(...)` factories validate exact types, finite numbers, shape/order, identities, cross-field invariants, and immutable copies. `__attrs_post_init__` repeats structural invariants so raw construction cannot bypass safety. Package `__init__.py` files remain minimal and define no `__all__` registry.

### 5.3 Content identity and manifests

Every artifact contains `BuildIdentity(schema_name, schema_version, compas_fab_version, producer_version, native_versions, ordered_input_digests, payload_digest)`. SHA-256 consumes length-prefixed canonical UTF-8 fields and exact payload bytes. Ordered programs/joints/commands retain order; declared sets sort by typed identity. `TreeContentDigest` and `BranchContentDigest` hash content only: ordered paths, empty branches, item/null bitmap, item order, all three port declarations, match/cross-product/policy values, exact item payload identities, and source-coordinate maps. Runtime `SolveGeneration` and `BranchRequestGeneration` never enter `BuildIdentity` or a content digest. Canonical JSON is UTF-8 with sorted keys and no insignificant whitespace; controller paths use forward slashes.

Schemas are `compas_fab.{robot_artifact,native_scene,native_program,planning_job,trajectory_projection,rapid_program,rapid_deployment,binary_program,command_receipt,recorded_rws_session}/v1`. Each has JSON Schema, `json.loads`/validate/re-encode/digest tests, and downstream round trips. Manifests exclude secrets, auth data, sessions, object addresses, wall time, cache roots, and nondeterministic paths.

### 5.4 Lifetimes and backpressure

- artifact owns exact source bytes; environment owner owns materialization/runtime; scene application clones baseline and returns a new scene revision; a planning worker exclusively owns its native clone; preview owns immutable sampled frames/display data;
- controller owner exclusively owns credentials, async RWS client, WebSocket, reducer, durable journal writer, reconciliation, and task-mode lease; binary playback is a task-exclusive mode, not another client;
- document close cancels queued local jobs, requests running cancellation, releases controller leases/subscriptions, discards late-generation callbacks, and preserves redacted receipts. The last lease closes the controller.
- async batch owners isolate every `(SolveGeneration, BranchContentDigest, BranchRequestGeneration)`; bounded concurrency is derived from clone/session capacity, never branch count alone. One branch cannot overwrite another branch's snapshot, failure, cancellation, or retry state.
- component runtime state is keyed by `ComponentInstanceId`, `SolveGeneration`, `TreeRootId`, `GhPath`, `BranchContentDigest`, and `BranchRequestGeneration`; positional sticky slots and iteration-blind component keys are forbidden because Grasshopper may invoke one component repeatedly across branches/iterations in one solution.

All queues use immutable `BackpressurePolicy`; no capacity/delay is an inline literal. Observation capacity is `ceil(measured_peak_rate_per_source × source_count × maximum_UI_drain_interval × measurement_uncertainty_factor)`, with calibration inputs serialized. Level state coalesces by exact resource key while retaining first/last revision/count; event-log edges, acknowledgements, faults, and transitions never coalesce. Edge overflow enters `GAP`, raises `ObservationGapError`, and requires full reconciliation. Local capacity derives from isolated clone count and job policy; superseded queued branch requests cancel independently, running results never masquerade as a newer `(content digest, solve/request generation)`, and saturation raises `PlanningBackpressureError` with the affected coordinates.

`PerformancePolicy` records runner-specific measured p99 UI enqueue/snapshot cost, native progress availability, request/handshake distributions, event cadence, controller connection limits, recovery distribution, display/materialization cost, confidence interval, and uncertainty factor. From these it derives UI budgets, request deadline, staleness, bounded reconnect schedule, arm validity, and deterministic preview decimation. CI stores calibration evidence and checks regression against the accepted same-runner baseline. Product code contains no arbitrary sleep, poll, reconnect, stale, timeout, arm-expiry, or tolerance literal.

## 6. State, journal, race, and replay contracts

Each machine is a pure validated reducer returning frozen next state plus declared effects. Illegal transitions raise named errors and issue no effect.

```text
Session: DISCONNECTED→CONNECTING→CONNECTED→CLOSING→CLOSED
                     ↘FAULTED   ↘DEGRADED→RECONNECTING→CONNECTED|FAULTED
Subscription: UNSUBSCRIBED→SUBSCRIBING→LIVE→CANCELLING→CLOSED
                              ↘FAULTED ↘GAP→RESYNCING→LIVE|FAULTED
Command: CREATED→VALIDATED→QUEUED→DISPATCHING→ACKNOWLEDGED→VERIFYING→SUCCEEDED|FAILED
             ↘FAILED   ↘CANCELLED       ↘INDETERMINATE
                                      ↘CANCEL_REQUESTED→CANCELLED|ACKNOWLEDGED|INDETERMINATE
Deployment: DRAFT→SEALED→STAGING→STAGED→VERIFYING→VERIFIED→LOADING→LOADED→READY
                         any effect state → FAILED; cleanup is a separate command
Execution: IDLE→PREFLIGHT→ARMED→STARTING→RUNNING→COMPLETING→COMPLETED
                    ↘FAILED ↘EXPIRED ↘INDETERMINATE ↘STOP_REQUESTED→STOPPING→STOPPED|FAULTED
Reconcile: UNRECONCILED→READING→CONSISTENT; CONSISTENT→STALE→READING; READING→CONFLICT|FAILED;
           CONFLICT→READING only through explicit adopt-observed or discard-local resolution
```

RW version is explicit before connect; probing is a separate async command. Reconnect increments `SessionGeneration`; old handles cannot mutate. Reconnect enters `GAP`, reads all subscribed levels plus event-log watermark, then atomically publishes a reconciled snapshot.

Queued cancellation proves no effect. After dispatch the command enters `CANCEL_REQUESTED` and then a proven terminal state; lost transport yields `INDETERMINATE`, never guessed success/failure or blind replay. Deployment seals bytes/task/path/versions, stages to a content-derived path, reads back/hash-verifies, then loads only into a reconciled stopped task. `ARMED` binds intent digest, controller, generation, task, observation revision, and derived validity; any change expires it.

The owner assigns increasing `CommandSequence`; durable append is admission's linearization point. `CommandId` hashes schema/version, intent identity, controller/generation/task or signal, expected revision, and user-issued `InvocationNonce`. Same ID returns the same handle/receipt; intentional repetition needs a new nonce, which recompute cannot create. Journal records are canonical, append-only, and redacted.

Replay: never-dispatched commands may resume only with unchanged preconditions; observations reconcile; staged upload resumes only through readback digest; uncertain load/reset/start/I/O never replay; stop reissue requires a new token or a profile-specific idempotency proof. Indeterminate commands reconcile before further mutation.

Races are deterministic and follow an explicit dependency closure. `SolveGeneration` is the root/shared-input epoch: it increments only when topology or a shared/broadcast scene/runtime/profile/input changes. A branch-local elementwise edit leaves `SolveGeneration` and unchanged siblings untouched, recomputes that branch content digest, and increments only the affected `BranchRequestGeneration`; any item/null/order edit feeding a sequence reduction invalidates that whole aggregate branch under the same scoped rule. Currentness requires an exact match of root `SolveGeneration`, branch `BranchContentDigest`, and branch `BranchRequestGeneration`. Late completion cannot publish into another path/generation, while unchanged siblings remain current and publishable. Batch publication follows the declared `BatchPublicationPolicy` (`PUBLISH_INDEPENDENT` or `FAIL_BATCH`) and always emits output status, source-shaped item diagnostics, and branch diagnostics. Newer deployment expires start arm; stop before start dispatch cancels start, after dispatch forces reconcile then stop; the single journal orders I/O and readback by command revision; full snapshots supersede level events but never edges; late document/session generations are discarded. RAPID program-stop and binary stop have declared safety-priority lanes, need no arm, are never batched, and cancel/displace other commands according to their dispatch state.

## 7. Exact Grasshopper node families

Category is `COMPAS FAB`; subcategories are `Tesseract · Resources/Scene/Author/Profiles/Plan/Inspect/Preview` and `ABB · RAPID/Observe/Control/Binary`. `in → out` below is the binding port contract; every node also outputs a typed summary/diagnostic where relevant.

### 7.1 Resources and native scene

| Node | Exact boundary |
|---|---|
| `Tesseract Robot Artifact` | cell, URDF/SRDF paths, resource roots, groups, mesh/IK/contact-manager selections → exact `RobotArtifact`, summary, ID |
| `Tesseract Runtime` | artifact, matching cell/state, warmup request, build token → runtime handle, isolated native `Robot`, pipeline registry |
| `Tesseract Group Select` | runtime/filter → `GroupId`, ordered joints, base/tip |
| `Tesseract Pipeline Select` | runtime/capability filter → `PipelineId`, exact metadata |
| `Tesseract Tool` | name, exact geometry/resources, tool frame, parent, collision role → content-addressed tool artifact, native link/joint payload |
| `Tesseract Rigid Body` | name, visual/collision geometry, frame, role → body artifact, native link payload |
| `Tesseract Attach Body` | tool/body, parent `LinkId`, transform, joint name → exact add/move command tuple, attachment intent |
| `Tesseract Touch Links` | attachment, exact link set, reason → exact allowed-collision command, touch intent |
| `Tesseract Scene Command` | one exact released environment `Command` → retained exact command + deterministic subtype/field manifest |
| `Tesseract Scene Batch` | ordered tool/body/attachment/touch/custom commands → exact ordered batch, manifest, digest |
| `Tesseract Apply Scene` | runtime, batch, token → scene handle, isolated native `Robot`, revision, exact delta |

`Tesseract Scene Command` accepts the released command family for link/scene-graph addition; link/joint move/remove/replace; link-origin/visibility/collision changes; allowed collisions/margins; kinematics information; and joint limit changes. It validates exact type and every identity-bearing subtype field without cloning/narrowing the command; the field manifest provides scene-batch identity and is round-tripped by applying the command to a clone and inspecting the exact delta. Touch links are explicit inspectable allowed-collision pairs. Detach/removal is a new batch, never mutation of an earlier artifact.

### 7.2 Native authoring and exact profiles

| Node | Exact boundary |
|---|---|
| `Tesseract Pose` | Plane/Frame tree, broadcastable `MetersPerUserUnit` and working `FrameId` scalars → exact working-frame `Pose` tree/status; sole geometry scale boundary |
| `Tesseract Cartesian Target` | exact pose tree, declared scalar/tree native move type/profile matching → exact `CartesianTarget` tree/status |
| `Tesseract Joint Target` | native robot/group, ordinary position branches, optional ordinary name branches, move type, profile → group-tagged fixed vectors and exact `JointTarget` tree/status in native radians/metres; absent names remain native `None` |
| `Tesseract State Target` | native robot/group, ordinary position branches and optional name/velocity/acceleration/time branches → group-tagged fixed vectors and exact `StateTarget` tree/status; null topology and native absence distinguished |
| `Tesseract Pose Profile Cross Product` | exact left pose tree, right profile-name tree, explicit finite maximum → additive projection of exact native product pairs into aligned `poses_expanded`/`profiles_expanded` trees/status/source-coordinate map/identity; pure, deterministic, bounded before allocation |
| `Tesseract Motion Program` | native robot, ordered target branches, scalar group/TCP/working frame/program profile → one exact `MotionProgram`/authored `CompositeInstruction` per branch, identity/source-coordinate map/status |
| `Tesseract Program Compose` | ordered exact composite/instruction branches + metadata → one exact nested `CompositeInstruction` per branch, digest/source-coordinate map/status |
| `Tesseract Cartesian Profiles` | every exact `create_cartesian_pipeline_profiles` input → exact `ProfileDictionary`, manifest |
| `Tesseract Freespace Profiles` | every exact `create_freespace_pipeline_profiles` input → exact dictionary, manifest |
| `Tesseract Freespace IFOPT Profiles` | every exact `create_freespace_ifopt_pipeline_profiles` input → exact dictionary, manifest |
| `Tesseract Descartes Profiles` | every exact `create_descartes_pipeline_profiles` input → exact dictionary, manifest |
| `Tesseract OMPL Profiles` | exact default-profile/configurator inputs → exact dictionary, manifest |
| `Tesseract TrajOpt Profiles` | every exact `create_trajopt_default_profiles` input → exact dictionary, manifest |
| `Tesseract TrajOpt IFOPT Profiles` | every exact `create_trajopt_ifopt_default_profiles` input → exact dictionary, manifest |
| `Tesseract Profile Merge` | profile artifacts, duplicate policy fixed to error → exact merged dictionary/manifest |

Omitted profile controls remain `None`; exact native custom profiles remain exact. Because 0.35.0.6 exposes no deterministic dictionary enumeration/serialization, factory nodes emit an identity-bearing supplied-argument `ProfileManifest`. Externally mutated dictionaries are `UNVERIFIABLE`, require a fresh compute token, and are ineligible for reproducible cache reuse. Program construction uses native target-preserving addition, not convenience calls that overwrite move type. Exact command-language I/O/wait instructions remain reachable through composition.

### 7.3 Planning, inspection, projection, preview

| Node | Exact boundary |
|---|---|
| `Tesseract Planning Job` | runtime/scene, exact program tree/pipeline/profiles+manifest, seed, compute token, match/publication/job policies → one handle/snapshot per program coordinate with content digests and solve/request generations |
| `Tesseract Cancel Planning` | job-handle tree, cancel token tree under exact matching → topology-preserving cancellation receipts/snapshots |
| `Tesseract Planning Result` | completed job tree → one-to-one `TesseractPlanningResult`/exact `PlanningResult`/raw program plus parallel status; source-shaped diagnostics remain separate |
| `Tesseract Result Inspect` | exact result tree → one-to-one request/native result/raw program/message; deterministic per-result sample branches with exact points and fixed-vector names/positions/optional velocities/accelerations plus scalar times/source maps/status |
| `Tesseract Contact Request` | exact contact-test/evaluator/config/margins/filter → exact native request/config |
| `Tesseract Collision Inspect` | scene, typed state or exact program, request, token → exact `ContactResultMap`/vector, contact summary, job |
| `Tesseract Diagnostics` | planning/collision job or native result tree → parallel typed diagnostics with source coordinate/native index/links/action |
| `Tesseract Project Result` | exact result tree, matching cell, `ProjectionPolicy` → one projection aggregate per result coordinate at its original path, exact source, `ProjectionReport`, ID/status/source-shaped diagnostics |
| `Tesseract Trajectory Scrub` | exact result/projection tree, broadcastable normalized parameter or matched point-index tree, interpolation policy → selected current sample plus complete deterministic expanded sample branches with one composed source-coordinate map and typed fixed-vector state/scalar time/index/report/status |
| `Tesseract Scene Preview` | scene, selected current-sample tree, display policy → branch-preserving prepared Rhino geometry, exact link frames/collision set/status |
| `Tesseract Path Preview` | complete exact sample-series/trajectory tree, TCP/link, decimation policy → branch-preserving Rhino paths, sample→program→source-coordinate maps/report |
| `Tesseract Bake Prepared Geometry` | immutable prepared-geometry tree, `BakeRequestToken`, destination policy → `BakeReceipt` and deterministic source/build-identity→Rhino-object map; edge-triggered local document mutation |

Planning content identity covers tree/branch content digests, source coordinates, artifact, scene revision, program digest, pipeline, profile manifest/object generation, seed, policies, native version, and planner version. Runtime currentness additionally requires the root/shared-input `SolveGeneration` plus each branch's matching content digest and `BranchRequestGeneration`; a branch-local edit does not invalidate unchanged sibling generations. Runtime generations are never hashed into content identity. Native non-cooperative cancellation reports `CANCEL_REQUESTED`, discards only the superseded request generation's completion, and never kills Rhino or claims immediate cancellation.

`ProjectionPolicy` declares required positions/names/time/velocity/acceleration and allowed missing fields. Missing required data fails at the exact source coordinate; permitted absence remains absent; effort is never invented. Report records unit/joint mapping, reordering, omissions, numerical checks, source-coordinate map, and both identities. Contacts retain exact links, shapes/subshapes, points, normals, distance, continuous time/type, transforms, and native objects. Scene Preview consumes the immutable selected current sample; Path Preview consumes the immutable complete sample series/trajectory. No preview calls a controller; changed branches remove only their old geometry while shared-root changes remove the whole display. Display preparation and decimation preserve branch boundaries and never flatten paths to improve frame rate.

Preview is pure preparation/drawing; baking is a separate local effect. `BakeRequestToken` is an edge-triggered unique token scoped to the component instance and complete prepared-geometry identity. Baking accepts only immutable current prepared geometry, rejects stale solve/branch generations before document mutation, records source coordinate and `BuildIdentity` on every created object, and returns a deterministic `BakedObjectMap` plus receipt. Recompute with the same token is idempotent and never creates duplicates; a new token is required for another bake. Partial failure is explicit and retains the exact created-object subset for reconciliation.

### 7.4 Lifting, batching, and topology observability

Pure nodes may lift only according to their declared topology role, branch semantics, item shape, and `MatchPolicy`; the tables above show the authoritative lifted boundaries. Profiles and exact native objects remain domain-atomic even when carried as tree items. Fixed vectors such as axes and joint states never become series merely because Grasshopper encodes their elements with list access. A branch-local ordered series is consumed as a sequence, not lifted item-by-item, where the node contract says “one program per branch.”

Every lifted component exposes topology and failure locally: input mismatch reports exact port names, paths, lengths, null/branch coordinates, and the required correction; output status trees exactly match output value topology. `SourceCoordinateMap` composes through authoring, program construction, planning, inspection, projection, scrub, and preview so a native instruction/result/sample can always be traced to its originating path and item without renumbering. A reduction from one ordered input branch to one program/result emits one aggregate value/status item at the original branch path and also emits a source-shaped item diagnostic tree for every original item. Diagnostics for empty or branch-wide failures occupy the separate `BranchDiagnosticMap` at the exact `BranchCoordinate`; an empty source-shaped item branch stays empty.

Cardinality expansion is deterministic rather than “same topology.” `ExpandedBranchCoordinate` contains the original `BranchCoordinate` plus appended `RequestOrdinal` and `ResultOrdinal`; samples remain ordered items beneath it. The Grasshopper adapter uses one canonical bijective length-prefixed path codec, so source paths that are prefixes of other source paths cannot collide after expansion. Every sample's joint names, positions, velocities, and accelerations are `FIXED_VECTOR[GroupShape]` items; time and source index are scalar items. Projection reductions stay at the original result path; preview geometry derived from samples stays under the expanded coordinate. Exact value/status pairs share topology, while detailed diagnostics retain the topology of the source surface they diagnose.

`CrossProductPolicy` is opt-in and pure and declares the finite `maximum_results`; validation rejects a product above the bound before allocation or native work. The exact product result owns deterministic left-major pairs, collision-free output paths, and source coordinates. A later Grasshopper projection exposes aligned, explicitly named left-pose/right-profile output trees without changing that result. No authoring helper, Grasshopper default matching mode, or async scheduler may create an implicit product.

### 7.5 Native RAPID, RWS observation/control, binary playback

| Node | Exact boundary |
|---|---|
| `ABB RAPID Profile` | Tesseract profile names + native typed speed/zone/tool/workobject names → exact native `RapidProfile` map/manifest |
| `ABB RAPID Generate` | authored composite, profile maps, module/procedure → immutable `RapidProgram`, exact source, ID/report |
| `ABB RAPID Bundle` | program, task, controller directory, metadata → sealed `RapidDeploymentBundle`, canonical manifest/digest |
| `ABB Controller` | controller ID, URL, explicit RW6/RW7, credential handle, TLS/connection policy → immutable endpoint/redacted summary; no probe |
| `ABB RWS Session` | endpoint, connect/disconnect token → session handle/snapshot |
| `ABB RWS Observe` | session, exact resource specs, subscribe token → subscription handle, atomic controller snapshot/freshness/reconcile |
| `ABB RWS State` | session/snapshot → controller/operation/execution/cycle/tasks/event watermark/freshness |
| `ABB RWS I/O Read` | session/snapshot + signal specs → typed samples/revisions/freshness |
| `ABB RWS Event Log` | session/snapshot + log/watermark → ordered events/new watermark/gap evidence |
| `ABB RWS Reconcile` | session, expected deployment/execution IDs, token → atomic snapshot or exact conflict |
| `ABB Command Arm` | immutable intent + reconciled snapshot + token → armed command bound to digest/session/revision/derived expiry |
| `ABB RAPID Deploy` | session, sealed bundle, armed command, token → deployment handle plus stage/readback/load receipts |
| `ABB RAPID Load` | session, verified staged artifact, task, arm, token → load receipt/reconciled program snapshot |
| `ABB RAPID Reset` | session, loaded deployment/task, arm, token → reset receipt/observed postcondition |
| `ABB RAPID Start` | session, ready deployment, cycle/task, arm, token → execution handle/snapshot/receipt |
| `ABB RAPID Program Stop` | session, tasks, token → stop receipt/observed stopped state |
| `ABB RWS I/O Write` | session, exact signal/value, arm, token → receipt/exact readback |
| `ABB RWS Command Status` | command/deployment/execution handle → complete state history/terminal receipt/reconciliation |
| `ABB Ordered Command Batch` | explicit bounded ordered mutation intents, per-item arms/tokens, failure policy → ordered per-item handles/receipts/reconciliation; no tree lifting/cross-product |
| `ABB Binary Tool` | exact backend-native tool/load values → exact value/manifest |
| `ABB Binary Workobject` | exact backend-native workobject values → exact value/manifest |
| `ABB Binary Command` | one exact supported non-EGM command + native typed args → exact backend command/manifest |
| `ABB Binary Program` | tool/workobject/load, ordered commands, explicit format version/timestamp/sequence → exact backend `MotionProgram`, bytes, binary artifact/digest |
| `ABB Binary Execute` | compatible session, artifact, task, arm, token → binary execution/result-log handles |
| `ABB Binary Monitor` | execution handle → run state/current+queued command where available/log state |
| `ABB Binary Stop` | one execution handle + unique token → immediate unarmed scalar-only safety-priority stop receipt/observed state; forbidden in ordered batches |
| `ABB Binary Result Log` | completed log handle → exact bytes, typed columns/table, identity |

RAPID generation is pure and may map authored program branches one-to-one; each result equals a direct native emitter call byte-for-byte and rejects results/trajectories. Armed controller/deployment/execution nodes remain scalar unless the user constructs `ABB Ordered Command Batch`; each batch item retains its own command ID, arm, precondition revision, journal sequence, receipt, and reconciliation state, and admission is sequential in declared order. RAPID program-stop and binary stop are never members of that batch: both remain immediate unarmed scalar-only safety-priority actions. Deploy exposes every stage despite being one ergonomic command. Binary scope admits exact backend non-EGM absolute-joint, joint, linear, circular, wait-time, and circular-path-mode commands; interpreter installation and file-format version are explicit preflight evidence. Binary timestamp/sequence are explicit identity inputs, so the backend's wall-clock default is never used. No conversion edge exists from native result, projection, or RAPID artifact to binary program.

`LoadedProgramEvidence` distinguishes local intent, verified controller file digest, load acknowledgement, and facts directly observable after load. If the selected RWS profile cannot prove the active module identity after reconnect, reconciliation reports `UNCONFIRMED` and blocks start until explicit readback and reload; it never equates an acknowledgement or local receipt with controller-observed identity.

Typed transformations are fixed and unidirectional:

```text
sources→RobotArtifact→runtime→scene batch→scene handle→native targets/program→profiles+pipeline→planning job→exact result
exact result→exact inspection→explicit projection+loss report→sample→preview (projection is never fed back into native planning)
authored CompositeInstruction+RapidProfile map→RapidProgram→sealed bundle→verified deployment→loaded task→execution receipt
binary-native values→binary MotionProgram→binary artifact→binary execution→exact result log (no edge from either trajectory type)
```

For pure lifted nodes, each arrow is also a topology-preserving `Tree` transformation under the declared matching policy. An arrow never implies flattening, automatic product, controller lifting, or loss of the composed source-coordinate map.

## 8. Errors, credentials, and downstream contracts

Every `Diagnostic` has stable code/named exception, severity, subsystem/operation, safe identities, exact input/index/link, redacted message, causal chain, next action, and retry safety. Components catch only owned or explicitly mapped upstream errors; no generic swallowing, silent retry, fallback, optional import, or alternate parser exists.

Every failure mode receives one concrete subtype; this minimum catalog includes:

- resource/scene: `RobotArtifactBuildError`, `RobotArtifactIdentityMismatchError`, `UnknownNativeSceneLinkError`, `InvalidNativeSceneCommandError`, `NativeSceneCommandConflictError`, `NativeSceneApplyError`, `TouchLinkPairError`;
- native/planning: `FrameIdentityMismatchError`, `UnitIdentityMismatchError`, `GroupIdentityMismatchError`, `InvalidNativeTargetError`, `NativeProgramConsistencyError`, `UnverifiableProfileDictionaryError`, `UnknownPipelineError`, `PlanningBackpressureError`, `NativePlanningFailedError`, `PlanningCancellationUnsupportedError`;
- topology/matching: `InvalidTreeTopologyError`, `DuplicateTreePathError`, `TreeBranchSetMismatchError`, `BranchLengthMismatchError`, `ImplicitSingletonBroadcastError`, `NullTreeItemError`, `EmptyOrderedSeriesError`, `InvalidSequenceItemError`, `CrossProductLimitError`, `InvalidExpandedCoordinateError`, `MissingSourceCoordinateError`, `StaleBranchGenerationError`, `BranchBatchFailureError`;
- inspect/project: `MalformedNativeResultError`, `MalformedContactResultError`, `MissingTrajectoryFieldError`, `InconsistentJointOrderError`, `NonMonotonicTrajectoryTimeError`, `ProjectionLossRejectedError`, `MissingTrajectoryTimeError`;
- runtime/RWS: `StaleHandleGenerationError`, `QueueCapacityDerivationError`, `CredentialProviderUnavailableError`, `CredentialResolutionError`, `RwsAuthenticationError`, `RwsConnectionLimitError`, `RwsSubscriptionCreateError`, `UnknownRwsSubscriptionMessageError`, `ObservationGapError`, `ReconciliationConflictError`;
- effect: `CommandArmExpiredError`, `CommandPreconditionError`, `CommandJournalWriteError`, `CommandDispatchIndeterminateError`, `RapidUploadError`, `RapidReadbackMismatchError`, `RapidLoadError`, `LoadedProgramIdentityUnconfirmedError`, `RapidResetError`, `RapidStartError`, `RapidProgramStopError`, `IoWriteVerificationError`;
- mutation cardinality: `AutomaticMutationLiftingError`, `MutationBatchCardinalityError`, `MutationBatchOrderError`, `MutationBatchPreconditionError`, `BatchedImmediateStopError`, `UnprovenMutationIndependenceError`, `MutationBatchIndeterminateBlockError`, `MutationBatchReconciliationError`;
- build proof: `InvalidComponentizerRevisionError`, `UnpinnedGrasshopperAssemblyError`, `BuildReceiptDigestMismatchError`, `RhinoArtifactRebuildError`;
- binary: `BinaryInterpreterCompatibilityError`, `InvalidBinaryCommandError`, `BinaryProgramSerializationError`, `BinaryPlaybackStartError`, `BinaryResultLogVersionError`, `BinaryResultLogShapeError`.

GHX stores `CredentialHandle` only; embedded URL credentials are rejected. Only `ControllerOwner` resolves secrets through a hard-dependency provider. Fake/recorded tests inject an in-memory provider; physical packaging requires the platform credential-store provider and fails if unavailable—never an insecure fallback. Secrets/auth headers/cookies/tokens never enter sticky data, pickles, logs, exceptions, manifests, journals, recordings, or diffs. Redaction is safe-field allowlisting before disk write. Controller paths are rooted/grammar-validated; deployment readback hashes exact bytes. TLS is default for physical endpoints; explicit insecure-local policy is restricted to declared isolated RobotStudio endpoints and remains visible.

Required consumer contracts: URDF/SRDF parse and resources stay in materialization root; scene batches apply to clone and native environment reports exact delta; profile nodes return exact types/manifest; `Tree` JSON round-trips paths/order/empty branches/null bitmap/root/content digests while runtime generations serialize only in snapshots; every lifted/reduced/expanded consumer round-trips output status, source-shaped item diagnostics, `BranchDiagnosticMap`, source-coordinate maps, and the expansion path codec; planning retains exact requests/results/messages per root epoch and branch content+generation without invalidating unchanged siblings; contacts retain exact objects/counts; projection JSON validates and COMPAS trajectory round-trips; native RAPID equals direct output; deployment JSON validates and controller bytes hash; ordered mutation batches round-trip per-item journal/receipt/reconciliation order and independence proof, reject both immediate stop types, and block after indeterminate state; build receipts round-trip and self-hosted Rhino rehashes exact public artifacts; binary bytes/version and result-log shape validate through backend contract; recorded RW6/RW7 exchanges parse/replay deterministically.

## 9. Platform, dependencies, tests, and productization

Runtime pins are exact:

```text
tesseract-robotics-nanobind==0.35.0.6
abb-robot-client[aio]==0.5.0
abb-motion-program-exec==0.8.0
```

The CPython componentizer and its invocation wrapper are reproducible build inputs, not floating tooling. CI resolves either a clean local checkout plus expected commit SHA or an upstream URL plus mandatory verified 40-hex commit SHA, never a default branch; metadata supplies stable component/port keys and deterministic parameter GUIDs. The public build job publishes one immutable artifact bundle named by receipt digest. Its build receipt records that verified componentizer SHA, generator version, pinned GH_IO/Grasshopper assembly package+version+digest, ordered input digests, and every output digest. The self-hosted Rhino job consumes that exact bundle, verifies the receipt and every file digest before loading Rhino, and is forbidden from invoking the component build task or substituting workspace outputs. Existing GHX embeds its stock script and full parameter layout and must continue to open unchanged; replacing a `.ghuser` affects new drops only and is never described as automatic migration.

Progressive runtime state uses the component `ValueTable`; undo uses `RecordUndoEvent`; parameter changes use registered Grasshopper parameter APIs between solves. A custom menu requires a supported component-owner hook, and physical port removal must prove Python invocation arity, wire/value retention, undo/redo, and save/reopen on actual Rhino before release. GH_IO-only compilation/deserialization is not evidence for menu, solve, or event behavior.

The Pixi lock records exact transitive artifacts/hashes. Imports are unconditional; no `HAS_*`, conditional import, transport fallback, reduced component set, skip, conditional skip, or expected failure. Nanobind/OpenMP plus pthreads OpenBLAS remains a lock contract.

| Environment | Binding evidence |
|---|---|
| macOS ARM64 Python 3.12 | primary dev; strict mypy/ruff, native resource/scene/plan/contact/projection/RAPID, deterministic artifacts/docs |
| Windows x64 CPython 3.9 public runner | exact-lock component build, deterministic GH_IO round trip/build receipts; no Rhino runtime claim |
| Windows x64 Rhino 8 CPython 3.9 self-hosted | downloads exact public-job artifacts/receipt, verifies every digest and pinned assembly version without rebuilding, then proves real item/list/tree access, multi-iteration solve, progressive menu/arity/undo/reopen, installed GHX, fake/recorded RWS |
| RobotStudio RW6/RW7 | claimed RWS observe/deploy/control contracts; RW6 binary playback |
| physical RW6 | supervised terminal/fault/cancel/reconnect acceptance |
| physical RW7 | future evidence only; no current claim |

Tests combine factory/bypass-safety, Hypothesis identity/frame/unit/joint/program/state/queue/redaction/tree-topology/matching properties, model-based state machines, numerical fidelity with named COMPAS tolerances and rationale, deterministic scheduling, and fault injection. Tree generators cover every valid/invalid topology-role+branch-semantics+item-shape combination; ragged ordered fixed-vector trees; frame series; typed scalar broadcast; multiple/nonzero/prefix-related paths; empty branches; explicit null slots; equal-length paired-series zip and unequal-length paired-series mismatch rejection; one-item series that must not broadcast; explicit bounded pure cross-products with deterministic paths and over-limit rejection; and topology-sensitive content digests. Identity tests prove identical content has identical `TreeContentDigest`/`BranchContentDigest` across different solve/request generations and that `BuildIdentity` never changes from generation alone. Reduction tests require one aggregate output/status per branch, `BranchDiagnosticMap` errors for empty series with an empty item diagnostic branch, whole-branch invalidation for one bad instruction, and source-shaped item diagnostics; expansion tests prove collision-free paths and fixed-vector sample dynamics. Currentness tests prove a branch-local edit changes only its branch request generation/content and leaves unchanged sibling outputs current, while root topology/shared/broadcast edits increment `SolveGeneration` and invalidate their full dependency closure. The host-independent Grasshopper harness emulates item/list/tree access and exact branch matching; it may not represent trees as flattened Python lists. Actual Rhino tests prove exact capture and post-audited emission of nonzero/prefix-related paths, branch order, empty branches, explicit null positions, item/list/tree marshalling, ordinary-branch-to-group-fixed-vector reduction, zero-source-and-zero-persistent optional absence versus present null slots, exact native `JointTarget.names=None`, additive aligned projection over unchanged cross-product pairs, and multiple `RunScript` iterations without state collisions. They change Grasshopper matching modes and prove typed matching is unaffected, then reorder/insert/remove branches and prove values, status, diagnostics, async state, current/full-series source identity, and preview geometry remain attached to exact coordinates; unchanged preview siblings remain visible. Forced interleavings cover duplicate solves; one-branch async failure while siblings succeed; item-versus-reduction-versus-broadcast/shared-input dependency closure; independent branch completion/cancel; close during every effect; duplicate/stale/partially failed bake; disconnect before/after dispatch/ack; edge overflow; late generations; start/stop/deploy/I/O races; pre-dispatch independent continuation; indeterminate ordered-batch blocking; RAPID and binary stop batch rejection; journal failure; malformed/unknown/gapped subscriptions; native completion after cancel; auth/connection-limit/HTTP/malformed/partial-file/readback/load/execution faults. Assertions require at-most-once effect, exact content/generation separation, topology/source map/parallel-output-status/source-shaped-item-diagnostic/branch-diagnostic/state history, branch isolation, bounded memory, no stale output/secrets, and byte-identical replay.

Test levels are fake controller, sanitized recorded RW6/RW7, RobotStudio RW6, RobotStudio RW7, physical RW6, and later physical RW7. Hardware suites are explicit jobs whose absence fails that job; the main suite never pretends hardware ran.

Grasshopper gates assert metadata/ports/access/topology role/branch semantics/item shape/match/cross-product policies/docs/icons/pin directives; progressive-port undo/persistence; compiled user-object load/deserialization; all installed GHX opening with no missing objects; item/list/tree harness parity with Rhino; exact capture and post-audited emission of paths/order/empty branches/null positions; matching-mode immunity; fixed-vector atomicity and optional-absence distinction; source-coordinate, parallel-status, source-shaped item-diagnostic, and branch-diagnostic maps; branch reorder/insert/remove stability; current-sample/full-series identity; preview sibling retention; edge-triggered bake idempotency/stale rejection/object mapping; remote work reaching terminal state without UI blocking; scoped branch-local stale-output invalidation with unchanged siblings retained; controller single-command-default/cardinality rejection; and Yak inventory of `.ghuser`, GHX, schemas, icons, docs.

Installed definitions use packaged compiled objects only:

- Basic: `01 Fixed-Base Plan and Preview`, `02 RAPID Deploy and Monitor`, `03 Controller State and I-O`;
- Advanced: `01 Tools Bodies Attachments and Contacts`, `02 Exact Pipelines and Projection`, `03 Binary Authored Playback`;
- Expert: `01 Native Scene Commands`, `02 RWS Reconnect and Reconciliation`, `03 Command Journal Replay`.

The primary Basic definition authors an ordered frame/pose/target series into one program per branch; its one-item ordered-series example is the concise convenience case. Companion examples demonstrate: broadcastable typed scale/frame scalars; equal-length paired target/profile series plus a visible unequal-length mismatch rejection; ragged multi-branch ordered programs with source-coordinate diagnostics; an explicit bounded pure Cartesian cross-product with deterministic paths; isolated async planning where one branch fails while siblings publish; topology-preserving output status, source-shaped diagnostics, and preview; and an explicit bounded controller-command batch. Examples include empty branches, a visible null failure, and a failed branch without shifting neighboring coordinates. No example relies on Grasshopper longest-list matching, flatten/simplify, implicit cross-product, or automatic mutation lifting.

Remote groups open disabled with explicit safety steps; offline/fake operation appears only in examples whose stated purpose is simulation, never as runtime fallback. Every node's MkDocs page specifies authority, typed ports/frames/units/IDs, defaults/progressive ports, async/effect/lifetime, state diagram where remote, errors/actions, minimal/composition examples, versions/platforms, and non-safety-rated warnings.

## 10. Tranches and dual-axis Definition of Done

Tranches: **0** typed values/tree topology/matching/source maps/identity/manifests/performance/handles/current-output; **1** resources/runtime/selectors/scalar+series authoring/all profiles/branch-isolated async planning; **2** tools/bodies/attachments/touch links/exact scene/contact; **3** topology-preserving result/projection/scrub/preview/native RAPID/bundles; **4** credentials/RWS observe/session/subscription/reconcile; **5** journal/scalar-default and explicit ordered-batch RWS deployment/control/execution/I/O; **6** advanced binary authored playback; **7** deterministic componentizer/Windows compiled+real-Rhino product, docs/examples, performance calibration, RobotStudio and physical RW6 acceptance. Public terminal completeness requires all.

| Boundary | Ergonomics DoD | Engineering DoD |
|---|---|---|
| Resources/runtime | one artifact wire; generated group/pipeline; readable digest/version | content-addressed/parser round trip, isolated lifetime, strict types, no UI block |
| Scene | one ordered batch; exact link selectors; readable delta | exact commands retained, clone-only apply, identity/conflict/contact contracts |
| Author/profile | ordered pose/frame/target series is the Basic path; one-item branch is concise convenience | typed frame/unit/group/role; exact topology/source map; absence and null distinguished; every released control/exact output |
| Plan | one compute; per-branch pending/cancel/failure visible; responsive/no stale result | bounded clone ownership/concurrency, content-digest plus solve/request-generation currentness, dependency-closure/isolation/race/cancel/failure proof |
| Inspect/project | source-coordinate diagnostics; projection one explicit choice | topology-preserving exact result/contact/status, named loss/match/malformed errors, numerical/schema proof |
| Preview | one scalar or matched slider tree; immediate branch-local stale-geometry removal | deterministic topology-preserving sampling/identity, measured display budget, no controller |
| RAPID | readable exact source/bundle without file wiring | byte equality, exact profiles/identity, no dense-result path |
| Observe | one connect+observe; freshness/reconnect visible | single owner, bounded reducer/state/reconcile/redaction, RW6/RW7 contracts |
| Deploy/control | one armed deploy pair; explicit ordered batch only; separate reset/start; immediate RAPID/binary stops | no automatic lifting/product; both stops unarmed/scalar-only/non-batch; bounded batch order, per-item arm/journal/idempotency/readback/pre-post/indeterminate/reconcile proof |
| Binary | distinct advanced authoring/result UX | separate types/owner/identity, no conversion, version/log/interpreter proof |
| Package/docs | searchable compiled nodes; installed examples solve; help complete | CPython 3.9 compile/deserialize/GHX, exact pins/inventory, no hidden paths |

## 11. Build gates and terminal acceptance

Required gates: ruff; strict mypy on Tesseract/ABB/runtime/tree/matching/contracts; `pytest --testmon -n auto` after changes; full `pytest -n auto`; property/model/topology/concurrency/fault suites; macOS ordered-series and one-item-convenience native examples; deterministic Windows 3.9 exact-lock component build receipt with verified 40-hex componentizer SHA and pinned assembly evidence; every `.ghuser` GH_IO-deserialize; self-hosted Rhino downloads and digest-verifies the public-job artifacts without rebuilding, then runs every installed series/one-item GHX smoke, real item/list/tree access and multi-iteration branch-matching parity, and progressive menu/arity/undo/reopen; fake/recorded RW6/RW7 on 3.12 and 3.9; RobotStudio RW6/RW7 for claimed single-command and ordered-batch surfaces; supervised physical RW6 record; MkDocs strict; JSON Schema/package inventory/secret/provenance/license scans; `git diff --check`. Reference tests are never weakened.

Terminal scenarios:

1. **Offline native ordered series:** installed Basic graph builds artifact/scene including tool/body/attachment/touch links, then ordered frame/pose/target series with typed scalar broadcast, equal-length paired-series zip, unequal-length mismatch rejection, ragged branches, and an explicit bounded pure cross-product author one program per branch; a one-item branch proves concise use. Planning isolates branch requests so one-branch async failure leaves successful siblings publishable; inspection emits topology-preserving aggregate status, source-shaped item diagnostics, `BranchDiagnosticMap`, results, and source maps; projection/scrub/preview never flatten; native RAPID remains exact; a branch-local edit retains unchanged siblings while shared-root invalidation closes over every dependency on macOS 3.12 and real Windows Rhino 3.9 item/list/tree multi-iteration solves.
2. **RWS native RAPID:** RobotStudio RW6/RW7 graph connects/reconciles, seals/deploys/readback-verifies/loads, separately arms reset/start-once, monitors/program-stops, and injects disconnect at every mutation boundary with exact cancelled/failed/indeterminate/reconciled outcomes.
3. **Observe/I-O and ordered mutation:** subscription covers controller/operation/execution/event log/signals, visibly gaps/reconciles after disconnect, performs scalar armed write/readback, rejects an implicitly lifted write, executes one explicit bounded ordered batch with per-item arms/receipts/reconciliation, and never silently loses an edge under pressure.
4. **Binary:** compatible RobotStudio RW6 interpreter runs exact non-EGM authored artifact, exposes current/queued command, performs immediate unarmed scalar-only binary stop and rejects batching it, downloads/version+shape-validates typed result log, with no Tesseract result/projection/RAPID input.
5. **Package:** clean Windows profile installs exact package, deserializes every node, opens every installed GHX, and completes local/fake smoke with no source-tree dependency.

Final audit must answer yes: ordered frame/pose/target/program series are the primary path and one-item ordered series are only convenience; exact native access/options/object identity preserved inside tree items; paths/order/empty branches/null slots/parallel output status/source-shaped item diagnostics/branch diagnostics/source coordinates survive every lifted or reduced boundary and host emission self-audit; fixed vectors never become series and disconnected optional dynamics remain distinct from connected null slots; matching never repeats/truncates/skips/flattens/simplifies/renumbers or treats singleton series as scalars; cross-products are explicit/bounded/pure; coordinate labels never mutate native names or identity; content digests exclude runtime generations; root/shared edits advance `SolveGeneration`, branch-local edits advance only affected `BranchRequestGeneration`, currentness requires root epoch+branch generation+content, and unchanged siblings remain valid; preview retains topology and unchanged sibling display; bake is separate, edge-triggered, identity-mapped, stale-rejecting, and duplicate-free under recompute; fixed-base scene complete; four artifact kinds non-interchangeable; controller mutations default to cardinality one, RAPID program-stop and binary stop are unarmed/scalar-only/non-batch safety-priority actions, and explicit ordered batches retain per-item safety evidence and block after indeterminate state; recompute cannot repeat effects; all pending/stale/failure/cancel/gap/conflict/indeterminate states visible; one controller owner/bounded derived queues; linear idempotent journal/deterministic replay; secrets absent; effects disclose pre/effect/post/cancel/reconnect; factories bypass-safe; strict types carry frames/units/groups/tasks/controllers/topology; responsibilities/lifetimes separated; downstream round trips pass; packaged ordered-series/one-item examples use compiled nodes; exact macOS/Windows pins pass; evidence labels are precise; no placeholder, silent fallback, arbitrary timing literal, skipped/expected-failure test, native weakening, or unsupported hardware claim remains.
