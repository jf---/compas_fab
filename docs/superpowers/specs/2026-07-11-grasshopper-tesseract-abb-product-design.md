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

### 2.3 Controller separation and safety

- **RWS observe** owns reads/subscriptions; **RWS control** owns upload/readback/load/reset/start/program-stop/I/O writes; **binary playback** owns its backend-native authored program/interpreter flow.
- Native RAPID source, dense Tesseract results, COMPAS projections, and binary bytes are non-interchangeable types with separate identities, nodes, icons, owners, and docs.
- Deploy never stops, resets, starts, changes motor state/mode, or writes unrelated I/O. Reset/start/I/O writes require arm then issue; program-stop is one immediate action and is explicitly a normal RAPID stop, not safety-rated or an emergency stop.
- Controller configuration installation, safety-system configuration, and automatic motor-state changes are out of scope.

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

Included: fixed base, one controller, one motion task; released 0.35.0.6 profile factories; RW6/RW7 RWS profiles; fake/recorded/RobotStudio tests; physical RW6 acceptance; macOS ARM64 Python 3.12 development; Rhino 8 Windows CPython 3.9 product builds.

Deferred: mobile bases, root-changing external axes, VKC, MultiMove, concurrent TrajOpt, EGM, lossy dense-result-to-RAPID/binary conversion, and any current physical RW7 validation claim. RW7 release claims stop at fake, recorded, and RobotStudio evidence until a supervised physical record exists.

## 4. Canonical workflow task matrix

This matrix comes from user intent, before component design, and is the interaction acceptance oracle. Counts exclude optional expert/diagnostic wires but include safety actions.

| Task | Authoritative input/output | Explicit decision/action budget | Canonical wire budget |
|---|---|---:|---:|
| Load robot | sources/plugins → artifact, digest, summary | one build token on source change | artifact to runtime: 1 |
| Add/attach scene object | geometry, parent, transform, touch links → exact command/batch | one authoring action | object to batch: 1 |
| Author Cartesian move | framed SI-scaled pose, move type, profile → exact target | one target action | pose→target→program: 2 |
| Author joint/state move | group-typed native units, optional dynamics → exact target | one target action | target→program: 1 |
| Choose planning | runtime pipeline registry, exact profiles → identities/summaries | one selector choice | scene/program/profiles→job: 3 |
| Plan/inspect | exact job → exact result, contacts, diagnostics | one compute action | job→inspect: 1 |
| Project/preview | exact result + explicit policy → projection/report/sample/display | one projection choice; slider only thereafter | result→project/scrub→preview: 2 |
| Generate RAPID | authored program + profiles → exact source/bundle identity | one pure generation | program/profiles→generate: 2 |
| Observe controller | endpoint/session/resources → reconciled snapshot | one connect and one subscribe | session→observe: 1 |
| Deploy RAPID | sealed bundle + session/task → verified loaded deployment | arm then deploy: 2 actions | bundle/session→deploy: 2 |
| Reset/start | ready deployment + reconciled state → execution handle | arm+reset; arm+start | deployment/session→command: 2 |
| Program-stop | live session/task → acknowledged observed stop | one press | session→stop: 1 |
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

Strict mypy is a release gate. Non-interchangeable `NewType` values cover `ControllerId`, `SessionId`, `SessionGeneration`, `SubscriptionId`, `ObservationRevision`, `CommandId`, `CommandSequence`, `DeploymentId`, `ExecutionId`, `DocumentId`, `ComponentInstanceId`, `RobotArtifactId`, `SceneRevision`, `GroupId`, `TaskId`, `MechanicalUnitId`, `FrameId`, `LinkId`, `JointId`, `SignalId`, `ProfileId`, `PipelineId`, `CredentialHandle`, `MetersPerUserUnit`, `Seconds`, `Radians`, and `Meters`.

`TaggedFrame[FrameTag, UnitTag]` distinguishes user/world/base/working/tool/controller frames and metres/user units. `JointVector[GroupTag, QuantityTag]` retains group, ordered joint identities, and position/velocity/acceleration/effort meaning. Conversion consumes one exact tag and returns another; bare floats/strings do not cross subsystem boundaries.

Boundary values are frozen/slotted `attrs` classes. Public `Class.build(...)` factories validate exact types, finite numbers, shape/order, identities, cross-field invariants, and immutable copies. `__attrs_post_init__` repeats structural invariants so raw construction cannot bypass safety. Package `__init__.py` files remain minimal and define no `__all__` registry.

### 5.3 Content identity and manifests

Every artifact contains `BuildIdentity(schema_name, schema_version, compas_fab_version, producer_version, native_versions, ordered_input_digests, payload_digest)`. SHA-256 consumes length-prefixed canonical UTF-8 fields and exact payload bytes. Ordered programs/joints/commands retain order; declared sets sort by typed identity. Canonical JSON is UTF-8 with sorted keys and no insignificant whitespace; controller paths use forward slashes.

Schemas are `compas_fab.{robot_artifact,native_scene,native_program,planning_job,trajectory_projection,rapid_program,rapid_deployment,binary_program,command_receipt,recorded_rws_session}/v1`. Each has JSON Schema, `json.loads`/validate/re-encode/digest tests, and downstream round trips. Manifests exclude secrets, auth data, sessions, object addresses, wall time, cache roots, and nondeterministic paths.

### 5.4 Lifetimes and backpressure

- artifact owns exact source bytes; environment owner owns materialization/runtime; scene application clones baseline and returns a new scene revision; a planning worker exclusively owns its native clone; preview owns immutable sampled frames/display data;
- controller owner exclusively owns credentials, async RWS client, WebSocket, reducer, durable journal writer, reconciliation, and task-mode lease; binary playback is a task-exclusive mode, not another client;
- document close cancels queued local jobs, requests running cancellation, releases controller leases/subscriptions, discards late-generation callbacks, and preserves redacted receipts. The last lease closes the controller.

All queues use immutable `BackpressurePolicy`; no capacity/delay is an inline literal. Observation capacity is `ceil(measured_peak_rate_per_source × source_count × maximum_UI_drain_interval × measurement_uncertainty_factor)`, with calibration inputs serialized. Level state coalesces by exact resource key while retaining first/last revision/count; event-log edges, acknowledgements, faults, and transitions never coalesce. Edge overflow enters `GAP`, raises `ObservationGapError`, and requires full reconciliation. Local capacity derives from isolated clone count and job policy; superseded queued jobs cancel, running results never masquerade as newer solves, and saturation raises `PlanningBackpressureError`.

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

Races are deterministic: newer scene revision invalidates component output for older jobs; newer deployment expires start arm; stop before start dispatch cancels start, after dispatch forces reconcile then stop; the single journal orders I/O and readback by command revision; full snapshots supersede level events but never edges; late document/session generations are discarded. Program-stop has a declared safety-priority lane and cancels/displaces other commands according to their dispatch state.

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
| `Tesseract Pose` | Plane/Frame, `MetersPerUserUnit`, working `FrameId` → exact working-frame `Pose`; sole geometry scale boundary |
| `Tesseract Cartesian Target` | exact pose, native move type, profile → exact `CartesianTarget` |
| `Tesseract Joint Target` | group-typed positions/names, move type, profile → exact `JointTarget` in native radians/metres |
| `Tesseract State Target` | typed positions and optional names/velocity/acceleration/time → exact `StateTarget`; absence preserved |
| `Tesseract Motion Program` | native robot, ordered targets, group/TCP/working frame/program profile → exact `MotionProgram`, authored `CompositeInstruction`, identity/index map |
| `Tesseract Program Compose` | ordered exact composite/instruction values + metadata → exact nested `CompositeInstruction`, digest/index map |
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
| `Tesseract Planning Job` | runtime/scene, exact program/pipeline/profiles+manifest, seed, compute token, job policy → non-blocking job handle/snapshot/input ID |
| `Tesseract Cancel Planning` | job, cancel token → cancellation receipt/snapshot |
| `Tesseract Planning Result` | completed job → complete `TesseractPlanningResult`, exact `PlanningResult`/raw program/diagnostics |
| `Tesseract Result Inspect` | exact result → request/native result/raw program/message/exact points/names/optional dynamics/time/index map |
| `Tesseract Contact Request` | exact contact-test/evaluator/config/margins/filter → exact native request/config |
| `Tesseract Collision Inspect` | scene, typed state or exact program, request, token → exact `ContactResultMap`/vector, contact summary, job |
| `Tesseract Diagnostics` | planning/collision job or native result → ordered typed diagnostics with native message/index/links/action |
| `Tesseract Project Result` | exact result, matching cell, `ProjectionPolicy` → COMPAS `JointTrajectory`, exact source, `ProjectionReport`, ID |
| `Tesseract Trajectory Scrub` | exact result/projection view, normalized parameter or point index, interpolation policy → typed sample/state/time/index/report |
| `Tesseract Scene Preview` | scene, sample, display policy → prepared Rhino geometry, exact link frames/collision set |
| `Tesseract Path Preview` | exact trajectory view, TCP/link, decimation policy → Rhino path, sample→program map/report |

Planning identity covers artifact, scene revision, program digest, pipeline, profile manifest/object generation, seed, native version, and planner version. Native non-cooperative cancellation reports `CANCEL_REQUESTED`, discards a superseded completion, and never kills Rhino or claims immediate cancellation.

`ProjectionPolicy` declares required positions/names/time/velocity/acceleration and allowed missing fields. Missing required data fails; permitted absence remains absent; effort is never invented. Report records unit/joint mapping, reordering, omissions, numerical checks, and both identities. Contacts retain exact links, shapes/subshapes, points, normals, distance, continuous time/type, transforms, and native objects. No preview calls a controller; changed inputs remove old geometry while the new snapshot is pending.

### 7.4 Native RAPID, RWS observation/control, binary playback

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
| `ABB Binary Tool` | exact backend-native tool/load values → exact value/manifest |
| `ABB Binary Workobject` | exact backend-native workobject values → exact value/manifest |
| `ABB Binary Command` | one exact supported non-EGM command + native typed args → exact backend command/manifest |
| `ABB Binary Program` | tool/workobject/load, ordered commands, explicit format version/timestamp/sequence → exact backend `MotionProgram`, bytes, binary artifact/digest |
| `ABB Binary Execute` | compatible session, artifact, task, arm, token → binary execution/result-log handles |
| `ABB Binary Monitor` | execution handle → run state/current+queued command where available/log state |
| `ABB Binary Stop` | execution handle, token → stop receipt/observed state |
| `ABB Binary Result Log` | completed log handle → exact bytes, typed columns/table, identity |

RAPID generation equals a direct native emitter call byte-for-byte and rejects results/trajectories. Deploy exposes every stage despite being one ergonomic command. Binary scope admits exact backend non-EGM absolute-joint, joint, linear, circular, wait-time, and circular-path-mode commands; interpreter installation and file-format version are explicit preflight evidence. Binary timestamp/sequence are explicit identity inputs, so the backend's wall-clock default is never used. No conversion edge exists from native result, projection, or RAPID artifact to binary program.

`LoadedProgramEvidence` distinguishes local intent, verified controller file digest, load acknowledgement, and facts directly observable after load. If the selected RWS profile cannot prove the active module identity after reconnect, reconciliation reports `UNCONFIRMED` and blocks start until explicit readback and reload; it never equates an acknowledgement or local receipt with controller-observed identity.

Typed transformations are fixed and unidirectional:

```text
sources→RobotArtifact→runtime→scene batch→scene handle→native targets/program→profiles+pipeline→planning job→exact result
exact result→exact inspection→explicit projection+loss report→sample→preview (projection is never fed back into native planning)
authored CompositeInstruction+RapidProfile map→RapidProgram→sealed bundle→verified deployment→loaded task→execution receipt
binary-native values→binary MotionProgram→binary artifact→binary execution→exact result log (no edge from either trajectory type)
```

## 8. Errors, credentials, and downstream contracts

Every `Diagnostic` has stable code/named exception, severity, subsystem/operation, safe identities, exact input/index/link, redacted message, causal chain, next action, and retry safety. Components catch only owned or explicitly mapped upstream errors; no generic swallowing, silent retry, fallback, optional import, or alternate parser exists.

Every failure mode receives one concrete subtype; this minimum catalog includes:

- resource/scene: `RobotArtifactBuildError`, `RobotArtifactIdentityMismatchError`, `UnknownNativeSceneLinkError`, `InvalidNativeSceneCommandError`, `NativeSceneCommandConflictError`, `NativeSceneApplyError`, `TouchLinkPairError`;
- native/planning: `FrameIdentityMismatchError`, `UnitIdentityMismatchError`, `GroupIdentityMismatchError`, `InvalidNativeTargetError`, `NativeProgramConsistencyError`, `UnverifiableProfileDictionaryError`, `UnknownPipelineError`, `PlanningBackpressureError`, `NativePlanningFailedError`, `PlanningCancellationUnsupportedError`;
- inspect/project: `MalformedNativeResultError`, `MalformedContactResultError`, `MissingTrajectoryFieldError`, `InconsistentJointOrderError`, `NonMonotonicTrajectoryTimeError`, `ProjectionLossRejectedError`, `MissingTrajectoryTimeError`;
- runtime/RWS: `StaleHandleGenerationError`, `QueueCapacityDerivationError`, `CredentialProviderUnavailableError`, `CredentialResolutionError`, `RwsAuthenticationError`, `RwsConnectionLimitError`, `RwsSubscriptionCreateError`, `UnknownRwsSubscriptionMessageError`, `ObservationGapError`, `ReconciliationConflictError`;
- effect: `CommandArmExpiredError`, `CommandPreconditionError`, `CommandJournalWriteError`, `CommandDispatchIndeterminateError`, `RapidUploadError`, `RapidReadbackMismatchError`, `RapidLoadError`, `LoadedProgramIdentityUnconfirmedError`, `RapidResetError`, `RapidStartError`, `RapidProgramStopError`, `IoWriteVerificationError`;
- binary: `BinaryInterpreterCompatibilityError`, `InvalidBinaryCommandError`, `BinaryProgramSerializationError`, `BinaryPlaybackStartError`, `BinaryResultLogVersionError`, `BinaryResultLogShapeError`.

GHX stores `CredentialHandle` only; embedded URL credentials are rejected. Only `ControllerOwner` resolves secrets through a hard-dependency provider. Fake/recorded tests inject an in-memory provider; physical packaging requires the platform credential-store provider and fails if unavailable—never an insecure fallback. Secrets/auth headers/cookies/tokens never enter sticky data, pickles, logs, exceptions, manifests, journals, recordings, or diffs. Redaction is safe-field allowlisting before disk write. Controller paths are rooted/grammar-validated; deployment readback hashes exact bytes. TLS is default for physical endpoints; explicit insecure-local policy is restricted to declared isolated RobotStudio endpoints and remains visible.

Required consumer contracts: URDF/SRDF parse and resources stay in materialization root; scene batches apply to clone and native environment reports exact delta; profile nodes return exact types/manifest; planning retains exact requests/results/messages; contacts retain exact objects/counts; projection JSON validates and COMPAS trajectory round-trips; native RAPID equals direct output; deployment JSON validates and controller bytes hash; binary bytes/version and result-log shape validate through backend contract; recorded RW6/RW7 exchanges parse/replay deterministically.

## 9. Platform, dependencies, tests, and productization

Runtime pins are exact:

```text
tesseract-robotics-nanobind==0.35.0.6
abb-robot-client[aio]==0.5.0
abb-motion-program-exec==0.8.0
```

The Pixi lock records exact transitive artifacts/hashes. Imports are unconditional; no `HAS_*`, conditional import, transport fallback, reduced component set, skip, conditional skip, or expected failure. Nanobind/OpenMP plus pthreads OpenBLAS remains a lock contract.

| Environment | Binding evidence |
|---|---|
| macOS ARM64 Python 3.12 | primary dev; strict mypy/ruff, native resource/scene/plan/contact/projection/RAPID, deterministic artifacts/docs |
| Windows x64 Rhino 8 CPython 3.9 | exact-lock install, every `.ghuser` compile/deserialize, installed GHX smoke, fake/recorded RWS |
| RobotStudio RW6/RW7 | claimed RWS observe/deploy/control contracts; RW6 binary playback |
| physical RW6 | supervised terminal/fault/cancel/reconnect acceptance |
| physical RW7 | future evidence only; no current claim |

Tests combine factory/bypass-safety, Hypothesis identity/frame/unit/joint/program/state/queue/redaction properties, model-based state machines, numerical fidelity with named COMPAS tolerances and rationale, deterministic scheduling, and fault injection. Forced interleavings cover duplicate solves; close during every effect; disconnect before/after dispatch/ack; edge overflow; late generations; start/stop/deploy/I/O races; journal failure; malformed/unknown/gapped subscriptions; native completion after cancel; auth/connection-limit/HTTP/malformed/partial-file/readback/load/execution faults. Assertions require at-most-once effect, exact state history, bounded memory, no stale output/secrets, and byte-identical replay.

Test levels are fake controller, sanitized recorded RW6/RW7, RobotStudio RW6, RobotStudio RW7, physical RW6, and later physical RW7. Hardware suites are explicit jobs whose absence fails that job; the main suite never pretends hardware ran.

Grasshopper gates assert metadata/ports/access/docs/icons/pin directives; progressive-port undo/persistence; compiled user-object load/deserialization; all installed GHX opening with no missing objects; remote work reaching terminal state without UI blocking; solve-stack watchdog forbidding file/network/wait/sleep; stale-output invalidation; and Yak inventory of `.ghuser`, GHX, schemas, icons, docs.

Installed definitions use packaged compiled objects only:

- Basic: `01 Fixed-Base Plan and Preview`, `02 RAPID Deploy and Monitor`, `03 Controller State and I-O`;
- Advanced: `01 Tools Bodies Attachments and Contacts`, `02 Exact Pipelines and Projection`, `03 Binary Authored Playback`;
- Expert: `01 Native Scene Commands`, `02 RWS Reconnect and Reconciliation`, `03 Command Journal Replay`.

Remote groups open disabled with explicit safety steps; offline/fake operation appears only in examples whose stated purpose is simulation, never as runtime fallback. Every node's MkDocs page specifies authority, typed ports/frames/units/IDs, defaults/progressive ports, async/effect/lifetime, state diagram where remote, errors/actions, minimal/composition examples, versions/platforms, and non-safety-rated warnings.

## 10. Tranches and dual-axis Definition of Done

Tranches: **0** typed values/identity/manifests/performance/handles/current-output; **1** resources/runtime/selectors/authoring/all profiles/async planning; **2** tools/bodies/attachments/touch links/exact scene/contact; **3** result/projection/scrub/preview/native RAPID/bundles; **4** credentials/RWS observe/session/subscription/reconcile; **5** journal/RWS deployment/control/execution/I/O; **6** advanced binary authored playback; **7** Windows compiled product, docs/examples, performance calibration, RobotStudio and physical RW6 acceptance. Public terminal completeness requires all.

| Boundary | Ergonomics DoD | Engineering DoD |
|---|---|---|
| Resources/runtime | one artifact wire; generated group/pipeline; readable digest/version | content-addressed/parser round trip, isolated lifetime, strict types, no UI block |
| Scene | one ordered batch; exact link selectors; readable delta | exact commands retained, clone-only apply, identity/conflict/contact contracts |
| Author/profile | canonical target ≤2 wires to program; native ports persist | typed frame/unit/group; absence preserved; every released control/exact output |
| Plan | one compute; visible pending/cancel; responsive/no stale result | bounded clone ownership, deterministic signature, race/cancel/failure proof |
| Inspect/project | local indexed diagnostics; projection one explicit choice | exact result/contact, named loss/malformed errors, numerical/schema proof |
| Preview | one slider; immediate stale-geometry removal | deterministic sampling/identity, measured display budget, no controller |
| RAPID | readable exact source/bundle without file wiring | byte equality, exact profiles/identity, no dense-result path |
| Observe | one connect+observe; freshness/reconnect visible | single owner, bounded reducer/state/reconcile/redaction, RW6/RW7 contracts |
| Deploy/control | one armed deploy pair; separate reset/start; immediate stop | linear journal/idempotency/readback/pre-post/indeterminate/RobotStudio proof |
| Binary | distinct advanced authoring/result UX | separate types/owner/identity, no conversion, version/log/interpreter proof |
| Package/docs | searchable compiled nodes; installed examples solve; help complete | CPython 3.9 compile/deserialize/GHX, exact pins/inventory, no hidden paths |

## 11. Build gates and terminal acceptance

Required gates: ruff; strict mypy on Tesseract/ABB/runtime/contracts; `pytest --testmon -n auto` after changes; full `pytest -n auto`; property/model/concurrency/fault suites; macOS native examples; Windows 3.9 exact-lock component build; every `.ghuser` deserialize; every installed GHX smoke; fake/recorded RW6/RW7 on 3.12 and 3.9; RobotStudio RW6/RW7 for claimed surfaces; supervised physical RW6 record; MkDocs strict; JSON Schema/package inventory/secret/provenance/license scans; `git diff --check`. Reference tests are never weakened.

Terminal scenarios:

1. **Offline native:** installed Basic graph builds artifact/scene including tool/body/attachment/touch links, plans exact pipeline/profile, exposes contacts/diagnostics/result, projects explicitly, scrubs/previews, emits native RAPID, and proves no stale output on every identity change on macOS 3.12 and Windows Rhino 3.9.
2. **RWS native RAPID:** RobotStudio RW6/RW7 graph connects/reconciles, seals/deploys/readback-verifies/loads, separately arms reset/start-once, monitors/program-stops, and injects disconnect at every mutation boundary with exact cancelled/failed/indeterminate/reconciled outcomes.
3. **Observe/I-O:** subscription covers controller/operation/execution/event log/signals, visibly gaps/reconciles after disconnect, performs armed write/readback, and never silently loses an edge under pressure.
4. **Binary:** compatible RobotStudio RW6 interpreter runs exact non-EGM authored artifact, exposes current/queued command, program-stops, downloads/version+shape-validates typed result log, with no Tesseract result/projection/RAPID input.
5. **Package:** clean Windows profile installs exact package, deserializes every node, opens every installed GHX, and completes local/fake smoke with no source-tree dependency.

Final audit must answer yes: exact native access/options preserved; fixed-base scene complete; four artifact kinds non-interchangeable; recompute cannot repeat effects; all pending/stale/failure/cancel/gap/conflict/indeterminate states visible; one controller owner/bounded derived queues; linear idempotent journal/deterministic replay; secrets absent; effects disclose pre/effect/post/cancel/reconnect; factories bypass-safe; strict types carry frames/units/groups/tasks/controllers; responsibilities/lifetimes separated; downstream round trips pass; packaged examples use compiled nodes; exact macOS/Windows pins pass; evidence labels are precise; no placeholder, silent fallback, arbitrary timing literal, skipped/expected-failure test, native weakening, or unsupported hardware claim remains.
