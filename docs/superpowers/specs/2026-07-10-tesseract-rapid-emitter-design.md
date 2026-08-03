# Tesseract RAPID Emitter Design

## Objective

Expose the RAPID emitter shipped by `tesseract-robotics-nanobind` 0.35.0.6 through a typed COMPAS FAB API and compiled Grasshopper components. COMPAS FAB must not implement a second instruction dispatcher, formatter, or RAPID dialect.

The emitter consumes one exact native `CompositeInstruction` and produces RAPID source code. It does not interpret how the instruction was created; source provenance is outside the emitter API.

## Native capability boundary

`TesseractRapidEmitter.emit(...)` accepts:

- an exact `tesseract_robotics.tesseract_command_language.CompositeInstruction`;
- a mapping from exact Tesseract profile names to native `RapidProfile` objects;
- explicit RAPID module and procedure names.

It calls `tesseract_robotics.emitters.rapid.emit_rapid(...)` directly. The returned source must be byte-for-byte identical to a direct native call with the same inputs. The adapter does not walk instructions, select motion commands, convert units, format numbers, or substitute missing profiles.

The native emitter's `RapidEmitterError` hierarchy propagates unchanged. COMPAS FAB adds named errors only for adapter-owned failures such as an invalid source type, invalid profile-map structure, empty module/procedure names, or an explicit file-write failure.

The program check uses the native `CompositeInstruction` class directly. Planning requests, planning results, COMPAS trajectories, and objects exposing a similarly named attribute are rejected; the adapter never unwraps or projects them.

## Python architecture

Each file owns one responsibility:

- `rapid_emitter.py`: validate the adapter boundary, call native `emit_rapid`, and return a typed artifact.
- `rapid_program.py`: immutable emitted-program value and explicit `write(Path)` operation.
- `rapid_identity.py`: SHA-256 identity for the emitted UTF-8 source and emitter configuration.
- `rapid_profiles.py`: validate and merge native `RapidProfile` mappings without cloning or replacing their values.

The public entry point is:

```python
TesseractRapidEmitter.emit(
    program: CompositeInstruction,
    profiles: Mapping[str, RapidProfile],
    module_name: str = "main_program",
    procedure_name: str = "main",
) -> RapidProgram
```

`RapidProgram` is a frozen, slotted attrs value containing:

- the exact `CompositeInstruction` reference supplied by the caller;
- the emitted RAPID source string;
- module and procedure names;
- a `RapidProgramIdentity`.

`RapidProgram.build(...)` owns validation and constructs the identity; direct attrs construction remains an internal bypass-safe representation. `RapidProgramIdentity.build(...)` likewise owns identity validation and hashing.

`RapidProgramIdentity` contains a schema version, COMPAS FAB version, nanobind distribution version, and SHA-256 digest. The digest uses length-prefixed UTF-8 fields covering the emitted source, module name, procedure name, and version fields. Identical emitted artifacts therefore compare structurally; different code or configuration changes the identity. This is a RAPID-specific build-identity primitive rather than an extension of the robot artifact's existing `BuildIdentity`; changing the latter would invalidate established robot artifact digests.

`RapidProgram.write(path: Path) -> Path` is the only filesystem mutation. It requires a `pathlib.Path`, writes the exact UTF-8 source to the exact caller path, does not invent an extension or create parent directories, and raises named `MissingRapidProgramParentError`, `RapidProgramTargetIsDirectoryError`, or `RapidProgramWriteError` failures. A string path is an invalid boundary value rather than a second calling convention.

## Profile mapping

Python callers pass native `RapidProfile` objects directly. COMPAS FAB validates that every mapping key is a non-empty string and every value is a native `RapidProfile`, then creates the concrete `dict` required by the native emitter while retaining the same profile objects.

Grasshopper profile components output native profile maps. Multiple maps are merged before emission. Duplicate Tesseract profile names can arise only at this merge boundary and raise `DuplicateRapidProfileError` instead of using connection order as an implicit override. Invalid keys and values raise `InvalidRapidProfileMapError`.

## Grasshopper components

Two CPython components ship together and compile through the existing locked Windows/Rhino 8 Python 3.9 job.

### Tesseract RAPID Profile

Inputs:

- `profile_names` list of exact Tesseract profile names;
- RAPID speed, zone, tool, and work-object variable names.

Defaults use ABB controller built-ins `v200`, `z10`, `tool0`, and `wobj0`. The output is a mapping whose values are native `RapidProfile` instances with native typed name wrappers.

### Tesseract RAPID

Inputs:

- exact native `CompositeInstruction`;
- list of profile mappings;
- module name;
- procedure name.

Outputs:

- complete `RapidProgram` artifact;
- RAPID source string;
- SHA-256 artifact identity.

The component performs no filesystem write and has no save path or recomputation-triggered side effect. It catches adapter errors and native `RapidEmitterError` values only to report them on the Grasshopper component; it does not replace or retry emission.

Windows build, Yak, and release workflows must require both new `.ghuser` artifacts in addition to the existing Tesseract artifact and planner components.

## Documentation example

A dedicated example loads Tesseract's ABB IRB 2400 support model and authors one local-TCP-Z axis-symmetric Cartesian program. Native Descartes plans that program across the explicit full -pi-to-pi range at 1-degree resolution with redundant joint solutions enabled. Independently, the authored program is converted to an exact Cartesian `CompositeInstruction` and passed to `TesseractRapidEmitter`; the dense `StateWaypoint` planning result is not reinterpreted as controller code. The example writes no file, uses an explicit native profile map, and prints the content identity plus generated source.

The emitter remains independent of planning provenance: it sees only the authored `CompositeInstruction`, never the planning request or result.

## Verification

- The released nanobind RAPID suite remains green; its current 25 tests are an upstream capability baseline, not copied into COMPAS FAB.
- Adapter output equals direct native `emit_rapid` output byte-for-byte for Cartesian, joint, nested, and I/O programs.
- Native `MissingProfileError`, `UnsupportedInstructionError`, and `EmptyProgramError` propagate unchanged.
- The exact input program reference is retained by `RapidProgram`.
- Identity is deterministic and changes with source, module, procedure, or emitter version.
- Profile-map duplicates and invalid values raise named adapter errors.
- `write(Path)` round-trips exact UTF-8 source and fails loudly for invalid destinations.
- Grasshopper source/metadata contracts cover inputs, outputs, native imports, pure behavior, Python 3.9 dependency directives, icons, and required Windows artifacts.
- The real ABB IRB 2400 axis-symmetric planning-and-emission example executes on macOS ARM64 with nanobind 0.35.0.6.
- Ruff, strict mypy, affected testmon, Python 3.12, Rhino Python 3.9, full pytest, MkDocs, and `git diff --check` remain gates.

## Deferred scope

- No ABB controller upload, Robot Web Services integration, or execution control.
- No RAPID parser, simulator, formatter fork, or third-party action model.
- No automatic conversion from COMPAS `JointTrajectory`, `TesseractPlanningRequest`, or `TesseractPlanningResult`.
- No circular-motion reconstruction when the native emitter rejects a `CompositeInstruction` lacking a via point.
- No automatic file saving from Grasshopper.
