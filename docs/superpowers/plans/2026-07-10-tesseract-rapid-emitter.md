# Tesseract RAPID Emitter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Ship a typed, content-addressed COMPAS FAB adapter and compiled Grasshopper components around Tesseract nanobind's native RAPID emitter.

**Architecture:** The adapter accepts only an exact native `CompositeInstruction`, validates native `RapidProfile` maps, serializes calls to Tesseract's process-global `RapidWriter`, and returns immutable RAPID source plus identity. Grasshopper constructs native profiles and invokes the same adapter; Windows CI compiles and requires both new user objects. An ABB IRB 2400 Descartes example demonstrates local-TCP-Z redundancy before emission without making planning provenance part of the emitter.

**Tech Stack:** Python 3.9/3.12, attrs, pathlib, SHA-256, tesseract-robotics-nanobind 0.35.0.6, Grasshopper CPython componentizer, pytest-xdist/testmon, Ruff, mypy strict, MkDocs.

## Global Constraints

- Require `tesseract-robotics-nanobind==0.35.0.6`; never install or import `tesseract_python` or `tesseract-python`.
- Python 3.9 compatibility is mandatory for Rhino 8; macOS ARM64/Python 3.12 remains the primary runtime.
- `TesseractRapidEmitter.emit` accepts only a native `CompositeInstruction`; it never unwraps planning requests/results or converts COMPAS trajectories.
- Native `emit_rapid` remains the sole dispatcher, unit converter, and formatter; output must equal a direct native call byte-for-byte.
- Native `RapidEmitterError` subclasses propagate unchanged; adapter-owned failures use named `TesseractBackendError` subclasses.
- No filesystem mutation occurs during emission or Grasshopper recomputation; only explicit `RapidProgram.write(Path)` writes.
- No fallback, retry, alternate RAPID formatter, conditional import, skip, xfail, `HAS_*`, or `__all__` path.
- Run every pytest command with `-n auto`; run affected tests with `--testmon` after changes.
- Do not commit or push without Jelle's explicit instruction.

---

### Task 1: Native RAPID Profile Boundary

**Files:**
- Modify: `src/compas_fab/backends/tesseract/errors.py`
- Create: `src/compas_fab/backends/tesseract/rapid_profiles.py`
- Create: `tests/backends/tesseract/test_rapid_profiles.py`

**Interfaces:**
- Consumes: native `RapidProfile` from `tesseract_robotics.emitters.rapid`.
- Produces: `normalize_rapid_profiles(profiles: Mapping[str, RapidProfile]) -> dict[str, RapidProfile]` and `merge_rapid_profile_maps(profile_maps: Iterable[Mapping[str, RapidProfile]]) -> dict[str, RapidProfile]`.

- [x] **Step 1: Write failing normalization and merge tests**

```python
def test_normalize_retains_exact_native_profile_objects():
    profile = RapidProfile()
    normalized = normalize_rapid_profiles({"DEFAULT": profile})
    assert normalized == {"DEFAULT": profile}
    assert normalized["DEFAULT"] is profile


@pytest.mark.parametrize("profiles", [{"": RapidProfile()}, {"   ": RapidProfile()}, {1: RapidProfile()}, {"P": object()}])
def test_normalize_rejects_invalid_native_map(profiles):
    with pytest.raises(InvalidRapidProfileMapError):
        normalize_rapid_profiles(profiles)


def test_merge_rejects_duplicate_exact_tesseract_name():
    with pytest.raises(DuplicateRapidProfileError, match="DEFAULT"):
        merge_rapid_profile_maps([{"DEFAULT": RapidProfile()}, {"DEFAULT": RapidProfile()}])
```

- [x] **Step 2: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_rapid_profiles.py -n auto -q`

Expected: collection fails because `rapid_profiles` and named errors do not exist.

- [x] **Step 3: Add exact named errors and implementation**

```python
class InvalidRapidProfileMapError(TesseractBackendError):
    """A RAPID profile map does not contain exact native values."""


class DuplicateRapidProfileError(TesseractBackendError):
    """More than one profile map defines the same Tesseract profile name."""
```

```python
def normalize_rapid_profiles(profiles: Mapping[str, RapidProfile]) -> dict[str, RapidProfile]:
    if not isinstance(profiles, Mapping):
        raise InvalidRapidProfileMapError("RAPID profiles must be a mapping.")
    normalized: dict[str, RapidProfile] = {}
    for name, profile in profiles.items():
        if not isinstance(name, str) or not name.strip():
            raise InvalidRapidProfileMapError("RAPID profile names must be non-empty strings.")
        if not isinstance(profile, RapidProfile):
            raise InvalidRapidProfileMapError("RAPID profile {!r} must be RapidProfile, got {}.".format(name, type(profile).__name__))
        normalized[name] = profile
    return normalized


def merge_rapid_profile_maps(profile_maps: Iterable[Mapping[str, RapidProfile]]) -> dict[str, RapidProfile]:
    merged: dict[str, RapidProfile] = {}
    for profiles in profile_maps:
        for name, profile in normalize_rapid_profiles(profiles).items():
            if name in merged:
                raise DuplicateRapidProfileError("Duplicate Tesseract RAPID profile {!r}.".format(name))
            merged[name] = profile
    return merged
```

- [x] **Step 4: Verify GREEN**

Run: `pixi run pytest tests/backends/tesseract/test_rapid_profiles.py -n auto -q`

Expected: all profile tests pass.

### Task 2: Content-Addressed RAPID Artifact

**Files:**
- Modify: `src/compas_fab/backends/tesseract/errors.py`
- Create: `src/compas_fab/backends/tesseract/rapid_identity.py`
- Create: `src/compas_fab/backends/tesseract/rapid_program.py`
- Create: `tests/backends/tesseract/test_rapid_identity.py`
- Create: `tests/backends/tesseract/test_rapid_program.py`

**Interfaces:**
- Consumes: exact native `CompositeInstruction`, emitted UTF-8 source, module/procedure names.
- Produces: semantic `NewType` values, `RapidProgramIdentity.build(...)`, `RapidProgram.build(...)`, and `RapidProgram.write(Path)`.

- [x] **Step 1: Write failing identity tests**

```python
def test_identity_is_deterministic_and_versioned():
    left = RapidProgramIdentity.build("MODULE M\nENDMODULE\n", "M", "main")
    right = RapidProgramIdentity.build("MODULE M\nENDMODULE\n", "M", "main")
    assert left == right
    assert len(left.digest) == 64
    assert left.schema_version == RAPID_IDENTITY_SCHEMA_VERSION
    assert left.compas_fab_version
    assert left.tesseract_version == "0.35.0.6"


@pytest.mark.parametrize(
    ("source", "module_name", "procedure_name"),
    [("changed", "M", "main"), ("source", "Changed", "main"), ("source", "M", "changed")],
)
def test_identity_changes_with_every_covered_field(source, module_name, procedure_name):
    baseline = RapidProgramIdentity.build("source", "M", "main")
    assert RapidProgramIdentity.build(source, module_name, procedure_name) != baseline
```

- [x] **Step 2: Write failing artifact and exact-write tests**

```python
def test_program_retains_exact_composite_reference(native_program):
    artifact = RapidProgram.build(native_program, "MODULE M\nENDMODULE\n", "M", "main")
    assert artifact.program is native_program
    assert artifact.source == "MODULE M\nENDMODULE\n"


def test_program_write_round_trips_exact_utf8(native_program, tmp_path):
    artifact = RapidProgram.build(native_program, "MODULE M\n  ! λ\nENDMODULE\n", "M", "main")
    target = tmp_path / "program.mod"
    assert artifact.write(target) == target
    assert target.read_bytes() == artifact.source.encode("utf-8")


def test_program_write_rejects_string_path(native_program, tmp_path):
    artifact = RapidProgram.build(native_program, "source", "M", "main")
    with pytest.raises(InvalidRapidProgramPathError):
        artifact.write(str(tmp_path / "program.mod"))
```

```python
def test_program_write_rejects_missing_parent(native_program, tmp_path):
    artifact = RapidProgram.build(native_program, "source", "M", "main")
    with pytest.raises(MissingRapidProgramParentError):
        artifact.write(tmp_path / "missing" / "program.mod")


def test_program_write_rejects_directory_target(native_program, tmp_path):
    artifact = RapidProgram.build(native_program, "source", "M", "main")
    with pytest.raises(RapidProgramTargetIsDirectoryError):
        artifact.write(tmp_path)


def test_program_write_translates_os_error(native_program, tmp_path, mocker):
    artifact = RapidProgram.build(native_program, "source", "M", "main")
    mocker.patch("pathlib.Path.open", side_effect=OSError("denied"))
    with pytest.raises(RapidProgramWriteError, match="denied"):
        artifact.write(tmp_path / "program.mod")
```

- [x] **Step 3: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_rapid_identity.py tests/backends/tesseract/test_rapid_program.py -n auto -q`

Expected: collection fails because identity/artifact modules and errors do not exist.

- [x] **Step 4: Implement semantic identity types and length-prefixed hashing**

```python
RapidSource = NewType("RapidSource", str)
RapidModuleName = NewType("RapidModuleName", str)
RapidProcedureName = NewType("RapidProcedureName", str)
Sha256Digest = NewType("Sha256Digest", str)
RAPID_IDENTITY_SCHEMA_VERSION = "1"


def _field_bytes(value: str) -> bytes:
    payload = value.encode("utf-8")
    return len(payload).to_bytes(8, byteorder="big") + payload


@define(frozen=True, slots=True)
class RapidProgramIdentity:
    digest: Sha256Digest
    schema_version: str
    compas_fab_version: str
    tesseract_version: str

    @classmethod
    def build(cls, source: str, module_name: str, procedure_name: str) -> RapidProgramIdentity:
        compas_fab_version = version("compas-fab")
        tesseract_version = version("tesseract-robotics-nanobind")
        fields = (RAPID_IDENTITY_SCHEMA_VERSION, compas_fab_version, tesseract_version, source, module_name, procedure_name)
        digest = hashlib.sha256(b"".join(_field_bytes(field) for field in fields)).hexdigest()
        return cls(Sha256Digest(digest), RAPID_IDENTITY_SCHEMA_VERSION, compas_fab_version, tesseract_version)
```

- [x] **Step 5: Implement the frozen artifact and explicit writer**

Add these named `TesseractBackendError` subclasses: `InvalidRapidProgramError`, `InvalidRapidProgramNameError`, `InvalidRapidProgramPathError`, `MissingRapidProgramParentError`, `RapidProgramTargetIsDirectoryError`, and `RapidProgramWriteError`. Implement the artifact contract exactly:

```python
def validate_rapid_program(program: object) -> CompositeInstruction:
    if not isinstance(program, CompositeInstruction):
        raise InvalidRapidProgramError("RAPID emission requires CompositeInstruction, got {}.".format(type(program).__name__))
    return program


def validate_rapid_name(value: object, kind: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise InvalidRapidProgramNameError("RAPID {} name must be a non-empty string.".format(kind))
    return value


@define(frozen=True, slots=True)
class RapidProgram:
    program: CompositeInstruction
    source: RapidSource
    module_name: RapidModuleName
    procedure_name: RapidProcedureName
    identity: RapidProgramIdentity

    @classmethod
    def build(cls, program: object, source: str, module_name: object, procedure_name: object) -> RapidProgram:
        native_program = validate_rapid_program(program)
        if not isinstance(source, str) or not source:
            raise InvalidRapidProgramError("Emitted RAPID source must be a non-empty string.")
        module = validate_rapid_name(module_name, "module")
        procedure = validate_rapid_name(procedure_name, "procedure")
        return cls(
            native_program,
            RapidSource(source),
            RapidModuleName(module),
            RapidProcedureName(procedure),
            RapidProgramIdentity.build(source, module, procedure),
        )

    def write(self, path: Path) -> Path:
        if not isinstance(path, Path):
            raise InvalidRapidProgramPathError("RAPID output path must be pathlib.Path.")
        if not path.parent.exists():
            raise MissingRapidProgramParentError("RAPID output parent does not exist: {}.".format(path.parent))
        if path.is_dir():
            raise RapidProgramTargetIsDirectoryError("RAPID output target is a directory: {}.".format(path))
        try:
            with path.open("w", encoding="utf-8", newline="") as stream:
                stream.write(self.source)
        except OSError as write_error:
            raise RapidProgramWriteError("Cannot write RAPID output {}: {}.".format(path, write_error)) from write_error
        return path
```

- [x] **Step 6: Verify GREEN**

Run: `pixi run pytest tests/backends/tesseract/test_rapid_identity.py tests/backends/tesseract/test_rapid_program.py -n auto -q`

Expected: all identity and artifact tests pass.

### Task 3: Thin Native Emitter Adapter

**Files:**
- Create: `src/compas_fab/backends/tesseract/rapid_emitter.py`
- Modify: `src/compas_fab/backends/tesseract/__init__.py`
- Create: `tests/backends/tesseract/test_rapid_emitter.py`

**Interfaces:**
- Consumes: `CompositeInstruction`, profile normalization from Task 1, `RapidProgram.build` from Task 2, native `emit_rapid`.
- Produces: `TesseractRapidEmitter.emit(program, profiles, module_name="main_program", procedure_name="main") -> RapidProgram`.

- [x] **Step 1: Write failing native parity and reference tests**

```python
def test_adapter_output_is_byte_identical_to_native(native_linear_program, native_profiles):
    expected = emit_rapid(native_linear_program, native_profiles, module_name="Cell", proc_name="Run")
    actual = TesseractRapidEmitter.emit(native_linear_program, native_profiles, "Cell", "Run")
    assert actual.source == expected
    assert actual.program is native_linear_program
```

Parameterize parity across Cartesian linear/freespace, six-axis joint, nested composite, Wait/Timer/SetDigital/SetAnalog/SetTool programs using native instruction constructors. Do not copy upstream golden source; compare direct native calls.

- [x] **Step 2: Write failing exact-boundary and native-error tests**

```python
@pytest.mark.parametrize("invalid", [object(), TesseractPlanningRequest, TesseractPlanningResult])
def test_adapter_rejects_non_composite_values(invalid, native_profiles):
    with pytest.raises(InvalidRapidProgramError):
        TesseractRapidEmitter.emit(invalid, native_profiles)


def test_native_missing_profile_error_propagates_unchanged(native_linear_program):
    with pytest.raises(MissingProfileError) as caught:
        TesseractRapidEmitter.emit(native_linear_program, {})
    assert type(caught.value) is MissingProfileError
```

```python
def test_native_unsupported_instruction_error_propagates_unchanged(native_circular_program, native_profiles):
    with pytest.raises(UnsupportedInstructionError) as caught:
        TesseractRapidEmitter.emit(native_circular_program, native_profiles)
    assert type(caught.value) is UnsupportedInstructionError


def test_native_empty_program_error_propagates_unchanged():
    with pytest.raises(EmptyProgramError) as caught:
        TesseractRapidEmitter.emit(CompositeInstruction("empty"), {})
    assert type(caught.value) is EmptyProgramError


@pytest.mark.parametrize(("module_name", "procedure_name"), [("", "main"), ("M", "   ")])
def test_empty_rapid_names_fail_before_native_call(native_linear_program, native_profiles, module_name, procedure_name):
    with pytest.raises(InvalidRapidProgramNameError):
        TesseractRapidEmitter.emit(native_linear_program, native_profiles, module_name, procedure_name)
```

- [x] **Step 3: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_rapid_emitter.py -n auto -q`

Expected: collection fails because `TesseractRapidEmitter` does not exist.

- [x] **Step 4: Implement direct, serialized delegation**

```python
_RAPID_WRITER_LOCK = RLock()


class TesseractRapidEmitter:
    @staticmethod
    def emit(
        program: CompositeInstruction,
        profiles: Mapping[str, RapidProfile],
        module_name: str = "main_program",
        procedure_name: str = "main",
    ) -> RapidProgram:
        validated_program = validate_rapid_program(program)
        validated_profiles = normalize_rapid_profiles(profiles)
        validated_module = validate_rapid_name(module_name, "module")
        validated_procedure = validate_rapid_name(procedure_name, "procedure")
        with _RAPID_WRITER_LOCK:
            source = emit_rapid(
                validated_program,
                validated_profiles,
                module_name=validated_module,
                proc_name=validated_procedure,
            )
        return RapidProgram.build(validated_program, source, validated_module, validated_procedure)
```

Do not catch native exceptions. Re-export only the new COMPAS FAB values from the backend's minimal `__init__.py`; native errors/profiles remain imported from their native package.

- [x] **Step 5: Verify GREEN and upstream baseline**

Run: `pixi run pytest tests/backends/tesseract/test_rapid_emitter.py -n auto -q`

Run: `pixi run pytest /Users/jelle/Code/CADCAM/tesseract_python_nanobind/tests/emitters/rapid -n auto -q`

Expected: adapter tests pass and the 25 released-emitter baseline tests remain green.

### Task 4: Grasshopper Components and Windows Artifact Gates

**Files:**
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractRapidProfile/code.py`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractRapidProfile/metadata.json`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractRapidProfile/icon.svg`
- Create by rendering SVG: `src/compas_fab/ghpython/components_cpython/Cf_TesseractRapidProfile/icon.png`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractRapid/code.py`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractRapid/metadata.json`
- Create: `src/compas_fab/ghpython/components_cpython/Cf_TesseractRapid/icon.svg`
- Create by rendering SVG: `src/compas_fab/ghpython/components_cpython/Cf_TesseractRapid/icon.png`
- Modify: `tests/backends/tesseract/test_grasshopper_components.py`
- Modify: `.github/workflows/build.yml`
- Modify: `.github/workflows/publish_yak.yml`
- Modify: `.github/workflows/release.yml`
- Modify: `docs/developer/grasshopper.md`

**Interfaces:**
- Consumes: native typed `SpeedName`, `ZoneName`, `ToolName`, `WobjName`, native `RapidProfile`, Task 1 merge function, Task 3 emitter.
- Produces: pure `Tesseract RAPID Profile` and `Tesseract RAPID` components plus four-artifact Windows gates.

- [x] **Step 1: Write failing source, metadata, icon, and workflow contract tests**

Assert exact profile inputs `profile_names`, `speed`, `zone`, `tool`, `workobject`; exact emitter inputs `program`, `profile_maps`, `module_name`, `procedure_name`; exact outputs `profiles` and `rapid_program`, `source`, `program_id`. Assert native typed wrappers appear in profile code, `merge_rapid_profile_maps` and `TesseractRapidEmitter.emit` appear in emitter code, no save path exists, both sources contain the released nanobind directive, both PNGs are 24×24, and every Windows workflow requires:

```text
Cf_TesseractRobotArtifact.ghuser
Cf_TesseractPlanner.ghuser
Cf_TesseractRapidProfile.ghuser
Cf_TesseractRapid.ghuser
```

- [x] **Step 2: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Expected: missing component directories and workflow names fail.

- [x] **Step 3: Implement the profile component**

Construct one native profile using `SpeedName(speed or "v200")`, `ZoneName(zone or "z10")`, `ToolName(tool or "tool0")`, and `WobjName(workobject or "wobj0")`. Build one-item maps for every supplied exact profile name and call `merge_rapid_profile_maps`; catch only `TesseractBackendError` for Grasshopper error reporting. Return `None` when no names are connected.

- [x] **Step 4: Implement the pure emitter component**

Return `(None, None, None)` for an absent program. Otherwise merge connected profile maps, call `TesseractRapidEmitter.emit(program, profiles, module_name or "main_program", procedure_name or "main")`, and return `(rapid_program, str(rapid_program.source), str(rapid_program.identity.digest))`. Catch only `(TesseractBackendError, RapidEmitterError)` and never write, retry, unwrap, or convert the program.

- [x] **Step 5: Add metadata, icons, and required artifacts**

Use the existing Tesseract grey/teal icon system. The profile icon depicts four stacked parameter bands; the emitter icon depicts a document with `MODULE`-like code lines. Render exact 24×24 PNGs using:

Run: `rsvg-convert -w 24 -h 24 <icon.svg> -o <icon.png>`

Extend each PowerShell gate to this exact array and update the developer guide from two to four required Tesseract user objects:

```powershell
$required = @(
  "Cf_TesseractRobotArtifact.ghuser",
  "Cf_TesseractPlanner.ghuser",
  "Cf_TesseractRapidProfile.ghuser",
  "Cf_TesseractRapid.ghuser"
)
```

- [x] **Step 6: Verify GREEN**

Run: `pixi run pytest tests/backends/tesseract/test_grasshopper_components.py -n auto -q`

Expected: component contracts and all Windows artifact-presence tests pass.

### Task 5: ABB IRB 2400 Axis-Symmetric Planning and Emission Example

**Files:**
- Create: `docs/backends/tesseract/files/03_rapid_emitter.py`
- Modify: `docs/backends/tesseract.md`
- Modify: `tests/backends/tesseract/test_documented_examples.py`

**Interfaces:**
- Consumes: native `Robot.from_tesseract_support("abb_irb2400")`, `MotionProgram`, `CartesianTarget`, `create_descartes_pipeline_profiles`, `TaskComposer`, exact authored `CompositeInstruction`, `TesseractRapidEmitter`.
- Produces: executable ABB example proving local-TCP-Z sampling and emission with no file write.

- [x] **Step 1: Write failing documentation source contract**

```python
def test_rapid_example_plans_abb_axis_symmetry_before_native_emission():
    source = RAPID_EXAMPLE.read_text(encoding="utf-8")
    assert 'Robot.from_tesseract_support("abb_irb2400")' in source
    assert 'pipeline="DescartesFPipeline"' in source
    assert "TOOL_Z_AXIS = (0.0, 0.0, 1.0)" in source
    assert "sample_axis=TOOL_Z_AXIS" in source
    assert "use_redundant_joint_solutions=True" in source
    assert "TesseractRapidEmitter.emit(" in source
    assert ".write(" not in source
```

- [x] **Step 2: Verify RED**

Run: `pixi run pytest tests/backends/tesseract/test_documented_examples.py -n auto -q`

Expected: the third example file is absent.

- [x] **Step 3: Implement the exact native ABB workflow**

Load `abb_irb2400`, use two proven-reachable Cartesian targets, build a Cartesian `MotionProgram`, and plan it directly with native `TaskComposer`:

```python
TOOL_Z_AXIS = (0.0, 0.0, 1.0)
TOOL_AXIS_SAMPLE_STEP = Radians(radians(1.0))
TOOL_AXIS_SAMPLE_MIN = Radians(-pi)
TOOL_AXIS_SAMPLE_MAX = Radians(pi)

profiles = create_descartes_pipeline_profiles(
    sample_axis=TOOL_Z_AXIS,
    sample_resolution=TOOL_AXIS_SAMPLE_STEP,
    sample_min=TOOL_AXIS_SAMPLE_MIN,
    sample_max=TOOL_AXIS_SAMPLE_MAX,
    use_redundant_joint_solutions=True,
)
result = TaskComposer.from_config().plan(
    robot,
    program,
    pipeline="DescartesFPipeline",
    profiles=profiles,
)
assert result.successful, result.message
authored_program = program.to_composite_instruction(joint_names, "tool0")
rapid_program = TesseractRapidEmitter.emit(
    authored_program,
    {"DEFAULT": RapidProfile()},
    module_name="AxisSymmetricPath",
    procedure_name="main",
)
```

Supply only `{"DEFAULT": RapidProfile()}` for the authored program; never reinterpret the dense planned `StateWaypoint` result as controller code. Assert planning success, print identity and source, and perform no filesystem write.

- [x] **Step 4: Document RAPID capability and Grasshopper flow**

Add a RAPID section stating that only exact `CompositeInstruction` is accepted, native errors pass through, profiles map Tesseract names to ABB variables, and `RapidProgram.write(Path)` is explicit. Embed `03_rapid_emitter.py`; update the Grasshopper section to explain profile-map merging and pure source output.

- [x] **Step 5: Verify source contract and execute example**

Run: `pixi run pytest tests/backends/tesseract/test_documented_examples.py -n auto -q`

Run: `pixi run python docs/backends/tesseract/files/03_rapid_emitter.py`

Expected: tests pass; Descartes reports success; output contains a RAPID `MODULE`; process exits zero on macOS ARM64 with nanobind 0.35.0.6.

### Task 6: Regression Gates

**Files:**
- Verify all files in Tasks 1–5 plus the approved design and this plan.

**Interfaces:**
- Consumes: complete RAPID adapter, Grasshopper components, workflows, and documentation.
- Produces: formatted, typed, regression-tested branch state without a commit.

- [x] **Step 1: Run affected tests through testmon**

Run: `pixi run pytest tests/backends/tesseract --testmon -n auto -q`

Expected: every selected test passes; no new skip or xfail exists.

- [x] **Step 2: Format, lint, and strict-type the backend**

Run: `pixi run format-tesseract`

Run: `pixi run lint-tesseract`

Run: `pixi run typecheck-tesseract`

Expected: Ruff changes are retained, Ruff reports no errors, and mypy strict reports success.

- [x] **Step 3: Verify Python 3.9 and 3.12 backend suites**

Run: `pixi run pytest tests/backends/tesseract -n auto -q`

Run: `pixi run -e rhino39 pytest tests/backends/tesseract -n auto -q`

Expected: every Tesseract backend test passes in both environments.

- [x] **Step 4: Run the complete repository suite and documentation build**

Run: `pixi run pytest -n auto -q`

Run: `pixi run invoke docs`

Expected: full tests pass with only existing opt-in ROS skips; MkDocs succeeds with at most the pre-existing print-site ordering warning.

- [x] **Step 5: Inspect final repository state**

Run: `git diff --check`

Run: `git status --short --branch`

Expected: diff check is silent; all RAPID changes remain uncommitted until explicit authorization.
