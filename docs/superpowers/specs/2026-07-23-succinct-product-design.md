# COMPAS FAB × Tesseract × ABB — succinct product contract

**Status:** supersedes `2026-07-11-grasshopper-tesseract-abb-product-design.md` (retained as reference)
**Date:** 2026-07-23

Grasshopper in, Tesseract out, robot on the wire.

## Principles

1. A component is a thin marshal: GH inputs → one backend call → native outputs. Grasshopper owns iteration and matching (item access lifts over trees; list access runs once per branch — "one program per branch" is native behavior). A component adds only the checks that prevent a wrong toolpath — lengths, DOF, finite numbers — reported via `AddRuntimeMessage`.
2. The backend stays a thin typed layer over a heavily-tested lib (`tesseract-robotics-nanobind`): load cell, author targets/programs, plan, emit RAPID. Typed quantities guard the native boundary, one named exception per failure mode, strict mypy. The `NativePlanCall` content cache is the plan cache; nothing more.
3. Controller mutations are armed, fire exactly once (edge-triggered token — Grasshopper recompute must never re-fire a start), and are believed only after readback. Stop is instant and unconditional. One small safety module, one thin RWS client. RWS client dependency: chosen by Jelle before W2 starts.
4. Grasshopper's nastiness — state across solves, canvas mutation timing — is answered with the small idiomatic helpers that already exist (`sticky_cache`, `ScheduleSolution`, `ensure_*` value lists), never with frameworks.
5. Good gardeners weed: anything not consumed by a shipped component or the backend gets cut.

## Weeded 2026-07-23

The unwired tree/identity/series framework (26 `ghpython` modules + 20 test files): it re-implemented Grasshopper's own tree handling in Python, plus a provenance/currentness model sized for a problem the product does not have. Deleted, not deferred. The neutral `IdentityVerification` enum and the backend plan cache stay.

## Map

- **W1** — tree-enable the authoring components via GH access declarations + equal-length zip guards where silent repetition would mean a wrong toolpath.
- **W2** — controller parity (the RobotComponents gap): ~9 scalar components — controller endpoint, session, coarse state, sealed bundle + deploy with readback verification, reset, start, program stop, I/O read/write — plus the safety core. Evidence: fake + recorded + RobotStudio RW6; no physical claim.
- **W3** — scene components (tool, rigid body, attach, touch links) over native environment commands.
- **W4** — as demand dictates: projection polish, external axes (track/positioner — the one permanent RobotComponents gap), binary playback, packaging.

Out unless demand proves otherwise: journals/replay, subscriptions, ordered command batches, JSON schema manifests, calibrated performance policies, per-node documentation ceremony.
