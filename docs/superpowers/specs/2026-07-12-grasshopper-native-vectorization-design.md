# Grasshopper-Native Tesseract Vectorization

## Goal

Make Tesseract authoring natural in Grasshopper without building a second tree runtime.
Frames, poses, targets, and programs are series. A single value is only the one-item case.

## Design

Grasshopper owns iteration, matching, paths, grafting, and cross-reference.
COMPAS FAB components remain thin adapters to exact Tesseract nanobind functions.

| Component | Access | Meaning |
|---|---|---|
| Tesseract Pose | item | Map every Frame/Plane to one exact Pose. |
| Tesseract Cartesian Target | item | Map every Pose to one exact target using standard GH matching. |
| Tesseract Joint Target | list | Treat each positions branch as one atomic joint vector. |
| Tesseract State Target | list | Treat each positions/dynamics branch as one atomic state vector. |
| Tesseract Motion Program | list for targets | Reduce each ordered target branch to one exact program. |
| Tesseract Native Plan | item | Plan each program supplied by Grasshopper iteration. |

Pure itemwise nodes use standard Grasshopper longest-list matching. Ordered target branches are
never flattened or reordered inside COMPAS FAB.

## Constraints

- Keep exact Tesseract native objects, move types, profiles, joint order, working frames, and defaults.
- Never add a production Tree, coordinate, identity, generation, authority, or matching framework.
- Prefer existing component scripts and metadata over new Python modules.
- No new production file over 150 lines without explicit review.
- No class when a function or existing type suffices.
- Errors use the existing Tesseract backend hierarchy; no class-per-message taxonomy.
- Rhino 8 remains Python 3.9; `tesseract-robotics-nanobind==0.35.0.6` remains exact.

## Proof

Tests pin every relevant port to item or list access and exercise exact native factories.
Examples start with ragged, multi-branch inputs and include the one-item convenience case.
Windows CI compiles and inventories every required `.ghuser` artifact.
Actual Rhino acceptance must prove nonzero paths, multiple branches, joint-vector atomicity,
one program per target branch, and exact native output types before runtime parity is claimed.

## Next Slice

After authoring access is proven, add kinematics, planning inspection, scrub, and preview nodes
using the same item/list model. Mobile/VKC and concurrent TrajOpt remain out of scope.
