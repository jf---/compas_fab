# Native Tesseract Kinematics in Grasshopper

## Goal

Expose existing native FK, IK, and collision APIs as vectorized Grasshopper components.

## Components

All inputs use item access; Grasshopper owns iteration and matching.

| Component | Native call | Outputs |
|---|---|---|
| Tesseract Native FK | `forward_kinematics_native` | exact metre `Pose` |
| Tesseract Native IK | `inverse_kinematics_native` | exact result/input/pose, group, joint names, all solutions |
| Tesseract Contact Request | `build_contact_request` | exact native `ContactRequest` |
| Tesseract Native Collision | `check_collision_native` | exact result/map, all contacts, colliding contacts, collision flag |

IK solutions retain native solver order. Collision retains every native contact; convenience
filtering uses the existing distance predicate. Collision requires an exact native request.
The request node exposes all released `ContactTestType` values and native calculation flags.

## Constraints

- Add no backend code, shared helper, tree model, or wrapper type.
- Each component script stays under 100 lines.
- Preserve exact native objects and catch only `TesseractBackendError`.
- Pin `tesseract-robotics-nanobind==0.35.0.6`.
- Windows CI must compile and inventory all three `.ghuser` artifacts.
- Metadata and emulation tests do not claim actual Rhino path behavior.
