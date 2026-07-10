# Collision Mesh Policy Loading Design

## Objective

Expose collision-mesh treatment at every Tesseract robot-loading boundary and default it to `CollisionMeshPolicy.CONVEX_HULL`. Explicit `CollisionMeshPolicy.PRESERVE` selection remains supported and is never replaced silently.

The example suite must also demonstrate why native Tesseract access is retained: its native planning example plans an axis-symmetric Cartesian path by allowing Descartes to sample rotations about the local TCP Z axis.

## API contract

- `RobotArtifactLoader.load(collision_mesh_policy=CollisionMeshPolicy.CONVEX_HULL)` provides the Python loading default.
- `CompasRobotArtifactCompiler.build(..., collision_mesh_policy=CollisionMeshPolicy.CONVEX_HULL, ...)` exposes the same default at the higher-level COMPAS loading boundary.
- `Cf_TesseractRobotArtifact` keeps `collision_mesh_policy` as an input and defaults it to `convex_hull`.
- `RobotArtifact.from_compas_urdf(...)` remains explicit because it is a low-level artifact constructor, not a loading convenience API.

The selected policy remains part of the content-addressed artifact through the compiled URDF and therefore changes `BuildIdentity` when changed.

## Axis-symmetric planning example

- The conventional example remains the Phase 1 `ConfigurationTarget` baseline and loads through the convex-hull default.
- The native example derives two reachable Cartesian poses from UR5 forward kinematics, then submits them as an exact `MotionProgram` to `DescartesFPipeline`.
- `create_descartes_pipeline_profiles(sample_axis=(0, 0, 1), sample_resolution=radians(30), use_redundant_joint_solutions=True)` delegates both local tool-Z rotational sampling and redundant joint-solution enumeration to Tesseract 0.35.0.6.
- The backend must not discretize rotations, select an inverse-kinematics branch, or lower the native program through a COMPAS target type.

This deliberately keeps conventional Cartesian lowering out of Phase 1. Adding it now would create a second semantics path before COMPAS has a type capable of expressing axis symmetry without losing Tesseract controls.

## macOS OpenMP runtime contract

The released nanobind wheel bundles the `libomp.dylib` used by `libdescartes_light.dylib`. Conda-forge's OpenMP-flavoured OpenBLAS initializes a second, differently identified `libomp.dylib` through NumPy and causes OMP Error #15 when Descartes begins solving. Both Pixi environments therefore constrain `libopenblas` to the conda-forge pthreads build on macOS ARM64 and Windows. The lockfile is a tested part of this contract; `KMP_DUPLICATE_LIB_OK` is forbidden.

## Failure model

Unknown Grasshopper policy values continue to raise `InvalidTesseractSelectionError`. A source URDF containing a conflicting explicit `tesseract:make_convex` value continues to raise `CollisionMeshPolicyConflictError`. No fallback or automatic retry changes the caller's selection.

## Verification

- A loader call with no policy produces `tesseract:make_convex="true"`.
- A compiler built with no policy retains `CollisionMeshPolicy.CONVEX_HULL` and compiles a convex-hull artifact.
- Explicit `PRESERVE` still produces `tesseract:make_convex="false"`.
- Grasshopper source and metadata expose the input and declare `convex_hull` as its default.
- Documentation and examples describe convex hull as the loading default while explaining how to select exact triangle meshes.
- Both examples omit the defaulted mesh-policy argument, proving the public loading default.
- The native example executes successfully against the bundled UR5 resources with `DescartesFPipeline` and the released nanobind 0.35.0.6 profile API.
- Source-contract tests require the local TCP Z sampling axis, a named angular sampling resolution, redundant joint-solution enumeration, and the native Descartes pipeline.
- The Pixi manifest and lockfile require pthreads OpenBLAS, and the native Descartes example executes without an OpenMP escape hatch.
