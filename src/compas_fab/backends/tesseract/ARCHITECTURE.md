# compas_fab Tesseract Backend Architecture

## Overview

This backend integrates [Tesseract Motion Planning Framework](https://github.com/tesseract-robotics/tesseract) 
with compas_fab via the nanobind Python bindings from [tesseract_nanobind](https://github.com/tesseract-robotics/tesseract_nanobind).

## Key Capabilities

| Feature | Tesseract Component | compas_fab Interface |
|---------|---------------------|---------------------|
| Forward Kinematics | `KinematicsPluginFactory` | `ForwardKinematics` |
| Inverse Kinematics | `KinematicsPluginFactory` | `InverseKinematics` |
| Motion Planning (OMPL) | `OMPLMotionPlanner` | `PlanMotion` |
| Motion Planning (TrajOpt) | `TrajOptMotionPlanner` | `PlanMotion` |
| Cartesian Planning | `SimpleMotionPlanner` | `PlanCartesianMotion` |
| Collision Checking | `DiscreteContactManager` | `AddCollisionMesh`, etc. |
| Time Parameterization | `TOTG`, `ISP` | Post-processing |

## Module Structure

```
compas_fab_tesseract/
├── __init__.py
├── client.py              # TesseractClient (ClientInterface)
├── planner.py             # TesseractPlanner (PlannerInterface)
├── backend_features/
│   ├── __init__.py
│   ├── forward_kinematics.py
│   ├── inverse_kinematics.py
│   ├── plan_motion.py
│   ├── plan_cartesian_motion.py
│   ├── collision_mesh.py
│   └── planning_scene.py
├── conversions.py         # Type conversions between compas/tesseract
├── environment.py         # Tesseract Environment management
├── exceptions.py          # Backend-specific exceptions
└── utils.py               # Utilities
```

## Type Mappings

### Geometry Conversions

| compas_fab | Tesseract |
|------------|-----------|
| `compas.geometry.Frame` | `Eigen::Isometry3d` |
| `compas.geometry.Point` | `Eigen::Vector3d` |
| `compas.geometry.Quaternion` | `Eigen::Quaterniond` |
| `compas_fab.robots.Configuration` | `JointWaypoint` / `StateWaypoint` |
| `compas_fab.robots.JointTrajectory` | `CompositeInstruction` result |
| `compas.datastructures.Mesh` | `tesseract_geometry.Mesh` |

### Robot Model Mapping

| compas_fab | Tesseract |
|------------|-----------|
| `compas.robots.RobotModel` (URDF) | `Environment` (URDF/SRDF) |
| Planning group | `KinematicGroup` |
| Joint names | Joint names (direct) |

## Usage Pattern

```python
from compas.geometry import Frame
from compas_fab_tesseract import TesseractClient

# Context manager handles Environment lifecycle
with TesseractClient() as client:
    # Load robot from URDF/SRDF
    robot = client.load_robot(urdf_path, srdf_path)
    
    # Forward kinematics
    frame = robot.forward_kinematics(configuration)
    
    # Inverse kinematics  
    config = robot.inverse_kinematics(frame, start_configuration)
    
    # Motion planning with OMPL
    trajectory = robot.plan_motion(
        goal_constraints,
        start_configuration,
        options={'planner': 'ompl', 'planner_id': 'RRTConnect'}
    )
    
    # Motion planning with TrajOpt
    trajectory = robot.plan_motion(
        goal_constraints,
        start_configuration,
        options={'planner': 'trajopt'}
    )
    
    # Cartesian motion
    trajectory = robot.plan_cartesian_motion(
        frames,
        start_configuration,
        options={'planner': 'simple'}
    )
```

## Implementation Notes

### Cross-Module RTTI Workaround

Due to nanobind's type erasure across shared library boundaries, the tesseract bindings 
use helper functions for type recovery:

```python
# Instead of instruction.as<MoveInstructionPoly>()
move_instr = InstructionPoly_as_MoveInstructionPoly(instruction)
```

This is handled internally in the conversions module.

### Explicit Cleanup Order

Nanobind requires explicit cleanup in the correct order to avoid segfaults:

```python
# Correct cleanup order
del trajectory
del planner
del environment
```

The TesseractClient context manager handles this automatically.

### Profile Dictionary Cross-Module Registration

Profiles must be registered using type-specific helpers:

```python
# OMPL profiles
ProfileDictionary_addOMPLProfile(profile_dict, "DEFAULT", ompl_profile)

# TrajOpt profiles  
ProfileDictionary_addTrajOptProfile(profile_dict, "DEFAULT", trajopt_profile)
```

## Planner Options

### OMPL Options

```python
options = {
    'planner': 'ompl',
    'planner_id': 'RRTConnect',  # or 'RRTstar', 'PRM', 'BKPIECE', etc.
    'planning_time': 5.0,
    'simplify': True,
}
```

### TrajOpt Options

```python
options = {
    'planner': 'trajopt',
    'collision_cost_config': {...},
    'collision_constraint_config': {...},
}
```

### Simple (Interpolation) Options

```python
options = {
    'planner': 'simple',
    'interpolation': 'linear',  # or 'cubic'
}
```

## Dependencies

- `tesseract_robotics` (nanobind bindings)
- `compas` >= 2.0
- `compas_fab` >= 1.0
- `numpy`

## References

- [Tesseract Motion Planning](https://github.com/tesseract-robotics/tesseract_planning)
- [tesseract_nanobind PR #83](https://github.com/tesseract-robotics/tesseract_python/pull/83)
- [compas_fab Backend Architecture](https://gramaziokohler.github.io/compas_fab/latest/developer/backends.html)
