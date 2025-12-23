# compas_fab_tesseract

Tesseract motion planning backend for [compas_fab](https://github.com/compas-dev/compas_fab).

This package integrates the [Tesseract Motion Planning Framework](https://github.com/tesseract-robotics/tesseract_planning) 
with compas_fab, providing access to powerful motion planning algorithms including OMPL, TrajOpt, and more.

## Features

- **OMPL Motion Planning**: RRTConnect, RRT*, PRM, and other sampling-based planners
- **TrajOpt Optimization**: Trajectory optimization with collision avoidance
- **Simple Interpolation**: Fast linear/cubic interpolation between waypoints
- **KDL Kinematics**: Forward and inverse kinematics via KDL
- **Collision Checking**: Bullet-based discrete and continuous collision detection
- **Time Parameterization**: TOTG and ISP trajectory timing

## Installation

### Prerequisites

Install the Tesseract nanobind bindings:

```bash
# Clone tesseract_nanobind (once available publicly)
git clone https://github.com/tesseract-robotics/tesseract_nanobind
cd tesseract_nanobind

# Build with conda environment
conda env create -f environment.yml
conda activate tesseract
pip install -e .
```

### Install compas_fab_tesseract

```bash
pip install compas_fab_tesseract
```

Or for development:

```bash
git clone https://github.com/compas-dev/compas_fab_tesseract
cd compas_fab_tesseract
pip install -e ".[dev]"
```

## Quick Start

```python
from compas.geometry import Frame
from compas_fab_tesseract import TesseractClient

# Load robot and plan motion
with TesseractClient() as client:
    robot = client.load_robot(
        'path/to/robot.urdf',
        'path/to/robot.srdf'
    )
    
    # Forward kinematics
    frame = robot.forward_kinematics(configuration)
    
    # Inverse kinematics
    config = robot.inverse_kinematics(target_frame, start_configuration)
    
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
    
    # Cartesian path planning
    trajectory = robot.plan_cartesian_motion(
        frames,
        start_configuration
    )
```

## Planner Options

### OMPL Planner

```python
options = {
    'planner': 'ompl',
    'planner_id': 'RRTConnect',  # or 'RRTstar', 'PRM', 'BKPIECE', etc.
    'planning_time': 5.0,
}
```

### TrajOpt Planner

```python
options = {
    'planner': 'trajopt',
    'collision_cost_config': {
        'enabled': True,
        'buffer_margin': 0.01,
    },
}
```

### Simple Interpolation

```python
options = {
    'planner': 'simple',
}
```

## Architecture

This backend follows the compas_fab backend interface pattern:

```
TesseractClient (ClientInterface)
    └── TesseractPlanner (PlannerInterface)
            ├── forward_kinematics()
            ├── inverse_kinematics()
            ├── plan_motion()
            ├── plan_cartesian_motion()
            ├── add_collision_mesh()
            └── remove_collision_mesh()
```

See [ARCHITECTURE.md](ARCHITECTURE.md) for detailed design documentation.

## Development

Based on the [tesseract-python nanobind refactor](https://github.com/tesseract-robotics/tesseract_python/pull/83)
which modernizes the Python bindings using nanobind and scikit-build-core.

### Running Tests

```bash
pytest tests/
```

### Building Documentation

```bash
cd docs
make html
```

## Related Projects

- [tesseract_planning](https://github.com/tesseract-robotics/tesseract_planning) - C++ motion planning framework
- [tesseract_python](https://github.com/tesseract-robotics/tesseract_python) - Original SWIG bindings
- [tesseract_nanobind](https://github.com/tesseract-robotics/tesseract_nanobind) - Modern nanobind bindings
- [compas_fab](https://github.com/compas-dev/compas_fab) - Robotic fabrication for COMPAS

## License

MIT License - see [LICENSE](LICENSE) for details.

## Credits

- Tesseract Motion Planning Framework by [tesseract-robotics](https://github.com/tesseract-robotics)
- Original tesseract_python bindings by [John Wason](https://github.com/johnwason)
- Nanobind refactor by [Jelle Feringa](https://github.com/jf---)
- compas_fab by [COMPAS](https://github.com/compas-dev)
