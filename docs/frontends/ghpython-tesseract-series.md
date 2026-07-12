# Tesseract series in Grasshopper

Tesseract authoring components declare Grasshopper item or list access instead
of introducing a COMPAS FAB tree or series wrapper:

```text
Frame items -> Pose items -> Cartesian Target items --\
Joint-vector branches -> one Joint/State Target each --+-> ordered target branch -> Motion Program
```

`Tesseract Pose` reads `frame` as an item, and `Tesseract Cartesian Target`
reads `pose` as an item. These ports use standard Grasshopper item matching;
shared item inputs such as move type, profile, and working frame participate in
that matching normally.

`Tesseract Joint Target` and `Tesseract State Target` instead read each joint
vector as a list. One ordered branch of positions, names, velocities, or
accelerations forms one atomic target. The scalar values inside that branch are
joint coordinates, not a series of targets.

`Tesseract Motion Program` likewise reads `targets` as a list. Treat each
ordered target branch as one program. For the one-item convenience case, wire a
single Cartesian, Joint, or State target directly to `targets`; no custom
container is part of the component contract.

## Evidence and acceptance

The automated metadata test pins the declared item/list access contract in the
component sources. Windows Rhino CI compiles the components and inventories the
expected `.ghuser` files. It does not run a Grasshopper canvas: exact Rhino
path preservation, branch matching, and one-item list coercion remain pending
Rhino acceptance testing.
