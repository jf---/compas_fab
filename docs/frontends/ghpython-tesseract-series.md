# Tesseract series in Grasshopper

Tesseract authoring components use Grasshopper's standard item matching and
data-tree behavior. No COMPAS FAB tree or series wrapper is needed:

```text
Frames -> Tesseract Pose -> Tesseract Cartesian Target -> Tesseract Motion Program
                         \-> Tesseract Joint/State Target -/
```

`Tesseract Pose` and the target components produce one output item for each
matched input item. Grasshopper preserves branches and broadcasts single-item
inputs in the usual way, so shared move types, profiles, or frames can remain
single items while frames vary across a list or tree.

## One program per branch

`Tesseract Motion Program` reads `targets` as a list. Each target branch is
therefore one ordered program: the component runs once per branch and passes
that branch's complete target sequence to Tesseract. With several target
branches, the output contains the corresponding program branches.

Joint data has the same atomic rule. `positions`, `joint_names`, `velocities`,
and `accelerations` are list inputs, so one branch is one joint vector, not a
series of scalar target items. To author several joint or state targets, put
each complete vector on its own branch and let Grasshopper match those branches.

For the common one-item case, connect one frame through Pose and Cartesian
Target directly into Motion Program. Grasshopper supplies the lone target as a
one-item target list; no grafting, wrapping, or custom container is required.
