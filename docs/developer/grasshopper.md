# Grasshopper components

The Rhino 8 Grasshopper user objects are built with the
[COMPAS GitHub Action componentizer](https://github.com/compas-dev/compas-actions.ghpython_components)
on Windows using CPython 3.9, matching Rhino 8's embedded interpreter.

1. Apply your changes to the component source code under
   [`src/compas_fab/ghpython/components_cpython`](https://github.com/compas-dev/compas_fab/tree/main/src/compas_fab/ghpython/components_cpython).
2. Push the component source. The Windows `build-cpython-components` CI job
   pins Python 3.9 and rebuilds them with:

    ```bash
    pixi run -e rhino39 build-gh-components
    ```

3. Download the `compas_fab_components` CI artifact and install the generated
   `.ghuser` files by dropping them into Grasshopper.

Build, Yak, and release workflows fail before upload or publication unless all
twelve Tesseract user objects were generated:

- runtime/artifact: `Cf_TesseractRobotArtifact.ghuser`,
  `Cf_TesseractPlanner.ghuser`;
- RAPID: `Cf_TesseractRapidProfile.ghuser`, `Cf_TesseractRapid.ghuser`;
- authoring: `Cf_TesseractPose.ghuser`,
  `Cf_TesseractCartesianTarget.ghuser`, `Cf_TesseractJointTarget.ghuser`,
  `Cf_TesseractStateTarget.ghuser`, `Cf_TesseractMotionProgram.ghuser`;
- native planning/inspection: `Cf_TesseractDescartesProfile.ghuser`,
  `Cf_TesseractNativePlan.ghuser`, `Cf_TesseractNativeResult.ghuser`.

Their source-level `# r:` directive requires released
`tesseract-robotics-nanobind` 0.35.0.6 on Rhino's Windows CPython 3.9 runtime.
The CI artifact is the installable source for these compiled user objects. The
artifact component exposes collision-mesh treatment as a value-list input;
`convex_hull` is the visible default and `preserve` is the exact-triangle opt-in.
Native planning components retain exact Tesseract poses, targets, programs,
profiles, requests, and results. RAPID emission remains a separate pure path
from an authored `CompositeInstruction`.

The CPython componentizer uses `System.Drawing` through `GH_IO.dll`; it is not
part of the macOS development gate. macOS validates component code and metadata
contracts, while Windows CI produces the actual user objects.
