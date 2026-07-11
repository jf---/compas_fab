# Configuration Target

`Configuration Target` uses native joint units: metres for prismatic joints and radians for revolute or continuous joints. Existing definitions keep the 0.001 m / pi-per-180 rad defaults because an unwired `tolerance_policy` takes the legacy branch.

The existing inputs remain first and unchanged. `tolerance_policy` is appended. When policy is connected, each connected tolerance list must contain one finite non-negative float per joint; a connected empty list is invalid. `legacy_defaults` fills absent lists. `preserve_absent` retains `None`, including when supplied directly from a planner's declared `configuration_tolerance_policy`.

Unknown policies and invalid list shapes fail with named local errors. No planner policy is guessed.
