# D435 Wrist-Cam Mount compatibility and integration status

## Result

The supplied `RealSenseD435_Camera_Mount.step` is the official **RealSenseD435
Wrist-Cam Mount (Franka Hand)** asset. Franka's current 3D Assets page lists it
alongside the FR3 Franka Hand CAD asset and publishes both STEP and STL forms.
The associated official mounting guide states that it applies to Franka Research
3 and installs the mount with a Franka Hand M6 threaded hole. This establishes
the product-level FR3 Hand compatibility despite the legacy `Panda_...` internal
name embedded in the STEP file.

## Verified assets

| Item | Source | SHA-256 |
| --- | --- | --- |
| Supplied STEP | `RealSenseD435_Camera_Mount.step` | `77885b417867c6340162cfb5473af38374579f619a2c5de7148fcac8824ce81d` |
| Official mount STL | Franka 3D Assets | `a06a5a4fa4080c6bf11761f8190093f513351d518cc8074e9e71a093be74c66c` |
| FR3 Hand CAD package | Franka 3D Assets | `1aca627ff49ded96938107166c55356cce98754d23f2c9acba67ffad2b95f8cd` |
| D435 DAE | `realsense-ros@4.58.3` | `42f3b66f47a1f8f425a2e4dc07c1d9c283183167d8441f520a15623d98f9bf78` |

The official mount STL bounds are 48.0 x 60.0 x 49.95 mm. It was converted to
a metre-scale OBJ by 0.15 mm vertex-cluster tessellation reduction; its mesh is
visual-only. The D435 mesh bounds are 89.914 x 25.0 x 25.055 mm, consistent with
the 90 x 25 x 25 mm published exterior dimensions.

## Installation-pose gate

No numerical `T_hand_mount` can be derived safely from the available files:

The mounting guide's Figure 1 and Figure 2 are important **orthographic
validation drawings**, not merely illustrative photographs. Figure 1 constrains
the RealSense assembly by the shown 50.0 mm and 65.0 mm dimensions; Figure 2
shows the corresponding generic-mount values of 52.5 mm and 65.5 mm. These
must be checked after CAD mating (including the correct camera-facing
orientation). They do not, however, label their dimension endpoints with the
`fr3v2_1_hand` coordinate system or identify which of the two M6 holes is used,
so they cannot by themselves define all six degrees of freedom of
`T_hand_mount`.

- STEP inspection confirms the file is a **single**
  `ADVANCED_BREP_SHAPE_REPRESENTATION`, with CAD origin `(0, 0, 0)`, Z axis
  `(0, 0, 1)` and X axis `(1, 0, 0)`. It contains useful local feature axes;
  for example, the 5.5 mm-radius circular feature centered at
  `(-25, 0, 17.5)` mm has a `+Z` axis. These are mount-local construction
  references, not a robot assembly frame.
- The STEP contains zero `NEXT_ASSEMBLY_USAGE_OCCURRENCE`,
  `ASSEMBLY_COMPONENT_USAGE`, `ITEM_DEFINED_TRANSFORMATION`, and
  `REPRESENTATION_RELATIONSHIP` entities. Thus it has no exported hand mate or
  hand-relative transform.
- `franka_description@2.8.1` contains visual/collision tessellations, not a
  named M6-hole axis or mating-plane datum.
- The public FR3 Hand CAD package is SolidWorks-only; it contains no neutral
  assembly export that fixes the mount relative to the hand frame.

No CAD-derived standalone attachment is generated. The user-authorized Figure
The mount remains visual-only and its hand-to-mount pose remains a figure-fit:
`(-0.035, 0, 0)` m with identity rotation.  It is not a hardware extrinsic or
collision claim.

The camera-to-mount pose is now CAD-derived.  The mounting guide specifies two
M3x10 screws for the RealSense-specific mount (the 1/4-inch D435 bottom socket
is used only by the generic mount).  The D435 STEP has matching M3 centres at
`(-22.5, -25.05, 0)` and `(22.5, -25.05, 0)` mm; the mount STEP has centres
at `(-50, -22.5, 22.45)` and `(-50, 22.5, 22.45)` mm.  The official ROS mesh
visual transform is part of the D435 coordinate chain: it maps the CAD M3
centres to `(-20.75, -/+22.5, 0)` mm in `d435_link`.  Their 45 mm spacing and
mating-face normals identify the D435 side branch.  The pair is a 10 mm-deep
through-hole: its STEP axis reference is at the rear, while the physical
camera mates at the front mouth.  Using that front mouth gives
`pos=(5.7, 17.5, 11)` mm with identity rotation in the existing mount frame.
This replaces the former 5 mm visual seat correction.  The D435-to-mount
relationship is therefore geometrically constrained, while hand-to-mount
installation still needs a measured or official hand datum for hardware
accuracy.

To unblock a CAD-derived integration, provide either a Franka CAD assembly
containing both parts, or the official hand-frame-to-mount-frame transform. The
variant can then be replaced without changing the baseline robot model.
