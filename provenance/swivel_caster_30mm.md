<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Swivel caster with a 30 mm wheel (DiffBot)

A pressed-steel swivel caster with a 30 mm plastic wheel, as on DiffBot. DiffBot is built on the Joy-IT Robot Car Kit 2WD (robot05), and this is the kit's caster. The model was created independently, from public product photos, the robot's own URDF numbers and photos of DiffBot's caster. No geometry was imported, converted, traced or sampled from another 3D model, and no vendor or third-party 3D model was downloaded.

Generator: [`generators/swivel_caster_30mm.py`](../generators/swivel_caster_30mm.py).

## Meshes

| Mesh | What | Materials |
|---|---|---|
| `swivel_caster_30mm_base` | The fixed part: four brass hex standoffs (M3, 5 mm across flats, 10 mm long), a plate 32 × 38 × 1.5 mm with four corner lugs and concave sides, a pressed raceway dome on top (Ø26 mm) with a domed rivet head, a rounded bearing ring between plate and fork, the stepped race cap under the fork (Ø14.8 mm) with a large flat rivet head (Ø10 mm), four Phillips pan heads under the lugs | `steel` (zinc-plated), `brass` |
| `swivel_caster_30mm_wheel_support` | The swivelling fork, bent from one 1.5 mm sheet: a round top plate (r 13 mm) with tight bends into two arms with a concave front edge, and a pressed boss around the axle on each arm. The axle rivet with domed heads and washers on both sides | `steel`, `axle` |
| `swivel_caster_30mm_wheel` | The wheel: Ø30 mm, hub 15.6 mm wide, crowned 12 mm tread with 3 mm rounds, dished sides | `wheel` (white) |

Each STL file is closed and consistently oriented. The three meshes, placed as DiffBot's caster macro places them, only touch. The axle has 0.1 mm clearance in the wheel's bore.

## Frames

The STL and glTF files are in metres, the COLLADA files are in millimetres, all z up. Positions on this page are in millimetres.

- `swivel_caster_30mm_base`: z = 0 is the swivel bearing's lower face. The standoffs end at z = +15 mm, at the chassis. The swivel axis is at x = -1 mm, y = 0, because DiffBot's hub joint is 1 mm behind its base joint (`dx` -0.077 and -0.076 m in `caster_wheel.yaml`).
- `swivel_caster_30mm_wheel_support`: the swivel axis at x = y = 0, the fork's top at z = 0. The wheel axle runs along y at x = -16 mm, z = -20 mm.
- `swivel_caster_30mm_wheel`: centred on its axle, the axle along y.

DiffBot's caster macro, [`diffbot_caster_macro.urdf.xacro`](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/diffbot_description/urdf/include/diffbot_caster_macro.urdf.xacro), with [`caster_wheel.yaml`](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/diffbot_description/config/diffbot/caster_wheel.yaml), attaches the three meshes:

- `rear_caster_base_joint` puts `rear_caster_base_link` 76 mm behind and 4.5 mm above `chassis_link`. The macro draws `swivel_caster_30mm_base` 20 mm below that link, so the standoffs' tops lie 0.5 mm below the chassis plate's top face, inside the plate where the screws hold them.
- `rear_caster_rotation_joint` puts the swivelling `rear_caster_rotation_link` 77 mm behind `chassis_link`, 1 mm behind the base link, at the same height. The macro draws `swivel_caster_30mm_wheel_support` 20 mm below it. So the swivel axis lies at x = -1 mm in the base's frame and at x = 0 in the fork's frame.
- `rear_caster_wheel_joint` puts `rear_caster_wheel_link` 16 mm behind and 40 mm below the rotation link, on the fork's axle at x = -16 mm, z = -20 mm. `swivel_caster_30mm_wheel` is drawn at that link's origin.

The wheel's lowest point is 0.2 mm below DiffBot's `base_footprint`, from the URDF's numbers.

## Sources

| What | Source |
|---|---|
| Which caster: DiffBot is built on the Joy-IT Robot Car Kit 2WD | ros-mobile-robots.com: [Components](https://ros-mobile-robots.com/components/) and [Assembly](https://ros-mobile-robots.com/hardware_setup/assembly/), steps 8 and 9 (four screws) |
| The caster from above and below, on four standoffs | Joy-IT's product page [joy-it.net/en/products/robot05](https://joy-it.net/en/products/robot05): the product photos `Robot05-01.png` and `Robot05-02.png` (underside). The assembly manual [Robot05-Anleitung.pdf](https://joy-it.net/files/files/Produkte/robot05/Robot05-Anleitung.pdf), page 2 ("4x Abstandshalter") and page 6 |
| Shapes on DiffBot's own caster: the lobed plate with Phillips pan heads at the corners, brass hex standoffs, the fork's round top plate around a race cap with concentric pressed steps and a large flat rivet head, arms with a concave front edge and a pressed boss at the axle, a domed axle rivet with a washer, a white crowned wheel | Photos and video frames of DiffBot's caster, taken by the project (not published) |
| Proportions: race cap about 0.6 of the fork's top plate, flat rivet head about 10 mm, plate hole spacing about the wheel's diameter, standoffs about a third of the hole spacing | The same video frames (they have no scale) |
| Wheel radius 15 mm. Joints: base `dx` -0.076 m, hub `dx` -0.077 m, both `dz` 0.0045 m. Wheel `dx` -0.016 m, `dz` -0.04 m below the hub. Visual origins 20 mm below the base and hub links | DiffBot's [`caster_wheel.yaml`](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/diffbot_description/config/diffbot/caster_wheel.yaml) and `diffbot_caster_macro.urdf.xacro` in the diffbot repository |
| Choices | Sheet thickness, bend radius, race step heights, screw and rivet sizes, the curve of the arm's outline. They are parameters at the top of the generator |

## Simplified or estimated

- The fork's height follows the URDF: the axle sits 20 mm below the fork's top, from the wheel joint's `dz`. On the real caster the wheel stands a little further from the race cap, so the real fork is probably longer.
- The bearing is a closed rounded ring between plate and fork, without visible balls. The race cap is a revolved profile with sharp steps.
- The fork's arms are flat sheets without the rolled edges of the real arms. The fork's inner width, 17.2 mm, is a design choice that leaves room for the 15.6 mm wheel, the washers and the rivet heads.
- The generator refuses parameters that leave no room for the rivet head or make the race cap wider than the gap between the fork's bends.
- The wheel is one solid part, opaque white instead of translucent.

## Measurements that would improve the model

With DiffBot upside down and calipers:

1. The axle position: from the underside of the caster's plate to the axle's centre, and from the swivel rivet's centre to the axle's centre. The model has 23.5 mm and 16 mm.
2. The plate's hole spacing, centre to centre, in both directions (model: 24 × 30 mm), and the standoffs' length (model: 10 mm).
3. The fork's inner width between the arms at the axle (model: 17.2 mm).
4. The wheel's diameter and width over the tread (model: 30 × 12 mm, the hub 15.6 mm).
