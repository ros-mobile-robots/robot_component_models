<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Swivel caster with a 30 mm wheel (DiffBot)

A pressed-steel swivel caster with a 30 mm plastic wheel, as on DiffBot. DiffBot is built on the Joy-IT Robot Car Kit 2WD (robot05), and this is the kit's caster. The model was created independently, from public product photos and a datasheet of the same caster family, the robot's own URDF numbers and photos of DiffBot's caster. No geometry was imported, converted, traced or sampled from another 3D model, and no vendor or third-party 3D model was downloaded.

Generator: [`generators/swivel_caster_30mm.py`](../generators/swivel_caster_30mm.py).

## Meshes

| Mesh | What | Materials |
|---|---|---|
| `swivel_caster_30mm_base` | The fixed part: four brass hex standoffs (M3, 5 mm across flats, 10 mm long, 0.3 mm chamfers at both ends, the M3 thread at the chassis end shown as a smooth bore), a plate 32 × 38 × 1.3 mm with four corner lugs and concave sides. The plate is pressed into a raceway: a trough that bulges 3.35 mm below it (Ø26.8 mm, seen from the side as a smooth lens), an arch over one row of 12 balls (Ø3.2 mm on a Ø15 mm circle), and a centre that carries the kingpin's flat head (Ø7.5 × 2 mm). Inside the fork's cup: the stepped race cap (Ø11.2 mm) and the kingpin's flat lower head (Ø9.2 mm). Four Phillips pan heads under the lugs | `steel` (zinc-plated, metallic), `brass` (metallic) |
| `swivel_caster_30mm_wheel_support` | The swivelling fork, bent from one 1.2 mm sheet. Its top is a shallow cup (rim Ø24.5 mm) that opens towards the wheel, its floor pressed into the lower ball track (a raised ring inside the cup). On the axle sides the cup's wall runs on into the arms: curved at the cup, between 48° and 132° around it, flat and parallel to the wheel from 9 mm below the top. Seen along the axle each arm narrows to a rounded tip (r 4.75 mm) around the axle, with a diagonal front edge that curves into the rim. The edges are plain cut. The axle with a domed head and a washer on the -y side and a Phillips pan head on the +y side | `steel`, `axle` (metallic) |
| `swivel_caster_30mm_wheel` | The wheel: Ø30 mm, hub 15.6 mm wide, crowned 12 mm tread with 3 mm rounds, dished sides | `wheel` (matte white plastic) |

Each STL file is closed and consistently oriented: the base's balls are separate closed shells. The three meshes, placed as DiffBot's caster macro places them, don't intersect: the base and the fork are at least 0.09 mm apart. The axle has 0.1 mm clearance in the wheel's bore.

## Frames

The files are in metres, z up. Positions on this page are in millimetres.

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
| Shapes and materials on DiffBot's own caster: zinc-plated steel plate, raceway, fork and rivets, hexagonal brass standoffs, a white plastic wheel. The lobed plate with Phillips pan heads at the corners, the smooth lens under the plate, the kingpin's flat head above it, the fork's cup with a raised ring, the stepped race cap and the flat lower head inside, broad arms that run on from the cup's wall and narrow to a rounded tip, a domed axle head with a washer on one side and a Phillips pan head on the other, a crowned wheel | Photos and video frames of DiffBot's caster, taken by the project (not published) |
| Proportions, measured on the frames and the shop photos, scaled with the wheel's 30 mm and the 38 × 32 mm plate: the cup's rim Ø about 24–25 mm, the raised ring inside it Ø about 15 mm, the race cap Ø about 12.5 mm, the kingpin's heads Ø about 7.5 and 9 mm, the arm's tip about 9.5 mm wide | The same video frames and photos |
| The same caster family: one ball race, the plate's dome Ø26.5 mm, the bearing stack in section, an offset of 11 mm between swivel axis and axle | [Tente, datasheet 1430POI032P60-38X32](https://cdn.tente.com/datasheets/1430POI032P60_38X32_en.pdf) (a caster with the same 38 × 32 mm plate) |
| The fork's shape: the cup, the arms that run on from its wall, bow out at the cup and run parallel lower down, plain cut edges, the ball row in the gap under the plate | Product photos of the same caster type: [Adafruit 2942](https://www.adafruit.com/product/2942) and [Bits & Parts](https://www.bitsandparts.nl/en/caster-wheel-swivel-wheel-for-robot-smart-cars-cars-nylon-p100105) |
| Sheet thickness: the fork 1.2 mm, the plate 1.3 mm | Listings of the same caster type: [Makerfabs](https://www.makerfabs.com/universal-caster-whee.html) (1.2 mm cold-rolled steel) and [Electrokit](https://www.electrokit.com/en/stodhjul-25mm) (plate 38 × 32 × 1.3 mm) |
| Wheel radius 15 mm. Joints: base `dx` -0.076 m, hub `dx` -0.077 m, both `dz` 0.0045 m. Wheel `dx` -0.016 m, `dz` -0.04 m below the hub. Visual origins 20 mm below the base and hub links | DiffBot's [`caster_wheel.yaml`](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/diffbot_description/config/diffbot/caster_wheel.yaml) and `diffbot_caster_macro.urdf.xacro` in the diffbot repository |
| Choices | The raceway's and the cup's profiles between the measured diameters, the balls' size, the race cap's steps, screw and rivet sizes, the arms' angles and outline, the standoffs' chamfers and bores, the materials' values. They are parameters at the top of the generator |

## Simplified or estimated

- The axle follows the URDF: 16 mm behind the swivel axis and 20 mm below the fork's top (`caster_wheel.yaml`). The same caster family has an offset of 11 mm (Tente's datasheet, and about 11 ± 2 mm on Joy-IT's photo), so the model's arms lean further back than the real ones. Matching the real caster needs the measured offset in the wheel's `dx` in `caster_wheel.yaml` and in this fork's axle position, together.
- The plate stands 3.7 mm above the fork's cup (standoffs of 10 mm, not measured). Its lens bulges 3.35 mm, more than on the real caster (about 2.4 mm), so the slit between lens and cup stays as thin as on the photos.
- The balls are one row of 12 spheres, Ø3.2 mm (the real ones about 3.5–4 mm). The plate's arch and the cup's ball track are smooth revolved sheets. The race cap is a revolved profile with sharp steps.
- The standoffs' M3 internal thread is a smooth blind bore of the tapping diameter (2.5 mm, 3 mm deep), without a thread profile.
- Each arm's root is a ruled loft through 21 sections that blend from the cup's wall into the flat sheet with a half cosine. It is smooth across the arm, with bends of a few degrees between the sections along it: about 6° in the middle of the arm, up to 13° at its rear edge just below the rim. Where the arm leaves the cup's rim its faces step 0.15 mm inside the wall's faces. The fork's inner width, 17.2 mm, is a design choice that leaves room for the 15.6 mm wheel.
- The generator refuses parameters that leave no room for the rivet head, let a ball touch the plate's raceway or the cup's ball track, put the race cap into the ball track, or the flat arms outside the cup's wall. It checks that the twelve balls stay separate solids.
- The wheel is one solid part, opaque white instead of translucent.
- The materials are Phong values in COLLADA and metallic or not in glTF, without textures: the zinc plating's sheen and the brass's colour come from a darker diffuse colour and strong highlights (COLLADA) or from metallic, low-roughness surfaces (glTF). A viewer without an environment light shows the glTF metals dark.

## Measurements that would improve the model

With DiffBot upside down and calipers:

1. The axle position: from the underside of the caster's plate to the axle's centre, and from the swivel rivet's centre to the axle's centre (the offset). The model has 23.7 mm and 16 mm.
2. The plate's hole spacing, centre to centre, in both directions (model: 24 × 30 mm), and the standoffs' length (model: 10 mm).
3. The fork's inner width between the arms at the axle (model: 17.2 mm).
4. The wheel's diameter and width over the tread (model: 30 × 12 mm, the hub 15.6 mm).
