<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Wheel 65 mm for TT gear motors

The common 65 mm wheel for TT gear motors: a yellow five-spoke plastic rim with a double-D bore and a black rubber tyre with a block tread. DiffBot and Remo drive on it. The model was created independently, from caliper measurements and photos of the real wheel, photos and video frames of the robots, product listings and shop photos. No geometry was imported, converted, traced or sampled from another 3D model, and no third-party 3D model was downloaded.

Generator: [`generators/tt_motor_wheel_65mm.py`](../generators/tt_motor_wheel_65mm.py).

## Mesh

`tt_motor_wheel_65mm`: one mesh with two sub-parts, `rim` (material `rim`, yellow) and `tyre` (material `tyre`, black). Each is a closed solid, and they don't intersect. The STL file is closed and consistently oriented: two closed shells, 0.02 mm apart where the tyre sits on the rim (a modelling clearance).

- **Rim:** a barrel with a flange at each end (Ø51.22 mm, 25.00 mm flange to flange), three ridges between them, plain bands next to the flanges and two pockets between the ridges (Ø47.91 mm), seven axial ribs across the pockets, Ø45.1 mm inside. On the outer face the flange's ring curves inward into the windows. In the centre a bowl is recessed into the spokes' central face: a rounded ring (Ø20.64 mm outside, Ø17.62 mm inside), its wall curving down to a floor with five small raised pads, and the boss (Ø9.69 mm) standing in it. Five tapered, swept spokes with a bevel and a mark, curved blends into the central face, and behind each spoke a narrow web and a fin up the hub tube. On the motor side the hub tube (Ø9.95 mm) on a collar (Ø15.27 mm), the double-D bore (Ø5.44 mm across the round sides), the tube standing 3.42 mm past the flange.
- **Tyre:** Ø66.6 mm, a thin lip at each edge that hooks over a flange (inner Ø49.4 mm), two inner rings in the plain bands, a wall of 7.46 mm to the blocks' tops. Its own inner surface (Ø51.68 mm) clears the flanges: the tyre is held by its lips and rings. Six block columns and 42 rows of nearly square blocks.

## Frame

Millimetres in the STL file, metres in the COLLADA and glTF files. The axle runs along z at x = y = 0. The tyre's outer face is at z = -13 mm, the rim's outer face (flange and boss) 0.74 mm behind it, the rim's motor-side flange face at z = +12.74 mm, and the hub tube's end at z = +16.16 mm. Both robots turn the mesh so that +z points towards the robot.

The frame is the frame of the wheel mesh both robots used before. Only that mesh's bounding box was read, to align the frame:

| | min (mm) | max (mm) |
|---|---|---|
| Earlier mesh | (-34.46, -34.46, -13.00) | (34.46, 34.41, 17.00) |
| This mesh | (-33.29, -33.30, -13.00) | (33.29, 33.30, 16.16) |

- The outer face and the axle are the same.
- The radius is 1.17 mm smaller: the measured Ø66.6 mm. DiffBot's URDF radius is 33.3 mm, so the tyre touches the ground. Remo's is 33.0 mm, so the visual tyre reaches 0.3 mm below the ground (the earlier mesh 1.46 mm).
- The motor-side end is at +16.16 mm instead of +17.00 mm: the real rim is 28.42 mm long. The end depends on how far the tyre's side stands in front of the flange, which isn't measured (see below). Nothing in the robots' URDFs refers to that end. The tyre is 26.48 mm wide instead of 24.82 mm, so its centre plane is 0.74 mm nearer the robot than before. Joints, collisions and inertias don't depend on the mesh.

## Sources

| What | Source |
|---|---|
| Rim and tyre dimensions (the parameters marked pNN in the generator) | Caliper measurements of a wheel with the tyre taken off the rim, photos p01–p41, taken by the project (not published). Flange Ø51.22 (p01, p41: 51.25), bands and pockets Ø47.91 (p02), barrel wall with flange 3.06 (p28), flange to flange 25.00 (p40), rim length 28.42 (p33), hub cup Ø20.64 (p24) and its inside Ø17.62 (p21), boss Ø9.69 (p23), spoke width 5.35 at the ring (p25), the spoke's back 3.47 (p18, p19), fin 1.51 (p31), hub tube Ø9.97 and Ø9.92 (p29, p30), collar Ø15.27 (p15), bore Ø5.44 across the round sides (p14), tyre wall 7.46 (p06), tyre edge lip to tread 8.38 and 8.82 (p07, p08) |
| The barrel's profile along the axle (ridges, bands, pockets, ribs) | The silhouette of side photo p37, scaled with the 25.00 mm flange to flange |
| The outer face's radii and spoke widths (9.3 at r 11.7, 7.7 at r 15.6, 5.4 at r 19.5 mm) | Face-on photo p41, scaled with its own 51.25 mm reading |
| Shapes not measured: the ring's curl, the spokes' bevel, the bowl, the pads, the tread | Video frames of DiffBot's wheel, taken by the project (not published). The photo of Remo on ros-mobile-robots.com: [remo_front_side.jpg](https://ros-mobile-robots.com/resources/remo/remo_front_side.jpg) |
| Tyre outer diameter 66.6 mm | Measured on DiffBot: 66.3 and 66.8 mm, in DiffBot's [`front_wheel.yaml`](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/diffbot_description/config/diffbot/front_wheel.yaml) |
| Bore flats 3.66 mm (not measured) | Product listings: [X2 Robotics, "Rubber Tire Wheel (Yellow 65mm)"](https://x2robotics.ca/wheels/rubber-tire-wheel-yellow-65mm) ("Hole Width Axis: 3.66 mm", "Long Shaft Hole: 5.3 mm") and [VXB](https://vxb.com/products/four-pack-68mm-tt-motor-hub-replacement-diy-kit-fo) ("hole 3.66mm × 5.3mm") |
| Comparison photos | X2 Robotics' product photos (link above) and [Otronic, "Wheel with rubber for robot or smart car, yellow"](https://otronic.nl/en/wheel-with-rubber-for-robot-or-smart-car-yellow.html) |
| Colours | remo_description's wheel colours: rim yellow `0.95 0.72 0.05`, tyre black `0.10 0.10 0.10` |

## Simplified or estimated

- The tyre's lips' axial thickness (0.74 mm, which also places the rim in z and sets the tyre's width) is an estimate. So are the bowl's depths, the ring's height, the pads' height and the tread's groove width and depth (0.7 and 0.6 mm).
- The spokes taper linearly through two of the photo's width estimates: 7.35 mm at r 15.6 mm, where the photo gives 7.7 mm.
- The tyre is solid between its rings and the tread; the real tyre's inside has more rings. Flat sidewalls with 3 mm shoulders.
- The barrel's inside is plain, and the motor-side collar is round; the real collar is scalloped between the fins.
- The ridges, flanges and ribs have flat faces and sharp edges, except the flanges' outer edges.

## Measurements that would improve the model

With calipers, the tyre on the rim:

1. The tyre's width over the tread, and how far the tyre's side stands in front of the yellow flange's face on both sides (the step measurement, 0 if flush).
2. On the outer face, with the step measurement: the ring around the boss above the spokes' face, the spokes' face above the bowl's floor, and whether the boss's top is flush with the flange's face.
3. The tread's groove depth, at a groove between two blocks.
