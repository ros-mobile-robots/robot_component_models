<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Wheel 65 mm for TT gear motors

The common 65 mm wheel for TT gear motors: a yellow five-spoke plastic rim with a double-D bore and a black rubber tyre with a block tread. DiffBot and Remo drive on it. The model was created independently, from caliper measurements and photos of the real wheel, photos and video frames of the robots, product listings and shop photos. No geometry was imported, converted, traced or sampled from another 3D model, and no third-party 3D model was downloaded.

Generator: [`generators/tt_motor_wheel_65mm.py`](../generators/tt_motor_wheel_65mm.py).

## Mesh

`tt_motor_wheel_65mm`: one mesh with two sub-parts, `rim` (material `rim`, yellow) and `tyre` (material `tyre`, black). Each is a closed solid, and they don't intersect. The STL file is closed and consistently oriented: two closed shells, with a modelling clearance of 0.02 mm where the tyre sits on the rim (at least 0.0097 mm between them).

- **Rim:** a barrel with a flange at each end (Ø51.22 mm, 25.00 mm flange to flange), three ridges between them, plain bands next to the flanges and two pockets between the ridges (Ø47.91 mm), seven axial ribs across the pockets, Ø45.1 mm inside. On the outer face the flange's ring curves inward into the windows. In the centre a bowl is recessed into the spokes' flat central face, its edge in that face (Ø17.9 mm). Its wall runs almost straight down to the depth of the boss's top (Ø17.62 mm there), then rounds into a flat floor (Ø13.8 mm) with four small pins. The boss (Ø9.69 mm) stands on the floor, its top 1.2 mm below the spokes' face. Behind the face the bowl's outside (Ø20.64 mm) shows through the windows. Five tapered, swept spokes with a bevel and a mark, curved blends into the central face, and behind each spoke a narrow web and a fin up the hub tube. On the motor side the hub tube (Ø9.95 mm) on a collar (Ø15.27 mm), the double-D bore (Ø5.44 mm across the round sides), the tube standing 3.42 mm past the flange.
- **Tyre:** Ø66.6 mm, a hollow section like a car tyre's. The beads at its inner diameter (Ø49.4 mm) sit inside the rim, each in the plain band between a flange and the outer ridge, held there because the bead is smaller than both. The sidewalls rise over the flanges and are widest halfway up (Ø59.2 mm), flush with the flanges' faces, 25.00 mm across. The crown is a little narrower and slightly crowned. Inside, an arched channel spans the barrel's three ridges: its ceiling lies 5.8 mm below the tread in the middle and 7.5 mm near the beads (the wall readings, their places across the tyre assumed). Six block columns and 42 rows of nearly square blocks.

## Frame

The files are in metres. Positions on this page are in millimetres. The axle runs along z at x = y = 0. The wheel's outer face is at z = -13 mm: the rim's outer flange face, flush with the tyre's sidewalls at their widest. The rim's motor-side flange face is at z = +12.00 mm, and the hub tube's end at z = +15.42 mm. Both robots turn the mesh so that +z points towards the robot.

**Frame alignment:** the reference wheel mesh's bounding box was read to position the model:

| | min (mm) | max (mm) |
|---|---|---|
| Reference mesh | (-34.46, -34.46, -13.00) | (34.46, 34.41, 17.00) |
| This model | (-33.30, -33.30, -13.00) | (33.30, 33.30, 15.42) |

- The outer face and the axle are the reference's.
- The radius is 1.17 mm smaller than the reference's: the measured Ø66.6 mm. DiffBot's URDF radius is 33.3 mm, so the tyre touches the ground. Remo's is 33.0 mm, so the visual tyre reaches 0.3 mm below the ground.
- The motor-side end is at +15.42 mm, the reference's at +17.00 mm: the real rim is 28.42 mm long, and its outer flange face is the wheel's outer face. Nothing in the robots' URDFs refers to that end. The tyre is 25.00 mm wide, the reference's 24.82 mm, so the two centre planes are 0.09 mm apart. Joints, collisions and inertias don't depend on the mesh.

## Sources

| What | Source |
|---|---|
| Rim and tyre dimensions (the parameters marked pNN in the generator) | Caliper measurements of a wheel with the tyre taken off the rim, photos p01–p41, taken by the project (not published). Flange Ø51.22 (p01, p41: 51.25), bands and pockets Ø47.91 (p02), barrel wall with flange 3.06 (p28), flange to flange 25.00 (p40), rim length 28.42 (p33), the bowl's outside behind the face Ø20.64 (p24) and its inside Ø17.62 at the boss's top (p21), boss Ø9.69 (p23), spoke width 5.35 at the ring (p25), the spoke's back 3.47 (p18, p19), fin 1.51 (p31), hub tube Ø9.97 and Ø9.92 (p29, p30), collar Ø15.27 (p15), bore Ø5.44 across the round sides (p14), tread to the tyre's inside 5.77, 6.88 and 7.46 (p04–p06), bead to tread 8.38 and 8.82 (p07, p08) |
| The tyre's section: hollow like a car tyre's, the beads narrower and inside the rim between the flanges, the sidewalls widest halfway up and flush with the flanges, the crown a little narrower | The project's description of the real tyre, with photos p03–p09 |
| The barrel's profile along the axle (ridges, bands, pockets, ribs) | The silhouette of side photo p37, scaled with the 25.00 mm flange to flange |
| The outer face's radii and spoke widths (9.3 at r 11.7, 7.7 at r 15.6, 5.4 at r 19.5 mm), the bowl's edge Ø17.9 and floor Ø13.8 mm, four pins on the floor at r 6.15 mm, 90° apart | Face-on photo p41, scaled with its own 51.25 mm reading |
| The bowl's and the boss's depths (the floor 2.8 mm and the boss's top 1.2 mm behind the spokes' face) | Oblique photo p24, scaled with the bowl's 17.62 mm |
| Shapes not measured: the ring's curl, the spokes' bevel, the tread | Video frames of DiffBot's wheel, taken by the project (not published). The photo of Remo on ros-mobile-robots.com: [remo_front_side.jpg](https://ros-mobile-robots.com/resources/remo/remo_front_side.jpg) |
| Tyre outer diameter 66.6 mm | Measured on DiffBot: 66.3 and 66.8 mm, in DiffBot's [`front_wheel.yaml`](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/diffbot_description/config/diffbot/front_wheel.yaml) |
| Bore flats 3.66 mm (not measured) | Product listings: [X2 Robotics, "Rubber Tire Wheel (Yellow 65mm)"](https://x2robotics.ca/wheels/rubber-tire-wheel-yellow-65mm) ("Hole Width Axis: 3.66 mm", "Long Shaft Hole: 5.3 mm") and [VXB](https://vxb.com/products/four-pack-68mm-tt-motor-hub-replacement-diy-kit-fo) ("hole 3.66mm × 5.3mm") |
| Comparison photos | X2 Robotics' product photos (link above) and [Otronic, "Wheel with rubber for robot or smart car, yellow"](https://otronic.nl/en/wheel-with-rubber-for-robot-or-smart-car-yellow.html) |
| Colours | remo_description's wheel colours: rim yellow `0.95 0.72 0.05`, tyre black `0.10 0.10 0.10` |

## Simplified or estimated

- The tyre's width (flush with the flanges) follows the project's description, not a caliper reading. Estimated: the crown's width (18.6 mm) and drop (0.25 mm), the beads' width (1.8 mm), the channel's width (15.6 mm) and its half-elliptical arch, which takes the p04 reading as the middle and the p06 reading as near a bead (where each was taken isn't recorded), the bowl's and the boss's depths from one oblique photo, the pins' height, and the tread's groove width and depth (0.7 and 0.6 mm).
- The spokes taper linearly through two of the photo's width estimates: 7.35 mm at r 15.6 mm, where the photo gives 7.7 mm.
- The tyre's sidewalls and shoulders are quarter ellipses. The real tyre's inside has thin rings that the model leaves out.
- The barrel's inside is plain, and the motor-side collar is round. The real collar is scalloped between the fins.
- The ridges, flanges and ribs have flat faces and sharp edges, except the flanges' outer edges.

## Measurements that would improve the model

With calipers:

1. The tyre on the rim: its width at the sidewalls' widest point and over the tread, and the step from each flange's face to the sidewall (0 if flush).
2. The tyre off the rim: the wall from the tread's top to the tyre's inside, in the middle of the tread and 7 mm to each side of it.
3. On the outer face, with the depth rod on the spokes' face: down to the bowl's floor and down to the boss's top.
4. The tread's groove depth, at a groove between two blocks.
