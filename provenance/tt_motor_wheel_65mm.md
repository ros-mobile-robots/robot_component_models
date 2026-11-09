<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Wheel 65 mm for TT gear motors

The common 65 mm wheel for TT gear motors: a yellow five-spoke plastic rim with a double-D bore and a black rubber tyre with a block tread. DiffBot and Remo drive on it. The model was created independently, from caliper measurements and photos of the real wheel, photos and video frames of the robots, product listings and shop photos. No geometry was imported, converted, traced or sampled from another 3D model, and no third-party 3D model was downloaded.

Generator: [`generators/tt_motor_wheel_65mm.py`](../generators/tt_motor_wheel_65mm.py).

## Mesh

`tt_motor_wheel_65mm`: one mesh with two sub-parts, `rim` (material `rim`, yellow) and `tyre` (material `tyre`, black). Each is a closed solid, and they don't intersect. The STL file is closed and consistently oriented: two closed shells, with a modelling clearance of 0.02 mm where the tyre sits on the rim.

- **Barrel:** 25.00 mm wide, with a flange at each end (Ø51.22 mm), four grooves and three ridges between them (groove bottoms Ø47.91 mm, ridges Ø50.9 mm), and seven axial ribs across the two middle grooves. From the outer flange towards the motor side: a rectangular groove 3.9 mm wide, a groove 5.8 mm wide at the top and 4.1 mm at the bottom with two sloped walls, a groove 4.8 mm wide at the top and 3.9 mm at the bottom whose wall on the outer-face side is vertical, and a rectangular groove 3.9 mm wide. The flanges' outer corners are sharp, their inner corners round.
- **Outer face:** the outer flange's face is flat and 3.2 mm high, with a fine groove 0.6 mm wide, 1.3 mm from each edge. Inside the flange the barrel is Ø44.82 mm and plain for 3.5 mm. Then a lip 1.7 mm wide stands inward to Ø40.5 mm. Behind the lip the barrel is Ø45.1 mm to the motor-side end.
- **Spokes and hub:** the lip, five spokes and the hub's flat face are one plate, 1.7 mm thick, its face 3.5 mm behind the outer flange face. The spokes are flat and symmetric about a radial centre line. They taper from 8.9 mm near the hub to 5.35 mm where they run into the lip, with curved blends at both ends and a round mark on each. In the centre a bowl is recessed into the face, its edge in that face (Ø17.9 mm). Its wall runs almost straight down to the depth of the boss's top (Ø17.62 mm there), then rounds into a flat floor (Ø13.8 mm) with four small pins, 5.0 mm below the face. The boss (Ø9.69 mm) stands on the floor, its top 3.0 mm below the face. Behind the face the bowl's outside (Ø20.64 mm) shows through the windows. Behind each spoke are a narrow web and a fin up the hub tube.
- **Motor side:** the hub tube (Ø9.95 mm) on a collar (Ø15.27 mm), the double-D bore (Ø5.44 mm across the round sides), the tube standing 3.42 mm past the flange.
- **Tyre:** Ø66.6 mm, a hollow section like a car tyre's. The beads at its inner diameter (Ø49.4 mm) sit inside the rim, each in the groove next to a flange, held there because the bead is smaller than the flange and the ridge. The bead lip, where the bead touches the rim, is 0.74 mm thick. The sidewalls rise over the flanges and are widest halfway up (Ø59.2 mm), flush with the flanges' faces, 25.00 mm across. The crown is a little narrower and slightly crowned. Inside, an arched channel spans the barrel's three ridges: its ceiling lies 5.8 mm below the tread in the middle and 7.5 mm near the beads (the wall readings, their places across the tyre assumed). Six block columns and 42 rows of nearly square blocks.

## Frame

The files are in metres. Positions on this page are in millimetres. The axle runs along z at x = y = 0. The wheel's outer face is at z = -13 mm: the rim's outer flange face, flush with the tyre's sidewalls at their widest. The rim's motor-side flange face is at z = +12.00 mm, and the hub tube's end at z = +15.42 mm. Both robots turn the mesh so that +z points towards the robot.

In [DiffBot's description](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/diffbot_description/urdf/include/common_macros.urdf.xacro) and [Remo's description](https://github.com/ros-mobile-robots/remo_description/blob/main/urdf/include/common_macros.urdf.xacro), the `wheel` macro's `front_left_wheel_joint` and `front_right_wheel_joint` attach `front_left_wheel` and `front_right_wheel` to `chassis_link`: on DiffBot at x = 18.5, y = ±69 and z = -17 mm, on Remo at x = 36, y = ±72 and z = -33.6 mm. The joint's origin is the axle's centre in the mesh's frame. The visual turns the mesh by 90° about x, and on the right side then by 180° about z, so +z points towards the robot on both sides. The wheel's outer face lies 13 mm outboard of the joint's origin, 0.5 mm beyond the URDF's 25 mm wheel cylinder.

The tyre's radius is 33.3 mm. The axle lies one URDF wheel radius above `base_footprint`. DiffBot's [`front_wheel.yaml`](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/diffbot_description/config/diffbot/front_wheel.yaml) gives 33.3 mm, so the tyre touches the ground. Remo's [`front_wheel.yaml`](https://github.com/ros-mobile-robots/remo_description/blob/main/config/remo/front_wheel.yaml) gives 33.0 mm, so the tyre reaches 0.3 mm below the ground. Joints, collisions and inertias don't depend on the mesh. Nothing in the robots' descriptions refers to the motor-side end.

## Sources

| What | Source |
|---|---|
| Rim and tyre dimensions (the parameters marked pNN in the generator) | Caliper measurements of a wheel with the tyre taken off the rim, photos p01–p41, taken by the project (not published). Flange Ø51.22 (p01, p41: 51.25), groove bottoms Ø47.91 (p02), barrel wall with flange 3.06 at the motor-side end (p28), flange to flange 25.00 (p40), rim length 28.42 (p33), the bowl's outside behind the face Ø20.64 (p24) and its inside Ø17.62 at the boss's top (p21), boss Ø9.69 (p23), spoke width 5.35 at its tip (p25), the spoke's back 3.47 (p18, p19), fin 1.51 (p31), hub tube Ø9.97 and Ø9.92 (p29, p30), collar Ø15.27 (p15), bore Ø5.44 across the round sides (p14), tread to the tyre's inside 5.77, 6.88 and 7.46 (p04–p06), bead to tread 8.38 and 8.82 (p07, p08), the bead lip's thickness where it touches the rim 0.74 (p03) |
| The barrel's grooves, the outer flange's face, the barrel's inside and the hub's depths (the parameters marked "measured" in the generator) | Caliper measurements of the same wheel, taken by the project without photos. The four grooves' widths at their tops and bottoms and their wall shapes. The face's 3.2 mm, with the fine groove 1.3 mm from each edge. 3.5 mm from the outer face to the lip and the lip's 1.7 mm. Sharp outer and round inner corners on both flanges. The bowl's floor 5.0 mm and the boss's top 3.0 mm below the spokes' face |
| The tyre's section: hollow like a car tyre's, the beads narrower and inside the rim between the flanges, the sidewalls widest halfway up and flush with the flanges, the crown a little narrower | The project's description of the real tyre, with photos p03–p09 |
| The flanges' top widths (1.8 and 2.2 mm), the ridges' diameter and the ribs | The silhouette of side photo p37, scaled with the 25.00 mm flange to flange. The three ridges' tops (0.87 mm each) share what the grooves and the flanges leave of the width |
| The outer face's radii: the lip's inner edge Ø40.5 mm, the spokes' width near the hub (8.9 mm at r 13.2 mm) and their symmetry, the hub's flat face Ø22.4 mm, the bowl's edge Ø17.9 and floor Ø13.8 mm, four pins on the floor at r 6.15 mm, the spokes' marks at r 15.5 mm | Face-on photo p41, scaled with its own 51.25 mm reading. The lip's inner edge also on photo p25 |
| The lip, the spokes and the hub's face as one plate, the spokes' sides curving into the lip | A close photo across the outer face, taken by the project (not published) |
| Shapes not measured: the tread | Video frames of DiffBot's wheel, taken by the project (not published). The photo of Remo on ros-mobile-robots.com: [remo_front_side.jpg](https://ros-mobile-robots.com/resources/remo/remo_front_side.jpg) |
| Tyre outer diameter 66.6 mm | Measured on DiffBot: 66.3 and 66.8 mm, in DiffBot's [`front_wheel.yaml`](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/diffbot_description/config/diffbot/front_wheel.yaml) |
| Bore flats 3.66 mm (not measured) | Product listings: [X2 Robotics, "Rubber Tire Wheel (Yellow 65mm)"](https://x2robotics.ca/wheels/rubber-tire-wheel-yellow-65mm) ("Hole Width Axis: 3.66 mm", "Long Shaft Hole: 5.3 mm") and [VXB](https://vxb.com/products/four-pack-68mm-tt-motor-hub-replacement-diy-kit-fo) ("hole 3.66mm × 5.3mm") |
| Comparison photos | X2 Robotics' product photos (link above) and [Otronic, "Wheel with rubber for robot or smart car, yellow"](https://otronic.nl/en/wheel-with-rubber-for-robot-or-smart-car-yellow.html) |
| Colours | remo_description's wheel colours: rim yellow `0.95 0.72 0.05`, tyre black `0.10 0.10 0.10` |

## Simplified or estimated

- Estimated from photos: how far the lip stands inward (2.16 mm, to Ø40.5 mm), the flanges' top widths, the spokes' width near the hub, the radii where the spokes' sides curve into the lip (0.5 mm) and into the hub's face (3.0 mm), the round inner corners (0.5 mm) and the fine groove's depth (0.2 mm).
- The spokes' face lies level with the lip's front, and the plate is as thick as the lip is wide. The photos show the two as one surface. A step of a few tenths of a millimetre between them can't be ruled out.
- The spokes taper linearly to 5.35 mm where they run into the lip. Where along the spoke's tip this width was measured isn't recorded. The face-on photo shows about 5.9 mm at the lip.
- Behind the plate, these are estimates: the webs' depth (1.8 mm), the thickness of the bowl's floor (1.2 mm) and of the boss's closed top (0.8 mm), the collar's height (2.0 mm) and how far the fins reach up the hub tube.
- The barrel's inside diameter behind the lip is measured at the motor-side end. The model keeps it constant up to the lip.
- The tyre's width (flush with the flanges) follows the project's description, not a caliper reading. Estimated: the crown's width (18.6 mm) and drop (0.25 mm), the beads' width (1.8 mm), the channel's width (15.6 mm) and its half-elliptical arch, which takes the p04 reading as the middle and the p06 reading as near a bead (where each was taken isn't recorded), the pins' height, and the tread's groove width and depth (0.7 and 0.6 mm).
- The tyre's sidewalls and shoulders are quarter ellipses. The real tyre's inside has thin rings that the model leaves out.
- The motor-side collar is round. The real collar is scalloped between the fins.
- The ridges, flanges and ribs have flat faces and sharp edges, except the flanges' inner corners.

## Measurements that would improve the model

With calipers:

1. With the depth rod on the outer flange face: down to a spoke's face near its tip, and down to the lip between two spokes.
2. How far the lip stands inward: one outside jaw through a window on the lip's inner edge, the other on the flange's outer rim.
3. A spoke's width where it runs into the lip and, if the jaws reach, next to the barrel. Its thickness, with one jaw on its face and one on its back.
4. The flanges' top widths along the axle, and one ridge's top.
5. The tyre on the rim: its width at the sidewalls' widest point and over the tread, and the step from each flange's face to the sidewall (0 if flush).
6. The tyre off the rim: the wall from the tread's top to the tyre's inside, in the middle of the tread and 7 mm to each side of it.
7. The tread's groove depth, at a groove between two blocks.
