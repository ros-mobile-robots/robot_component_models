<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# 360 degree laser scanner, RPLIDAR A1 size

A 360 degree laser scanner with the outline of the Slamtec RPLIDAR A1 (A1M8): a black base shaped like a drop, the motor below its narrow end, a flat belt from the motor's pulley to the rotating head, the head with its two openings, and the circuit board with its connectors under the base. The model was created independently, from the public datasheet's mechanical drawing and Slamtec's product pictures. No geometry was imported, converted, traced or sampled from another 3D model, and no vendor 3D model was downloaded. No logos and no printed text: the badge on the head keeps its ring, without the brand name.

Generator: [`generators/rplidar_a1.py`](../generators/rplidar_a1.py).

## Mesh

`rplidar_a1`: one mesh with eight sub-parts, each a named node for an exploded view.

| Sub-part | What | Materials |
|---|---|---|
| `base` | The plate with its outline, the four mounting holes with a spot face on top, the wall, bosses and the housing under it | `body` (black) |
| `pcb` | The circuit board under the base, a square and a round part on its underside, four screws | `pcb` (green), `part` (black), `screw` (dark steel) |
| `connectors` | Three vertical headers on the board's underside: 2, 3 and 4 pins at 1.25 mm pitch, with their pins and fixing tabs | `connector` (off-white), `contact` (tin) |
| `motor` | The motor's can below the base with its bearing boss, two screws on the base beside the pulley | `steel`, `screw` |
| `pulley` | The motor's pulley above the base: two flanges and a stepped cap | `body` |
| `belt` | The flat belt round the head's pulley and the motor's pulley | `rubber` |
| `head` | The rotating head: its pulley with a flange, the cylinder with a rounded top edge, the badge's ring, three screw recesses, the two openings with the lenses' barrels | `body` |
| `window` | The dark glass at the bottom of both openings and the two lenses | `window` |

The metal materials are metallic in the glTF file. Each solid is closed and consistently oriented. The sub-parts touch but don't intersect.

## Frame

The files are in metres. Positions on this page are in millimetres.

The origin lies on the head's rotation axis, in the scan plane: 42 mm above the motor's bottom, the middle of the datasheet's 10 mm laser working area. +z points up. +x points from the motor to the base's round end, Slamtec's 0° direction for the A1. The motor's axis is at x = -45.5 mm. The bottom is at z = -42 mm, the head's top at z = +9 mm, the base's top face at z = -15.5 mm.

The head turns when the scanner runs, so the window's direction in the mesh is a choice. The model draws the window's middle, between the two openings, towards -x, the motor's side.

No robot description uses this mesh yet. The axes follow the RPLIDAR A2 model's: +x points to the scanner's 0° direction, away from the motor here and away from the cable on the A2. So a description attaches both scanners alike. [rplidar_ros](https://github.com/Slamtec/rplidar_ros) publishes the scan in a frame whose +x points to the scanner's 180° direction, towards the A1's motor, and DiffBot's and Remo's `rplidar_laser_link` is that frame. The `rplidar_laser_link`'s visual turns the mesh by π about z, as for the A2. A scanner mounted with its motor towards the robot's front, on a parent frame whose +x points to the front, then has a `rplidar_laser_joint` with a yaw of 0, and the window faces the robot's front. Mounted the other way round, like the A2 with its cable towards the rear, the joint's yaw is π.

## Sources

Primary source: Slamtec, *RPLIDAR A1 Low Cost 360 Degree Laser Range Scanner, Introduction and Datasheet, Model A1M8*, rev. 3.0, 2020-10-15, chapter "Mechanical and Customization Options", Figure 5-2 "The Mechanical Dimensions of RPLIDAR A1" (page 16). Slamtec's support page links it as "A1M8 Datasheet": <https://www.slamtec.com/en/support>. The drawing has a top view, a side view, a bottom view, a view from the round end and a detail of the connectors. Values without a dimension are read off the drawing's scale, at 8.65 pixels per mm in a 400 dpi rendering of page 16.

| What | Value used | Where |
|---|---|---|
| Base and height | The base 96.74 mm long and 70.28 mm wide, 51 mm from the motor's bottom to the head's top. The belt stands out 0.36 mm beyond the base's round end and sides, so the mesh measures 97.08 × 71.00 × 51.00 mm | Figure 5-2, side view and view from the round end |
| Head | Ø70.04 mm, its cylinder from 31.50 mm above the bottom to the top | Figure 5-2, side view |
| Scan plane | 42 mm above the bottom, in a laser working area 10 mm high | Figure 5-2, side view, "Laser Working Area" |
| Base | From 21 to 26.50 mm above the bottom, its round end 35.14 mm from the head's axis | Figure 5-2, side view |
| Motor | Ø32 mm, its axis 45.50 mm from the head's axis | Figure 5-2, side and top views |
| Mounting holes | 4 × Ø3 mm: two 56 mm apart, 28 mm from the head's axis towards the round end, two 40 mm apart, 42 mm from it towards the motor | Figure 5-2, top view |
| Board | Its underside 15 mm above the bottom (6 mm below the base), the lowest part 7.50 mm below it | Figure 5-2, side view |
| Connectors | Headers with 1.25 mm pitch, 3 pins for the motor and 4 pins for the data, both vertical as in the figure. Page 10 calls the 3-pin header horizontal, the figure vertical. The model follows the figure. The 2-pin header beside them has no label, and its 1.25 mm pitch is assumed | Figure 5-2, connector detail. Page 10 names them PH1.25-3P and PH1.25-4P |
| Base outline | The long sides are arcs through the widest point at the head's axis, tangent to the arc round the motor end (R16.10 mm, from the overall length). So their radius follows from the dimensions: 79.99 mm. The round end is an arc of R90 mm between two ears of R3.3 mm round the holes | Figure 5-2, top view, read off the scale |
| Read off the drawing's scale | Head's top edge R1 mm. Badge ring R14.44 to R14.96 mm. Three screw recesses Ø5 mm on R28 mm, at 3°, -128° and 138° from the window's middle, anticlockwise seen from above. The two openings 32.5° either side of the window's middle: the receiver's at +32.5°, 17.5 × 10 mm with R3.5 mm corners and a Ø6 mm lens, the emitter's at -32.5°, 9.6 × 7.8 mm with a Ø2.6 mm lens. Motor can from 3.5 to 20.9 mm, its boss Ø7 mm below it. Motor pulley Ø18 mm with a step of R6 mm and a boss of R2.5 mm. Belt's outer radius 35.5 mm round the head and 9.5 mm round the pulley, between 28.2 and 30.8 mm. Two screws beside the pulley at y = ±11.25 mm. Board from x = -28.5 to +17.2 mm, 57.2 mm wide, with four screws at x, y = ±11.2 mm and two holes at y = ±22.5 mm. Connectors' centres at y = -19.1, -9.0 and +2.7 mm, 4.6 mm high. A square part of 6.9 × 7.3 mm, 3.8 mm high, and a round part of Ø5.6 mm. Housing under the base R29.5 mm from 17.8 mm up. Spot faces and bosses Ø5.7 mm | Figure 5-2, all views |
| Direction of the openings | Seen from above, the emitter's opening lies clockwise of the receiver's, about 65° apart | Figure 5-2, side view and view from the round end, and Slamtec's product pictures |
| 0° direction | Away from the motor, angles clockwise seen from above | Slamtec, *RPLIDAR Interface Protocol and Application Notes* (file `LR001_SLAMTEC_rplidar_protocol_v2.4_en.pdf`), Figure 4-6 (page 17), linked as "Protocol" on Slamtec's support page |
| Look and colours | Matte black head and base, a silver motor can, black pulley and belt, a green board | Slamtec's A1 product page, <https://www.slamtec.com/en/lidar/a1>, its banner picture, and the datasheet's Figure 1-1 (page 3) |
| Choices | Plate 2.5 mm and wall 1.6 mm thick, spot faces 0.4 mm and screw recesses 2 mm deep, the badge's groove 0.3 mm deep with edges rounded by 0.15 mm, openings 5 mm deep, glass 0.8 mm, belt 0.8 mm thick, heights of the pulleys' flanges, screw heads | Parameters at the top of the generator |

## Simplified

- The head's insides, the optics and the screws in the head's recesses are left out. Each opening has a dark pane and a lens at its bottom.
- The base's underside is a plain shell with four bosses and a round housing, without the ribs of the bottom view.
- The motor's can has no terminals, and its end cap no ribs.
- The board carries only the parts that the drawing shows: three connectors, a square and a round part, four screws. No traces and no text.
- The legs of Slamtec's development kit aren't part of the scanner and are left out.
- The head doesn't rotate: the mesh is one rigid model.

## Measurements that would improve the model

The datasheet fixes the main sizes. These would refine the details:

1. The openings' size and position on the head, and the angle between them, from a photo straight at the head.
2. The motor pulley's and the belt's heights above the base.
3. The board's parts: the round part's height and the connectors' height below the board.
4. The base plate's thickness and the underside's ribs.
