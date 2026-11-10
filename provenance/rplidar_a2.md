<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# 360 degree laser scanner, RPLIDAR A2 size

A 360 degree laser scanner with the outline of the Slamtec RPLIDAR A2 (A2M8): a black base, a rotating head with a copper-coloured band, and the optical window with two dark openings. DiffBot and Remo carry this scanner. The model was created independently, from the public datasheet's dimensions and a public photo. No geometry was imported, converted, traced or sampled from another 3D model, and no vendor 3D model was downloaded. No cable, no logos and no printed text: the head's badge is left out.

Generator: [`generators/rplidar_a2.py`](../generators/rplidar_a2.py).

## Mesh

`rplidar_a2`: one mesh with three sub-parts. `housing` (material `body`, black) is the base and the head. `trim` (copper-coloured) is the band and the window's bezel. `windows` (dark glass) are the two panes. The STL file is closed and consistently oriented: several closed shells that touch.

## Frame

The STL and glTF files are in metres, the COLLADA file is in millimetres. Positions on this page are in millimetres.

The origin lies on the scanner's rotation axis, 29.85 mm above its bottom. +z points to the top, +x towards the optical window, and the cable exit is at -x. The bottom is at z = -29.85 mm, the top at z = +10.95 mm. The optical window's centre is at z = +0.95 mm, in the middle of the 6 mm laser band, so the origin lies in the laser band.

In [DiffBot's](https://github.com/ros-mobile-robots/diffbot/blob/noetic-devel/diffbot_description/urdf/include/common_sensors.xacro) and [Remo's](https://github.com/ros-mobile-robots/remo_description/blob/main/urdf/remo.urdf.xacro) descriptions, `rplidar_laser_joint` attaches `rplidar_laser_link` to `chassis_link` (DiffBot, 40 mm above it) and to `lidar_platform_link` (Remo, 70 mm above it). Both turn the joint and the visual by π about z, so the window faces the robot's front. `rplidar_laser_link` is also the frame of the simulated laser in Gazebo, so its scan plane passes through the window.

## Sources

Primary source: Slamtec, *RPLIDAR A2 Low Cost 360 Degree Laser Range Scanner, Introduction and Datasheet, Model A2M8*, rev. 2.5, 2019-10-25, chapter "Mechanical Dimensions", Figure 5-1 (page 18). A copy is hosted by Farnell: <https://www.farnell.com/datasheets/3178291.pdf>. Slamtec's product page: <https://www.slamtec.com/en/Lidar/A2>.

| What | Value used | Where |
|---|---|---|
| Base | Ø75.70 mm, 18.60 mm high | Figure 5-1, side view |
| Head | Ø72.50 mm, from 19.40 mm above the bottom | Figure 5-1 |
| Band at the head's lower edge | 1.30 mm high | Figure 5-1 |
| Overall height | 40.80 mm | Figure 5-1 |
| Optical window's centre | 30.80 mm above the bottom, laser band 6 mm | Figure 5-1, with the note 激光作用范围 (laser working range) |
| Mounting holes | 4 × M3 on R29.50, at 45° to the cable axis | Figure 5-1, bottom view. The hole depth isn't given. The model's 4 mm follows the note on page 18: the M3 screws in the bottom should be no longer than 4 mm |
| Locating hole | Ø3 × 1.0 mm on R29.50, opposite the cable | Figure 5-1, bottom view, 定位孔 (locating hole) |
| Cable cover on the bottom | inner edge 25 mm from the centre, about 26 mm wide, two screws 4.75 mm further out | Figure 5-1, bottom view |
| Read off the drawing's scale | Head's top edge radius 11 mm. Neck Ø71 mm between 18.60 and 19.40 mm. Window bezel 58 mm long and 14.9 mm high, rim 1.1 mm wide. Two openings 18.7 × 10 mm, 17.7 mm either side of the window's centre. Six case screws on R34 at 30°, 90°, 150°, 210°, 270° and 330° | Figure 5-1, side and bottom views |
| Look and colours | Black matte base and head, copper-coloured bezel and band, two dark oval openings | A photo of Remo on ros-mobile-robots.com, [remo_front_side.jpg](https://ros-mobile-robots.com/resources/remo/remo_front_side.jpg) |
| Cable direction | Cable exit on the laser frame's +x, so on the mesh's -x | The [laser range scanner](https://ros-mobile-robots.com/laser-range-scanner/) page on ros-mobile-robots.com, and the robots' URDF |
| Choices | Recess depths (bezel 1.0 mm, band 1.2 mm, openings 7.8 mm), bezel 0.3 mm proud, glass 0.8 mm, a cable boot of 3.0 × 9 × 4.5 mm | Parameters at the top of the generator |

## Simplified

- The head is one rounded body. The real head's raised badge with the brand name is left out.
- The openings are plain stadium-shaped cuts with a dark pane at the bottom. No emitter or lens inside.
- No cable, no connector and no label on the bottom. The six case screws are shallow round recesses.
- The head doesn't rotate: both robots have a fixed link for the scanner.

## Measurements that would improve the model

The datasheet fixes the main sizes. These would refine details:

1. The neck's diameter between base and head, and the copper band's height.
2. The window bezel's outer length and height, how far it stands out, and each opening's size.
3. The radius of the head's top edge, from a side photo with a ruler.
4. The cable boot's width, height and how far it stands out from the base.
