<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Raspberry Pi Camera Module v2

A display model of the Raspberry Pi Camera Module v2: the board with its four mounting holes, the camera module with its lens, the module's flex and connector on the front, the 15-way ribbon connector and the small parts on the back. Remo carries this camera. The model was created independently, from Raspberry Pi's public mechanical drawing, documentation and product photo. No geometry was imported, converted, traced or sampled from another 3D model, and no third-party 3D model was downloaded. No logos and no printed text.

Generator: [`generators/raspberry_pi_camera_v2.py`](../generators/raspberry_pi_camera_v2.py).

## Mesh

`raspberry_pi_camera_v2`: one mesh with 12 materials: green solder mask, FR4 edges, gold hole rings and test pads, the black camera module, dark lens glass, the flex's orange edge, black and dark brown connectors, silver leads, black ICs and beige capacitors. Its sub-parts are the board, the pads, the camera module, the flex, the ribbon connector and the back's small parts. Raspberry Pi's documentation gives the v2's sensor as the Sony IMX219 or, for modules made from November 2026, the Sony IMX355. The model shows no sensor detail, so it fits both, and its name doesn't name a sensor.

## Frame

The files are in metres. Positions on this page are in millimetres. The board stands in the yz plane with its back face at x = -1.645 mm and the lens looking along +x. The board's edges are at y = -12.16 mm and z = -12.283 mm. The mounting holes are at (y, z) = (-10.16, -10.283), (-10.16, 2.217), (10.84, -10.283) and (10.84, 2.217) mm. The edge 2 mm from the holes is at -z. The ribbon connector's edge is at +z, on the back.

The model spans (-4.145, -12.160, -12.283) to (4.575, 12.840, 11.579) mm: from the ribbon connector on the back to the camera module on the front. The camera module's centre is at (y, z) = (0.34, 2.12) mm.

In [Remo's description](https://github.com/ros-mobile-robots/remo_description/blob/main/urdf/remo.urdf.xacro), `camera_joint` attaches `camera_link` to `raspi-cam_link`, the frame of the printed camera holder, 5 mm in front of it and 28 mm above it, tilted by 0.35 rad about y. There, the holder's plate is 0.8 mm behind the board's back face. The holder's screw bores and the board's mounting holes don't line up (see "Simplified or estimated"). `camera_link` is also the frame of the simulated camera in Gazebo.

## Sources

| Fact | Value used | Source |
|---|---|---|
| Board | 25 × 23.862 mm, 1 mm thick, corners R 2.0 mm | [Camera Module v2.1 mechanical drawing](https://datasheets.raspberrypi.com/camera/camera-module-2-mechanical-drawing.pdf) (RPI-CAM-V2_1, 2015) |
| Holes | 4 × Ø2.2 mm, 2 mm from the edges, 21 mm and 12.5 mm apart | The same drawing |
| Camera module | 8.5 × 8.5 mm, centred across, 9.462 mm from the connector edge, in line with the inner hole pair | The same drawing |
| Module height | About 5.2 mm on the board: foam 0.3 mm, body 2.4 mm, holder Ø7.0 × 1.4 mm, barrel Ø5.2 × 1.1 mm, glass Ø2.4 mm | Estimated. The [camera documentation](https://www.raspberrypi.com/documentation/accessories/camera.html) gives the whole camera as "Around 25 × 24 × 9 mm". Proportions from the product photo |
| Module flex connector | a 3.6 to 5.8 mm, b 7.05 to 15.2 mm (8.2 mm long), 1 mm high, black, with the flex's orange edge | The drawing: outline and centre 4.7 / 13.8 mm. The position matches the product photo |
| Ribbon connector | On the back, 5.5 mm from the edge, 20.8 mm long. Body 2.0 mm and latch 2.5 mm high | The drawing (5.5 and 20.8 mm). The heights are estimated, so that the total depth is about 9 mm |
| Back parts | 14 capacitors and resistors, 5 ICs (SOT-23-5, two SOT-23-6, two small square packages) at the drawing's positions | The drawing. The product photo shows the front without them |
| Gold rings and test pads | Rings Ø4.0 mm around the holes, two round pads Ø2.0 mm at (a 20.3, b 3.2) and (a 10.8, b 22.6) mm | The product photo on [raspberrypi.com/products/camera-module-v2](https://www.raspberrypi.com/products/camera-module-v2/), positions by a projective fit to its 4 holes |
| Colours | Green solder mask 0.10 0.42 0.22 (Remo's colour for the camera), gold, black module | The product photo |

Drawing coordinates: a from the edge next to the holes (along +z), b from the drawing's top edge (along +y, the drawing being the front view).

## Simplified or estimated

- The heights of the module, the connectors and the back parts are estimates: the drawing has no side view.
- Small parts are plain boxes. No traces, no silkscreen and no protective lens sticker.
- The module's flex is a black strip with an orange edge between the module and its connector.
- On Remo, some back parts reach into the printed holder's plate: the ribbon connector's lowest 0.9 mm by up to 1.2 mm, and three ICs by 0.2 mm. They are hidden inside the plate in every view.
- On Remo, the printed holder's four screw bores lie about 2 mm from the board's mounting holes: 1.9 mm towards +z and 0.4 mm towards -y, in the mesh's frame. The mesh shows where the camera sits, not a screw fit.

## Measurements that would improve the model

1. A side photo of the camera on Remo's holder: whether the board sits flat on the holder's plate or on its corner blocks, where the screws sit and where the ribbon connector is.
2. Calipers on the module's height and on the ribbon connector's height.
