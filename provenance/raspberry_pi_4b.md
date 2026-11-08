<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Raspberry Pi 4 Model B

A display model of the Raspberry Pi 4 Model B: the board with its connectors, chips and small parts on both sides. Remo carries one. The model was created independently, from Raspberry Pi's public mechanical drawing and product brief. No geometry was imported, converted, traced or sampled from another 3D model. No logos and no printed text.

Generator: [`generators/raspberry_pi_4b.py`](../generators/raspberry_pi_4b.py).

## Mesh

`raspberry_pi_4b`: one mesh with 13 materials (green solder mask, gold pads and pins, silver shells, black and blue USB tongues, LEDs, tan latches and others). Its sub-parts are the board, the headers, the Ethernet jack, the two USB stacks, USB-C, the two micro-HDMI receptacles, the AV jack, the camera and display connectors, the chips, the small parts and the bottom side. Every part's solid is closed with outward normals. The STL file holds all parts together, so touching parts share faces, as in any assembly.

## Frame

Millimetres in the STL file, metres in the COLLADA and glTF files. The board's mid-plane is at z = 0. The board lies from x = -7.569 to 48.431 mm and from y = -42.123 to 42.877 mm. The USB and Ethernet end is at +y, the GPIO header along -x and the micro-HDMI side at +x. The drawing's (x, y) map to (48.431 - y, x - 42.123): a turn by +90° about z and a shift.

The frame is the frame of the Raspberry Pi mesh Remo used before. Only that mesh's bounding box, origin and axes were read, to align the frame:

| | min (mm) | max (mm) |
|---|---|---|
| Earlier mesh | (-7.569, -42.123, -2.900) | (50.831, 46.867, 17.000) |
| This mesh | (-7.569, -42.123, -2.800) | (50.921, 45.977, 16.800) |

The board sits exactly where the earlier mesh's board sat. In +y the mesh is 0.89 mm shorter: the drawing ends the Ethernet and USB fronts 3.0 mm past the board's edge. The top is the USB height, 16.0 mm above a 1.6 mm board. The bottom is the GPIO solder tails, 2.0 mm below the board.

Independent check: the centres of the four standoff bores of Remo's printed Raspberry Pi deck lie within 0.071 mm of this model's mounting holes, at the URDF's position of the board. This checks the holes' positions, not the clearance: the deck's standoffs end 7.0 mm above the deck, where the URDF puts the board's mid-plane, so the board's lower half (0.8 mm) overlaps the standoffs, as with the earlier mesh. A frame with z = 0 at the board's bottom face would change the URDF.

## Sources

1. **Raspberry Pi 4 Model B mechanical drawing**, PDF and DXF: <https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-mechanical-drawing.pdf> and <https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-mechanical-drawing.dxf>.
   - PDF: board 85 × 56 mm, corner radius 3.0 mm, holes 3.5 mm from the edges and 58 × 49 mm apart. Connector centres along the bottom edge: USB-C at 3.5 + 7.7 = 11.2 mm, micro-HDMI 0 at 26.0 mm (+14.8), micro-HDMI 1 at 39.5 mm (+13.5), AV jack at 54.0 mm (+7 +7.5). Along the right edge from the bottom: USB 2.0 at 9 mm, USB 3.0 at 27 mm, Ethernet at 45.75 mm. GPIO header's centre 29 mm from the left hole. Heights above the board ("Z"): GPIO 8.5, Ethernet 13.5, USB 16.0, USB-C 3.2, micro-HDMI 3.0, AV jack 6.0, camera and display connectors 5.5, processor 2.4 mm.
   - DXF, layers `BOARD_OUTLINE`, `PARTS_TOP`, `PARTS_BOTTOM` and `PINS_TOP`: the outlines of the parts in the drawing, for example the Ethernet jack (66.65 to 88.0 × 37.99 to 53.51 mm), the USB stacks to x = 88.0 mm, the AV barrel to y = -2.5 mm, the processor (21.75 to 36.75 × 25 to 40 mm), the RAM (39.05 to 50.55 × 25 to 40 mm), the wireless module (6.6 to 17.4 × 36.1 to 49.1 mm) and the microSD socket on the bottom (1.69 to 13.13 × 22.11 to 34.10 mm). Also the 40 GPIO pin centres (x = 8.37 + 2.54 k, y = 51.23 and 53.77 mm), the 4 PoE pins, the J2 pads, the through-hole pins of the Ethernet and USB connectors, and 108 small top and 100 small bottom part outlines, the generator's tables. The mounting pads are Ø6.0 mm, the holes Ø2.7 mm. Only coordinates were read from the DXF. Text, logos and the silkscreen layers aren't used.
2. **Raspberry Pi 4 Model B product brief**: <https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-product-brief.pdf>, page 3, "Physical specification": the same dimensions and the Ø2.7 mm holes.
3. **Colours and shapes of the parts only**, no dimensions: the public photos <https://commons.wikimedia.org/wiki/File:Raspberry_Pi_4_Model_B_-_Side.jpg> and <https://commons.wikimedia.org/wiki/File:Raspberry_Pi_4_Model_B_-_Top.jpg>. They show the green board, gold pads and pins, silver shells, blue USB 3.0 and black USB 2.0 tongues, the Ethernet latch notch at the bottom with green and yellow LEDs, the processor's silver lid with a rim, the wireless module's silver shield and the tan latches of the flat cable connectors.

Raspberry Pi's [licensing page](https://www.raspberrypi.com/licensing/) puts its PDF product documentation under CC BY-ND 4.0. The model doesn't copy or modify a document. It uses dimensions and positions, which are facts, to draw new geometry.

## Estimated or simplified

- **Board thickness 1.6 mm**: not in Raspberry Pi's documents. 1.6 mm is the common FR-4 thickness.
- **Heights the drawing doesn't give**: RAM 1.0 mm, wireless shield 1.5 mm, microSD socket 1.4 mm below the board, header plastic 2.5 mm (standard 2.54 mm headers), PoE header pins as high as the GPIO pins (8.5 mm), solder tails 2.0 mm, small parts 0.3 to 0.9 mm by size, the inductor 2.0 mm, two larger parts 2.0 and 2.2 mm.
- **Connector insides**: standard opening sizes (USB-A 12.5 × 5.12 mm, RJ45 11.7 mm wide with the latch notch, USB-C 8.94 × 3.2 mm shell), simplified tongues and contacts. Micro-HDMI is a 6.5 × 3.0 mm shell with a chamfered opening and a gold tongue.
- **Processor lid**: a 14.2 mm rim and a raised 12.2 mm square on a 15 × 15 mm substrate, shaped after the photo.
- **Small parts**: boxes or prisms from the drawing's outlines. The drawing notes that not all of the board's components are shown, so the model has fewer small parts than the real board.
- **Bottom**: the microSD socket without a card, the GPIO and PoE tails, the solder joints of the Ethernet and USB pins and pegs, and the small bottom parts. The bottom part J5 near the USB 2.0 stack is left out: its height is unknown.
- **J2** (RUN, GLOBAL_EN) is three gold rings on top of the board, without pins as on the real board. The rings aren't drilled through the board.

## Measurements that would improve the model

- Calipers on the board's thickness and on the microSD socket's height. Both change the bottom and the height by tenths of a millimetre.
- A side photo with a ruler, to check the connector heights the drawing doesn't give.
