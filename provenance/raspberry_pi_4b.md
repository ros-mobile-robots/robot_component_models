<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Raspberry Pi 4 Model B

A display model of the Raspberry Pi 4 Model B: the board with its connectors, chips and small parts on both sides. Remo carries one. The model was created independently, from Raspberry Pi's public mechanical drawing and product brief, and its look from Raspberry Pi's product photo. No geometry was imported, converted, traced or sampled from another 3D model. No logos and no printed text.

Generator: [`generators/raspberry_pi_4b.py`](../generators/raspberry_pi_4b.py).

## Mesh

`raspberry_pi_4b`: one mesh with 16 materials: semi-gloss green solder mask, pale FR-4 on the board's cut edges, gold pads, pins and hole plating, nickel connector shells and shields, the processor's steel lid, tin end caps and solder joints, black plastic, chip packages, ceramic part bodies, the processor's substrate, the inductor, blue USB 3.0 tongues, the cream bodies of the flat cable connectors and three LED colours. The metals are metallic in the glTF file (PBR metallic 1, roughness 0.17 to 0.23). In the COLLADA file they have strong, small highlights (specular 0.7 to 0.9, shininess 35 to 70); how strong they look depends on the viewer's lights: clear in Gazebo, flatter in RViz's default light. Its sub-parts are the board, the headers, the Ethernet jack, the two USB stacks, USB-C, the two micro-HDMI receptacles, the AV jack, the camera and display connectors, the chips, the small parts and the bottom side. Every part's solid is closed with outward normals. The STL file holds all parts together, so touching parts share faces, as in any assembly.

## Frame

The files are in metres. Positions on this page are in millimetres. The board's mid-plane is at z = 0. The board lies from x = -7.569 to 48.431 mm and from y = -42.123 to 42.877 mm. The USB and Ethernet end is at +y, the GPIO header along -x and the micro-HDMI side at +x. The drawing's (x, y) map to (48.431 - y, x - 42.123): a turn by +90° about z and a shift.

**Frame alignment:** the reference Raspberry Pi mesh's bounding box, origin and axes were read to position the model:

| | min (mm) | max (mm) |
|---|---|---|
| Reference mesh | (-7.569, -42.123, -2.900) | (50.831, 46.867, 17.000) |
| This model | (-7.569, -42.123, -2.800) | (50.921, 45.977, 16.800) |

The boards coincide. In +y the model ends 0.89 mm short of the reference: the drawing ends the Ethernet and USB fronts 3.0 mm past the board's edge. The top is the USB height, 16.0 mm above a 1.6 mm board. The bottom is the GPIO solder tails, 2.0 mm below the board.

Independent check: the centres of the four standoff bores of Remo's printed Raspberry Pi deck lie within 0.071 mm of this model's mounting holes, at the URDF's position of the board. This checks the holes' positions, not the clearance: the deck's standoffs end 7.0 mm above the deck, where the URDF puts the board's mid-plane, so the board's lower half (0.8 mm) overlaps the standoffs. A frame with z = 0 at the board's bottom face would change the URDF.

## Sources

1. **Raspberry Pi 4 Model B mechanical drawing**, PDF and DXF: <https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-mechanical-drawing.pdf> and <https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-mechanical-drawing.dxf>.
   - PDF: board 85 × 56 mm, corner radius 3.0 mm, holes 3.5 mm from the edges and 58 × 49 mm apart. Connector centres along the bottom edge: USB-C at 3.5 + 7.7 = 11.2 mm, micro-HDMI 0 at 26.0 mm (+14.8), micro-HDMI 1 at 39.5 mm (+13.5), AV jack at 54.0 mm (+7 +7.5). Along the right edge from the bottom: USB 2.0 at 9 mm, USB 3.0 at 27 mm, Ethernet at 45.75 mm. GPIO header's centre 29 mm from the left hole. Heights above the board ("Z"): GPIO 8.5, Ethernet 13.5, USB 16.0, USB-C 3.2, micro-HDMI 3.0, AV jack 6.0, camera and display connectors 5.5, processor 2.4 mm.
   - DXF, layers `BOARD_OUTLINE`, `PARTS_TOP`, `PARTS_BOTTOM` and `PINS_TOP`: the outlines of the parts in the drawing, for example the Ethernet jack (66.65 to 88.0 × 37.99 to 53.51 mm), the USB stacks to x = 88.0 mm, the AV barrel to y = -2.5 mm, the processor (21.75 to 36.75 × 25 to 40 mm), the RAM (39.05 to 50.55 × 25 to 40 mm), the wireless module (6.6 to 17.4 × 36.1 to 49.1 mm) and the microSD socket on the bottom (1.69 to 13.13 × 22.11 to 34.10 mm). Also the 40 GPIO pin centres (x = 8.37 + 2.54 k, y = 51.23 and 53.77 mm), the 4 PoE pins, the J2 pads, the through-hole pins of the Ethernet and USB connectors, and 108 small top and 100 small bottom part outlines, the generator's tables. The mounting pads are Ø6.0 mm, the holes Ø2.7 mm. Only coordinates were read from the DXF. Text, logos and the silkscreen layers aren't used.
2. **Raspberry Pi 4 Model B product brief**: <https://datasheets.raspberrypi.com/rpi4/raspberry-pi-4-product-brief.pdf>, page 3, "Physical specification": the same dimensions and the Ø2.7 mm holes.
3. **Raspberry Pi's product photo** of the Pi 4 Model B, in its documentation repository: <https://github.com/raspberrypi/documentation/blob/master/documentation/asciidoc/computers/raspberry-pi/images/4-model-b.jpg>. Used for the look only, no dimensions: the colours were sampled from it (medians of small patches, sRGB): solder mask about 0.30 to 0.38, 0.60 to 0.71, 0.50 to 0.58 where lit, FR-4 edge 0.83 0.77 0.66, gold pads 0.94 0.74 0.25, the processor lid 0.79 0.75 0.73, the connector shells 0.78 0.73 0.73, USB 3.0 tongues 0.46 0.69 0.80. It also shows the shapes the model follows: the USB shells' front frames, ears, divider and spring fingers, the Ethernet shield's rounded front edge, the processor's stamped lid, the end caps of the small parts, the pads of the chips and the flat cable connectors' cream bodies with dark latches.
4. **Colours and shapes of the parts only**, no dimensions: the public photos <https://commons.wikimedia.org/wiki/File:Raspberry_Pi_4_Model_B_-_Side.jpg> and <https://commons.wikimedia.org/wiki/File:Raspberry_Pi_4_Model_B_-_Top.jpg>. They show the green board, gold pads and pins, silver shells, blue USB 3.0 and black USB 2.0 tongues, the Ethernet latch notch at the bottom with green and yellow LEDs, the processor's silver lid with a rim, the wireless module's silver shield and the flat cable connectors' cream bodies with dark latches.

Raspberry Pi's [licensing page](https://www.raspberrypi.com/licensing/) puts its PDF product documentation under CC BY-ND 4.0. The model doesn't copy or modify a document. It uses dimensions and positions, which are facts, to draw new geometry.

## Estimated or simplified

- **Board thickness 1.6 mm**: not in Raspberry Pi's documents. 1.6 mm is the common FR-4 thickness.
- **Heights the drawing doesn't give**: RAM 1.0 mm, wireless shield 1.5 mm, microSD socket 1.4 mm below the board, header plastic 2.5 mm (standard 2.54 mm headers), PoE header pins as high as the GPIO pins (8.5 mm), solder tails 2.0 mm, small parts 0.3 to 0.9 mm by size, the inductor 2.0 mm, two larger parts 2.0 and 2.2 mm.
- **Connector insides**: standard opening sizes (USB-A 12.5 × 5.12 mm, RJ45 11.7 mm wide with the latch notch, USB-C 8.94 × 3.2 mm shell), simplified tongues and contacts. Micro-HDMI is a 6.5 × 3.0 mm shell with rounded top edges, a chamfered opening, a dark lining and a tongue with ten gold contacts.
- **Connector details**, shaped after the photo, sizes estimated, inside the drawing's outlines except the USB fronts' 0.35 mm ears and the Ethernet LEDs' 0.1 mm: the USB stacks' front frame with 0.35 mm ears, two spring fingers in the top (1.4 × 3.2 mm, 0.15 mm deep), two dimples, snap recesses in the sides, a dark lining in each port and the USB 3.0 tongues' second contact row. The Ethernet shield's rounded front edge (1.0 mm) and vertical edges (0.5 mm), ground-tab recesses in its sides. The USB-C receptacle's dark lining and 2 × 12 contacts. The AV jack's metal clips on its top and side. Header blocks one per pin pair, with chamfered top edges, and pins with pointed ends.
- **Processor lid**: a 14.2 mm rim and a raised 12.2 mm square with rounded edges on a 15 × 15 mm substrate, shaped after the photo.
- **Small parts**: from the drawing's outlines. Rectangles below 3.5 mm² are chip capacitors and resistors with tin end caps, each 22 % of their length. Larger rectangles on the top are chips with tin pads at 0.5 mm along their edges; the pad count isn't from a drawing. Rotated or other outlines are plain prisms. The drawing notes that not all of the board's components are shown, so the model has fewer small parts than the real board.
- **Bottom**: the microSD socket without a card, the GPIO and PoE tails, the solder joints of the Ethernet and USB pins and pegs, and the small bottom parts. The bottom part J5 near the USB 2.0 stack is left out: its height is unknown.
- **J2** (RUN, GLOBAL_EN): three Ø1.0 mm holes through the board with gold rings on both sides, without pins as on the real board.
- **Board**: the cut edges show pale FR-4, the mounting holes' and J2's walls are plated (gold). No copper traces and no silkscreen.

## Measurements that would improve the model

- Calipers on the board's thickness and on the microSD socket's height. Both change the bottom and the height by tenths of a millimetre.
- A side photo with a ruler, to check the connector heights the drawing doesn't give.
