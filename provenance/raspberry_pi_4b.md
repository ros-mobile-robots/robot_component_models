<!-- SPDX-FileCopyrightText: 2026 ROS Mobile Robots
     SPDX-License-Identifier: BSD-3-Clause -->
# Raspberry Pi 4 Model B

A display model of the Raspberry Pi 4 Model B: the board with its connectors, chips and small parts on both sides. Remo carries one. The model was created independently, from Raspberry Pi's public mechanical drawing and product brief, the connector makers' public drawings for the insides of the connectors, and Raspberry Pi's and Wikimedia Commons' product photos for the look and a size check. No geometry was imported, converted, traced or sampled from another 3D model. No logos and no printed text.

Generator: [`generators/raspberry_pi_4b.py`](../generators/raspberry_pi_4b.py).

## Mesh

`raspberry_pi_4b`: one mesh with 18 materials: semi-gloss green solder mask, pale FR-4 on the board's cut edges, gold pads, pins, hole plating and contacts, nickel connector shells and shields, the processor's steel lid, tin end caps and solder joints, black plastic, chip packages, the processor's silicon die and the thermal paste on it, ceramic part bodies, the processor's substrate, the inductor, blue USB 3.0 tongues, the cream bodies of the flat cable connectors and three LED colours. The metals are metallic in the glTF file (PBR metallic 1, roughness 0.17 to 0.23). In the COLLADA file they have strong, small highlights (specular 0.7 to 0.9, shininess 35 to 70). How strong they look depends on the viewer's lights: clear in Gazebo, flatter in RViz's default light. Each sub-part is a named node in the COLLADA and glTF files, so a viewer can move the parts apart for an exploded view:

- `board`, `gpio_header`, `poe_header`
- connectors: `ethernet`, `usb_2`, `usb_3`, `usb_c`, `micro_hdmi_0`, `micro_hdmi_1`, `audio_jack`, `camera_connector`, `display_connector`
- the processor in three layers: `soc_substrate`, `soc_die` (the die with thermal paste on it) and `soc_lid`, a hollow stamped lid over the die
- `ram`, and the wireless module in two layers: `wireless_chip` (the chip and its small parts) under the hollow `wireless_shield`
- `usb_controller`, `ethernet_phy`, `pmic`, `small_parts_top`
- on the bottom: `micro_sd_socket`, `small_parts_bottom`, `solder_joints`

Every part's solid is closed with outward normals. The STL file holds all parts together, so touching parts share faces, as in any assembly.

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
4. **Public photos on Wikimedia Commons**: <https://commons.wikimedia.org/wiki/File:Raspberry_Pi_4_Model_B_-_Side.jpg> and <https://commons.wikimedia.org/wiki/File:Raspberry_Pi_4_Model_B_-_Top.jpg>. They show the green board, gold pads and pins, silver shells, blue USB 3.0 and black USB 2.0 tongues, the Ethernet latch notch at the bottom with green and yellow LEDs, the processor's silver lid with a rim, the wireless module's silver shield and the flat cable connectors' cream bodies with dark latches. The top photo (3965 × 2647 pixels, about 44 pixels per mm) is also the reference of the [photo check](#photo-check).
5. **Insides of the USB-A receptacles**: GCT's drawing of the stacked USB 3.0 Type-A receptacle USB1100, <https://gct.co/files/drawings/usb1100.pdf>, sheet 1. The opening 12.50 × 5.12 mm, the tongue 11.10 × 1.84 mm with its top 0.40 mm below the opening's top, the rear housing 10.25 mm behind the front, the two ports 8.50 mm apart, the four USB 2.0 contacts 2.0 and 7.0 mm apart (pairs), about 0.95 mm wide and 0.35 mm below the tongue (2.59 mm below the opening's top), the five SuperSpeed contacts within 3.0 mm of the tongue's tip.
6. **Insides of the micro-HDMI receptacles**: Kycon's drawing of the micro-HDMI (Type D) receptacle KMMDMIX-NS3-WS-BTRT, <https://anacapa.kycon.com/Pub_Eng_Draw/KMMDMIX-NS3-WS-BTRT.pdf>, sheet 1. The opening 5.90 × 2.30 mm with chamfered lower corners, the tongue's contact faces 0.80 and 1.30 mm below the opening's top, 10 contacts on the upper face and 9 on the lower one, 0.40 mm apart, the body 6.50 mm wide.
7. **Insides of the USB-C receptacle**: GCT's drawing of the USB Type-C receptacle USB4525, <https://gct.co/files/drawings/usb4525.pdf>, sheet 1. The opening 8.34 × 2.56 mm, the shell 8.94 × 3.16 mm, the insulator 4.65 mm behind the front, the tongue 6.69 mm wide, 0.70 mm thick (1.20 mm near its root) with its tip 0.20 mm behind the front, 12 contacts on each face at 0.50 mm.
8. **RJ45 contacts**: the eight contacts at the modular plug's 1.02 mm (0.040 in) pitch. The makers' public drawings of Gigabit jacks with magnetics give only the outside (for example Same Sky's CRJ013-ML1-TH, <https://www.sameskydevices.com/product/resource/crj013-ml1-th.pdf>, page 3: the LEDs at the bottom corners beside the latch notch, the contacts at the top of the opening).

Raspberry Pi's [licensing page](https://www.raspberrypi.com/licensing/) puts its PDF product documentation under CC BY-ND 4.0. The model doesn't copy or modify a document. It uses dimensions and positions, which are facts, to draw new geometry.

## Photo check

The model was rendered through the camera of the Wikimedia Commons top photo (source 4) and compared with it: side by side, as a 50 % overlay and as outlines. The camera was fitted to the four mounting holes and the tips of the 40 GPIO and 4 PoE pins (8.5 mm above the board): they fit within 0.04 and 0.09 mm (RMS). Each part's top face was then resampled from the photo at 80 pixels per mm, and its edges were read by eye. The photo itself isn't part of this repository. No public photo of the Pi 4's bottom was found (Wikimedia Commons, Raspberry Pi's documentation), so the bottom follows the drawing only.

Inside the board, every part's centre in the photo lies 0.1 to 0.45 mm towards +x and 0.25 to 0.55 mm towards -y of the drawing's position. J2's three holes show the same shift (0.39 and 0.50 mm), while the mounting holes and pin tips the camera was fitted to agree within 0.1 mm. The photo alone can't tell whether the camera fit, the photographed board or the drawing causes the shift, so the positions follow the drawing. The table compares sizes (x along the 85 mm edge, y along the 56 mm edge):

| Part | Model (mm) | Photo (mm) | Result |
|---|---|---|---|
| Processor lid, top | 12.6 × 12.6 | 12.52 × 12.76 | From the photo. Its top edges are rounded by 0.7 mm, as the photo's 0.8 to 1.0 mm wide bright band shows. |
| Processor lid, rim | 14.45 × 14.45 | 14.51 × 14.45 | Within 0.1 mm |
| RAM | 10.0 × 14.5 | 10.25 × 14.47 | Within 0.25 mm, probably because the package's +x edge merges with its shadow. 10.0 × 14.5 mm is a 200-ball LPDDR4 package's size. |
| Wireless shield | 10.6 × 12.8 | 10.54 × 12.80 | Within 0.1 mm |
| USB controller | 8.0 × 8.0 | 8.31 × 7.94 | Within 0.31 mm, probably the +x edge with its shadow |
| Ethernet controller | 6.0 × 6.0 | 6.04 × 6.03 | Within 0.05 mm |
| Power management chip | 5.0 × 5.0 | 5.03 × 4.97 | Within 0.05 mm |
| Tall part at x 65.55 | 4.3 × 8.3 | 4.30 × 8.27 | From the photo: its tin terminals reach 0.5 mm past both ends of the drawing's 7.3 mm. |
| USB-C | 8.94 wide | 8.74 | Within 0.2 mm, GCT's 8.94 mm shell |
| Micro-HDMI 0 and 1 | 6.5 wide | 6.46 and 6.50 | Within 0.05 mm, Kycon's 6.50 mm body |
| AV jack | 6.6 × 12.25 | 6.58 × 12.25 | From the photo, inside the drawing's 7.0 × 12.5 mm outline. The photo also gives its metal band over the front 3.6 mm of the top, the two contact legs along its sides and the tab between them. |
| Camera connector, latch | 1.85 × 22.1 | 1.76 × 22.08 | From the photo |
| Camera and display connectors, solder tails | 4.55 wide | 4.86 and 4.35 | From the photo: tails on both sides, reaching 0.3 mm past the drawing's 3.95 mm. The two connectors' tails read 0.5 mm apart: their ends blur into the solder. |
| GPIO header, plastic | 50.8 long | 50.8 | Same |
| Ethernet jack | 21.35 × 15.52 | 21.51 × 15.62 | Within 0.2 mm |
| USB 2.0 and USB 3.0 stacks, top | 17.4 × 13.92 and 17.2 × 13.82 | 16.2 × 11.76 and 16.7 × 12.48 | The drawing's outlines. A USB-A receptacle's shell is at least 13.1 mm wide: the 12.5 mm opening and two walls. Plausible reasons for the narrow reading: the stacks' rounded, dark top edges, and the 16 mm high tops lie furthest from the points the camera was fitted to. |

## Estimated or simplified

- **Board thickness 1.6 mm**: not in Raspberry Pi's documents. 1.6 mm is the common FR-4 thickness.
- **Heights the drawing doesn't give**: RAM 1.0 mm, wireless shield 1.5 mm, microSD socket 1.4 mm below the board, header plastic 2.5 mm (standard 2.54 mm headers), PoE header pins as high as the GPIO pins (8.5 mm), solder tails 2.0 mm, small parts 0.3 to 0.9 mm by size, the inductor 2.0 mm, two larger parts 2.0 and 2.2 mm.
- **Connector insides**, from the drawings of sources 5 to 8: the inside walls of the metal shells show up to a black insulator, the tongues carry their contacts, the USB 2.0 contacts are springs that bulge below the tongue, the USB 3.0 tongues also have their five flat contacts near the tip, and two dimples in each USB port's floor hold the plug. The Pi's own connectors may differ from these makers' parts in details the photos don't show. Estimated: the contacts' lengths and widths where the drawings give only the solder tails (USB-C 0.25 mm, micro-HDMI 0.2 mm), the micro-HDMI tongue's 4.6 mm width (scaled from Kycon's drawing), the SuperSpeed contacts' 2.3 mm pitch, and the RJ45 springs: 0.4 mm wide, from the top of the opening at the front down and back at about 23 degrees, their ends in a black comb.
- **Connector details**, shaped after the photo, sizes estimated, inside the drawing's outlines except the USB fronts' 0.35 mm ears and the Ethernet LEDs' 0.1 mm: the USB stacks' front frame with 0.35 mm ears, two spring fingers in the top (1.4 × 3.2 mm, 0.15 mm deep), two dimples and snap recesses in the sides. The Ethernet shield's rounded front edge (1.0 mm) and vertical edges (0.5 mm), ground-tab recesses in its sides. The USB-C tongue's metal middle plate, which shows at its edges. The AV jack's metal band and legs on its top, in 0.2 mm recesses, flush with the body's top (see the [photo check](#photo-check)). Header blocks one per pin pair, with chamfered top edges, and pins with pointed ends.
- **Processor**: a 15 × 15 mm substrate, a 9 × 9 × 0.75 mm die with thermal paste on it, and a hollow lid of 0.3 mm sheet over them: a 14.45 mm rim and a raised 12.6 mm square with rounded edges (see the [photo check](#photo-check)). The die's size is estimated, the lid hides it on the real board.
- **Wireless module**: under the 0.15 mm sheet of its shield, a 5 × 4.5 mm chip and twelve small parts, estimated: the shield hides them on the real board.
- **Small parts**: from the drawing's outlines. Rectangles below 3.5 mm² are chip capacitors and resistors with tin end caps, each 22 % of their length. Larger rectangles on the top are chips with tin pads at 0.5 mm along their edges. The pad count isn't from a drawing. Rotated or other outlines are plain prisms. The drawing notes that not all of the board's components are shown, so the model has fewer small parts than the real board.
- **Bottom**: the microSD socket without a card, the GPIO and PoE tails, the solder joints of the Ethernet and USB pins and pegs, and the small bottom parts. The bottom part J5 near the USB 2.0 stack is left out: its height is unknown.
- **J2** (RUN, GLOBAL_EN): three Ø1.0 mm holes through the board with gold rings on both sides, without pins as on the real board.
- **Board**: the cut edges show pale FR-4, the mounting holes' and J2's walls are plated (gold). No copper traces and no silkscreen.

## Measurements that would improve the model

- Calipers on the board's thickness and on the microSD socket's height. Both change the bottom and the height by tenths of a millimetre.
- A side photo with a ruler, to check the connector heights the drawing doesn't give.
- A top photo taken from further away (less perspective), to check the USB stacks' width.
