# Jetson Nano Developer Kit B01

A display model of the NVIDIA Jetson Nano Developer Kit B01: the carrier board P3449-B01 with the module P3448 and its heatsink. Remo carries one. The model was created independently, from NVIDIA's public documents and pictures and a public forum post. No geometry was imported, converted, traced or sampled from another 3D model. NVIDIA's own 3D model of the kit is behind a developer login and wasn't downloaded or looked at. No logos and no printed text.

Generator: [`generators/jetson_nano_dev_kit_b01.py`](../generators/jetson_nano_dev_kit_b01.py).

## Mesh

`jetson_nano_dev_kit_b01`: one mesh with 11 materials (black carrier, green module, dark heatsink, silver shells, gold pins, blue USB 3.0 tongues, white fan header, brown latches, LEDs and others). Its sub-parts are the carrier, the DC jack, the HDMI and DisplayPort stack, the USB stacks, the Ethernet jack, micro-USB, the module, the heatsink, the carrier's other parts (headers, camera connectors, fan header, M.2 slot) and the bottom side. Every part's solid is closed with outward normals.

## Frame

Millimetres in the STL file, metres in the COLLADA and glTF files. The carrier's bottom face is at z = 0. The carrier lies from x = -28.982 to 71.018 mm and from y = -38.847 to 40.153 mm. The port row is at +y and the 40-pin header at -x. While drawing, the generator uses the carrier's own coordinates: u from the left edge (DC jack side) to the right edge (40-pin header side), v from the port edge inwards. They map to (71.018 - u, 40.153 - v): a turn by 180° about z and a shift.

The frame is the frame of the Jetson Nano mesh Remo used before. Only that mesh's bounding box, origin and axes were read, to align the frame:

| | min (mm) | max (mm) |
|---|---|---|
| Earlier mesh | (-28.982, -38.847, -2.230) | (71.018, 42.175, 28.276) |
| This mesh | (-28.982, -38.847, -1.950) | (71.018, 42.223, 28.250) |

Independent check: the centres of the four standoff bores of Remo's printed Jetson deck lie within 0.051 mm of this model's mounting holes, at the URDF's position of the kit.

## Sources

1. **NVIDIA Jetson Nano Developer Kit User Guide**, DA_09402_003, 17 Dec 2019: <https://developer.download.nvidia.com/embedded/L4T/r32-3-1_Release_v1.0/Jetson_Nano_Developer_Kit_User_Guide.pdf>. Page 2, "Developer kit module and carrier board": front, rear and top views. The figure shows the A02 carrier. The port row, the 40-pin header, the PoE header, the fan header, the M.2 slot, the SO-DIMM connector and the module's standoffs are the same on B01. Page 3: J25 is 9.5 mm deep for a 2.1 × 5.5 mm plug, J32 and J33 are stacks of two USB 3.0 Type A, J6 is the HDMI and DisplayPort stack.
2. **NVIDIA Jetson Nano Developer Kit data sheet** (2022), as hosted by Farnell: <https://www.farnell.com/datasheets/3799946.pdf>. "Mechanical 100mm x 79mm x 30.21mm (height includes carrier board, module, and thermal solution.)", two 15-pin MIPI CSI-2 camera connectors, 4 USB 3.0 Type-A, 1 USB 2.0 Micro-B, 40-pin header, 4-pin fan header, 4-pin PoE header.
3. **NVIDIA, "Get Started With Jetson Nano Developer Kit"**: <https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit>. Its B01 picture shows the connector order and colours, and the heatsink's fins along the port edge with two flat-topped fins that carry the fan holes. Its microSD picture shows the microSD slot under the module's back edge and the green module board.
4. **NVIDIA Jetson Nano product page**: <https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-nano/product-development/>. Module 69.6 × 45 mm, 260-pin SO-DIMM.
5. **Mounting holes**: NVIDIA developer forum, "Jetson Nano Dev kit mounting holes?" (2023): <https://forums.developer.nvidia.com/t/jetson-nano-dev-kit-mounting-holes/239242>. The dimensioned board layout posted there: holes Ø2.75 mm at (0, 0), (86, 0), (0, 58) and (86, 58), the board from (-4, -17) to (96, 62), so 100 × 79 mm, the ports along the -17 edge. Only these numbers were used.
6. **B01 changes**: CNX Software, "NVIDIA Jetson Nano Developer Kit-B01 Gets an Extra Camera Connector", 17 Jan 2020: <https://www.cnx-software.com/2020/01/17/nvidia-jetson-nano-developer-kit-b01-gets-an-extra-camera-connector/>. Its B01 photo gives the two camera connectors along the left edge, the moved J48 jumper and the removed serial and button headers.
7. **Fan holes**: Noctua's FAQ, <https://www.noctua.at/cn/support/faqs/which-is-the-best-fan-for-the-nvidia-jetson-nano>. 40 mm fans fit the heatsink's holes, which are Ø2.5 mm and not threaded. A 40 mm fan's holes are 32 mm apart.

## How the numbers were read

Board coordinates: u from the left edge (DC jack side), v from the port edge inwards, z from the carrier's bottom face.

- **Top view (user guide)**: the carrier's outline at x = 201.5 and 709.5 px (100 mm, 5.08 px/mm), the back edge at y = 47.5 px, the port edge at y = 449 px (79.0 mm, as in the 2022 data sheet). u = (x - 201.5) / 5.08, v = (449 - y) / 5.08. Results in mm: J25 u 1.67 to 10.53, v -1.97 to 12.8. J6 u 15.0 to 33.0, v -1.52 to 16.3. J32 u 35.8 to 49.6. J33 u 52.9 to 66.6. J43 u 68.9 to 85.0, v -1.97 to 19.8. J28 u 88.3 to 95.8, v -0.94. DS3 at u 96 to 98.5, v 11. J41 pins at u 88.8 and 91.3, pitch 2.54 from v 70.0. J38 u 93.8 to 98.5, v 72.3 to 77.2. J15 u 72.7 to 82.9, v 21.5 to 27.4. J2 u 11.5 to 84.7 with its latches, v 28.0 to 36.6. Module standoffs at (16.3, 75.4) and (79.9, 75.4). J18 u 69.2 to 78.4, v 46.1 to 68.0. M.2 screw at (44.4, 57.3).
- **Front view (user guide)**: scale 3.71 px/mm, from J25's left edge (u 1.67) to J28's right edge (u 95.8). Heights above the carrier's top face: J25 11.05, J6 18.6, USB stacks 16.0, J43 13.7, J28 3.0, heatsink 26.5 mm. The HDMI opening from 1.6 to 5.7 mm and the DisplayPort opening from 12.7 to 17.5 mm above the top face.
- **Rear view (user guide)**: scale 3.80 px/mm. Heatsink top 26.8 mm above the top face, heatsink u 18.4 to 77.4, module board z 9.0 to 10.0, microSD slot u 37.1 to 48.4.
- **Heights**: heatsink top = 1.6 + (26.5 + 26.8) / 2 = 28.25 mm. With the data sheet's 30.21 mm total height, the solder tails reach 1.96 mm below the carrier.
- **Holes**: the forum layout's (0, 0) to (86, 58), in a board from (-4, -17), gives u 4 and 90, v 17 and 75. The 40-pin header's pins at u 88.8 and 91.3 lie on the line of the right holes, as in that layout.
- **Camera connectors (B01)**: in the CNX photo, the pin rows of the two connectors lie 7.8 to 22.1 mm and 33.0 to 46.2 mm from the back left hole along the left edge, scaled by the 58 mm between the two left holes. Centres at v ≈ 60.0 and 35.4.

## Estimated or simplified

- **Carrier thickness 1.6 mm** (not published), corner radius 1.0 mm.
- **Heatsink**: outline and height from the user guide's views. The fins follow NVIDIA's picture in a simplified way: a 4 mm base on the module, one low fin at the front, two flat-topped fins 32 mm apart with the four Ø2.5 mm fan holes, three fins between them, all straight. The real fins splay a little.
- **Module**: a plain 69.6 × 45 × 1.0 mm green board from z 9.2 to 10.2 mm, with its microSD slot under the back edge. The rear view reads 9.0 to 10.0 mm at ±0.3 mm per pixel. 0.2 mm higher leaves room for a 1.4 mm microSD socket underneath. The module's parts under the heatsink and its gold fingers aren't modelled.
- **Camera connectors**: two plain 15-pin flat cable connectors (20.8 × 5.5 × 2.5 mm) with a dark latch. Their position along the edge is read from a photo, ±2 mm.
- **J48** is a 2-pin header without a jumper cap, centred at u 13.0, v 20.5 mm, from the photo, ±3 mm. The B01's 12-pin button header J50 isn't modelled: its position isn't in the sources.
- **Connector insides**: standard opening sizes, simplified tongues and contacts. The carrier's small surface parts aren't modelled: none are in the public drawings.
- **Bottom**: the 40-pin, PoE and J48 tails and approximate solder joints under the port connectors.
- **No M.2 card, fan or camera cables**: the kit as shipped.

## Measurements that would improve the model

- The heatsink's depth, front to back, and the number and shape of its fins: a top photo with a ruler.
- The module's height above the carrier and the base of the heatsink: calipers from the carrier to the module's edge.
- A photo of the left edge, for the exact places of the two camera connectors, J48 and J50.
